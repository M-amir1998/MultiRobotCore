#include "multirobotcore/scenarios/warehouse_mission.hpp"

#include "multirobotcore/allocation/hungarian.hpp"
#include "multirobotcore/avoidance/avoidance.hpp"
#include "multirobotcore/avoidance/simulation_adapter.hpp"
#include "multirobotcore/mapf/conflict_based_search.hpp"
#include "multirobotcore/mapf/conflicts.hpp"
#include "multirobotcore/mapf/instance.hpp"
#include "multirobotcore/mapf/planners.hpp"
#include "multirobotcore/math/numerics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace multirobotcore::scenarios {
namespace {

using mapf::Cell;

/// Portable sampling: std::mt19937_64 output is fully specified by the
/// standard, the standard distributions are not, so results are identical
/// across compilers only with explicit transforms like these.
double uniform01(std::mt19937_64& rng) {
  return static_cast<double>(rng() >> 11U) * (1.0 / 9007199254740992.0);  // 53 bits
}

std::size_t uniform_index(std::mt19937_64& rng, std::size_t n) {
  return static_cast<std::size_t>(uniform01(rng) * static_cast<double>(n)) % n;
}

double exponential(std::mt19937_64& rng, double rate) {
  return -std::log(1.0 - uniform01(rng)) / rate;
}

}  // namespace

std::string_view to_string(WarehouseRobotState state) noexcept {
  switch (state) {
    case WarehouseRobotState::kIdle:
      return "idle";
    case WarehouseRobotState::kToPickup:
      return "to_pickup";
    case WarehouseRobotState::kPicking:
      return "picking";
    case WarehouseRobotState::kToDropoff:
      return "to_dropoff";
    case WarehouseRobotState::kDropping:
      return "dropping";
  }
  return "unknown";
}

// --- WarehouseCoordinator -----------------------------------------------------------

WarehouseCoordinator::WarehouseCoordinator(const WarehouseMissionConfig& config)
    : config_(config),
      warehouse_(mapf::make_warehouse(config.layout)),
      frame_{Vec2::Zero(), config.cell_size},
      rng_(config.seed) {
  if (config_.num_robots == 0 || !(config_.step_duration > 0.0) || !(config_.cell_size > 0.0)) {
    throw std::invalid_argument("WarehouseCoordinator: invalid configuration");
  }
  for (const Cell c : warehouse_.stations) {
    (c.x == 0 ? stations_ : homes_).push_back(c);
  }
  if (homes_.size() < config_.num_robots) {
    throw std::invalid_argument("WarehouseCoordinator: more robots than home cells");
  }
  robots_.resize(config_.num_robots);
  plan_.resize(config_.num_robots);
  for (std::size_t i = 0; i < robots_.size(); ++i) {
    robots_[i].home = homes_[(i * homes_.size()) / robots_.size()];
    robots_[i].goal = robots_[i].home;
    plan_[i] = {robots_[i].home};
  }
}

std::vector<Pose2> WarehouseCoordinator::home_poses() const {
  std::vector<Pose2> poses;
  for (const Robot& r : robots_) {
    poses.push_back({frame_.to_world(r.home), math::kPi});
  }
  return poses;
}

math::ObstacleSet WarehouseCoordinator::shelf_obstacles() const {
  math::ObstacleSet out;
  const auto& map = warehouse_.map;
  for (int y = 0; y < map.height(); ++y) {
    for (int x = 0; x < map.width(); ++x) {
      if (!map.is_free({x, y})) {
        const Vec2 lo = frame_.origin + config_.cell_size * Vec2(x, y);
        out.boxes.push_back({lo, lo + Vec2(config_.cell_size, config_.cell_size)});
      }
    }
  }
  return out;
}

Cell WarehouseCoordinator::planned_cell(std::size_t robot, std::size_t step) const {
  return mapf::position_at(plan_[robot], step >= plan_start_ ? step - plan_start_ : 0);
}

double WarehouseCoordinator::grid_distance(Cell from, Cell to) {
  const int key = warehouse_.map.index(to);
  auto it = std::find_if(distance_cache_.begin(), distance_cache_.end(),
                         [key](const auto& entry) { return entry.first == key; });
  if (it == distance_cache_.end()) {
    distance_cache_.emplace_back(key, warehouse_.map.distances_from(to));
    it = std::prev(distance_cache_.end());
  }
  const int d = it->second[static_cast<std::size_t>(warehouse_.map.index(from))];
  return d < 0 ? std::numeric_limits<double>::infinity() : static_cast<double>(d);
}

std::optional<std::size_t> WarehouseCoordinator::request_task(double time,
                                                              std::optional<Vec2> near) {
  std::vector<Cell> free_pickups;
  for (const Cell c : warehouse_.pickups) {
    const bool taken = std::any_of(tasks_.begin(), tasks_.end(), [&](const WarehouseTask& t) {
      return t.delivered_time < 0.0 && t.pickup == c;
    });
    if (!taken) {
      free_pickups.push_back(c);
    }
  }
  if (free_pickups.empty()) {
    return std::nullopt;
  }
  Cell chosen = free_pickups.front();
  if (near) {
    double best = std::numeric_limits<double>::infinity();
    for (const Cell c : free_pickups) {
      const double d = (frame_.to_world(c) - *near).norm();
      if (d < best) {
        best = d;
        chosen = c;
      }
    }
  } else {
    chosen = free_pickups[uniform_index(rng_, free_pickups.size())];
  }
  WarehouseTask task;
  task.id = tasks_.size();
  task.pickup = chosen;
  task.arrival_time = time;
  tasks_.push_back(task);
  return task.id;
}

bool WarehouseCoordinator::replan(std::size_t step, double time) {
  mapf::Instance instance;
  instance.map = warehouse_.map;
  for (std::size_t i = 0; i < robots_.size(); ++i) {
    instance.agents.push_back({planned_cell(i, step), robots_[i].goal});
  }
  const auto start = std::chrono::steady_clock::now();
  auto try_prioritized = [&] {
    mapf::PrioritizedOptions options;
    options.order = mapf::PriorityOrder::kShortestFirst;
    options.seed = config_.seed + step;
    options.max_attempts = config_.prioritized_attempts;
    return mapf::plan_prioritized(instance, options);
  };
  auto try_cbs = [&] {
    mapf::CbsOptions options;
    options.time_limit_s = config_.cbs_time_limit;
    return mapf::ConflictBasedSearch(options).solve(instance);
  };
  mapf::PlannerResult planned;
  if (config_.planner == "cbs") {
    planned = try_cbs();
    if (!planned.solved()) {
      planned = try_prioritized();
    }
  } else {
    planned = try_prioritized();
    if (!planned.solved()) {
      ++stats_.cbs_fallbacks;
      planned = try_cbs();
    }
  }
  const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  stats_.total_planning_ms += ms;
  stats_.max_planning_ms = std::max(stats_.max_planning_ms, ms);
  if (!planned.solved()) {
    ++stats_.planning_failures;
    events_.push_back({time, "planning failed: previous plan kept"});
    return false;
  }
  plan_ = planned.paths;
  plan_start_ = step;
  ++stats_.replans;
  stats_.plan_conflicts += mapf::count_conflicts(plan_);
  return true;
}

void WarehouseCoordinator::process_step(std::size_t k, double t, std::span<const Vec2> positions) {
  // Goal arrivals and service completion.
  for (std::size_t i = 0; i < robots_.size(); ++i) {
    Robot& r = robots_[i];
    const Cell here = planned_cell(i, k);
    const bool at_goal =
        here == r.goal && (positions[i] - frame_.to_world(r.goal)).norm() < 0.3 * config_.cell_size;
    switch (r.state) {
      case WarehouseRobotState::kToPickup:
        if (at_goal) {
          r.state = WarehouseRobotState::kPicking;
          r.service_until = k + config_.service_steps;
        }
        break;
      case WarehouseRobotState::kPicking:
        if (k >= r.service_until) {
          std::optional<Cell> best;
          double best_d = std::numeric_limits<double>::infinity();
          for (const Cell st : stations_) {
            const bool taken = std::any_of(robots_.begin(), robots_.end(), [&](const Robot& o) {
              return (o.state == WarehouseRobotState::kToDropoff ||
                      o.state == WarehouseRobotState::kDropping) &&
                     o.goal == st;
            });
            const double d = grid_distance(here, st);
            if (!taken && d < best_d) {
              best_d = d;
              best = st;
            }
          }
          if (best) {
            WarehouseTask& task = tasks_[*r.task];
            task.picked_time = t;
            task.dropoff = *best;
            r.state = WarehouseRobotState::kToDropoff;
            r.goal = *best;
            dirty_ = true;
          }
        }
        break;
      case WarehouseRobotState::kToDropoff:
        if (at_goal) {
          r.state = WarehouseRobotState::kDropping;
          r.service_until = k + config_.service_steps;
        }
        break;
      case WarehouseRobotState::kDropping:
        if (k >= r.service_until) {
          WarehouseTask& task = tasks_[*r.task];
          task.delivered_time = t;
          ++stats_.tasks_completed;
          events_.push_back(
              {t, "robot " + std::to_string(i) + " delivered task " + std::to_string(task.id)});
          r.task.reset();
          r.state = WarehouseRobotState::kIdle;
          r.goal = r.home;
          dirty_ = true;
        }
        break;
      case WarehouseRobotState::kIdle:
        break;
    }
  }

  // Allocation: idle robots x unassigned tasks, grid travel distance as cost.
  std::vector<std::size_t> idle;
  for (std::size_t i = 0; i < robots_.size(); ++i) {
    if (robots_[i].state == WarehouseRobotState::kIdle) {
      idle.push_back(i);
    }
  }
  std::vector<std::size_t> open;
  for (std::size_t j = 0; j < tasks_.size(); ++j) {
    if (!tasks_[j].robot) {
      open.push_back(j);
    }
  }
  if (!idle.empty() && !open.empty()) {
    allocation::CostMatrix costs(static_cast<Eigen::Index>(idle.size()),
                                 static_cast<Eigen::Index>(open.size()));
    for (std::size_t a = 0; a < idle.size(); ++a) {
      for (std::size_t b = 0; b < open.size(); ++b) {
        costs(static_cast<Eigen::Index>(a), static_cast<Eigen::Index>(b)) =
            grid_distance(planned_cell(idle[a], k), tasks_[open[b]].pickup);
      }
    }
    const auto assignment = allocation::solve_hungarian(costs);
    for (std::size_t a = 0; a < idle.size(); ++a) {
      if (const auto b = assignment.robot_to_task[a]) {
        Robot& r = robots_[idle[a]];
        WarehouseTask& task = tasks_[open[*b]];
        task.robot = idle[a];
        task.assigned_time = t;
        r.task = open[*b];
        r.state = WarehouseRobotState::kToPickup;
        r.goal = task.pickup;
        dirty_ = true;
      }
    }
  }

  if (dirty_ && replan(k, t)) {
    dirty_ = false;
  }
}

std::vector<Vec2> WarehouseCoordinator::update(double time, std::span<const Vec2> positions) {
  if (positions.size() != robots_.size()) {
    throw std::invalid_argument("WarehouseCoordinator::update: one position per robot required");
  }
  const auto k = static_cast<std::size_t>(std::floor(time / config_.step_duration + 1e-9));
  if (!last_step_ || k > *last_step_) {
    process_step(k, time, positions);
    last_step_ = k;
  }
  const double phase = std::clamp(
      (time - static_cast<double>(k) * config_.step_duration) / config_.step_duration, 0.0, 1.0);
  std::vector<Vec2> preferred(robots_.size());
  for (std::size_t i = 0; i < robots_.size(); ++i) {
    const Vec2 a = frame_.to_world(planned_cell(i, k));
    const Vec2 b = frame_.to_world(planned_cell(i, k + 1));
    const Vec2 reference = a + phase * (b - a);
    const Vec2 feedforward = (b - a) / config_.step_duration;
    const Vec2 error = reference - positions[i];
    stats_.max_tracking_error = std::max(stats_.max_tracking_error, error.norm());
    preferred[i] = math::clamp_norm(feedforward + config_.tracking_gain * error, config_.max_speed);
  }
  return preferred;
}

std::vector<WarehouseRobotState> WarehouseCoordinator::robot_states() const {
  std::vector<WarehouseRobotState> out;
  for (const Robot& r : robots_) {
    out.push_back(r.state);
  }
  return out;
}

std::vector<std::optional<std::size_t>> WarehouseCoordinator::robot_task_ids() const {
  std::vector<std::optional<std::size_t>> out;
  for (const Robot& r : robots_) {
    out.push_back(r.task ? std::optional<std::size_t>(tasks_[*r.task].id) : std::nullopt);
  }
  return out;
}

std::vector<Vec2> WarehouseCoordinator::planned_route(std::size_t robot, const Vec2& position,
                                                      std::size_t steps) const {
  const std::size_t k = last_step_.value_or(0);
  std::vector<Vec2> route{position};
  for (std::size_t s = 1; s <= steps; ++s) {
    route.push_back(frame_.to_world(planned_cell(robot, k + s)));
  }
  return route;
}

std::vector<std::pair<double, Vec2>> WarehouseCoordinator::timed_plan(std::size_t robot) const {
  std::vector<std::pair<double, Vec2>> out;
  const auto& path = plan_.at(robot);
  for (std::size_t s = 0; s < path.size(); ++s) {
    out.emplace_back(static_cast<double>(plan_start_ + s) * config_.step_duration,
                     frame_.to_world(path[s]));
  }
  return out;
}

// --- Simulated mission ----------------------------------------------------------------

WarehouseMissionResult run_warehouse_mission(const WarehouseMissionConfig& cfg) {
  if (!(cfg.dt > 0.0)) {
    throw std::invalid_argument("warehouse mission: dt must be positive");
  }
  const auto substeps = static_cast<std::size_t>(std::llround(cfg.step_duration / cfg.dt));
  if (substeps == 0 ||
      std::abs(static_cast<double>(substeps) * cfg.dt - cfg.step_duration) > 1e-9) {
    throw std::invalid_argument("warehouse mission: step_duration must be a multiple of dt");
  }
  WarehouseCoordinator coordinator(cfg);
  const std::size_t n = coordinator.num_robots();

  // Task request times: an initial batch, then a Poisson process.
  std::mt19937_64 arrivals(cfg.seed ^ 0xA5A5A5A5ULL);
  std::vector<double> request_times(cfg.initial_tasks, 0.0);
  if (cfg.arrival_rate > 0.0) {
    for (double t = exponential(arrivals, cfg.arrival_rate); t < cfg.arrival_horizon;
         t += exponential(arrivals, cfg.arrival_rate)) {
      request_times.push_back(t);
    }
  }
  std::size_t next_request = 0;

  simulation::WorldConfig world;
  world.dt = cfg.dt;
  world.record_every = cfg.record_every;
  const auto& map = coordinator.warehouse().map;
  world.bounds =
      math::AABB{Vec2::Zero(), Vec2(map.width() * cfg.cell_size, map.height() * cfg.cell_size)};
  world.obstacles = coordinator.shelf_obstacles();
  simulation::RobotSpec spec;
  spec.radius = cfg.robot_radius;
  spec.max_speed = cfg.max_speed;
  spec.max_acceleration = cfg.max_acceleration;
  simulation::Simulator sim(world, std::vector<simulation::RobotSpec>(n, spec),
                            coordinator.home_poses());

  avoidance::AvoidanceConfig avoid_cfg;
  avoid_cfg.method = avoidance::AvoidanceMethod::kOrca;
  avoid_cfg.time_horizon = cfg.orca_time_horizon;
  avoid_cfg.obstacle_time_horizon = 0.5 * cfg.orca_time_horizon;
  avoid_cfg.time_step = cfg.dt;
  avoid_cfg.neighbor_distance = 3.0;
  avoid_cfg.max_neighbors = 8;
  // A short horizon keeps ORCA passive while robots follow the coordinated
  // plan and active only when execution deviations make a contact imminent.
  const avoidance::CollisionAvoider avoider(
      avoid_cfg, cfg.avoid_shelves ? world.obstacles : math::ObstacleSet{});

  WarehouseMissionResult result;
  result.warehouse = coordinator.warehouse();
  result.frame = coordinator.frame();
  result.homes = coordinator.homes();
  result.stations = coordinator.stations();
  auto record = [&] {
    result.robot_states.push_back(coordinator.robot_states());
    result.robot_tasks.push_back(coordinator.robot_task_ids());
    std::vector<std::vector<Vec2>> routes;
    for (std::size_t i = 0; i < n; ++i) {
      routes.push_back(coordinator.planned_route(i, sim.robot(i).position, cfg.preview_steps));
    }
    result.routes.push_back(std::move(routes));
    std::vector<std::size_t> visible;
    for (const auto& task : coordinator.tasks()) {
      if (task.delivered_time < 0.0) {
        visible.push_back(task.id);
      }
    }
    result.visible_tasks.push_back(std::move(visible));
  };
  record();

  const auto total_steps = static_cast<std::size_t>(std::llround(cfg.duration / cfg.dt));
  for (std::size_t s = 0; s < total_steps; ++s) {
    const double t = sim.time();
    if (s % substeps == 0) {
      while (next_request < request_times.size() && request_times[next_request] <= t + 1e-9) {
        (void)coordinator.request_task(request_times[next_request]);
        ++next_request;
      }
    }
    const auto preferred = coordinator.update(t, sim.positions());
    if (cfg.local_avoidance) {
      sim.step(avoidance::safe_velocities(avoider, sim, preferred));
      result.orca_infeasible += avoider.last_infeasible_count();
    } else {
      sim.step(preferred);
    }
    if (sim.step_count() % cfg.record_every == 0) {
      record();
    }
  }

  const CoordinatorStats& stats = coordinator.stats();
  result.frames = sim.frames();
  result.metrics = sim.metrics();
  result.tasks = coordinator.tasks();
  result.events = coordinator.events();
  result.tasks_completed = stats.tasks_completed;
  result.replans = stats.replans;
  result.planning_failures = stats.planning_failures;
  result.cbs_fallbacks = stats.cbs_fallbacks;
  result.max_planning_ms = stats.max_planning_ms;
  result.plan_conflicts = stats.plan_conflicts;
  result.max_tracking_error = stats.max_tracking_error;
  const std::size_t planning_calls = stats.replans + stats.planning_failures;
  result.mean_planning_ms =
      planning_calls > 0 ? stats.total_planning_ms / static_cast<double>(planning_calls) : 0.0;
  double sum = 0.0;
  for (const auto& task : result.tasks) {
    if (task.delivered_time >= 0.0) {
      sum += task.delivered_time - task.arrival_time;
    }
  }
  result.mean_completion_time =
      result.tasks_completed > 0 ? sum / static_cast<double>(result.tasks_completed) : 0.0;
  result.throughput_per_minute = 60.0 * static_cast<double>(result.tasks_completed) / cfg.duration;
  return result;
}

// --- JSON ------------------------------------------------------------------------------

WarehouseMissionConfig warehouse_mission_config_from_json(const io::Json& j) {
  WarehouseMissionConfig c;
  if (j.contains("layout")) {
    const auto& l = j.at("layout");
    c.layout.shelf_block_columns = l.value("shelf_block_columns", c.layout.shelf_block_columns);
    c.layout.shelf_block_rows = l.value("shelf_block_rows", c.layout.shelf_block_rows);
    c.layout.shelf_length = l.value("shelf_length", c.layout.shelf_length);
    c.layout.shelf_depth = l.value("shelf_depth", c.layout.shelf_depth);
    c.layout.aisle_width = l.value("aisle_width", c.layout.aisle_width);
    c.layout.station_margin = l.value("station_margin", c.layout.station_margin);
  }
  c.num_robots = j.value("num_robots", c.num_robots);
  c.cell_size = j.value("cell_size", c.cell_size);
  c.step_duration = j.value("step_duration", c.step_duration);
  c.dt = j.value("dt", c.dt);
  c.duration = j.value("duration", c.duration);
  c.seed = j.value("seed", c.seed);
  c.initial_tasks = j.value("initial_tasks", c.initial_tasks);
  c.arrival_rate = j.value("arrival_rate", c.arrival_rate);
  c.arrival_horizon = j.value("arrival_horizon", c.arrival_horizon);
  c.service_steps = j.value("service_steps", c.service_steps);
  c.robot_radius = j.value("robot_radius", c.robot_radius);
  c.max_speed = j.value("max_speed", c.max_speed);
  c.max_acceleration = j.value("max_acceleration", c.max_acceleration);
  c.tracking_gain = j.value("tracking_gain", c.tracking_gain);
  c.local_avoidance = j.value("local_avoidance", c.local_avoidance);
  c.avoid_shelves = j.value("avoid_shelves", c.avoid_shelves);
  c.orca_time_horizon = j.value("orca_time_horizon", c.orca_time_horizon);
  c.planner = j.value("planner", c.planner);
  c.prioritized_attempts = j.value("prioritized_attempts", c.prioritized_attempts);
  c.cbs_time_limit = j.value("cbs_time_limit", c.cbs_time_limit);
  c.record_every = j.value("record_every", c.record_every);
  c.preview_steps = j.value("preview_steps", c.preview_steps);
  return c;
}

io::Json to_json(const WarehouseMissionResult& r) {
  auto cells = [](const std::vector<Cell>& v) {
    io::Json out = io::Json::array();
    for (const Cell c : v) {
      out.push_back(io::Json::array({c.x, c.y}));
    }
    return out;
  };
  io::Json states = io::Json::array();
  for (const auto& frame_states : r.robot_states) {
    io::Json row = io::Json::array();
    for (const auto s : frame_states) {
      row.push_back(static_cast<int>(s));
    }
    states.push_back(std::move(row));
  }
  io::Json routes = io::Json::array();
  for (const auto& frame_routes : r.routes) {
    io::Json row = io::Json::array();
    for (const auto& route : frame_routes) {
      row.push_back(io::to_json(std::span<const Vec2>(route), 3));
    }
    routes.push_back(std::move(row));
  }
  io::Json robot_tasks = io::Json::array();
  for (const auto& frame_tasks : r.robot_tasks) {
    io::Json row = io::Json::array();
    for (const auto& t : frame_tasks) {
      row.push_back(t ? io::Json(*t) : io::Json(nullptr));
    }
    robot_tasks.push_back(std::move(row));
  }
  io::Json tasks = io::Json::array();
  for (const auto& t : r.tasks) {
    tasks.push_back(io::Json{
        {"id", t.id},
        {"pickup", io::Json::array({t.pickup.x, t.pickup.y})},
        {"dropoff", t.dropoff ? io::Json::array({t.dropoff->x, t.dropoff->y}) : io::Json(nullptr)},
        {"arrival", io::round_to(t.arrival_time, 3)},
        {"assigned", io::round_to(t.assigned_time, 3)},
        {"picked", io::round_to(t.picked_time, 3)},
        {"delivered", io::round_to(t.delivered_time, 3)},
        {"robot", t.robot ? io::Json(*t.robot) : io::Json(nullptr)}});
  }
  io::Json events = io::Json::array();
  for (const auto& e : r.events) {
    events.push_back(io::Json{{"time", io::round_to(e.time, 3)}, {"label", e.label}});
  }
  io::Json state_names = io::Json::array();
  for (const auto s :
       {WarehouseRobotState::kIdle, WarehouseRobotState::kToPickup, WarehouseRobotState::kPicking,
        WarehouseRobotState::kToDropoff, WarehouseRobotState::kDropping}) {
    state_names.push_back(std::string(to_string(s)));
  }
  return io::Json{{"map", r.warehouse.map.to_ascii()},
                  {"cell_size", r.frame.cell_size},
                  {"homes", cells(r.homes)},
                  {"stations", cells(r.stations)},
                  {"pickups", cells(r.warehouse.pickups)},
                  {"state_names", state_names},
                  {"trajectory", io::frames_to_json(r.frames, 3)},
                  {"robot_states", states},
                  {"routes", routes},
                  {"visible_tasks", r.visible_tasks},
                  {"robot_tasks", robot_tasks},
                  {"tasks", tasks},
                  {"events", events},
                  {"summary",
                   {{"tasks_requested", r.tasks.size()},
                    {"tasks_completed", r.tasks_completed},
                    {"mean_completion_time", io::round_to(r.mean_completion_time, 3)},
                    {"throughput_per_minute", io::round_to(r.throughput_per_minute, 3)},
                    {"replans", r.replans},
                    {"planning_failures", r.planning_failures},
                    {"cbs_fallbacks", r.cbs_fallbacks},
                    {"mean_planning_ms", io::round_to(r.mean_planning_ms, 3)},
                    {"max_planning_ms", io::round_to(r.max_planning_ms, 3)},
                    {"plan_conflicts", r.plan_conflicts},
                    {"max_tracking_error", io::round_to(r.max_tracking_error, 4)},
                    {"orca_infeasible", r.orca_infeasible}}},
                  {"metrics", io::to_json(r.metrics)}};
}

}  // namespace multirobotcore::scenarios
