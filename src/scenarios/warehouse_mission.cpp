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
#include <map>
#include <random>
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

struct RobotPlanState {
  WarehouseRobotState state{WarehouseRobotState::kIdle};
  Cell home;
  Cell goal;
  std::optional<std::size_t> task;
  std::size_t service_until{0};
};

class DistanceCache {
 public:
  explicit DistanceCache(const mapf::GridMap& map) : map_(map) {}

  /// BFS distance (steps) from `from` to `to`; +infinity when unreachable.
  double distance(Cell from, Cell to) {
    const int key = map_.index(to);
    auto it = cache_.find(key);
    if (it == cache_.end()) {
      it = cache_.emplace(key, map_.distances_from(to)).first;
    }
    const int d = it->second[static_cast<std::size_t>(map_.index(from))];
    return d < 0 ? std::numeric_limits<double>::infinity() : static_cast<double>(d);
  }

 private:
  const mapf::GridMap& map_;
  std::map<int, std::vector<int>> cache_;
};

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

WarehouseMissionResult run_warehouse_mission(const WarehouseMissionConfig& cfg) {
  if (cfg.num_robots == 0 || !(cfg.step_duration > 0.0) || !(cfg.dt > 0.0)) {
    throw std::invalid_argument("warehouse mission: invalid configuration");
  }
  const auto substeps = static_cast<std::size_t>(std::llround(cfg.step_duration / cfg.dt));
  if (substeps == 0 ||
      std::abs(static_cast<double>(substeps) * cfg.dt - cfg.step_duration) > 1e-9) {
    throw std::invalid_argument("warehouse mission: step_duration must be a multiple of dt");
  }

  WarehouseMissionResult result;
  result.warehouse = mapf::make_warehouse(cfg.layout);
  const mapf::GridMap& map = result.warehouse.map;
  result.frame = mapf::WorldFrame{Vec2::Zero(), cfg.cell_size};
  const mapf::WorldFrame& frame = result.frame;
  for (const Cell c : result.warehouse.stations) {
    (c.x == 0 ? result.stations : result.homes).push_back(c);
  }
  if (result.homes.size() < cfg.num_robots) {
    throw std::invalid_argument("warehouse mission: more robots than home cells");
  }
  std::mt19937_64 rng(cfg.seed);
  DistanceCache distances(map);

  // Robots start at evenly spread home cells.
  const std::size_t n = cfg.num_robots;
  std::vector<RobotPlanState> robots(n);
  std::vector<Pose2> poses;
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t h = (i * result.homes.size()) / n;
    robots[i].home = result.homes[h];
    robots[i].goal = robots[i].home;
    poses.push_back({frame.to_world(robots[i].home), math::kPi});
  }

  // Task request times: an initial batch, then a Poisson process.
  std::vector<double> request_times(cfg.initial_tasks, 0.0);
  if (cfg.arrival_rate > 0.0) {
    for (double t = exponential(rng, cfg.arrival_rate); t < cfg.arrival_horizon;
         t += exponential(rng, cfg.arrival_rate)) {
      request_times.push_back(t);
    }
  }
  std::size_t next_request = 0;

  simulation::WorldConfig world;
  world.dt = cfg.dt;
  world.record_every = cfg.record_every;
  world.bounds =
      math::AABB{Vec2::Zero(), Vec2(map.width() * cfg.cell_size, map.height() * cfg.cell_size)};
  for (int y = 0; y < map.height(); ++y) {
    for (int x = 0; x < map.width(); ++x) {
      if (!map.is_free({x, y})) {
        const Vec2 lo = frame.origin + cfg.cell_size * Vec2(x, y);
        world.obstacles.boxes.push_back({lo, lo + Vec2(cfg.cell_size, cfg.cell_size)});
      }
    }
  }
  simulation::RobotSpec spec;
  spec.radius = cfg.robot_radius;
  spec.max_speed = cfg.max_speed;
  spec.max_acceleration = cfg.max_acceleration;
  simulation::Simulator sim(world, std::vector<simulation::RobotSpec>(n, spec), poses);

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

  std::vector<mapf::Path> plan(n);
  for (std::size_t i = 0; i < n; ++i) {
    plan[i] = {robots[i].home};
  }
  std::size_t plan_start = 0;
  bool dirty = true;
  double planning_ms_total = 0.0;
  std::size_t planning_calls = 0;

  auto planned_cell = [&](std::size_t i, std::size_t k) {
    return mapf::position_at(plan[i], k >= plan_start ? k - plan_start : 0);
  };
  auto active_pickup = [&](Cell c) {
    return std::any_of(result.tasks.begin(), result.tasks.end(), [&](const WarehouseTask& t) {
      return t.delivered_time < 0.0 && t.pickup == c;
    });
  };
  auto station_taken = [&](Cell c) {
    return std::any_of(robots.begin(), robots.end(), [&](const RobotPlanState& r) {
      return (r.state == WarehouseRobotState::kToDropoff ||
              r.state == WarehouseRobotState::kDropping) &&
             r.goal == c;
    });
  };
  auto event = [&](double t, std::string label) {
    result.events.push_back({t, std::move(label)});
  };

  auto replan = [&](std::size_t k, double t) {
    mapf::Instance instance;
    instance.map = map;
    for (std::size_t i = 0; i < n; ++i) {
      instance.agents.push_back({planned_cell(i, k), robots[i].goal});
    }
    const auto start = std::chrono::steady_clock::now();
    mapf::PlannerResult planned;
    auto try_prioritized = [&] {
      mapf::PrioritizedOptions options;
      options.order = mapf::PriorityOrder::kShortestFirst;
      options.seed = cfg.seed + k;
      options.max_attempts = cfg.prioritized_attempts;
      return mapf::plan_prioritized(instance, options);
    };
    auto try_cbs = [&] {
      mapf::CbsOptions options;
      options.time_limit_s = cfg.cbs_time_limit;
      return mapf::ConflictBasedSearch(options).solve(instance);
    };
    if (cfg.planner == "cbs") {
      planned = try_cbs();
      if (!planned.solved()) {
        planned = try_prioritized();
      }
    } else {
      planned = try_prioritized();
      if (!planned.solved()) {
        ++result.cbs_fallbacks;
        planned = try_cbs();
      }
    }
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    planning_ms_total += ms;
    ++planning_calls;
    result.max_planning_ms = std::max(result.max_planning_ms, ms);
    if (!planned.solved()) {
      ++result.planning_failures;
      event(t, "planning failed: previous plan kept");
      return false;
    }
    plan = planned.paths;
    plan_start = k;
    ++result.replans;
    result.plan_conflicts += mapf::count_conflicts(plan);
    return true;
  };

  const auto total_steps = static_cast<std::size_t>(std::llround(cfg.duration / cfg.dt));
  auto record = [&](std::size_t k) {
    result.robot_states.emplace_back();
    result.routes.emplace_back();
    result.robot_tasks.emplace_back();
    for (std::size_t i = 0; i < n; ++i) {
      result.robot_states.back().push_back(robots[i].state);
      std::vector<Vec2> route{sim.robot(i).position};
      for (std::size_t s = 1; s <= cfg.preview_steps; ++s) {
        route.push_back(frame.to_world(planned_cell(i, k + s)));
      }
      result.routes.back().push_back(std::move(route));
      result.robot_tasks.back().push_back(
          robots[i].task ? std::optional<std::size_t>(result.tasks[*robots[i].task].id)
                         : std::nullopt);
    }
    std::vector<std::size_t> visible;
    for (const auto& task : result.tasks) {
      if (task.delivered_time < 0.0) {
        visible.push_back(task.id);
      }
    }
    result.visible_tasks.push_back(std::move(visible));
  };
  record(0);

  for (std::size_t s = 0; s < total_steps; ++s) {
    const double t = sim.time();
    const std::size_t k = s / substeps;

    if (s % substeps == 0) {
      // --- Task requests ---------------------------------------------------------
      while (next_request < request_times.size() && request_times[next_request] <= t + 1e-9) {
        std::vector<Cell> free_pickups;
        for (const Cell c : result.warehouse.pickups) {
          if (!active_pickup(c)) {
            free_pickups.push_back(c);
          }
        }
        if (!free_pickups.empty()) {
          WarehouseTask task;
          task.id = result.tasks.size();
          task.pickup = free_pickups[uniform_index(rng, free_pickups.size())];
          task.arrival_time = request_times[next_request];
          result.tasks.push_back(task);
        }
        ++next_request;
      }

      // --- Goal arrivals and service completion -----------------------------------
      for (std::size_t i = 0; i < n; ++i) {
        RobotPlanState& r = robots[i];
        const Cell here = planned_cell(i, k);
        const bool at_goal =
            here == r.goal &&
            (sim.robot(i).position - frame.to_world(r.goal)).norm() < 0.3 * cfg.cell_size;
        switch (r.state) {
          case WarehouseRobotState::kToPickup:
            if (at_goal) {
              r.state = WarehouseRobotState::kPicking;
              r.service_until = k + cfg.service_steps;
            }
            break;
          case WarehouseRobotState::kPicking:
            if (k >= r.service_until) {
              std::optional<Cell> best;
              double best_d = std::numeric_limits<double>::infinity();
              for (const Cell st : result.stations) {
                const double d = distances.distance(here, st);
                if (!station_taken(st) && d < best_d) {
                  best_d = d;
                  best = st;
                }
              }
              if (best) {
                WarehouseTask& task = result.tasks[*r.task];
                task.picked_time = t;
                task.dropoff = *best;
                r.state = WarehouseRobotState::kToDropoff;
                r.goal = *best;
                dirty = true;
              }
            }
            break;
          case WarehouseRobotState::kToDropoff:
            if (at_goal) {
              r.state = WarehouseRobotState::kDropping;
              r.service_until = k + cfg.service_steps;
            }
            break;
          case WarehouseRobotState::kDropping:
            if (k >= r.service_until) {
              WarehouseTask& task = result.tasks[*r.task];
              task.delivered_time = t;
              ++result.tasks_completed;
              event(t, "robot " + std::to_string(i) + " delivered task " + std::to_string(task.id));
              r.task.reset();
              r.state = WarehouseRobotState::kIdle;
              r.goal = r.home;
              dirty = true;
            }
            break;
          case WarehouseRobotState::kIdle:
            break;
        }
      }

      // --- Allocation: idle robots x unassigned tasks (grid travel cost) -----------
      std::vector<std::size_t> idle;
      for (std::size_t i = 0; i < n; ++i) {
        if (robots[i].state == WarehouseRobotState::kIdle) {
          idle.push_back(i);
        }
      }
      std::vector<std::size_t> open;
      for (std::size_t j = 0; j < result.tasks.size(); ++j) {
        if (!result.tasks[j].robot) {
          open.push_back(j);
        }
      }
      if (!idle.empty() && !open.empty()) {
        allocation::CostMatrix costs(static_cast<Eigen::Index>(idle.size()),
                                     static_cast<Eigen::Index>(open.size()));
        for (std::size_t a = 0; a < idle.size(); ++a) {
          for (std::size_t b = 0; b < open.size(); ++b) {
            costs(static_cast<Eigen::Index>(a), static_cast<Eigen::Index>(b)) =
                distances.distance(planned_cell(idle[a], k), result.tasks[open[b]].pickup);
          }
        }
        const auto assignment = allocation::solve_hungarian(costs);
        for (std::size_t a = 0; a < idle.size(); ++a) {
          if (const auto b = assignment.robot_to_task[a]) {
            RobotPlanState& r = robots[idle[a]];
            WarehouseTask& task = result.tasks[open[*b]];
            task.robot = idle[a];
            task.assigned_time = t;
            r.task = open[*b];
            r.state = WarehouseRobotState::kToPickup;
            r.goal = task.pickup;
            dirty = true;
          }
        }
      }

      // --- Multi-agent path finding ---------------------------------------------------
      if (dirty && replan(k, t)) {
        dirty = false;
      }
    }

    // --- Trajectory tracking + local avoidance ---------------------------------------
    const double phase = (t - static_cast<double>(k) * cfg.step_duration) / cfg.step_duration;
    std::vector<Vec2> preferred(n);
    for (std::size_t i = 0; i < n; ++i) {
      const Vec2 a = frame.to_world(planned_cell(i, k));
      const Vec2 b = frame.to_world(planned_cell(i, k + 1));
      const Vec2 reference = a + phase * (b - a);
      const Vec2 feedforward = (b - a) / cfg.step_duration;
      const Vec2 error = reference - sim.robot(i).position;
      result.max_tracking_error = std::max(result.max_tracking_error, error.norm());
      preferred[i] = math::clamp_norm(feedforward + cfg.tracking_gain * error, cfg.max_speed);
    }
    if (cfg.local_avoidance) {
      sim.step(avoidance::safe_velocities(avoider, sim, preferred));
      result.orca_infeasible += avoider.last_infeasible_count();
    } else {
      sim.step(preferred);
    }
    if (sim.step_count() % cfg.record_every == 0) {
      record(sim.step_count() / substeps);
    }
  }

  result.frames = sim.frames();
  result.metrics = sim.metrics();
  double sum = 0.0;
  for (const auto& task : result.tasks) {
    if (task.delivered_time >= 0.0) {
      sum += task.delivered_time - task.arrival_time;
    }
  }
  result.mean_completion_time =
      result.tasks_completed > 0 ? sum / static_cast<double>(result.tasks_completed) : 0.0;
  result.throughput_per_minute = 60.0 * static_cast<double>(result.tasks_completed) / cfg.duration;
  result.mean_planning_ms =
      planning_calls > 0 ? planning_ms_total / static_cast<double>(planning_calls) : 0.0;
  return result;
}

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
