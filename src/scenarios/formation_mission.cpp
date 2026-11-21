#include "multirobotcore/scenarios/formation_mission.hpp"

#include "multirobotcore/communication/topology.hpp"
#include "multirobotcore/consensus/distributed.hpp"
#include "multirobotcore/formation/controllers.hpp"
#include "multirobotcore/formation/metrics.hpp"
#include "multirobotcore/formation/safety.hpp"
#include "multirobotcore/formation/transition.hpp"
#include "multirobotcore/math/numerics.hpp"
#include "multirobotcore/simulation/events.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

namespace multirobotcore::scenarios {
namespace {

constexpr std::uint32_t kLeaderTopic = 7;

enum class Phase { kGather, kCruise, kContracting, kNarrow, kRestoring };

struct LeaderBelief {
  Pose2 pose;
  Vec2 velocity{Vec2::Zero()};
  double stamp{-1.0};
  bool valid{false};
};

std::vector<Pose2> initial_poses(const FormationMissionConfig& cfg, std::mt19937_64& rng) {
  std::uniform_real_distribution<double> ux(cfg.spawn.min.x(), cfg.spawn.max.x());
  std::uniform_real_distribution<double> uy(cfg.spawn.min.y(), cfg.spawn.max.y());
  const double min_spacing = 2.0 * cfg.robot_radius + 2.0 * cfg.safety_margin;
  std::vector<Pose2> poses{{cfg.leader_path.front(), 0.0}};
  std::size_t attempts = 0;
  while (poses.size() < cfg.num_robots) {
    const Vec2 p(ux(rng), uy(rng));
    const bool clear =
        std::all_of(poses.begin(), poses.end(),
                    [&](const Pose2& q) { return (q.position - p).norm() >= min_spacing; }) &&
        cfg.obstacles.distance(p) > cfg.robot_radius + 0.2;
    if (clear) {
      poses.push_back({p, 0.0});
    }
    if (++attempts > 100000) {
      throw std::runtime_error("formation mission: cannot place robots in the spawn region");
    }
  }
  return poses;
}

double path_heading(const std::vector<Vec2>& path, std::size_t next_waypoint) {
  const std::size_t i = std::min(std::max<std::size_t>(next_waypoint, 1), path.size() - 1);
  const Vec2 d = path[i] - path[i - 1];
  return d.norm() > 1e-9 ? std::atan2(d.y(), d.x()) : 0.0;
}

}  // namespace

double polyline_progress(const std::vector<Vec2>& path, const Vec2& point) {
  if (path.size() < 2) {
    return 1.0;
  }
  double total = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    total += (path[i] - path[i - 1]).norm();
  }
  if (total <= 0.0) {
    return 1.0;
  }
  double best_dist = std::numeric_limits<double>::infinity();
  double best_arc = 0.0;
  double arc = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const math::Segment seg{path[i - 1], path[i]};
    const Vec2 q = math::closest_point(seg, point);
    const double d = (q - point).norm();
    if (d < best_dist) {
      best_dist = d;
      best_arc = arc + (q - path[i - 1]).norm();
    }
    arc += (path[i] - path[i - 1]).norm();
  }
  return std::clamp(best_arc / total, 0.0, 1.0);
}

FormationMissionResult run_formation_mission(const FormationMissionConfig& cfg) {
  if (cfg.num_robots < 2 || cfg.leader_path.empty()) {
    throw std::invalid_argument("formation mission: need >= 2 robots and a leader path");
  }
  const std::size_t n = cfg.num_robots;
  std::mt19937_64 rng(cfg.seed);
  std::normal_distribution<double> noise(0.0, 1.0);

  simulation::WorldConfig world;
  world.dt = cfg.dt;
  world.bounds = cfg.arena;
  world.obstacles = cfg.obstacles;
  world.record_every = cfg.record_every;
  world.stall.window = 6.0;
  simulation::RobotSpec spec;
  spec.radius = cfg.robot_radius;
  spec.max_speed = cfg.max_speed;
  spec.max_acceleration = cfg.max_acceleration;
  simulation::Simulator sim(world, std::vector<simulation::RobotSpec>(n, spec),
                            initial_poses(cfg, rng));

  communication::ChannelConfig channel;
  channel.drop_probability = cfg.drop_probability;
  channel.latency = cfg.latency;
  channel.max_range = cfg.comm_radius;
  communication::NetworkSimulator network(n, channel, cfg.seed ^ 0x9E3779B97F4A7C15ULL);

  // Shapes relative to the leader slot (slot 0 is the front of V and column).
  const auto cruise =
      formation::relative_to_slot(formation::make_shape(cfg.cruise_shape, n, cfg.spacing), 0);
  const auto narrow =
      formation::relative_to_slot(formation::make_shape(cfg.narrow_shape, n, cfg.spacing), 0);

  std::size_t leader = 0;
  std::size_t next_waypoint = cfg.leader_path.size() > 1 ? 1 : 0;
  double leader_heading = path_heading(cfg.leader_path, next_waypoint);

  // Initial slot assignment: robots take the cruise slots closest to them.
  std::vector<Vec2> offsets(n);
  {
    const auto world_slots =
        formation::to_world(Pose2{sim.robot(leader).position, leader_heading}, cruise);
    const auto assignment = formation::assign_slots(sim.positions(), world_slots,
                                                    std::pair<std::size_t, std::size_t>{leader, 0});
    for (std::size_t i = 0; i < n; ++i) {
      offsets[i] = cruise[assignment[i]];
    }
  }

  std::optional<formation::FormationTransition> transition;
  Phase phase = Phase::kGather;
  std::vector<LeaderBelief> beliefs(n);
  bool leader_failed = false;
  bool reelected = false;

  FormationMissionResult result;
  result.events.push_back(
      {0.0, "gathering into " + std::string(formation::to_string(cfg.cruise_shape))});
  const formation::ConsensusFormationConfig law{cfg.formation_gain, cfg.pinning_gain,
                                                std::numeric_limits<double>::infinity()};
  const double safety_distance = 2.0 * cfg.robot_radius + cfg.safety_margin;

  auto true_reference = [&]() {
    return Pose2{sim.robot(leader).position, leader_heading};
  };
  auto record = [&](const std::vector<Vec2>& current_offsets, const graph::Graph& comm,
                    const Pose2& reference) {
    std::vector<std::array<std::size_t, 2>> e;
    for (const auto& edge : comm.edges()) {
      e.push_back({edge.source, edge.target});
    }
    result.edges.push_back(std::move(e));
    std::vector<Vec2> slots(n);
    std::vector<Vec2> active_pos;
    std::vector<Vec2> active_slots;
    for (std::size_t i = 0; i < n; ++i) {
      if (models::is_operational(sim.robot(i).status)) {
        slots[i] = formation::to_world(reference, current_offsets[i]);
        active_pos.push_back(sim.robot(i).position);
        active_slots.push_back(slots[i]);
      } else {
        slots[i] = sim.robot(i).position;
      }
    }
    result.slots.push_back(std::move(slots));
    result.leader.push_back(leader);
    double sum_sq = 0.0;
    for (std::size_t k = 0; k < active_pos.size(); ++k) {
      sum_sq += (active_pos[k] - active_slots[k]).squaredNorm();
    }
    result.rmse.push_back(
        active_pos.empty() ? 0.0 : std::sqrt(sum_sq / static_cast<double>(active_pos.size())));
  };

  const auto steps = static_cast<std::size_t>(std::llround(cfg.duration / cfg.dt));
  Pose2 last_reference = true_reference();
  {
    const auto comm = communication::disk_graph(sim.positions(), cfg.comm_radius);
    record(offsets, comm, last_reference);
  }

  for (std::size_t step = 0; step < steps; ++step) {
    const double t = sim.time();
    std::vector<Vec2> positions = sim.positions();
    network.set_positions(positions);

    // --- Failure injection ---------------------------------------------------
    if (cfg.leader_failure_time && !leader_failed && t >= *cfg.leader_failure_time) {
      simulation::ScheduledEvent ev;
      ev.time = t;
      ev.type = simulation::EventType::kLeaderLoss;
      ev.robot = leader;
      simulation::apply_event(ev, sim, &network);
      leader_failed = true;
      result.events.push_back({t, "leader " + std::to_string(leader) + " failed"});
    }
    const std::vector<bool> operational = sim.operational_mask();

    // --- Communication graph (failed robots isolated) ------------------------
    graph::Graph comm = communication::disk_graph(positions, cfg.comm_radius);
    for (std::size_t i = 0; i < n; ++i) {
      if (!operational[i]) {
        comm.isolate_node(i);
      }
    }

    // --- Leader motion ---------------------------------------------------------
    Vec2 leader_velocity = Vec2::Zero();
    if (operational[leader] && t >= cfg.gather_time && next_waypoint < cfg.leader_path.size()) {
      const Vec2 to_target = cfg.leader_path[next_waypoint] - positions[leader];
      if (to_target.norm() < 0.15) {
        ++next_waypoint;
        if (next_waypoint >= cfg.leader_path.size()) {
          result.events.push_back({t, "goal reached"});
        }
      } else {
        const bool reconfiguring = phase == Phase::kContracting || phase == Phase::kRestoring;
        const double cruise_speed =
            reconfiguring ? cfg.transition_speed_factor * cfg.leader_speed : cfg.leader_speed;
        const double speed = std::min(cruise_speed, 0.8 * to_target.norm());
        leader_velocity = speed * to_target.normalized();
        leader_heading = path_heading(cfg.leader_path, next_waypoint);
      }
    }

    // --- Leader state dissemination (multi-hop relay) --------------------------
    // The leader broadcasts its state; every follower re-broadcasts the
    // freshest leader state it holds to its communication neighbours, so the
    // reference propagates one hop per control step. Messages carry the
    // leader identifier and the time the state was measured.
    if (operational[leader]) {
      beliefs[leader] = {Pose2{positions[leader], leader_heading}, leader_velocity, t, true};
    }
    for (std::size_t i = 0; i < n; ++i) {
      if (!operational[i] || !beliefs[i].valid) {
        continue;
      }
      const LeaderBelief& b = beliefs[i];
      const std::vector<double> payload{b.pose.position.x(),
                                        b.pose.position.y(),
                                        b.pose.heading,
                                        b.velocity.x(),
                                        b.velocity.y(),
                                        b.stamp,
                                        static_cast<double>(leader)};
      for (const graph::Neighbor& j : comm.out_neighbors(i)) {
        network.send(i, j.id, kLeaderTopic, payload, t);
      }
    }
    for (auto& inbox : network.deliver_inboxes(t)) {
      for (const auto& m : inbox) {
        if (m.topic != kLeaderTopic || m.payload.size() != 7 ||
            static_cast<std::size_t>(m.payload[6]) != leader || m.receiver == leader) {
          continue;
        }
        LeaderBelief& b = beliefs[m.receiver];
        if (!b.valid || m.payload[5] > b.stamp) {
          b.pose = Pose2{{m.payload[0], m.payload[1]}, m.payload[2]};
          b.velocity = Vec2(m.payload[3], m.payload[4]);
          b.stamp = m.payload[5];
          b.valid = true;
        }
      }
    }

    // --- Leader re-election after a detected failure -----------------------------
    if (leader_failed && cfg.reelect_leader && !reelected) {
      bool silent = false;
      for (std::size_t i = 0; i < n; ++i) {
        if (operational[i] && i != leader && t - beliefs[i].stamp > cfg.leader_timeout) {
          silent = true;
        }
      }
      if (silent) {
        const std::size_t rounds = n;
        const auto election = consensus::elect_leader_min_id(
            comm, operational, network, rounds, t, cfg.dt / static_cast<double>(rounds + 1));
        std::optional<std::size_t> elected;
        for (std::size_t i = 0; i < n; ++i) {
          if (operational[i] && election.leader[i]) {
            elected = elected ? std::min(*elected, *election.leader[i]) : *election.leader[i];
          }
        }
        if (elected) {
          // Finish any running transition, then re-express the shape relative
          // to the new leader's slot.
          if (transition) {
            offsets = transition->target();
            transition.reset();
            phase = phase == Phase::kContracting ? Phase::kNarrow : Phase::kCruise;
          }
          const Vec2 base = offsets[*elected];
          for (Vec2& d : offsets) {
            d -= base;
          }
          leader = *elected;
          reelected = true;
          // Continue from the first waypoint ahead of the new leader.
          const double progress = polyline_progress(cfg.leader_path, positions[leader]);
          double arc = 0.0;
          double total = 0.0;
          for (std::size_t i = 1; i < cfg.leader_path.size(); ++i) {
            total += (cfg.leader_path[i] - cfg.leader_path[i - 1]).norm();
          }
          next_waypoint = cfg.leader_path.size() - 1;
          for (std::size_t i = 1; i < cfg.leader_path.size(); ++i) {
            arc += (cfg.leader_path[i] - cfg.leader_path[i - 1]).norm();
            if (arc / total > progress + 1e-9) {
              next_waypoint = i;
              break;
            }
          }
          beliefs.assign(n, LeaderBelief{});
          beliefs[leader] = {Pose2{positions[leader], leader_heading}, Vec2::Zero(), t, true};
          result.events.push_back({t, "robot " + std::to_string(leader) + " elected leader (" +
                                          std::to_string(election.rounds) + " rounds)"});
        }
      }
    }

    // --- Shape reconfiguration around the passage ----------------------------------
    if (phase == Phase::kGather && t >= cfg.gather_time) {
      phase = Phase::kCruise;
      result.events.push_back({t, "cruise"});
    }
    if (operational[leader]) {
      const auto pinned = std::pair<std::size_t, std::size_t>{leader, 0};
      if (phase == Phase::kCruise &&
          positions[leader].x() >= cfg.passage_x_min - cfg.contract_distance &&
          positions[leader].x() < cfg.passage_x_max) {
        transition.emplace(offsets, narrow, t, cfg.transition_duration, true, pinned);
        phase = Phase::kContracting;
        result.events.push_back(
            {t, "contracting to " + std::string(formation::to_string(cfg.narrow_shape))});
      } else if (phase == Phase::kNarrow) {
        double rearmost = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < n; ++i) {
          if (operational[i]) {
            rearmost = std::min(rearmost, positions[i].x());
          }
        }
        if (rearmost >= cfg.passage_x_max + cfg.restore_clearance) {
          transition.emplace(offsets, cruise, t, cfg.transition_duration, true, pinned);
          phase = Phase::kRestoring;
          result.events.push_back(
              {t, "restoring " + std::string(formation::to_string(cfg.cruise_shape))});
        }
      }
    }
    std::vector<Vec2> current = offsets;
    std::vector<Vec2> rates(n, Vec2::Zero());
    if (transition) {
      current = transition->offsets_at(t);
      rates = transition->offset_rates_at(t);
      if (transition->finished(t)) {
        offsets = transition->target();
        current = offsets;
        transition.reset();
        phase = phase == Phase::kContracting ? Phase::kNarrow : Phase::kCruise;
        if (phase == Phase::kNarrow) {
          result.events.push_back({t, "passing the passage"});
        } else {
          result.events.push_back(
              {t, std::string(formation::to_string(cfg.cruise_shape)) + " restored"});
        }
      }
    }

    // --- Control ---------------------------------------------------------------
    std::vector<Vec2> estimates = positions;
    if (cfg.measurement_noise > 0.0) {
      for (Vec2& p : estimates) {
        p += cfg.measurement_noise * Vec2(noise(rng), noise(rng));
      }
    }
    // Failed robots remain physically present and are avoided as obstacles.
    math::ObstacleSet hazards = sim.obstacles();
    for (std::size_t i = 0; i < n; ++i) {
      if (!operational[i]) {
        hazards.circles.push_back({positions[i], cfg.robot_radius});
      }
    }
    std::vector<Vec2> commands(n, Vec2::Zero());
    for (std::size_t i = 0; i < n; ++i) {
      if (!operational[i]) {
        continue;
      }
      Vec2 u = Vec2::Zero();
      if (i == leader) {
        u = leader_velocity;
      } else if (beliefs[i].valid) {
        const double age = std::min(t - beliefs[i].stamp, cfg.extrapolation_horizon);
        const Pose2 reference{beliefs[i].pose.position + age * beliefs[i].velocity,
                              beliefs[i].pose.heading};
        const bool fresh = t - beliefs[i].stamp <= cfg.extrapolation_horizon;
        const Vec2 ref_velocity = fresh ? beliefs[i].velocity : Vec2::Zero();
        // Tracking is saturated first so that the separation barrier, which
        // exceeds max_speed near contact, always dominates there.
        const Vec2 tracking =
            math::clamp_norm(formation::consensus_formation_velocity(
                                 i, comm, estimates, current, reference, ref_velocity, 1.0, law) +
                                 math::rotate(rates[i], reference.heading),
                             cfg.max_speed);
        u = tracking +
            formation::separation_velocity(i, positions, safety_distance, cfg.separation_gain);
      }
      u += formation::obstacle_repulsion(positions[i], hazards, cfg.robot_radius,
                                         cfg.obstacle_influence, cfg.obstacle_gain);
      commands[i] = math::clamp_norm(u, cfg.max_speed);
    }
    sim.step(commands);

    // --- Recording ---------------------------------------------------------------
    if (operational[leader]) {
      last_reference = true_reference();
    } else {
      // Ghost reference: what the lowest-index operational follower believes.
      for (std::size_t i = 0; i < n; ++i) {
        if (operational[i] && beliefs[i].valid) {
          const double age = std::min(sim.time() - beliefs[i].stamp, cfg.extrapolation_horizon);
          last_reference =
              Pose2{beliefs[i].pose.position + age * beliefs[i].velocity, beliefs[i].pose.heading};
          break;
        }
      }
    }
    if (sim.step_count() % cfg.record_every == 0) {
      record(current, comm, last_reference);
    }
  }

  // --- Summary metrics ------------------------------------------------------------
  result.frames = sim.frames();
  result.metrics = sim.metrics();
  result.network = network.stats();
  result.final_leader = leader;
  result.path_progress = polyline_progress(cfg.leader_path, sim.robot(leader).position);
  // Standing still after reaching the goal is not a stall.
  result.stalled = sim.stalled() && result.path_progress < 0.99;
  double sum = 0.0;
  std::size_t count = 0;
  const double record_dt = cfg.dt * static_cast<double>(cfg.record_every);
  for (std::size_t k = 0; k < result.rmse.size(); ++k) {
    const double time = static_cast<double>(k) * record_dt;
    if (result.convergence_time < 0.0 && result.rmse[k] < 0.15) {
      result.convergence_time = time;
    }
    if (time >= cfg.gather_time) {
      sum += result.rmse[k];
      result.max_rmse = std::max(result.max_rmse, result.rmse[k]);
      ++count;
    }
  }
  result.mean_rmse = count > 0 ? sum / static_cast<double>(count) : 0.0;
  result.final_rmse = result.rmse.empty() ? 0.0 : result.rmse.back();
  return result;
}

FormationMissionConfig formation_mission_config_from_json(const io::Json& j) {
  FormationMissionConfig c;
  c.num_robots = j.value("num_robots", c.num_robots);
  c.spacing = j.value("spacing", c.spacing);
  c.robot_radius = j.value("robot_radius", c.robot_radius);
  c.max_speed = j.value("max_speed", c.max_speed);
  c.max_acceleration = j.value("max_acceleration", c.max_acceleration);
  c.leader_speed = j.value("leader_speed", c.leader_speed);
  c.dt = j.value("dt", c.dt);
  c.duration = j.value("duration", c.duration);
  c.gather_time = j.value("gather_time", c.gather_time);
  c.seed = j.value("seed", c.seed);
  if (j.contains("arena")) {
    c.arena = io::aabb_from_json(j.at("arena"));
  }
  if (j.contains("spawn")) {
    c.spawn = io::aabb_from_json(j.at("spawn"));
  }
  if (j.contains("leader_path")) {
    c.leader_path = io::vec2_list_from_json(j.at("leader_path"));
  }
  if (j.contains("cruise_shape")) {
    c.cruise_shape = formation::shape_from_string(j.at("cruise_shape").get<std::string>())
                         .value_or(c.cruise_shape);
  }
  if (j.contains("narrow_shape")) {
    c.narrow_shape = formation::shape_from_string(j.at("narrow_shape").get<std::string>())
                         .value_or(c.narrow_shape);
  }
  if (j.contains("obstacles")) {
    c.obstacles = io::obstacles_from_json(j.at("obstacles"));
  }
  c.passage_x_min = j.value("passage_x_min", c.passage_x_min);
  c.passage_x_max = j.value("passage_x_max", c.passage_x_max);
  c.contract_distance = j.value("contract_distance", c.contract_distance);
  c.restore_clearance = j.value("restore_clearance", c.restore_clearance);
  c.transition_duration = j.value("transition_duration", c.transition_duration);
  c.transition_speed_factor = j.value("transition_speed_factor", c.transition_speed_factor);
  c.comm_radius = j.value("comm_radius", c.comm_radius);
  c.formation_gain = j.value("formation_gain", c.formation_gain);
  c.pinning_gain = j.value("pinning_gain", c.pinning_gain);
  c.separation_gain = j.value("separation_gain", c.separation_gain);
  c.safety_margin = j.value("safety_margin", c.safety_margin);
  c.obstacle_gain = j.value("obstacle_gain", c.obstacle_gain);
  c.obstacle_influence = j.value("obstacle_influence", c.obstacle_influence);
  c.extrapolation_horizon = j.value("extrapolation_horizon", c.extrapolation_horizon);
  if (j.contains("leader_failure_time") && !j.at("leader_failure_time").is_null()) {
    c.leader_failure_time = j.at("leader_failure_time").get<double>();
  }
  c.reelect_leader = j.value("reelect_leader", c.reelect_leader);
  c.leader_timeout = j.value("leader_timeout", c.leader_timeout);
  c.drop_probability = j.value("drop_probability", c.drop_probability);
  c.latency = j.value("latency", c.latency);
  c.measurement_noise = j.value("measurement_noise", c.measurement_noise);
  c.record_every = j.value("record_every", c.record_every);
  return c;
}

io::Json to_json(const FormationMissionResult& r) {
  io::Json edges = io::Json::array();
  for (const auto& frame : r.edges) {
    io::Json e = io::Json::array();
    for (const auto& pair : frame) {
      e.push_back(io::Json::array({pair[0], pair[1]}));
    }
    edges.push_back(std::move(e));
  }
  io::Json slots = io::Json::array();
  for (const auto& frame : r.slots) {
    slots.push_back(io::to_json(std::span<const Vec2>(frame)));
  }
  io::Json rmse = io::Json::array();
  for (const double v : r.rmse) {
    rmse.push_back(io::round_to(v, 4));
  }
  io::Json events = io::Json::array();
  for (const auto& e : r.events) {
    events.push_back(io::Json{{"time", io::round_to(e.time, 3)}, {"label", e.label}});
  }
  return io::Json{{"trajectory", io::frames_to_json(r.frames)},
                  {"edges", edges},
                  {"slots", slots},
                  {"leader", r.leader},
                  {"rmse", rmse},
                  {"events", events},
                  {"summary",
                   {{"mean_rmse", io::round_to(r.mean_rmse, 4)},
                    {"max_rmse", io::round_to(r.max_rmse, 4)},
                    {"final_rmse", io::round_to(r.final_rmse, 4)},
                    {"convergence_time", io::round_to(r.convergence_time, 3)},
                    {"path_progress", io::round_to(r.path_progress, 4)},
                    {"stalled", r.stalled},
                    {"final_leader", r.final_leader}}},
                  {"metrics", io::to_json(r.metrics)},
                  {"network", io::to_json(r.network)}};
}

}  // namespace multirobotcore::scenarios
