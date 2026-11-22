#pragma once

// Shared scenario definitions and the simulation loop for the collision
// avoidance demos (apps/demos/avoidance_*.cpp) and the avoidance experiment
// runner (apps/benchmarks/avoidance_benchmark.cpp).

#include "multirobotcore/avoidance/avoidance.hpp"
#include "multirobotcore/avoidance/simulation_adapter.hpp"
#include "multirobotcore/io/json.hpp"
#include "multirobotcore/math/numerics.hpp"
#include "multirobotcore/math/random.hpp"
#include "multirobotcore/simulation/simulator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace mrc_avoid {

namespace mrc = multirobotcore;
namespace av = multirobotcore::avoidance;
using Json = multirobotcore::io::Json;
using mrc::Vec2;

struct Scenario {
  std::string name;
  std::vector<Vec2> starts;
  std::vector<Vec2> goals;
  // Optional intermediate waypoints per robot (a minimal global route, e.g.
  // through a door); a waypoint is reached within `waypoint_radius`.
  std::vector<std::vector<Vec2>> waypoints;
  double waypoint_radius{0.5};
  double robot_radius{0.3};
  double max_speed{1.0};
  double dt{0.05};
  double duration{60.0};
  double goal_tolerance{0.05};
  double perturbation{1e-3};  // [m/s] seeded symmetry-breaking noise on preferences
  double slowdown_distance{0.5};
  mrc::math::ObstacleSet obstacles;
  std::optional<mrc::math::AABB> bounds;
  std::uint64_t seed{1};
  std::size_t record_every{2};
  double stall_window{4.0};
};

struct Outcome {
  bool success{false};  // every robot reached its goal without any contact
  std::size_t reached{0};
  std::size_t robot_collisions{0};
  std::size_t obstacle_collisions{0};
  double min_pair_clearance{0.0};
  double min_obstacle_clearance{0.0};
  double makespan{0.0};           // time the last robot arrived (or end of run)
  double mean_time_to_goal{0.0};  // over robots that arrived
  double mean_path_length{0.0};
  double path_efficiency{0.0};          // straight-line distance / path length (mean over robots)
  double mean_velocity_deviation{0.0};  // mean ||v - v_pref|| over robot-steps
  bool stalled{false};
  double stall_time{-1.0};
  std::size_t infeasible_steps{0};
  double ms_per_step{0.0};
  std::vector<double> arrival_times;
  std::vector<mrc::simulation::Frame> frames;
  std::vector<mrc::simulation::CollisionEvent> collisions;
};

inline av::AvoidanceConfig avoidance_config(const Json& j, av::AvoidanceMethod method, double dt) {
  av::AvoidanceConfig c;
  c.method = method;
  c.time_step = dt;
  c.time_horizon = j.value("time_horizon", c.time_horizon);
  c.obstacle_time_horizon = j.value("obstacle_time_horizon", c.obstacle_time_horizon);
  c.neighbor_distance = j.value("neighbor_distance", c.neighbor_distance);
  c.max_neighbors = j.value("max_neighbors", c.max_neighbors);
  c.safety_margin = j.value("safety_margin", c.safety_margin);
  c.obstacle_disc_spacing = j.value("obstacle_disc_spacing", c.obstacle_disc_spacing);
  c.speed_rings = j.value("speed_rings", c.speed_rings);
  c.angle_samples = j.value("angle_samples", c.angle_samples);
  c.penalty_weight = j.value("penalty_weight", c.penalty_weight);
  return c;
}

/// Robots on a circle swap to the antipodal points (classic symmetric test).
inline Scenario circle_swap(std::size_t n, double radius, double jitter, std::uint64_t seed) {
  Scenario s;
  s.name = "circle_swap";
  s.seed = seed;
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> u(-jitter, jitter);
  for (std::size_t i = 0; i < n; ++i) {
    const double a = mrc::math::kTwoPi * static_cast<double>(i) / static_cast<double>(n);
    const Vec2 p = radius * Vec2(std::cos(a), std::sin(a));
    s.starts.push_back(p + mrc::math::sample_vec2(rng, u));
    s.goals.push_back(-p);
  }
  return s;
}

/// Random starts and goals in a square arena with a minimum spacing.
inline Scenario random_crossing(std::size_t n, double side, double min_spacing,
                                std::uint64_t seed) {
  Scenario s;
  s.name = "random_crossing";
  s.seed = seed;
  s.bounds =
      mrc::math::AABB{{-0.5 * side - 1.0, -0.5 * side - 1.0}, {0.5 * side + 1.0, 0.5 * side + 1.0}};
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> u(-0.5 * side, 0.5 * side);
  auto sample = [&](std::vector<Vec2>& out) {
    for (std::size_t attempt = 0; out.size() < n && attempt < 200000; ++attempt) {
      const Vec2 p = mrc::math::sample_vec2(rng, u);
      if (std::all_of(out.begin(), out.end(),
                      [&](const Vec2& q) { return (p - q).norm() >= min_spacing; })) {
        out.push_back(p);
      }
    }
    if (out.size() < n) {
      throw std::runtime_error("random_crossing: arena too small for the requested spacing");
    }
  };
  sample(s.starts);
  sample(s.goals);
  return s;
}

/// Wall across x = 0 with a centred door. `counterflow`: half of the robots
/// start on each side; otherwise every robot moves left to right.
inline Scenario doorway(std::size_t n, double door_width, bool counterflow, std::uint64_t seed) {
  Scenario s;
  s.name = counterflow ? "doorway_counterflow" : "doorway";
  s.seed = seed;
  const double half_height = 6.0;
  const double wall = 0.15;
  // Columns of robots behind the wall, goals mirrored on the other side; the
  // arena grows with the number of columns.
  const std::size_t per_side = counterflow ? (n + 1) / 2 : n;
  const double spacing = 0.9;
  const std::size_t rows = 5;
  const auto columns = static_cast<double>((per_side + rows - 1) / rows);
  const double half_width = std::max(9.0, 2.0 + spacing * columns + 3.0);
  s.bounds = mrc::math::AABB{{-half_width, -half_height}, {half_width, half_height}};
  s.obstacles.boxes.push_back({{-wall, -half_height}, {wall, -0.5 * door_width}});
  s.obstacles.boxes.push_back({{-wall, 0.5 * door_width}, {wall, half_height}});
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> u(-0.05, 0.05);
  for (std::size_t i = 0; i < n; ++i) {
    const bool left = !counterflow || i < per_side;
    const std::size_t k = left ? i : i - per_side;
    const double x = 2.0 + spacing * static_cast<double>(k / rows);
    const double y =
        spacing * (static_cast<double>(k % rows) - 0.5 * static_cast<double>(rows - 1));
    const Vec2 start(left ? -x : x, y);
    s.starts.push_back(start + mrc::math::sample_vec2(rng, u));
    s.goals.emplace_back(left ? x + 1.0 : -x - 1.0, y);
    // Route through the door: approach point, then exit point.
    const double side = left ? -1.0 : 1.0;
    s.waypoints.push_back({Vec2(0.6 * side, 0.0), Vec2(-0.6 * side, 0.0)});
  }
  return s;
}

/// Two groups swap ends of a corridor that is too narrow for two robots side
/// by side: a classic deadlock-prone case for purely local methods.
inline Scenario corridor_swap(std::size_t per_side, double corridor_width, std::uint64_t seed) {
  Scenario s;
  s.name = "corridor_swap";
  s.seed = seed;
  const double length = 4.0;
  s.bounds = mrc::math::AABB{{-8.0, -4.0}, {8.0, 4.0}};
  s.obstacles.boxes.push_back({{-0.5 * length, 0.5 * corridor_width}, {0.5 * length, 4.0}});
  s.obstacles.boxes.push_back({{-0.5 * length, -4.0}, {0.5 * length, -0.5 * corridor_width}});
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> u(-0.05, 0.05);
  for (std::size_t i = 0; i < 2 * per_side; ++i) {
    const bool left = i < per_side;
    const double offset = 0.8 * static_cast<double>(left ? i : i - per_side);
    const double x = 0.5 * length + 1.0 + offset;
    s.starts.push_back(Vec2(left ? -x : x, 0.0) + mrc::math::sample_vec2(rng, u));
    s.goals.emplace_back(left ? x : -x, 0.0);
  }
  return s;
}

inline Outcome run(const Scenario& s, const av::AvoidanceConfig& config, bool record) {
  const std::size_t n = s.starts.size();
  mrc::simulation::WorldConfig world;
  world.dt = s.dt;
  world.bounds = s.bounds;
  world.obstacles = s.obstacles;
  world.record = record;
  world.record_every = s.record_every;
  world.stall.window = s.stall_window;
  world.stall.min_progress = 0.05;
  mrc::simulation::RobotSpec spec;
  spec.radius = s.robot_radius;
  spec.max_speed = s.max_speed;
  std::vector<mrc::Pose2> poses;
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 d = s.goals[i] - s.starts[i];
    poses.push_back({s.starts[i], std::atan2(d.y(), d.x())});
  }
  mrc::simulation::Simulator sim(world, std::vector<mrc::simulation::RobotSpec>(n, spec), poses);
  const av::CollisionAvoider avoider(config, s.obstacles);

  std::mt19937_64 rng(s.seed ^ 0x5eedULL);
  std::uniform_real_distribution<double> angle(0.0, mrc::math::kTwoPi);
  Outcome out;
  out.arrival_times.assign(n, -1.0);
  std::vector<std::size_t> stage(n, 0);
  double deviation_sum = 0.0;
  std::size_t deviation_count = 0;
  const auto steps = static_cast<std::size_t>(std::llround(s.duration / s.dt));
  const auto t0 = std::chrono::steady_clock::now();
  std::size_t executed = 0;
  for (std::size_t k = 0; k < steps; ++k) {
    std::vector<Vec2> pref(n);
    for (std::size_t i = 0; i < n; ++i) {
      const Vec2& p = sim.robot(i).position;
      const std::vector<Vec2>* route = i < s.waypoints.size() ? &s.waypoints[i] : nullptr;
      while (route != nullptr && stage[i] < route->size() &&
             (p - (*route)[stage[i]]).norm() <= s.waypoint_radius) {
        ++stage[i];
      }
      const bool final_leg = route == nullptr || stage[i] >= route->size();
      const Vec2 target = final_leg ? s.goals[i] : (*route)[stage[i]];
      const double a = angle(rng);
      pref[i] =
          av::preferred_velocity(p, target, s.max_speed, final_leg ? s.slowdown_distance : 0.0) +
          s.perturbation * Vec2(std::cos(a), std::sin(a));
    }
    const auto v = av::safe_velocities(avoider, sim, pref);
    out.infeasible_steps += avoider.last_infeasible_count();
    for (std::size_t i = 0; i < n; ++i) {
      deviation_sum += (v[i] - pref[i]).norm();
      ++deviation_count;
    }
    sim.step(v);
    ++executed;
    bool all = true;
    for (std::size_t i = 0; i < n; ++i) {
      if (out.arrival_times[i] < 0.0 &&
          (sim.robot(i).position - s.goals[i]).norm() <= s.goal_tolerance) {
        out.arrival_times[i] = sim.time();
        sim.set_status(i, mrc::models::RobotStatus::kFinished);
      }
      all = all && out.arrival_times[i] >= 0.0;
    }
    if (all) {
      break;
    }
    if (sim.stalled()) {
      out.stalled = true;
      out.stall_time = sim.time();
      break;
    }
  }
  const double elapsed =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  out.ms_per_step = executed > 0 ? elapsed / static_cast<double>(executed) : 0.0;

  const auto& m = sim.metrics();
  out.robot_collisions = m.robot_collisions;
  out.obstacle_collisions = m.obstacle_collisions;
  out.min_pair_clearance = m.min_pair_clearance;
  out.min_obstacle_clearance = m.min_obstacle_clearance;
  double time_sum = 0.0;
  double path_sum = 0.0;
  double efficiency_sum = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    path_sum += m.distance_travelled[i];
    const double straight = (s.goals[i] - s.starts[i]).norm();
    efficiency_sum +=
        m.distance_travelled[i] > 1e-9 ? std::min(1.0, straight / m.distance_travelled[i]) : 0.0;
    if (out.arrival_times[i] >= 0.0) {
      ++out.reached;
      time_sum += out.arrival_times[i];
      out.makespan = std::max(out.makespan, out.arrival_times[i]);
    }
  }
  if (out.reached < n) {
    out.makespan = sim.time();
  }
  out.mean_time_to_goal = out.reached > 0 ? time_sum / static_cast<double>(out.reached) : 0.0;
  out.mean_path_length = n > 0 ? path_sum / static_cast<double>(n) : 0.0;
  out.path_efficiency = n > 0 ? efficiency_sum / static_cast<double>(n) : 0.0;
  out.mean_velocity_deviation =
      deviation_count > 0 ? deviation_sum / static_cast<double>(deviation_count) : 0.0;
  out.success = out.reached == n && out.robot_collisions == 0 && out.obstacle_collisions == 0;
  out.frames = sim.frames();
  out.collisions = sim.collision_events();
  return out;
}

inline Json rounded_or_null(double v, int decimals = 4) {
  return std::isfinite(v) ? Json(mrc::io::round_to(v, decimals)) : Json(nullptr);
}

inline Json metrics_json(const Outcome& o) {
  return Json{{"success", o.success},
              {"reached", o.reached},
              {"robot_collisions", o.robot_collisions},
              {"obstacle_collisions", o.obstacle_collisions},
              {"min_pair_clearance", rounded_or_null(o.min_pair_clearance)},
              {"min_obstacle_clearance", rounded_or_null(o.min_obstacle_clearance)},
              {"makespan", rounded_or_null(o.makespan, 2)},
              {"mean_time_to_goal", rounded_or_null(o.mean_time_to_goal, 2)},
              {"mean_path_length", rounded_or_null(o.mean_path_length, 3)},
              {"path_efficiency", rounded_or_null(o.path_efficiency, 4)},
              {"mean_velocity_deviation", rounded_or_null(o.mean_velocity_deviation, 4)},
              {"stalled", o.stalled},
              {"stall_time", rounded_or_null(o.stall_time, 2)},
              {"infeasible_steps", o.infeasible_steps},
              {"ms_per_step", rounded_or_null(o.ms_per_step, 4)}};
}

inline Json collisions_json(const Outcome& o) {
  Json events = Json::array();
  for (const auto& e : o.collisions) {
    events.push_back(Json{{"t", mrc::io::round_to(e.time, 3)},
                          {"robot", e.robot},
                          {"other", e.other ? Json(*e.other) : Json(nullptr)}});
  }
  return events;
}

inline Json scenario_json(const Scenario& s) {
  return Json{{"name", s.name},
              {"robot_radius", s.robot_radius},
              {"max_speed", s.max_speed},
              {"dt", s.dt},
              {"starts", mrc::io::to_json(s.starts)},
              {"goals", mrc::io::to_json(s.goals)},
              {"has_waypoints", !s.waypoints.empty()},
              {"obstacles", mrc::io::to_json(s.obstacles)},
              {"bounds", s.bounds ? mrc::io::to_json(*s.bounds) : Json(nullptr)}};
}

}  // namespace mrc_avoid
