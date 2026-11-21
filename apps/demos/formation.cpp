// Formation demonstrations:
//   mission      dispersed robots gather into a V around a leader, contract into
//                a column to pass a narrow passage, then restore the V
//   leader_loss  the same mission with a leader failure, with and without
//                min-identifier re-election
//   transitions  smooth line -> V -> circle -> triangle reconfiguration with
//                minimum-sum-of-squares slot assignment
//
// Output: JSON consumed by scripts/render/render_formation.py.
//
// mrc-modules: scenarios formation simulation io

#include "demo_common.hpp"

#include "multirobotcore/formation/controllers.hpp"
#include "multirobotcore/formation/metrics.hpp"
#include "multirobotcore/formation/safety.hpp"
#include "multirobotcore/formation/shapes.hpp"
#include "multirobotcore/formation/transition.hpp"
#include "multirobotcore/graph/topologies.hpp"
#include "multirobotcore/io/json.hpp"
#include "multirobotcore/math/numerics.hpp"
#include "multirobotcore/scenarios/formation_mission.hpp"
#include "multirobotcore/simulation/simulator.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace mrc = multirobotcore;
namespace io = multirobotcore::io;
namespace f = multirobotcore::formation;
using mrc_apps::Json;

namespace {

Json run_mission(const Json& cfg_json, const char* label) {
  const auto cfg = mrc::scenarios::formation_mission_config_from_json(cfg_json);
  const auto result = mrc::scenarios::run_formation_mission(cfg);
  std::cout << label << ": mean rmse " << result.mean_rmse << " m, progress "
            << result.path_progress << ", robot contacts " << result.metrics.robot_collisions
            << ", obstacle contacts " << result.metrics.obstacle_collisions << "\n";
  Json out = mrc::scenarios::to_json(result);
  out["arena"] = io::to_json(cfg.arena);
  out["obstacles"] = io::to_json(cfg.obstacles);
  out["leader_path"] = io::to_json(std::span<const mrc::Vec2>(cfg.leader_path));
  out["robot_radius"] = cfg.robot_radius;
  out["comm_radius"] = cfg.comm_radius;
  return out;
}

Json run_transitions(const Json& cfg) {
  const auto n = cfg.value("num_robots", std::size_t{8});
  const double spacing = cfg.value("spacing", 1.3);
  const double radius = cfg.value("robot_radius", 0.25);
  const double hold = cfg.value("hold_time", 3.0);
  const double duration = cfg.value("transition_duration", 4.0);
  const double vmax = cfg.value("max_speed", 1.0);
  std::vector<f::Shape> shapes;
  for (const auto& name : cfg.at("shapes")) {
    shapes.push_back(f::shape_from_string(name.get<std::string>()).value());
  }
  std::vector<std::vector<mrc::Vec2>> targets;
  for (const auto s : shapes) {
    targets.push_back(f::make_shape(s, n, spacing));
  }

  const mrc::Pose2 reference{{6.0, 5.0}, 0.0};
  std::vector<mrc::Pose2> initial;
  for (const auto& p : f::to_world(reference, targets.front())) {
    initial.push_back({p, 0.0});
  }
  mrc::simulation::WorldConfig world;
  world.dt = cfg.value("dt", 0.05);
  world.record_every = cfg.value("record_every", std::size_t{2});
  mrc::simulation::RobotSpec spec;
  spec.radius = radius;
  spec.max_speed = vmax;
  mrc::simulation::Simulator sim(world, std::vector<mrc::simulation::RobotSpec>(n, spec), initial);

  // Schedule: hold, transition, hold, transition, ..., hold.
  std::vector<f::FormationTransition> plan;
  std::vector<mrc::Vec2> offsets = targets.front();
  double t_start = hold;
  Json schedule = Json::array();
  for (std::size_t k = 1; k < targets.size(); ++k) {
    plan.emplace_back(offsets, targets[k], t_start, duration, true);
    schedule.push_back(Json{{"start", t_start},
                            {"end", t_start + duration},
                            {"from", std::string(f::to_string(shapes[k - 1]))},
                            {"to", std::string(f::to_string(shapes[k]))}});
    offsets = plan.back().target();
    t_start += duration + hold;
  }
  const double total_time = t_start;

  const mrc::graph::Graph graph = mrc::graph::complete_graph(n);
  const f::ConsensusFormationConfig law{cfg.value("gain", 1.5), cfg.value("gain", 1.5),
                                        std::numeric_limits<double>::infinity()};
  const double safety = 2.0 * radius + 0.25;

  Json slots = Json::array();
  Json rmse = Json::array();
  Json labels = Json::array();
  const auto steps = static_cast<std::size_t>(std::llround(total_time / world.dt));
  auto active_index = [&](double t) -> std::ptrdiff_t {
    std::ptrdiff_t idx = -1;
    for (std::size_t k = 0; k < schedule.size(); ++k) {
      if (t >= schedule[k]["start"].get<double>()) {
        idx = static_cast<std::ptrdiff_t>(k);
      }
    }
    return idx;
  };
  auto record = [&](double t, const std::vector<mrc::Vec2>& current) {
    const auto world_slots = f::to_world(reference, current);
    slots.push_back(io::to_json(std::span<const mrc::Vec2>(world_slots)));
    rmse.push_back(io::round_to(
        f::formation_error(sim.positions(), current, f::Alignment::kReference, reference).rmse, 4));
    const auto idx = active_index(t);
    std::string label = std::string(f::to_string(shapes.front()));
    if (idx >= 0) {
      const auto& s = schedule[static_cast<std::size_t>(idx)];
      label = t < s["end"].get<double>()
                  ? s["from"].get<std::string>() + " -> " + s["to"].get<std::string>()
                  : s["to"].get<std::string>();
    }
    labels.push_back(label);
  };
  record(0.0, targets.front());
  for (std::size_t step = 0; step < steps; ++step) {
    const double t = sim.time();
    std::vector<mrc::Vec2> current = targets.front();
    std::vector<mrc::Vec2> rates(n, mrc::Vec2::Zero());
    const auto idx = active_index(t);
    if (idx >= 0) {
      current = plan[static_cast<std::size_t>(idx)].offsets_at(t);
      rates = plan[static_cast<std::size_t>(idx)].offset_rates_at(t);
    }
    const auto positions = sim.positions();
    std::vector<mrc::Vec2> u(n);
    for (std::size_t i = 0; i < n; ++i) {
      u[i] = mrc::math::clamp_norm(
          f::consensus_formation_velocity(i, graph, positions, current, reference,
                                          mrc::Vec2::Zero(), 1.0, law) +
              rates[i] + f::separation_velocity(i, positions, safety, 0.3),
          vmax);
    }
    sim.step(u);
    if (sim.step_count() % world.record_every == 0) {
      record(sim.time(), current);
    }
  }
  return Json{{"robot_radius", radius}, {"reference", io::to_json(reference)},
              {"schedule", schedule},   {"trajectory", io::frames_to_json(sim.frames())},
              {"slots", slots},         {"rmse", rmse},
              {"labels", labels},       {"metrics", io::to_json(sim.metrics())}};
}

}  // namespace

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args =
        mrc_apps::parse_args(argc, argv, "formation", "configs/formation/formation.json");
    const Json cfg = mrc_apps::load_config(args);
    Json result = mrc_apps::metadata("mrc_demo_formation", cfg);

    const Json& mission = cfg.at("mission");
    result["mission"] = run_mission(mission, "mission");

    Json no_reelect = mission;
    no_reelect["leader_failure_time"] = cfg.at("leader_loss").at("failure_time");
    no_reelect["reelect_leader"] = false;
    Json reelect = no_reelect;
    reelect["reelect_leader"] = true;
    result["leader_loss"] = Json{{"without_reelection", run_mission(no_reelect, "leader loss")},
                                 {"with_reelection", run_mission(reelect, "leader re-election")}};

    result["transitions"] = run_transitions(cfg.at("transitions"));
    mrc_apps::write_result(args, result);
  });
}
