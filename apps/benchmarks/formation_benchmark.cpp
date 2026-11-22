// Formation experiments:
//   controllers  leader-follower / consensus / distance-based x line, V,
//                circle from perturbed starts (static reference)
//   moving       moving leader: steady-state tracking error
//   topology     consensus formation under random per-step edge dropout
//   robustness   integrated mission under 0/5/10/20% message loss and
//                position-estimate noise
//
// mrc-modules: formation scenarios graph communication io

#include "demo_common.hpp"
#include "experiment_common.hpp"

#include "multirobotcore/communication/topology.hpp"
#include "multirobotcore/formation/controllers.hpp"
#include "multirobotcore/formation/metrics.hpp"
#include "multirobotcore/formation/safety.hpp"
#include "multirobotcore/formation/shapes.hpp"
#include "multirobotcore/graph/topologies.hpp"
#include "multirobotcore/math/numerics.hpp"
#include "multirobotcore/math/random.hpp"
#include "multirobotcore/scenarios/formation_mission.hpp"

#include <limits>
#include <random>
#include <string>
#include <vector>

namespace mrc = multirobotcore;
namespace f = multirobotcore::formation;
namespace io = multirobotcore::io;
using mrc::Vec2;
using mrc_apps::Json;
using mrc_apps::Sample;

namespace {

struct RunMetrics {
  double rmse{0.0};
  double max_error{0.0};
  double convergence{-1.0};
  double min_distance{std::numeric_limits<double>::infinity()};
  double effort{0.0};
};

/// Integrates single integrators under `controller` and measures formation
/// error with the alignment appropriate for the controller.
template <typename Controller>
RunMetrics simulate(std::vector<Vec2> p, const std::vector<Vec2>& offsets, f::Alignment alignment,
                    const mrc::Pose2& reference_at_end, double duration, double dt,
                    Controller&& controller, const std::vector<Vec2>* leader_track = nullptr) {
  RunMetrics m;
  const auto steps = static_cast<std::size_t>(duration / dt);
  for (std::size_t k = 0; k < steps; ++k) {
    const double t = static_cast<double>(k) * dt;
    const std::vector<Vec2> u = controller(t, p);
    for (std::size_t i = 0; i < p.size(); ++i) {
      p[i] += dt * u[i];
      m.effort += u[i].squaredNorm() * dt;
    }
    m.min_distance = std::min(m.min_distance, f::min_separation(p));
    mrc::Pose2 ref = reference_at_end;
    if (leader_track != nullptr) {
      ref.position = (*leader_track)[k];
    }
    const auto e = f::formation_error(p, offsets, alignment, ref);
    if (m.convergence < 0.0 && e.rmse < 0.05) {
      m.convergence = t + dt;
    }
    if (m.convergence >= 0.0 && e.rmse >= 0.05) {
      m.convergence = -1.0;  // must stay converged
    }
    m.rmse = e.rmse;
    m.max_error = e.max_error;
  }
  return m;
}

std::vector<Vec2> perturbed(const std::vector<Vec2>& p, double amount, std::mt19937_64& rng) {
  std::uniform_real_distribution<double> u(-amount, amount);
  std::vector<Vec2> out = p;
  for (Vec2& x : out) {
    x += mrc::math::sample_vec2(rng, u);
  }
  return out;
}

Json metrics_json(const Sample& rmse, const Sample& conv, const Sample& mind, const Sample& effort,
                  std::size_t converged, std::size_t runs) {
  return Json{{"rmse", rmse.json(4)},         {"convergence_time", conv.json(3)},
              {"min_distance", mind.json(4)}, {"control_effort", effort.json(3)},
              {"converged_runs", converged},  {"runs", runs}};
}

}  // namespace

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args =
        mrc_apps::parse_args(argc, argv, "formation_benchmark", "configs/formation/benchmark.json");
    const Json cfg = mrc_apps::load_config(args);
    const auto seeds = cfg.value("seeds", std::size_t{10});
    const auto n = cfg.value("robots", std::size_t{7});
    const double spacing = cfg.value("spacing", 1.4);
    const double dt = cfg.value("dt", 0.02);
    const double duration = cfg.value("duration", 40.0);
    const double vmax = cfg.value("max_speed", 1.0);
    const mrc::Pose2 reference{{0.0, 0.0}, 0.0};

    Json controllers = Json::array();
    for (const auto& shape_name : cfg.at("shapes")) {
      const auto shape = f::shape_from_string(shape_name.get<std::string>()).value();
      const auto centred = f::make_shape(shape, n, spacing);
      const auto from_leader = f::relative_to_slot(centred, 0);
      const auto henneberg = f::henneberg_graph(centred);
      const auto ring = mrc::graph::ring_graph(n);
      for (const std::string controller : {"leader_follower", "consensus", "distance"}) {
        Sample rmse;
        Sample conv;
        Sample mind;
        Sample effort;
        std::size_t converged = 0;
        for (std::size_t s = 0; s < seeds; ++s) {
          std::mt19937_64 rng(500 + s);
          const auto start = perturbed(f::to_world(reference, centred), 1.5, rng);
          RunMetrics m;
          if (controller == "leader_follower") {
            auto p0 = start;
            p0[0] = reference.position + centred[0];  // leader already at its pose
            const mrc::Pose2 leader{p0[0], 0.0};
            m = simulate(p0, from_leader, f::Alignment::kReference, leader, duration, dt,
                         [&](double, const std::vector<Vec2>& p) {
                           return f::leader_follower_velocities(leader, Vec2::Zero(), 0, p,
                                                                from_leader, {1.0, vmax, true});
                         });
          } else if (controller == "consensus") {
            std::vector<double> pin(n, 0.0);
            pin[0] = 1.0;
            m = simulate(start, centred, f::Alignment::kReference, reference, duration, dt,
                         [&](double, const std::vector<Vec2>& p) {
                           return f::consensus_formation_velocities(
                               ring, p, centred, reference, Vec2::Zero(), pin, {1.0, 1.0, vmax});
                         });
          } else {
            m = simulate(start, centred, f::Alignment::kRigid, reference, duration, dt,
                         [&](double, const std::vector<Vec2>& p) {
                           return f::distance_formation_velocities(henneberg, p, centred,
                                                                   {0.3, vmax});
                         });
          }
          rmse.add(m.rmse);
          mind.add(m.min_distance);
          effort.add(m.effort);
          if (m.convergence >= 0.0) {
            conv.add(m.convergence);
            ++converged;
          }
        }
        controllers.push_back(
            Json{{"controller", controller},
                 {"formation", shape_name},
                 {"metrics", metrics_json(rmse, conv, mind, effort, converged, seeds)}});
      }
    }

    // Moving leader: steady-state error with and without velocity feedforward.
    Json moving = Json::array();
    {
      const auto centred = f::make_shape(f::Shape::kV, n, spacing);
      const auto from_leader = f::relative_to_slot(centred, 0);
      const Vec2 v(0.5, 0.0);
      for (const bool feedforward : {true, false}) {
        std::vector<Vec2> track;
        const auto steps = static_cast<std::size_t>(duration / dt);
        for (std::size_t k = 0; k < steps; ++k) {
          track.push_back(static_cast<double>(k + 1) * dt * v);
        }
        auto p0 = f::to_world(mrc::Pose2{}, from_leader);
        const auto m = simulate(
            p0, from_leader, f::Alignment::kReference, mrc::Pose2{}, duration, dt,
            [&](double t, const std::vector<Vec2>& p) {
              const mrc::Pose2 leader{t * v, 0.0};
              auto u = f::leader_follower_velocities(leader, v, 0, p, from_leader,
                                                     {1.0, vmax, feedforward});
              u[0] = v;
              return u;
            },
            &track);
        moving.push_back(Json{{"controller", "leader_follower"},
                              {"feedforward", feedforward},
                              {"leader_speed", v.norm()},
                              {"steady_state_rmse", io::round_to(m.rmse, 5)},
                              {"max_error", io::round_to(m.max_error, 5)}});
      }
    }

    // Topology changes: consensus formation with per-step random edge dropout.
    Json topology = Json::array();
    for (const auto& p_json : cfg.at("edge_dropout")) {
      const double p_drop = p_json.get<double>();
      const auto centred = f::make_shape(f::Shape::kCircle, n, spacing);
      Sample rmse;
      Sample conv;
      Sample mind;
      Sample effort;
      std::size_t converged = 0;
      for (std::size_t s = 0; s < seeds; ++s) {
        std::mt19937_64 rng(700 + s);
        const auto start = perturbed(f::to_world(reference, centred), 1.5, rng);
        const auto base = mrc::graph::complete_graph(n);
        std::vector<double> pin(n, 0.0);
        pin[0] = 1.0;
        const auto m =
            simulate(start, centred, f::Alignment::kReference, reference, duration, dt,
                     [&](double, const std::vector<Vec2>& p) {
                       const auto g = mrc::communication::apply_dropout(base, p_drop, rng);
                       return f::consensus_formation_velocities(
                           g, p, centred, reference, Vec2::Zero(), pin, {0.5, 1.0, vmax});
                     });
        rmse.add(m.rmse);
        mind.add(m.min_distance);
        effort.add(m.effort);
        if (m.convergence >= 0.0) {
          conv.add(m.convergence);
          ++converged;
        }
      }
      topology.push_back(
          Json{{"edge_dropout", p_drop},
               {"formation", "circle"},
               {"metrics", metrics_json(rmse, conv, mind, effort, converged, seeds)}});
    }

    // Integrated mission robustness: message loss and estimate noise.
    Json robustness = Json::array();
    const auto mission_cfg =
        io::read_json_file(mrc_apps::fs::path(MRC_SOURCE_DIR) / "configs/formation/formation.json");
    for (const auto& noise_json : cfg.at("noise_levels")) {
      for (const auto& p_json : cfg.at("drop_rates")) {
        Sample mean_rmse;
        Sample progress;
        Sample contacts;
        Sample messages;
        for (std::size_t s = 0; s < cfg.value("mission_seeds", std::size_t{3}); ++s) {
          auto mc = mrc::scenarios::formation_mission_config_from_json(mission_cfg.at("mission"));
          mc.drop_probability = p_json.get<double>();
          mc.measurement_noise = noise_json.get<double>();
          mc.seed = 11 + s;
          mc.record_every = 20;
          const auto r = mrc::scenarios::run_formation_mission(mc);
          mean_rmse.add(r.mean_rmse);
          progress.add(r.path_progress);
          contacts.add(
              static_cast<double>(r.metrics.robot_collisions + r.metrics.obstacle_collisions));
          messages.add(static_cast<double>(r.network.messages_sent));
        }
        robustness.push_back(Json{{"drop_rate", p_json.get<double>()},
                                  {"noise", noise_json.get<double>()},
                                  {"mean_rmse", mean_rmse.json(4)},
                                  {"path_progress", progress.json(4)},
                                  {"contacts", contacts.json(1)},
                                  {"messages_sent", messages.json(0)}});
      }
    }

    Json result = mrc_apps::metadata("mrc_experiment_formation_benchmark", cfg);
    result["controllers"] = controllers;
    result["moving_leader"] = moving;
    result["topology_changes"] = topology;
    result["robustness"] = robustness;
    mrc_apps::write_result(args, result);
  });
}
