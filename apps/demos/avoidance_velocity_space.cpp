// Velocity-space view of one robot avoiding three others with ORCA. Each
// recorded frame contains the world state plus the focal robot's velocity
// obstacles, ORCA half-planes, preferred and selected velocity.
//
// Output: JSON consumed by scripts/render/render_velocity_obstacles.py.
//
// mrc-modules: avoidance simulation io

#include "demo_common.hpp"

#include "multirobotcore/avoidance/avoidance.hpp"
#include "multirobotcore/avoidance/simulation_adapter.hpp"
#include "multirobotcore/io/json.hpp"
#include "multirobotcore/simulation/simulator.hpp"

#include <cmath>
#include <vector>

namespace mrc = multirobotcore;
namespace av = multirobotcore::avoidance;
namespace io = multirobotcore::io;
using mrc_apps::Json;

namespace {

Json cone_json(const av::VelocityObstacleCone& c) {
  return Json{{"apex", io::to_json(c.apex)},
              {"cutoff_center", io::to_json(c.cutoff_center)},
              {"cutoff_radius", io::round_to(c.cutoff_radius, 5)},
              {"left_leg", io::to_json(c.left_leg, 5)},
              {"right_leg", io::to_json(c.right_leg, 5)},
              {"overlapping", c.overlapping}};
}

}  // namespace

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args = mrc_apps::parse_args(argc, argv, "avoidance_velocity_space",
                                           "configs/avoidance/velocity_space.json");
    const Json cfg = mrc_apps::load_config(args);
    const double radius = cfg.value("robot_radius", 0.35);
    const double vmax = cfg.value("max_speed", 1.0);
    const auto focus = cfg.value("focus", std::size_t{0});

    std::vector<mrc::Pose2> poses;
    std::vector<mrc::Vec2> goals;
    for (const auto& r : cfg.at("robots")) {
      const mrc::Vec2 start = io::vec2_from_json(r.at("start"));
      goals.push_back(io::vec2_from_json(r.at("goal")));
      const mrc::Vec2 d = goals.back() - start;
      poses.push_back({start, std::atan2(d.y(), d.x())});
    }
    const std::size_t n = poses.size();
    mrc::simulation::WorldConfig world;
    world.dt = cfg.value("dt", 0.05);
    world.record = false;
    mrc::simulation::RobotSpec spec;
    spec.radius = radius;
    spec.max_speed = vmax;
    mrc::simulation::Simulator sim(world, std::vector<mrc::simulation::RobotSpec>(n, spec), poses);

    av::AvoidanceConfig config;
    const Json acfg = cfg.value("avoidance", Json::object());
    config.method = av::AvoidanceMethod::kOrca;
    config.time_step = world.dt;
    config.time_horizon = acfg.value("time_horizon", 3.0);
    config.obstacle_time_horizon = acfg.value("obstacle_time_horizon", 1.0);
    config.neighbor_distance = acfg.value("neighbor_distance", 8.0);
    config.max_neighbors = acfg.value("max_neighbors", std::size_t{8});
    config.safety_margin = acfg.value("safety_margin", 0.0);
    const av::CollisionAvoider avoider(config);

    const auto steps =
        static_cast<std::size_t>(std::llround(cfg.value("duration", 11.0) / world.dt));
    const auto every = cfg.value("record_every", std::size_t{2});
    Json frames = Json::array();
    for (std::size_t k = 0; k <= steps; ++k) {
      std::vector<mrc::Vec2> pref(n);
      for (std::size_t i = 0; i < n; ++i) {
        pref[i] = av::preferred_velocity(sim.robot(i).position, goals[i], vmax, 0.5);
      }
      const auto agents = av::agent_states(sim);
      const auto speeds = av::max_speeds(sim);
      const auto v = avoider.compute(agents, pref, speeds);
      if (k % every == 0) {
        const auto d = avoider.diagnose(focus, agents, pref, speeds);
        Json cones = Json::array();
        Json planes = Json::array();
        for (const auto& nb : d.neighbors) {
          Json c = cone_json(nb.vo);
          c["neighbor"] = nb.index;
          c["is_robot"] = nb.is_robot;
          cones.push_back(std::move(c));
          planes.push_back(Json{{"neighbor", nb.index},
                                {"is_robot", nb.is_robot},
                                {"normal", io::to_json(nb.orca.half_plane.normal, 5)},
                                {"offset", io::round_to(nb.orca.half_plane.offset, 5)}});
        }
        frames.push_back(Json{{"t", io::round_to(sim.time(), 3)},
                              {"positions", io::to_json(sim.positions())},
                              {"velocities", io::to_json(sim.velocities())},
                              {"preferred", io::to_json(pref)},
                              {"selected", io::to_json(d.selected)},
                              {"feasible", d.feasible},
                              {"vo", cones},
                              {"orca", planes}});
      }
      if (k < steps) {
        sim.step(v);
      }
    }
    Json result = mrc_apps::metadata("mrc_demo_avoidance_velocity_space", cfg);
    result["robot_radius"] = radius;
    result["max_speed"] = vmax;
    result["focus"] = focus;
    result["goals"] = io::to_json(goals);
    result["time_horizon"] = config.time_horizon;
    result["frames"] = frames;
    result["metrics"] = io::to_json(sim.metrics());
    mrc_apps::write_result(args, result);
  });
}
