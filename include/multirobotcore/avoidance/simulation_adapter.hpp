#pragma once

/// @file simulation_adapter.hpp
/// Header-only glue between the avoidance module and the simulator. Including
/// this header requires linking MultiRobotCore::simulation; the avoidance
/// library itself depends only on the core module.

#include "multirobotcore/avoidance/avoidance.hpp"
#include "multirobotcore/simulation/simulator.hpp"

#include <span>
#include <vector>

namespace multirobotcore::avoidance {

/// Agent states (position, velocity, radius) of every simulated robot.
[[nodiscard]] inline std::vector<AgentState> agent_states(const simulation::Simulator& sim) {
  std::vector<AgentState> agents;
  agents.reserve(sim.num_robots());
  for (std::size_t i = 0; i < sim.num_robots(); ++i) {
    agents.push_back({sim.robot(i).position, sim.robot(i).velocity, sim.spec(i).radius});
  }
  return agents;
}

[[nodiscard]] inline std::vector<double> max_speeds(const simulation::Simulator& sim) {
  std::vector<double> speeds;
  speeds.reserve(sim.num_robots());
  for (const auto& spec : sim.specs()) {
    speeds.push_back(spec.max_speed);
  }
  return speeds;
}

/// Safe velocities for the current simulator state; non-operational robots are
/// treated as static obstacles.
[[nodiscard]] inline std::vector<Vec2> safe_velocities(const CollisionAvoider& avoider,
                                                       const simulation::Simulator& sim,
                                                       std::span<const Vec2> preferred) {
  const auto agents = agent_states(sim);
  const auto speeds = max_speeds(sim);
  return avoider.compute(agents, preferred, speeds, sim.operational_mask());
}

}  // namespace multirobotcore::avoidance
