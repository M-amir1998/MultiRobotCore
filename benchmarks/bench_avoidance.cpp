// Micro-benchmarks for local collision avoidance: one team-wide velocity
// update (ORCA, sampled RVO) for N robots packed in a ring, and the LP kernel.
//
// mrc-modules: avoidance

#include "multirobotcore/avoidance/avoidance.hpp"
#include "multirobotcore/avoidance/linear_program.hpp"
#include "multirobotcore/math/numerics.hpp"

#include <benchmark/benchmark.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace av = multirobotcore::avoidance;
using multirobotcore::Vec2;

namespace {

struct Team {
  std::vector<av::AgentState> agents;
  std::vector<Vec2> preferred;
  std::vector<double> speeds;
};

/// Robots on a circle (about 1 m apart) heading to the antipodes, with
/// velocities already pointing inwards: every robot has a full neighbourhood.
Team ring_team(std::size_t n) {
  Team team;
  const double ring = std::max(4.0, static_cast<double>(n) / multirobotcore::math::kTwoPi);
  for (std::size_t i = 0; i < n; ++i) {
    const double a = multirobotcore::math::kTwoPi * static_cast<double>(i) / static_cast<double>(n);
    const Vec2 p = ring * Vec2(std::cos(a), std::sin(a));
    team.agents.push_back({0.5 * p, -0.1 * p.normalized(), 0.25});
    team.preferred.push_back(-p.normalized());
    team.speeds.push_back(1.0);
  }
  return team;
}

void run_team_update(benchmark::State& state, av::AvoidanceMethod method) {
  const Team team = ring_team(static_cast<std::size_t>(state.range(0)));
  av::AvoidanceConfig config;
  config.method = method;
  config.time_horizon = 4.0;
  config.neighbor_distance = 6.0;
  config.max_neighbors = 16;
  const av::CollisionAvoider avoider(config);
  for (auto _ : state) {
    benchmark::DoNotOptimize(avoider.compute(team.agents, team.preferred, team.speeds));
  }
  state.SetItemsProcessed(state.iterations() * state.range(0));
}

void BM_OrcaTeamUpdate(benchmark::State& state) {
  run_team_update(state, av::AvoidanceMethod::kOrca);
}
void BM_RvoSamplingTeamUpdate(benchmark::State& state) {
  run_team_update(state, av::AvoidanceMethod::kReciprocalVelocityObstacle);
}

void BM_VelocityLp(benchmark::State& state) {
  std::mt19937_64 rng(1);
  std::uniform_real_distribution<double> angle(0.0, multirobotcore::math::kTwoPi);
  std::uniform_real_distribution<double> offset(-0.8, 0.3);
  std::vector<av::HalfPlane> constraints;
  for (int k = 0; k < state.range(0); ++k) {
    const double a = angle(rng);
    constraints.push_back({{std::cos(a), std::sin(a)}, offset(rng)});
  }
  for (auto _ : state) {
    benchmark::DoNotOptimize(av::solve_velocity_lp(constraints, 0, {1.0, 0.0}, 1.0));
  }
}

}  // namespace

BENCHMARK(BM_OrcaTeamUpdate)->Arg(10)->Arg(50)->Arg(100)->Arg(200)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_RvoSamplingTeamUpdate)->Arg(10)->Arg(50)->Arg(100)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_VelocityLp)->Arg(4)->Arg(16)->Arg(32)->Unit(benchmark::kNanosecond);
