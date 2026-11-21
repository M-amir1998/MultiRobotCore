// Micro-benchmarks for graph, consensus and formation kernels.
//
// mrc-modules: consensus formation graph communication

#include "multirobotcore/communication/topology.hpp"
#include "multirobotcore/consensus/consensus.hpp"
#include "multirobotcore/formation/controllers.hpp"
#include "multirobotcore/formation/shapes.hpp"
#include "multirobotcore/graph/connectivity.hpp"
#include "multirobotcore/graph/topologies.hpp"

#include <benchmark/benchmark.h>

#include <random>
#include <vector>

namespace mrc = multirobotcore;

namespace {

std::vector<mrc::Vec2> random_positions(std::size_t n, std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> u(0.0, 2.0 * std::sqrt(static_cast<double>(n)));
  std::vector<mrc::Vec2> p;
  for (std::size_t i = 0; i < n; ++i) {
    p.emplace_back(u(rng), u(rng));
  }
  return p;
}

void BM_DiskGraph(benchmark::State& state) {
  const auto p = random_positions(static_cast<std::size_t>(state.range(0)), 1);
  for (auto _ : state) {
    benchmark::DoNotOptimize(mrc::communication::disk_graph(p, 3.0));
  }
  state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_DiskGraph)->RangeMultiplier(2)->Range(16, 512)->Complexity();

void BM_AlgebraicConnectivity(benchmark::State& state) {
  const auto g = mrc::graph::ring_graph(static_cast<std::size_t>(state.range(0)));
  for (auto _ : state) {
    benchmark::DoNotOptimize(mrc::graph::algebraic_connectivity(g));
  }
  state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_AlgebraicConnectivity)->RangeMultiplier(2)->Range(16, 256)->Complexity();

void BM_ConsensusStep(benchmark::State& state) {
  const auto n = static_cast<std::size_t>(state.range(0));
  const auto g = mrc::communication::disk_graph(random_positions(n, 2), 3.0);
  mrc::consensus::StateMatrix x =
      mrc::consensus::StateMatrix::Random(static_cast<Eigen::Index>(n), 2);
  const double eps = 0.5 * mrc::consensus::max_convex_step(g);
  for (auto _ : state) {
    x = mrc::consensus::consensus_step(g, x, eps);
    benchmark::DoNotOptimize(x.data());
  }
  state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_ConsensusStep)->RangeMultiplier(2)->Range(16, 1024)->Complexity();

void BM_ConsensusFormationVelocities(benchmark::State& state) {
  const auto n = static_cast<std::size_t>(state.range(0));
  const auto offsets = mrc::formation::make_shape(mrc::formation::Shape::kCircle, n, 1.0);
  const auto g = mrc::graph::ring_graph(n);
  const auto p = random_positions(n, 3);
  std::vector<double> pin(n, 0.0);
  pin[0] = 1.0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(mrc::formation::consensus_formation_velocities(
        g, p, offsets, mrc::Pose2{}, mrc::Vec2::Zero(), pin, {}));
  }
}
BENCHMARK(BM_ConsensusFormationVelocities)->Arg(8)->Arg(32)->Arg(128);

void BM_HennebergGraph(benchmark::State& state) {
  const auto offsets = mrc::formation::make_shape(mrc::formation::Shape::kSquare,
                                                  static_cast<std::size_t>(state.range(0)), 1.0);
  for (auto _ : state) {
    benchmark::DoNotOptimize(mrc::formation::henneberg_graph(offsets));
  }
}
BENCHMARK(BM_HennebergGraph)->Arg(8)->Arg(32)->Arg(64);

}  // namespace
