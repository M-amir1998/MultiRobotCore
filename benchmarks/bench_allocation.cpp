// Micro-benchmarks for assignment and distributed optimisation kernels.
//
// mrc-modules: allocation optimization graph

#include "multirobotcore/allocation/auction.hpp"
#include "multirobotcore/allocation/hungarian.hpp"
#include "multirobotcore/allocation/sequential_auction.hpp"
#include "multirobotcore/graph/topologies.hpp"
#include "multirobotcore/math/random.hpp"
#include "multirobotcore/optimization/distributed.hpp"

#include <benchmark/benchmark.h>

#include <random>
#include <vector>

namespace mrc = multirobotcore;
namespace al = multirobotcore::allocation;

namespace {

al::CostMatrix random_costs(std::size_t n, std::size_t m, std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> u(0.0, 100.0);
  al::CostMatrix c(static_cast<Eigen::Index>(n), static_cast<Eigen::Index>(m));
  for (Eigen::Index i = 0; i < c.rows(); ++i) {
    for (Eigen::Index j = 0; j < c.cols(); ++j) {
      c(i, j) = u(rng);
    }
  }
  return c;
}

void BM_Hungarian(benchmark::State& state) {
  const auto n = static_cast<std::size_t>(state.range(0));
  const auto c = random_costs(n, n, 1);
  for (auto _ : state) {
    benchmark::DoNotOptimize(al::solve_hungarian(c));
  }
  state.SetComplexityN(state.range(0));
}
BENCHMARK(BM_Hungarian)->RangeMultiplier(2)->Range(8, 256)->Complexity(benchmark::oNCubed);

void BM_AuctionJacobi(benchmark::State& state) {
  const auto n = static_cast<std::size_t>(state.range(0));
  const auto c = random_costs(n, n, 1);
  for (auto _ : state) {
    benchmark::DoNotOptimize(al::solve_auction(c, {al::BiddingMode::kJacobi, 0.0}));
  }
}
BENCHMARK(BM_AuctionJacobi)->RangeMultiplier(2)->Range(8, 128);

void BM_AuctionGaussSeidel(benchmark::State& state) {
  const auto n = static_cast<std::size_t>(state.range(0));
  const auto c = random_costs(n, n, 1);
  for (auto _ : state) {
    benchmark::DoNotOptimize(al::solve_auction(c, {al::BiddingMode::kGaussSeidel, 0.0}));
  }
}
BENCHMARK(BM_AuctionGaussSeidel)->RangeMultiplier(2)->Range(8, 128);

void BM_SequentialSingleItemAuction(benchmark::State& state) {
  std::mt19937_64 rng(2);
  std::uniform_real_distribution<double> u(0.0, 30.0);
  std::vector<al::Robot> robots(5);
  for (auto& r : robots) {
    r.position = mrc::math::sample_vec2(rng, u);
  }
  std::vector<al::Task> tasks(static_cast<std::size_t>(state.range(0)));
  for (auto& t : tasks) {
    t.position = mrc::math::sample_vec2(rng, u);
  }
  for (auto _ : state) {
    benchmark::DoNotOptimize(al::sequential_single_item_auction(robots, tasks));
  }
}
BENCHMARK(BM_SequentialSingleItemAuction)->Arg(10)->Arg(25)->Arg(50);

void BM_AdmmBearingLocalisation(benchmark::State& state) {
  const auto n = static_cast<std::size_t>(state.range(0));
  std::mt19937_64 rng(3);
  std::uniform_real_distribution<double> u(0.0, 20.0);
  std::vector<mrc::Vec2> robots;
  for (std::size_t i = 0; i < n; ++i) {
    robots.push_back(mrc::math::sample_vec2(rng, u));
  }
  const auto problem = mrc::optimization::bearing_localization_problem(
      robots, mrc::optimization::measure_bearings(robots, {10.0, 10.0}, 0.02, rng));
  const auto g = mrc::graph::ring_graph(n);
  const std::vector<Eigen::VectorXd> x0(n, Eigen::VectorXd::Zero(2));
  for (auto _ : state) {
    benchmark::DoNotOptimize(
        mrc::optimization::decentralized_admm(problem, g, x0, {0.5, 100, 0.0, false}));
  }
}
BENCHMARK(BM_AdmmBearingLocalisation)->Arg(10)->Arg(50);

}  // namespace
