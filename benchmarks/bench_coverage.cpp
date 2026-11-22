// Micro-benchmarks for Voronoi coverage: bounded partition, cell integrals and
// one controller evaluation for N robots in a 20 m x 20 m square.
//
// mrc-modules: coverage

#include "multirobotcore/coverage/coverage_control.hpp"
#include "multirobotcore/coverage/voronoi.hpp"
#include "multirobotcore/math/random.hpp"

#include <benchmark/benchmark.h>

#include <cmath>
#include <random>
#include <vector>

namespace cov = multirobotcore::coverage;
namespace math = multirobotcore::math;
using multirobotcore::Vec2;

namespace {

std::vector<Vec2> positions(std::size_t n) {
  std::mt19937_64 rng(42);
  std::uniform_real_distribution<double> u(0.0, 20.0);
  std::vector<Vec2> p;
  for (std::size_t i = 0; i < n; ++i) {
    p.push_back(math::sample_vec2(rng, u));
  }
  return p;
}

const math::ConvexPolygon& domain() {
  static const math::ConvexPolygon d = math::ConvexPolygon::from_box({{0.0, 0.0}, {20.0, 20.0}});
  return d;
}

void BM_BoundedVoronoi(benchmark::State& state) {
  const auto p = positions(static_cast<std::size_t>(state.range(0)));
  for (auto _ : state) {
    benchmark::DoNotOptimize(cov::bounded_voronoi(p, domain()));
  }
  state.SetComplexityN(state.range(0));
}

void BM_CoverageEvaluation(benchmark::State& state) {
  const auto p = positions(static_cast<std::size_t>(state.range(0)));
  const auto density = cov::DensityFunction::gaussian_mixture(
      {{{14.0, 13.0}, 2.5, 5.0}, {{5.0, 6.0}, 1.8, 3.0}}, 0.2);
  const cov::CoverageController controller(domain(), density, {1.0, 1.0, 4, {}});
  for (auto _ : state) {
    benchmark::DoNotOptimize(controller.evaluate(p));
  }
  state.SetComplexityN(state.range(0));
}

void BM_LimitedRangeEvaluation(benchmark::State& state) {
  const auto n = static_cast<std::size_t>(state.range(0));
  const auto p = positions(n);
  const double spacing = 20.0 / std::sqrt(static_cast<double>(n));
  const cov::CoverageController controller(domain(), cov::DensityFunction::uniform(),
                                           {1.0, 1.0, 4, 2.0 * spacing});
  for (auto _ : state) {
    benchmark::DoNotOptimize(controller.controller_state(p));
  }
}

}  // namespace

BENCHMARK(BM_BoundedVoronoi)->RangeMultiplier(2)->Range(8, 256)->Complexity();
BENCHMARK(BM_CoverageEvaluation)->RangeMultiplier(2)->Range(8, 256)->Complexity();
BENCHMARK(BM_LimitedRangeEvaluation)->RangeMultiplier(2)->Range(8, 256);
