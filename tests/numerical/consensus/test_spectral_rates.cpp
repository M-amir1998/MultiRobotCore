// Measured asymptotic consensus convergence rates versus the spectral
// predictions: discrete rho = max(|1 - eps lambda_2|, |1 - eps lambda_N|)
// and continuous decay exp(-lambda_2 t).

#include "multirobotcore/consensus/consensus.hpp"
#include "multirobotcore/graph/connectivity.hpp"
#include "multirobotcore/graph/topologies.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

namespace c = multirobotcore::consensus;
namespace g = multirobotcore::graph;

namespace {

double deviation(const c::StateMatrix& x) {
  return (x.rowwise() - c::average(x)).norm();
}

c::StateMatrix random_state(std::size_t n, std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  c::StateMatrix x(static_cast<Eigen::Index>(n), 1);
  for (Eigen::Index i = 0; i < x.rows(); ++i) {
    x(i, 0) = u(rng);
  }
  return x;
}

}  // namespace

TEST(SpectralRates, DiscreteAsymptoticRateEqualsContractionFactor) {
  std::mt19937_64 rng(17);
  int checked = 0;
  for (int trial = 0; trial < 20 && checked < 8; ++trial) {
    const g::Graph graph = g::random_graph(12, 0.35, rng);
    if (!g::is_connected(graph)) {
      continue;
    }
    const double eps = 0.6 * c::max_convex_step(graph);
    const double rho = c::discrete_contraction_factor(graph, eps);
    c::StateMatrix x = random_state(12, static_cast<std::uint64_t>(trial));
    // Let the transient decay, then measure the per-step ratio.
    for (int k = 0; k < 1000; ++k) {
      x = c::consensus_step(graph, x, eps);
      x /= std::max(deviation(x), 1e-300);  // normalise to avoid underflow
      x.rowwise() -= c::average(x);
    }
    const double before = deviation(x);
    const auto next = c::consensus_step(graph, x, eps);
    const double measured = deviation(next) / before;
    EXPECT_LE(measured, rho + 1e-12) << "trial " << trial;  // the bound holds
    EXPECT_NEAR(measured, rho, 1e-3) << "trial " << trial;  // and is attained asymptotically
    ++checked;
  }
  EXPECT_GE(checked, 5);
}

TEST(SpectralRates, ContinuousDecayRateEqualsAlgebraicConnectivity) {
  for (const std::size_t n : {6U, 10U, 16U}) {
    const g::Graph graph = g::ring_graph(n);
    const double lambda2 = g::algebraic_connectivity(graph);
    c::StateMatrix x = random_state(n, n);
    const double dt = 0.01;
    for (int k = 0; k < 1500; ++k) {  // transient
      x = c::consensus_flow_rk4(graph, x, dt);
    }
    const double d1 = deviation(x);
    const int steps = 300;
    for (int k = 0; k < steps; ++k) {
      x = c::consensus_flow_rk4(graph, x, dt);
    }
    const double measured = -std::log(deviation(x) / d1) / (steps * dt);
    EXPECT_NEAR(measured, lambda2, 1e-4 * std::max(1.0, lambda2)) << "n = " << n;
  }
}
