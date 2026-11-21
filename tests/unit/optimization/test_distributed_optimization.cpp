#include "multirobotcore/graph/topologies.hpp"
#include "multirobotcore/math/numerics.hpp"
#include "multirobotcore/optimization/distributed.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

namespace opt = multirobotcore::optimization;
namespace g = multirobotcore::graph;
using multirobotcore::Vec2;

namespace {

std::vector<Vec2> ring_of_robots(std::size_t n, double radius) {
  std::vector<Vec2> p;
  for (std::size_t i = 0; i < n; ++i) {
    const double a = multirobotcore::math::kTwoPi * static_cast<double>(i) / static_cast<double>(n);
    p.emplace_back(radius * std::cos(a), radius * std::sin(a));
  }
  return p;
}

std::vector<Eigen::VectorXd> zeros(std::size_t n) {
  return std::vector<Eigen::VectorXd>(n, Eigen::VectorXd::Zero(2));
}

}  // namespace

TEST(BearingLocalization, NoiselessCentralSolutionIsTheTarget) {
  const auto robots = ring_of_robots(6, 5.0);
  const Vec2 target(1.0, -0.7);
  std::mt19937_64 rng(1);
  const auto problem =
      opt::bearing_localization_problem(robots, opt::measure_bearings(robots, target, 0.0, rng));
  EXPECT_TRUE(problem.centralized_solution().isApprox(target, 1e-12));
  EXPECT_NEAR(problem.objective(target), 0.0, 1e-20);
  // A single bearing is rank one: the gradient vanishes along the bearing line.
  const Eigen::VectorXd along = target + 2.0 * (target - robots[0]).normalized();
  EXPECT_LT(problem.local_gradient(0, along).norm(), 1e-12);
}

TEST(BearingLocalization, SingularProblemIsRejected) {
  // Two robots with parallel lines of bearing cannot localise the target.
  const std::vector<Vec2> robots{{0.0, 0.0}, {1.0, 0.0}};
  const std::vector<double> bearings{0.0, 0.0};
  const auto problem = opt::bearing_localization_problem(robots, bearings);
  EXPECT_THROW((void)problem.centralized_solution(), std::runtime_error);
}

TEST(DistributedOptimization, AdmmConvergesToCentralisedSolution) {
  const auto robots = ring_of_robots(10, 6.0);
  std::mt19937_64 rng(4);
  const auto problem = opt::bearing_localization_problem(
      robots, opt::measure_bearings(robots, {0.5, 1.0}, 0.05, rng));
  const auto optimum = problem.centralized_solution();
  const auto run =
      opt::decentralized_admm(problem, g::ring_graph(10), zeros(10), {1.0, 5000, 1e-8, false});
  ASSERT_TRUE(run.converged);
  for (const auto& x : run.estimates) {
    EXPECT_LT((x - optimum).norm(), 1e-6);
  }
  EXPECT_EQ(run.messages, run.iterations * 2 * 10);
  // Error trend: the last record is far below the first.
  EXPECT_LT(run.history.back().solution_error, 1e-6 * run.history.front().solution_error + 1e-7);
}

TEST(DistributedOptimization, DgdDiminishingStepApproachesOptimum) {
  const auto robots = ring_of_robots(8, 4.0);
  std::mt19937_64 rng(9);
  const auto problem = opt::bearing_localization_problem(
      robots, opt::measure_bearings(robots, {0.2, -0.4}, 0.02, rng));
  opt::GradientDescentConfig cfg;
  cfg.step = 0.5;
  cfg.rule = opt::StepRule::kDiminishing;
  cfg.max_iterations = 20000;
  cfg.tolerance = 1e-3;
  const auto run = opt::distributed_gradient_descent(problem, g::complete_graph(8), zeros(8), cfg);
  EXPECT_TRUE(run.converged);
  EXPECT_LT(run.history.back().consensus_error, 1e-2);
}

TEST(DistributedOptimization, DgdConstantStepStallsInANeighbourhood) {
  // With a constant step, DGD's fixed points are not exact optima: the error
  // stops decreasing at a level that shrinks with the step size.
  const auto robots = ring_of_robots(8, 4.0);
  std::mt19937_64 rng(9);
  const auto problem = opt::bearing_localization_problem(
      robots, opt::measure_bearings(robots, {0.2, -0.4}, 0.02, rng));
  auto residual = [&](double step) {
    opt::GradientDescentConfig cfg;
    cfg.step = step;
    cfg.rule = opt::StepRule::kConstant;
    cfg.max_iterations = 20000;
    cfg.tolerance = 0.0;
    return opt::distributed_gradient_descent(problem, g::ring_graph(8), zeros(8), cfg)
        .history.back()
        .solution_error;
  };
  const double large = residual(0.4);
  const double small = residual(0.05);
  EXPECT_GT(large, 1e-4);
  EXPECT_LT(small, large);
}

TEST(DistributedOptimization, InputValidation) {
  const auto robots = ring_of_robots(4, 3.0);
  std::mt19937_64 rng(1);
  const auto problem = opt::bearing_localization_problem(
      robots, opt::measure_bearings(robots, {0.0, 0.0}, 0.0, rng));
  g::Graph split = g::path_graph(4);
  split.remove_edge(1, 2);
  EXPECT_THROW((void)opt::decentralized_admm(problem, split, zeros(4), {}), std::invalid_argument);
  EXPECT_THROW((void)opt::distributed_gradient_descent(problem, g::directed_ring(4), zeros(4), {}),
               std::invalid_argument);
  EXPECT_THROW((void)opt::decentralized_admm(problem, g::ring_graph(4), zeros(3), {}),
               std::invalid_argument);
}
