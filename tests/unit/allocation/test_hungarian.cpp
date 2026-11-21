#include "assignment_reference.hpp"

#include "multirobotcore/allocation/hungarian.hpp"
#include "multirobotcore/allocation/types.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

namespace a = multirobotcore::allocation;
using multirobotcore::Vec2;

TEST(Hungarian, KnownSquareInstance) {
  a::CostMatrix c(3, 3);
  c << 4, 1, 3,  //
      2, 0, 5,   //
      3, 2, 2;
  const a::Assignment s = a::solve_hungarian(c);
  EXPECT_DOUBLE_EQ(s.total_cost, 5.0);
  EXPECT_EQ(s.robot_to_task[0], 1U);
  EXPECT_EQ(s.robot_to_task[1], 0U);
  EXPECT_EQ(s.robot_to_task[2], 2U);
  EXPECT_TRUE(s.is_consistent());
}

TEST(Hungarian, ClassicFourByFour) {
  // Optimal value 140 (e.g. Munkres-style textbook example).
  a::CostMatrix c(4, 4);
  c << 82, 83, 69, 92,  //
      77, 37, 49, 92,   //
      11, 69, 5, 86,    //
      8, 9, 98, 23;
  const a::Assignment s = a::solve_hungarian(c);
  EXPECT_DOUBLE_EQ(s.total_cost, 140.0);
}

TEST(Hungarian, RectangularBothOrientations) {
  a::CostMatrix wide(2, 4);
  wide << 9, 2, 7, 1,  //
      6, 4, 3, 8;
  const a::Assignment sw = a::solve_hungarian(wide);
  EXPECT_EQ(sw.num_assigned(), 2U);
  EXPECT_DOUBLE_EQ(sw.total_cost, 4.0);  // 1 + 3

  const a::CostMatrix tall = wide.transpose();
  const a::Assignment st = a::solve_hungarian(tall);
  EXPECT_EQ(st.num_assigned(), 2U);
  EXPECT_DOUBLE_EQ(st.total_cost, 4.0);
  EXPECT_EQ(st.task_to_robot.size(), 2U);
  EXPECT_TRUE(st.is_consistent());
}

TEST(Hungarian, InfeasiblePairsAreNeverAssigned) {
  a::CostMatrix c(3, 3);
  c << 1, a::kInfeasible, a::kInfeasible,              //
      a::kInfeasible, a::kInfeasible, a::kInfeasible,  //
      2, 3, a::kInfeasible;
  const a::Assignment s = a::solve_hungarian(c);
  EXPECT_EQ(s.num_assigned(), 2U);
  EXPECT_FALSE(s.robot_to_task[1].has_value());
  EXPECT_DOUBLE_EQ(s.total_cost, 4.0);  // robot 0 -> 0, robot 2 -> 1
}

TEST(Hungarian, FeasibilityDominatesCost) {
  // Assigning robot 0 to its cheap task would leave robot 1 infeasible.
  a::CostMatrix c(2, 2);
  c << 0, 100,  //
      1, a::kInfeasible;
  const a::Assignment s = a::solve_hungarian(c);
  EXPECT_EQ(s.num_assigned(), 2U);
  EXPECT_DOUBLE_EQ(s.total_cost, 101.0);
}

TEST(Hungarian, EmptyAndInvalidInputs) {
  EXPECT_EQ(a::solve_hungarian(a::CostMatrix(0, 3)).num_assigned(), 0U);
  EXPECT_EQ(a::solve_hungarian(a::CostMatrix(2, 0)).robot_to_task.size(), 2U);
  a::CostMatrix bad(1, 1);
  bad << std::nan("");
  EXPECT_THROW((void)a::solve_hungarian(bad), std::invalid_argument);
}

TEST(Hungarian, MaximisationVariant) {
  Eigen::MatrixXd benefit(2, 2);
  benefit << 3, 1,  //
      4, 5;
  const a::Assignment s = a::solve_hungarian_max(benefit);
  EXPECT_DOUBLE_EQ(s.total_cost, 8.0);
}

TEST(Hungarian, MatchesBruteForceOnRandomTinyProblems) {
  std::mt19937_64 rng(2025);
  std::uniform_int_distribution<int> size(1, 6);
  std::uniform_real_distribution<double> value(-10.0, 50.0);
  std::bernoulli_distribution infeasible(0.15);
  for (int trial = 0; trial < 400; ++trial) {
    const int n = size(rng);
    const int m = size(rng);
    a::CostMatrix c(n, m);
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < m; ++j) {
        c(i, j) = infeasible(rng) ? a::kInfeasible : std::round(value(rng) * 10.0) / 10.0;
      }
    }
    const a::Assignment s = a::solve_hungarian(c);
    const auto ref = mrc_test::brute_force_assignment(c);
    ASSERT_TRUE(s.is_consistent());
    EXPECT_EQ(s.num_assigned(), ref.feasible_pairs) << "trial " << trial;
    EXPECT_NEAR(s.total_cost, ref.cost, 1e-9) << "trial " << trial;
    // Property: no task is used twice and no infeasible pair is reported.
    for (std::size_t r = 0; r < s.robot_to_task.size(); ++r) {
      if (s.robot_to_task[r]) {
        EXPECT_TRUE(std::isfinite(
            c(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(*s.robot_to_task[r]))));
      }
    }
  }
}

TEST(CostModels, DistanceTimeEnergyAndCapacity) {
  const std::vector<a::Robot> robots{{0, {0.0, 0.0}, 2.0, 3.0, 1.0}};
  std::vector<a::Task> tasks(2);
  tasks[0].position = {3.0, 4.0};
  tasks[0].service_time = 1.0;
  tasks[1].position = {1.0, 0.0};
  tasks[1].dropoff = Vec2(1.0, 2.0);
  tasks[1].demand = 5.0;
  const a::CostMatrix d = a::euclidean_costs(robots, tasks);
  EXPECT_DOUBLE_EQ(d(0, 0), 5.0);
  EXPECT_DOUBLE_EQ(d(0, 1), 3.0);
  EXPECT_DOUBLE_EQ(a::travel_time_costs(robots, tasks)(0, 0), 3.5);
  EXPECT_DOUBLE_EQ(a::energy_costs(robots, tasks)(0, 1), 9.0);
  const a::CostMatrix w = a::weighted_costs(robots, tasks, {1.0, 0.0, 0.0, 0.0, true});
  EXPECT_TRUE(std::isinf(w(0, 1)));  // demand exceeds capacity
  EXPECT_DOUBLE_EQ(w(0, 0), 5.0);
}

TEST(Assignment, MakeAssignmentRejectsDuplicates) {
  a::CostMatrix c = a::CostMatrix::Ones(2, 2);
  EXPECT_THROW((void)a::make_assignment(c, {0U, 0U}), std::invalid_argument);
  const auto ok = a::make_assignment(c, {1U, std::nullopt});
  EXPECT_EQ(ok.task_to_robot[1], 0U);
  EXPECT_DOUBLE_EQ(ok.total_cost, 1.0);
}
