#include "assignment_reference.hpp"

#include "multirobotcore/allocation/auction.hpp"
#include "multirobotcore/allocation/dynamic.hpp"
#include "multirobotcore/allocation/hungarian.hpp"
#include "multirobotcore/allocation/sequential_auction.hpp"
#include "multirobotcore/math/random.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <set>
#include <vector>

namespace a = multirobotcore::allocation;
using multirobotcore::Vec2;

namespace {

a::CostMatrix random_costs(int n, int m, std::mt19937_64& rng, double infeasible_p, bool integer) {
  std::uniform_real_distribution<double> value(0.0, 100.0);
  std::bernoulli_distribution infeasible(infeasible_p);
  a::CostMatrix c(n, m);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < m; ++j) {
      const double v = integer ? std::floor(value(rng)) : value(rng);
      c(i, j) = infeasible(rng) ? a::kInfeasible : v;
    }
  }
  return c;
}

}  // namespace

TEST(Auction, WithinEpsilonBoundOfHungarianBothModes) {
  std::mt19937_64 rng(31);
  std::uniform_int_distribution<int> size(1, 12);
  for (int trial = 0; trial < 200; ++trial) {
    const int n = size(rng);
    const int m = size(rng);
    const a::CostMatrix c = random_costs(n, m, rng, 0.1, false);
    const a::Assignment opt = a::solve_hungarian(c);
    for (const auto mode : {a::BiddingMode::kGaussSeidel, a::BiddingMode::kJacobi}) {
      a::AuctionConfig cfg;
      cfg.mode = mode;
      cfg.epsilon = 0.01;
      const a::AuctionResult r = a::solve_auction(c, cfg);
      ASSERT_TRUE(r.converged);
      ASSERT_TRUE(r.assignment.is_consistent());
      EXPECT_EQ(r.assignment.num_assigned(), opt.num_assigned()) << "trial " << trial;
      EXPECT_GE(r.assignment.total_cost, opt.total_cost - 1e-9);
      EXPECT_LE(r.assignment.total_cost, opt.total_cost + n * cfg.epsilon + 1e-9)
          << "trial " << trial;
      std::set<std::size_t> used;
      for (const auto& t : r.assignment.robot_to_task) {
        if (t) {
          EXPECT_TRUE(used.insert(*t).second);  // no task assigned twice
        }
      }
    }
  }
}

TEST(Auction, IntegerCostsWithSmallEpsilonAreOptimal) {
  std::mt19937_64 rng(8);
  for (int trial = 0; trial < 100; ++trial) {
    const int n = 1 + trial % 7;
    const a::CostMatrix c = random_costs(n, n, rng, 0.0, true);
    a::AuctionConfig cfg;
    cfg.epsilon = 0.99 / (n + 1);  // eps < 1/n
    const a::AuctionResult r = a::solve_auction(c, cfg);
    EXPECT_DOUBLE_EQ(r.assignment.total_cost, mrc_test::brute_force_assignment(c).cost);
  }
}

TEST(Auction, LargeEpsilonStillRespectsBound) {
  std::mt19937_64 rng(2);
  const a::CostMatrix c = random_costs(20, 20, rng, 0.0, false);
  const double optimal = a::solve_hungarian(c).total_cost;
  for (const double eps : {5.0, 1.0, 0.1}) {
    a::AuctionConfig cfg;
    cfg.epsilon = eps;
    const a::AuctionResult r = a::solve_auction(c, cfg);
    EXPECT_LE(r.assignment.total_cost, optimal + 20.0 * eps + 1e-9);
    EXPECT_GE(r.assignment.total_cost, optimal - 1e-9);
  }
}

TEST(Auction, TracedRoundsEndInFinalAssignment) {
  std::mt19937_64 rng(5);
  const a::CostMatrix c = random_costs(6, 6, rng, 0.0, false);
  std::vector<a::AuctionRound> trace;
  const a::AuctionResult r = a::solve_auction_traced(c, 0.5, trace);
  ASSERT_FALSE(trace.empty());
  EXPECT_EQ(trace.size(), r.rounds);
  for (std::size_t t = 0; t < 6; ++t) {
    EXPECT_EQ(trace.back().owner[t], r.assignment.task_to_robot[t]);
  }
  // Prices never decrease from round to round.
  for (std::size_t k = 1; k < trace.size(); ++k) {
    for (std::size_t t = 0; t < 6; ++t) {
      EXPECT_GE(trace[k].prices[t], trace[k - 1].prices[t]);
    }
  }
}

TEST(Auction, MoreRobotsThanTasksAndNoFeasibleTask) {
  a::CostMatrix c(3, 1);
  c << 5.0, 1.0, 3.0;
  const auto r = a::solve_auction(c);
  EXPECT_EQ(r.assignment.task_to_robot[0], 1U);
  a::CostMatrix none(2, 2);
  none << a::kInfeasible, a::kInfeasible, 1.0, a::kInfeasible;
  const auto s = a::solve_auction(none);
  EXPECT_FALSE(s.assignment.robot_to_task[0].has_value());
  EXPECT_EQ(s.assignment.robot_to_task[1], 0U);
}

TEST(DynamicAllocation, ArrivalsCancellationsAndHysteresis) {
  a::DynamicAllocatorConfig cfg;
  cfg.switching_penalty = 2.0;
  cfg.commit_distance = 0.5;
  a::DynamicTaskAllocator allocator(cfg);
  const std::vector<a::Robot> robots{{0, {0.0, 0.0}}, {1, {10.0, 0.0}}};
  a::Task t0;
  t0.position = {1.0, 0.0};
  const std::size_t id0 = allocator.add_task(t0);

  auto u =
      allocator.update(robots, std::vector<std::optional<std::size_t>>{std::nullopt, std::nullopt});
  EXPECT_EQ(u.robot_task[0], id0);
  EXPECT_FALSE(u.robot_task[1].has_value());
  EXPECT_EQ(u.newly_assigned, 1U);

  // A new task near robot 1 goes to the idle robot.
  a::Task t1;
  t1.position = {9.0, 0.0};
  const std::size_t id1 = allocator.add_task(t1);
  u = allocator.update(robots, u.robot_task);
  EXPECT_EQ(u.robot_task[0], id0);
  EXPECT_EQ(u.robot_task[1], id1);
  EXPECT_EQ(u.reassigned, 0U);

  // A slightly better task for robot 0 does not trigger a switch (penalty 2).
  a::Task t2;
  t2.position = {0.4, 0.6};  // ~0.72 m away vs 1.0 m for its current task
  allocator.add_task(t2);
  u = allocator.update(robots, u.robot_task);
  EXPECT_EQ(u.robot_task[0], id0);
  EXPECT_EQ(u.reassigned, 0U);

  // Cancelling robot 1's task releases it; it takes the remaining task.
  EXPECT_TRUE(allocator.cancel_task(id1));
  u = allocator.update(robots, u.robot_task);
  EXPECT_EQ(u.released, 1U);
  EXPECT_TRUE(u.robot_task[1].has_value());
  EXPECT_FALSE(allocator.cancel_task(999));
}

TEST(DynamicAllocation, CommittedRobotsKeepTheirTask) {
  a::DynamicAllocatorConfig cfg;
  cfg.commit_distance = 1.0;
  a::DynamicTaskAllocator allocator(cfg);
  a::Task far;
  far.position = {0.5, 0.0};
  const std::size_t id = allocator.add_task(far);
  const std::vector<a::Robot> robots{{0, {0.0, 0.0}}, {1, {0.5, 0.1}}};
  // Robot 0 is committed (0.5 m away) even though robot 1 is closer.
  const auto u =
      allocator.update(robots, std::vector<std::optional<std::size_t>>{id, std::nullopt});
  EXPECT_EQ(u.robot_task[0], id);
  EXPECT_FALSE(u.robot_task[1].has_value());
}

TEST(SequentialAuction, AllocatesEveryTaskWithinCapacity) {
  std::mt19937_64 rng(3);
  std::uniform_real_distribution<double> u(0.0, 20.0);
  std::vector<a::Robot> robots(3);
  for (std::size_t r = 0; r < 3; ++r) {
    robots[r].id = r;
    robots[r].position = multirobotcore::math::sample_vec2(rng, u);
  }
  std::vector<a::Task> tasks(9);
  for (std::size_t k = 0; k < 9; ++k) {
    tasks[k].id = k;
    tasks[k].position = multirobotcore::math::sample_vec2(rng, u);
  }
  for (const auto objective : {a::TeamObjective::kMiniSum, a::TeamObjective::kMiniMax}) {
    const auto plan = a::sequential_single_item_auction(robots, tasks, {objective, 4});
    EXPECT_TRUE(plan.unallocated.empty());
    std::set<std::size_t> seen;
    double total = 0.0;
    for (std::size_t r = 0; r < 3; ++r) {
      EXPECT_LE(plan.sequences[r].size(), 4U);
      for (const std::size_t k : plan.sequences[r]) {
        EXPECT_TRUE(seen.insert(k).second);
      }
      EXPECT_NEAR(plan.route_length[r],
                  a::route_length(robots[r].position, tasks, plan.sequences[r]), 1e-9);
      total += plan.route_length[r];
    }
    EXPECT_EQ(seen.size(), 9U);
    EXPECT_NEAR(plan.total_length, total, 1e-9);
  }
  // Capacity too small: leftover tasks are reported.
  const auto limited =
      a::sequential_single_item_auction(robots, tasks, {a::TeamObjective::kMiniSum, 2});
  EXPECT_EQ(limited.unallocated.size(), 3U);
}

TEST(SequentialAuction, MiniMaxBalancesRoutesBetterThanMiniSum) {
  // Two robots at the same depot, tasks on one side: MiniSum lets one robot
  // take everything; MiniMax splits the work.
  const std::vector<a::Robot> robots{{0, {0.0, 0.0}}, {1, {0.0, 0.0}}};
  std::vector<a::Task> tasks(4);
  for (std::size_t k = 0; k < 4; ++k) {
    tasks[k].position = Vec2(2.0 + static_cast<double>(k), 0.0);
  }
  const auto sum = a::sequential_single_item_auction(robots, tasks, {a::TeamObjective::kMiniSum});
  const auto max = a::sequential_single_item_auction(robots, tasks, {a::TeamObjective::kMiniMax});
  EXPECT_LE(sum.total_length, max.total_length + 1e-9);
  EXPECT_LE(max.max_length, sum.max_length + 1e-9);
}
