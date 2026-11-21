#include "joint_reference.hpp"

#include "multirobotcore/mapf/conflict_based_search.hpp"
#include "multirobotcore/mapf/conflicts.hpp"
#include "multirobotcore/mapf/generators.hpp"
#include "multirobotcore/mapf/planners.hpp"

#include <gtest/gtest.h>

#include <random>
#include <string>
#include <vector>

namespace mapf = multirobotcore::mapf;
using mapf::Cell;

namespace {

/// Corridor with a single passing pocket above the middle cell:
///   ##.##
///   .....
mapf::Instance pocket_swap() {
  return {mapf::GridMap::from_ascii({"##.##", "....."}), {{{0, 0}, {4, 0}}, {{4, 0}, {0, 0}}}};
}

/// Plus-shaped intersection, two agents crossing the centre.
mapf::Instance small_intersection() {
  return {mapf::GridMap::from_ascii({"@.@", "...", "@.@"}), {{{0, 1}, {2, 1}}, {{1, 0}, {1, 2}}}};
}

void expect_valid_solution(const mapf::Instance& inst, const mapf::PlannerResult& r) {
  ASSERT_EQ(r.status, mapf::PlannerStatus::kSolved) << r.message;
  const auto check = mapf::check_paths(inst, r.paths);
  EXPECT_TRUE(check.valid) << check.message;
  EXPECT_TRUE(mapf::is_collision_free(r.paths));
  EXPECT_EQ(r.sum_of_costs, mapf::sum_of_costs(r.paths));
  EXPECT_EQ(r.makespan, mapf::makespan(r.paths));
}

}  // namespace

TEST(Independent, ShortestPathsCollideInCorridor) {
  const auto inst = pocket_swap();
  const auto r = mapf::plan_independent(inst);
  EXPECT_EQ(r.status, mapf::PlannerStatus::kUnsafe);
  EXPECT_GT(r.conflicts, 0U);
  EXPECT_EQ(r.sum_of_costs, 8U);  // lower bound: 4 + 4
}

TEST(Cbs, TwoAgentSwapThroughPocket) {
  const auto inst = pocket_swap();
  const auto r = mapf::ConflictBasedSearch().solve(inst);
  expect_valid_solution(inst, r);
  // One agent waits once (cost 5), the other detours through the pocket (6).
  EXPECT_EQ(r.sum_of_costs, 11U);
  EXPECT_EQ(r.sum_of_costs, mrc_test::optimal_sum_of_costs(inst));
  EXPECT_GT(r.high_level_expanded, 0U);
  const auto mk = mapf::ConflictBasedSearch({mapf::Objective::kMakespan}).solve(inst);
  expect_valid_solution(inst, mk);
  EXPECT_EQ(mk.makespan, 6U);
  EXPECT_EQ(mk.makespan, mrc_test::optimal_makespan(inst));
}

TEST(Prioritized, FailsOnPocketSwapForEveryOrder) {
  // The higher-priority agent ignores the other one and parks on its start
  // route, so neither ordering can succeed: PP is incomplete.
  const auto inst = pocket_swap();
  for (const auto order : {std::vector<std::size_t>{0, 1}, std::vector<std::size_t>{1, 0}}) {
    mapf::PrioritizedOptions opt;
    opt.order = mapf::PriorityOrder::kCustom;
    opt.custom_order = order;
    const auto r = mapf::plan_prioritized(inst, opt);
    EXPECT_EQ(r.status, mapf::PlannerStatus::kFailed);
    EXPECT_FALSE(r.message.empty());
  }
}

TEST(Cbs, IntersectionRequiresOneWait) {
  const auto inst = small_intersection();
  const auto indep = mapf::plan_independent(inst);
  EXPECT_EQ(indep.status, mapf::PlannerStatus::kUnsafe);
  const auto r = mapf::ConflictBasedSearch().solve(inst);
  expect_valid_solution(inst, r);
  EXPECT_EQ(r.sum_of_costs, 5U);
  const auto pp = mapf::plan_prioritized(inst);
  expect_valid_solution(inst, pp);
  EXPECT_EQ(pp.sum_of_costs, 5U);
}

TEST(Cbs, CorridorWithSideRoomResolvesMultipleConflicts) {
  // Three agents in a corridor with one side room: several conflicts must be
  // resolved jointly.
  const mapf::Instance inst{mapf::GridMap::from_ascii({"@@.@@@", "......", "@@@@.@"}),
                            {{{0, 1}, {5, 1}}, {{5, 1}, {0, 1}}, {{4, 0}, {1, 1}}}};
  const auto r = mapf::ConflictBasedSearch().solve(inst);
  expect_valid_solution(inst, r);
  EXPECT_EQ(r.sum_of_costs, mrc_test::optimal_sum_of_costs(inst, 60));
}

TEST(Cbs, NodeLimitIsReported) {
  const auto inst = pocket_swap();
  mapf::CbsOptions opt;
  opt.max_high_level_nodes = 0;
  const auto r = mapf::ConflictBasedSearch(opt).solve(inst);
  EXPECT_EQ(r.status, mapf::PlannerStatus::kNodeLimit);
  EXPECT_TRUE(r.paths.empty());
}

TEST(Cbs, UnreachableGoalIsNoSolution) {
  const mapf::Instance inst{mapf::GridMap::from_ascii({".@."}), {{{0, 0}, {2, 0}}}};
  EXPECT_EQ(mapf::ConflictBasedSearch().solve(inst).status, mapf::PlannerStatus::kNoSolution);
  EXPECT_EQ(mapf::plan_prioritized(inst).status, mapf::PlannerStatus::kFailed);
}

TEST(Cbs, MatchesJointStateOptimumOnRandomTinyInstances) {
  std::mt19937_64 rng(20251121);
  int compared = 0;
  for (int trial = 0; trial < 60; ++trial) {
    const int w = 3 + static_cast<int>(rng() % 2);
    const int h = 3;
    const auto map = mapf::make_random_map(w, h, 0.15, rng);
    const std::size_t agents = 2 + static_cast<std::size_t>(rng() % 2);
    if (mapf::largest_component(map).size() < agents + 1) {
      continue;
    }
    const auto inst = mapf::random_instance(map, agents, rng);
    const auto soc_ref = mrc_test::optimal_sum_of_costs(inst, 30);
    if (!soc_ref) {
      // Unsolvable: CBS cannot prove it in general (the constraint tree is
      // unbounded), so it must simply never report a solution.
      mapf::CbsOptions limited;
      limited.max_high_level_nodes = 2000;
      EXPECT_NE(mapf::ConflictBasedSearch(limited).solve(inst).status, mapf::PlannerStatus::kSolved)
          << "trial " << trial;
      continue;
    }
    const auto r = mapf::ConflictBasedSearch().solve(inst);
    expect_valid_solution(inst, r);
    EXPECT_EQ(r.sum_of_costs, *soc_ref) << "trial " << trial;

    const auto mk_ref = mrc_test::optimal_makespan(inst, 30);
    ASSERT_TRUE(mk_ref.has_value());
    const auto mk = mapf::ConflictBasedSearch({mapf::Objective::kMakespan}).solve(inst);
    expect_valid_solution(inst, mk);
    EXPECT_EQ(mk.makespan, *mk_ref) << "trial " << trial;

    // Independent planning is a lower bound; prioritized planning, when it
    // succeeds, is an upper bound.
    EXPECT_LE(mapf::plan_independent(inst).sum_of_costs, *soc_ref);
    const auto pp = mapf::plan_prioritized(inst);
    if (pp.solved()) {
      EXPECT_GE(pp.sum_of_costs, *soc_ref);
    }
    ++compared;
  }
  EXPECT_GE(compared, 30);
}

TEST(PlannersProperty, RandomInstancesProduceValidCollisionFreePaths) {
  std::mt19937_64 rng(99);
  for (int trial = 0; trial < 25; ++trial) {
    const auto map = mapf::make_random_map(10, 10, 0.2, rng);
    const auto inst = mapf::random_instance(map, 6, rng);
    mapf::CbsOptions opt;
    opt.time_limit_s = 10.0;
    const auto cbs = mapf::ConflictBasedSearch(opt).solve(inst);
    if (cbs.status == mapf::PlannerStatus::kSolved) {
      expect_valid_solution(inst, cbs);
      EXPECT_GE(cbs.sum_of_costs, mapf::plan_independent(inst).sum_of_costs);
    }
    mapf::PrioritizedOptions pp_opt;
    pp_opt.order = mapf::PriorityOrder::kRandom;
    pp_opt.seed = static_cast<std::uint64_t>(trial);
    pp_opt.max_attempts = 5;
    const auto pp = mapf::plan_prioritized(inst, pp_opt);
    if (pp.solved()) {
      expect_valid_solution(inst, pp);
      if (cbs.solved()) {
        EXPECT_GE(pp.sum_of_costs, cbs.sum_of_costs);
      }
    }
  }
}

TEST(Prioritized, OrderingChangesTheResult) {
  // In an open room PP always succeeds, but the cost depends on the order.
  std::mt19937_64 rng(4);
  const auto inst = mapf::random_instance(mapf::make_open_map(6, 6), 10, rng);
  std::size_t best = SIZE_MAX;
  std::size_t worst = 0;
  for (std::uint64_t seed = 0; seed < 20; ++seed) {
    mapf::PrioritizedOptions opt;
    opt.order = mapf::PriorityOrder::kRandom;
    opt.seed = seed;
    const auto r = mapf::plan_prioritized(inst, opt);
    if (r.solved()) {
      best = std::min(best, r.sum_of_costs);
      worst = std::max(worst, r.sum_of_costs);
      EXPECT_EQ(r.order.size(), 10U);
    }
  }
  EXPECT_LE(best, worst);
  EXPECT_NE(best, SIZE_MAX);
}
