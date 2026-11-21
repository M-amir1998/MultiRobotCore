// Regression tests: small deterministic MAPF scenarios with recorded outputs.
//
// The generators use platform-independent random helpers, so these values are
// identical on every compiler. A change here means the planners' behaviour
// changed: confirm the new values are correct (e.g. against the joint-state
// reference in tests/unit/mapf) before updating them.

#include "multirobotcore/mapf/conflict_based_search.hpp"
#include "multirobotcore/mapf/conflicts.hpp"
#include "multirobotcore/mapf/generators.hpp"
#include "multirobotcore/mapf/planners.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <random>
#include <string>
#include <vector>

namespace mapf = multirobotcore::mapf;

namespace {

mapf::Instance plus_intersection() {
  const std::vector<std::string> rows{"######.######", "######.######", "######.######",
                                      "######.######", "######.######", ".............",
                                      "######.######", "######.######", "######.######",
                                      "######.######", "######.######"};
  return {mapf::GridMap::from_ascii(rows),
          {{{0, 5}, {12, 5}}, {{12, 5}, {0, 5}}, {{6, 10}, {6, 0}}, {{6, 0}, {6, 10}}}};
}

mapf::Instance warehouse_demo(std::size_t agents) {
  mapf::WarehouseLayout layout;
  layout.shelf_block_rows = 4;
  const auto wh = mapf::make_warehouse(layout);
  std::mt19937_64 rng(11);
  auto instance = mapf::random_instance(wh.map, 24, wh.stations, wh.pickups, rng);
  instance.agents.resize(agents);
  return instance;
}

}  // namespace

TEST(MapfRegression, PlusIntersection) {
  const auto inst = plus_intersection();
  const auto independent = mapf::plan_independent(inst);
  EXPECT_EQ(independent.sum_of_costs, 44U);
  EXPECT_EQ(independent.conflicts, 2U);
  EXPECT_EQ(mapf::plan_prioritized(inst).status, mapf::PlannerStatus::kFailed);
  const auto cbs = mapf::ConflictBasedSearch().solve(inst);
  ASSERT_TRUE(cbs.solved());
  EXPECT_EQ(cbs.sum_of_costs, 54U);
  EXPECT_EQ(cbs.makespan, 16U);
  EXPECT_TRUE(mapf::is_collision_free(cbs.paths));
}

TEST(MapfRegression, WarehouseDemoInstance) {
  const auto inst = warehouse_demo(24);
  const auto independent = mapf::plan_independent(inst);
  EXPECT_EQ(independent.sum_of_costs, 473U);
  EXPECT_EQ(independent.conflicts, 62U);
  mapf::PrioritizedOptions opt;
  opt.max_attempts = 20;
  opt.seed = 11;
  const auto pp = mapf::plan_prioritized(inst, opt);
  ASSERT_TRUE(pp.solved());
  EXPECT_EQ(pp.sum_of_costs, 562U);
  EXPECT_EQ(pp.attempts, 3U);
  EXPECT_TRUE(mapf::check_paths(inst, pp.paths).valid);
}

TEST(MapfRegression, WarehouseSubsetCbsBeatsPrioritized) {
  const auto inst = warehouse_demo(12);
  const auto pp = mapf::plan_prioritized(inst);
  const auto cbs = mapf::ConflictBasedSearch().solve(inst);
  ASSERT_TRUE(pp.solved());
  ASSERT_TRUE(cbs.solved());
  EXPECT_EQ(pp.sum_of_costs, 257U);
  EXPECT_EQ(cbs.sum_of_costs, 252U);
}

TEST(MapfRegression, CommittedWarehouseMapMatchesGenerator) {
  const std::filesystem::path file =
      std::filesystem::path(MRC_SOURCE_DIR) / "scenarios" / "warehouse" / "warehouse_small.map";
  mapf::WarehouseLayout layout;
  layout.shelf_block_rows = 4;
  EXPECT_EQ(mapf::GridMap::load_movingai(file), mapf::make_warehouse(layout).map);
}

TEST(MapfRegression, GeneratorsArePlatformIndependent) {
  std::mt19937_64 rng(1 * 7919 + 17);
  const auto maze = mapf::make_maze(31, 31, 0.15, rng);
  EXPECT_EQ(maze.num_free_cells(), 482U);
  std::mt19937_64 rng2(5);
  const auto inst = mapf::random_instance(mapf::make_open_map(8, 8), 3, rng2);
  EXPECT_EQ(inst.agents[0].start, (mapf::Cell{3, 0}));
  EXPECT_EQ(inst.agents[2].goal, (mapf::Cell{1, 3}));
}
