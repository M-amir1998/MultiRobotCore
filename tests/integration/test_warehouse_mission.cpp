// Task allocation -> MAPF -> execution -> all tasks completed.

#include "multirobotcore/scenarios/warehouse_mission.hpp"

#include <gtest/gtest.h>

namespace sc = multirobotcore::scenarios;

namespace {

sc::WarehouseMissionConfig small_mission() {
  sc::WarehouseMissionConfig cfg;
  cfg.layout.shelf_block_columns = 3;
  cfg.layout.shelf_block_rows = 3;
  cfg.num_robots = 8;
  cfg.initial_tasks = 10;
  cfg.arrival_rate = 0.0;  // fixed task set
  cfg.duration = 90.0;
  cfg.robot_radius = 0.25;
  cfg.orca_time_horizon = 0.6;
  cfg.record_every = 20;
  return cfg;
}

}  // namespace

TEST(WarehouseMission, AllocatesPlansAndDeliversEveryTask) {
  const auto result = sc::run_warehouse_mission(small_mission());
  ASSERT_EQ(result.tasks.size(), 10U);
  EXPECT_EQ(result.tasks_completed, 10U);
  for (const auto& task : result.tasks) {
    EXPECT_GE(task.delivered_time, task.picked_time);
    EXPECT_GE(task.picked_time, task.assigned_time);
    ASSERT_TRUE(task.dropoff.has_value());
  }
  EXPECT_EQ(result.plan_conflicts, 0U);  // every executed plan was conflict-free
  EXPECT_EQ(result.planning_failures, 0U);
  EXPECT_EQ(result.metrics.robot_collisions, 0U);
  EXPECT_EQ(result.metrics.obstacle_collisions, 0U);
  EXPECT_LT(result.max_tracking_error, 1.0);
}

TEST(WarehouseMission, ConflictBasedSearchPlannerAlsoCompletes) {
  auto cfg = small_mission();
  cfg.planner = "cbs";
  const auto result = sc::run_warehouse_mission(cfg);
  EXPECT_EQ(result.tasks_completed, result.tasks.size());
  EXPECT_EQ(result.plan_conflicts, 0U);
  EXPECT_EQ(result.metrics.robot_collisions, 0U);
}

TEST(WarehouseMission, RunsAreDeterministic) {
  const auto a = sc::run_warehouse_mission(small_mission());
  const auto b = sc::run_warehouse_mission(small_mission());
  ASSERT_EQ(a.frames.size(), b.frames.size());
  EXPECT_EQ(a.frames.back().poses[3].position, b.frames.back().poses[3].position);
  EXPECT_EQ(a.tasks_completed, b.tasks_completed);
}
