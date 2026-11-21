#include "multirobotcore/exploration/exploration.hpp"
#include "multirobotcore/exploration/frontier.hpp"
#include "multirobotcore/exploration/occupancy_grid.hpp"
#include "multirobotcore/exploration/planner.hpp"
#include "multirobotcore/exploration/sensor.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace ex = multirobotcore::exploration;
using ex::GridCell;
using ex::Occupancy;
using multirobotcore::Vec2;

namespace {

/// Two rooms connected by a door.
ex::OccupancyGrid two_rooms() {
  return ex::OccupancyGrid::from_ascii({
      "####################",
      "#........#.........#",
      "#........#.........#",
      "#..................#",
      "#........#.........#",
      "#........#.........#",
      "####################",
  });
}

}  // namespace

TEST(OccupancyGrid, AsciiOrientationAndConversions) {
  const auto g = ex::OccupancyGrid::from_ascii({"#..", "..."}, 0.5, Vec2(1.0, 2.0));
  EXPECT_EQ(g.width(), 3);
  EXPECT_EQ(g.height(), 2);
  EXPECT_EQ(g.at({0, 1}), Occupancy::kOccupied);  // top-left character
  EXPECT_EQ(g.at({0, 0}), Occupancy::kFree);
  EXPECT_EQ(g.at({-1, 0}), Occupancy::kOccupied);  // out of bounds
  EXPECT_TRUE(g.cell_center({1, 1}).isApprox(Vec2(1.75, 2.75)));
  EXPECT_EQ(g.world_to_cell({1.74, 2.76}), (GridCell{1, 1}));
  EXPECT_EQ(g.cell(g.index({2, 1})), (GridCell{2, 1}));
  EXPECT_EQ(g.count(Occupancy::kFree), 5U);
  EXPECT_THROW((void)ex::OccupancyGrid::from_ascii({"#.", "."}), std::invalid_argument);
  EXPECT_THROW((void)ex::OccupancyGrid::from_ascii({"#x"}), std::invalid_argument);
  EXPECT_THROW((void)ex::OccupancyGrid(0, 3), std::invalid_argument);
}

TEST(Sensor, RaysStopAtWallsAndRange) {
  const auto truth = ex::OccupancyGrid::from_ascii({
      ".........",
      "....#....",
      ".........",
  });
  ex::RangeSensorConfig cfg{5.0, 360};
  const auto seen = ex::simulate_scan(truth, truth.cell_center({0, 1}), cfg);
  const std::set<GridCell> set(seen.begin(), seen.end());
  EXPECT_TRUE(set.count({3, 1}));   // in front of the wall
  EXPECT_TRUE(set.count({4, 1}));   // the wall itself is observed
  EXPECT_FALSE(set.count({5, 1}));  // shadowed by the wall
  EXPECT_FALSE(set.count({8, 2}));  // beyond range (distance > 7.5 cells)
  EXPECT_TRUE(set.count({5, 2}));   // visible around the wall
  for (const GridCell& c : seen) {
    const Vec2 d = truth.cell_center(c) - truth.cell_center({0, 1});
    EXPECT_LE(d.norm(), 5.0 + 1.5);  // traversed cells start within range
  }
  ex::OccupancyGrid belief(truth.width(), truth.height());
  const std::size_t newly = ex::integrate_scan(belief, truth, seen);
  EXPECT_EQ(newly, seen.size());
  EXPECT_EQ(ex::integrate_scan(belief, truth, seen), 0U);  // known cells stay known
  EXPECT_EQ(belief.at({4, 1}), Occupancy::kOccupied);
}

TEST(Frontier, DetectionAndClustering) {
  ex::OccupancyGrid map(6, 5);  // all unknown
  for (int x = 0; x < 3; ++x) {
    for (int y = 0; y < 5; ++y) {
      map.set({x, y}, Occupancy::kFree);
    }
  }
  map.set({0, 4}, Occupancy::kOccupied);
  EXPECT_TRUE(ex::is_frontier(map, {2, 2}));
  EXPECT_FALSE(ex::is_frontier(map, {1, 2}));  // all neighbours known
  EXPECT_FALSE(ex::is_frontier(map, {0, 4}));  // occupied cells are never frontiers
  const auto frontiers = ex::detect_frontiers(map);
  EXPECT_EQ(frontiers.size(), 5U);  // the x = 2 column
  const auto clusters = ex::cluster_frontiers(map, frontiers, 1);
  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_EQ(clusters[0].representative, (GridCell{2, 2}));
  EXPECT_TRUE(clusters[0].centroid.isApprox(Vec2(2.0, 2.0)));
  EXPECT_TRUE(ex::cluster_frontiers(map, frontiers, 6).empty());

  // Two disjoint frontier segments form two clusters.
  map.set({2, 2}, Occupancy::kOccupied);
  const auto split = ex::cluster_frontiers(map, ex::detect_frontiers(map), 1);
  EXPECT_EQ(split.size(), 2U);
}

TEST(Frontier, TravelCostsAndInformationGain) {
  const auto map = two_rooms();
  const auto dist = ex::bfs_distances(map, {1, 1});
  EXPECT_EQ(dist[map.index({1, 1})], 0);
  EXPECT_EQ(dist[map.index({2, 1})], 1);
  EXPECT_EQ(dist[map.index({0, 0})], ex::kUnreachable);  // wall
  // Crossing the door at (9, 3).
  EXPECT_EQ(dist[map.index({10, 1})], 9 + 2 + 2);
  const auto path = ex::shortest_path(map, {1, 1}, {10, 1});
  ASSERT_EQ(path.size(), 14U);
  EXPECT_EQ(path.front(), (GridCell{1, 1}));
  EXPECT_EQ(path.back(), (GridCell{10, 1}));
  for (std::size_t k = 1; k < path.size(); ++k) {
    EXPECT_EQ(std::abs(path[k].x - path[k - 1].x) + std::abs(path[k].y - path[k - 1].y), 1);
    EXPECT_EQ(map.at(path[k]), Occupancy::kFree);
  }
  EXPECT_TRUE(ex::shortest_path(map, {1, 1}, {0, 0}).empty());
  const ex::OccupancyGrid unknown(5, 5);
  EXPECT_EQ(ex::information_gain(unknown, {2, 2}, 1.0), 5U);  // plus-shaped disc
}

TEST(Planner, StrategiesAssignDistinctTargetsWhenPossible) {
  ex::OccupancyGrid map(21, 5, 1.0, Vec2::Zero(), Occupancy::kUnknown);
  for (int x = 5; x <= 15; ++x) {
    for (int y = 0; y < 5; ++y) {
      map.set({x, y}, Occupancy::kFree);
    }
  }
  // Frontiers: the x = 5 and x = 15 columns.
  const auto clusters = ex::cluster_frontiers(map, ex::detect_frontiers(map), 1);
  ASSERT_EQ(clusters.size(), 2U);
  const std::vector<GridCell> robots{{9, 2}, {11, 2}};
  for (const auto s : {ex::Strategy::kHungarian, ex::Strategy::kUtilityDiscount}) {
    ex::PlannerConfig cfg;
    cfg.strategy = s;
    const auto a = ex::assign_targets(map, robots, clusters, cfg, 3.0);
    ASSERT_TRUE(a.targets[0] && a.targets[1]);
    EXPECT_NE(*a.clusters[0], *a.clusters[1]) << ex::to_string(s);
    EXPECT_EQ(a.targets[0]->x, 5);  // each robot takes the closer side
    EXPECT_EQ(a.targets[1]->x, 15);
  }
  // Uncoordinated: both robots near the left frontier choose it.
  ex::PlannerConfig nearest;
  nearest.strategy = ex::Strategy::kNearestFrontier;
  const std::vector<GridCell> left{{6, 1}, {7, 3}};
  const auto a = ex::assign_targets(map, left, clusters, nearest, 3.0);
  EXPECT_EQ(*a.clusters[0], *a.clusters[1]);
  EXPECT_EQ(ex::strategy_from_string("utility_discount"), ex::Strategy::kUtilityDiscount);
  EXPECT_FALSE(ex::strategy_from_string("random").has_value());
}

TEST(Planner, MoreRobotsThanFrontiersShareTargets) {
  ex::OccupancyGrid map(10, 3, 1.0, Vec2::Zero(), Occupancy::kFree);
  map.set({9, 1}, Occupancy::kUnknown);
  const auto clusters = ex::cluster_frontiers(map, ex::detect_frontiers(map), 1);
  ASSERT_EQ(clusters.size(), 1U);
  const std::vector<GridCell> robots{{0, 0}, {1, 1}, {2, 2}};
  const auto a = ex::assign_targets(map, robots, clusters, {}, 2.0);
  for (const auto& t : a.targets) {
    EXPECT_TRUE(t.has_value());
  }
}

TEST(Exploration, TwoRoomsAreFullyExploredAndTerminate) {
  for (const auto s :
       {ex::Strategy::kNearestFrontier, ex::Strategy::kHungarian, ex::Strategy::kUtilityDiscount}) {
    ex::ExplorationConfig cfg;
    cfg.planner.strategy = s;
    cfg.sensor = {3.0, 120};
    cfg.record_frames = true;
    ex::ExplorationSimulation sim(two_rooms(), {{1, 1}, {2, 1}}, cfg);
    const auto r = sim.run();
    EXPECT_TRUE(r.complete) << ex::to_string(s);
    EXPECT_NEAR(r.coverage.back(), 1.0, 1e-12);
    EXPECT_LT(r.steps, 200U);
    ASSERT_TRUE(r.steps_to_95.has_value());
    EXPECT_LE(*r.steps_to_90, *r.steps_to_95);
    // Coverage is monotone and the known map agrees with the truth.
    EXPECT_TRUE(std::is_sorted(r.coverage.begin(), r.coverage.end()));
    for (std::size_t i = 0; i < sim.truth().size(); ++i) {
      const auto known = sim.belief().data()[i];
      if (known != Occupancy::kUnknown) {
        EXPECT_EQ(known, sim.truth().data()[i]);
      }
    }
    EXPECT_GE(r.observed_cells, r.redundant_cells);
    EXPECT_EQ(r.frames.size(), r.steps + 1);
    EXPECT_GT(r.messages, 0U);
    // Paths are 4-connected walks on free cells.
    for (const auto& path : r.paths) {
      for (std::size_t k = 1; k < path.size(); ++k) {
        EXPECT_EQ(std::abs(path[k].x - path[k - 1].x) + std::abs(path[k].y - path[k - 1].y), 1);
      }
    }
  }
}

TEST(Exploration, EnclosedStartTerminatesImmediately) {
  const auto truth = ex::OccupancyGrid::from_ascii({"#####", "#...#", "#####"});
  ex::ExplorationConfig cfg;
  cfg.sensor = {5.0, 90};
  ex::ExplorationSimulation sim(truth, {{2, 1}}, cfg);
  EXPECT_TRUE(sim.finished());
  const auto r = sim.run();
  EXPECT_TRUE(r.complete);
  EXPECT_EQ(r.steps, 0U);
  EXPECT_THROW(ex::ExplorationSimulation(truth, {{0, 0}}, cfg), std::invalid_argument);
}
