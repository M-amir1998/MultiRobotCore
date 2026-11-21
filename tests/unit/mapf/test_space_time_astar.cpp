#include "multirobotcore/mapf/constraint_table.hpp"
#include "multirobotcore/mapf/grid_map.hpp"
#include "multirobotcore/mapf/space_time_astar.hpp"

#include <gtest/gtest.h>

#include <optional>

namespace mapf = multirobotcore::mapf;
using mapf::Cell;
using mapf::Path;

namespace {

std::optional<Path> plan(const mapf::GridMap& map, Cell s, Cell g, const mapf::ConstraintTable& c,
                         mapf::LowLevelStats* stats = nullptr) {
  mapf::LowLevelStats local;
  return mapf::space_time_astar(map, s, g, c, map.distances_from(g), stats ? *stats : local);
}

}  // namespace

TEST(SpaceTimeAStar, ShortestPathWithoutConstraints) {
  const auto map = mapf::GridMap::from_ascii({".....", ".@@@.", "....."});
  const mapf::ConstraintTable none(map);
  mapf::LowLevelStats stats;
  const auto p = plan(map, {0, 0}, {4, 2}, none, &stats);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(mapf::path_cost(*p), 6U);
  EXPECT_EQ(p->front(), (Cell{0, 0}));
  EXPECT_EQ(p->back(), (Cell{4, 2}));
  // Perfect heuristic: only nodes on an optimal path are expanded.
  EXPECT_EQ(stats.expansions, 7U);
}

TEST(SpaceTimeAStar, VertexConstraintForcesWait) {
  // Single-cell-wide corridor: the only way around a blocked (cell, time) is to wait.
  const auto map = mapf::GridMap::from_ascii({"...."});
  mapf::ConstraintTable c(map);
  c.add_vertex({1, 0}, 1);
  const auto p = plan(map, {0, 0}, {3, 0}, c);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(mapf::path_cost(*p), 4U);
  EXPECT_EQ((*p)[1], (Cell{0, 0}));  // waited at the start
}

TEST(SpaceTimeAStar, EdgeConstraintIsRespected) {
  const auto map = mapf::GridMap::from_ascii({"...", "..."});
  mapf::ConstraintTable c(map);
  c.add_edge({0, 0}, {1, 0}, 0);
  const auto p = plan(map, {0, 0}, {2, 0}, c);
  ASSERT_TRUE(p.has_value());
  EXPECT_FALSE((*p)[0] == (Cell{0, 0}) && (*p)[1] == (Cell{1, 0}));
  EXPECT_EQ(mapf::path_cost(*p), 3U);  // waiting once beats the detour (cost 4)
}

TEST(SpaceTimeAStar, GoalConstraintDelaysArrival) {
  // The goal is occupied at t = 6 by someone else: the agent may not finish
  // before t = 7 even though it can arrive at t = 2.
  const auto map = mapf::GridMap::from_ascii({"...."});
  mapf::ConstraintTable c(map);
  c.add_vertex({2, 0}, 6);
  const auto p = plan(map, {0, 0}, {2, 0}, c);
  ASSERT_TRUE(p.has_value());
  EXPECT_GE(mapf::path_cost(*p), 7U);
  EXPECT_NE((*p)[6], (Cell{2, 0}));
  EXPECT_EQ(p->back(), (Cell{2, 0}));
}

TEST(SpaceTimeAStar, PermanentlyBlockedGoalIsInfeasible) {
  const auto map = mapf::GridMap::from_ascii({"...."});
  mapf::ConstraintTable c(map);
  c.add_permanent({3, 0}, 10);
  EXPECT_FALSE(plan(map, {0, 0}, {3, 0}, c).has_value());
}

TEST(SpaceTimeAStar, ReservationsAvoidHigherPriorityPath) {
  const auto map = mapf::GridMap::from_ascii({"..", ".."});
  mapf::ConstraintTable reservations(map);
  // Higher-priority agent goes (0,0) -> (1,0) -> (1,1) and parks.
  reservations.reserve_path({{0, 0}, {1, 0}, {1, 1}});
  // Lower-priority agent (0,1) -> (1,0) must follow behind it.
  const auto p = plan(map, {0, 1}, {1, 0}, reservations);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(mapf::path_cost(*p), 2U);
  for (std::size_t t = 0; t < p->size(); ++t) {
    EXPECT_FALSE(reservations.is_vertex_blocked((*p)[t], static_cast<int>(t)));
  }
  // Parking: (1,1) is blocked forever from t = 2.
  EXPECT_TRUE(reservations.is_vertex_blocked({1, 1}, 50));
  EXPECT_TRUE(reservations.is_edge_blocked({1, 0}, {0, 0}, 0));  // no swap
}

TEST(SpaceTimeAStar, UnreachableGoalReturnsNullopt) {
  const auto map = mapf::GridMap::from_ascii({".@."});
  const mapf::ConstraintTable none(map);
  EXPECT_FALSE(plan(map, {0, 0}, {2, 0}, none).has_value());
}

TEST(SpaceTimeAStar, StartBlockedAtTimeZero) {
  const auto map = mapf::GridMap::from_ascii({"..."});
  mapf::ConstraintTable c(map);
  c.add_vertex({0, 0}, 0);
  EXPECT_FALSE(plan(map, {0, 0}, {2, 0}, c).has_value());
}
