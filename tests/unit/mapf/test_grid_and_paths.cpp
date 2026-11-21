#include "multirobotcore/mapf/conflicts.hpp"
#include "multirobotcore/mapf/generators.hpp"
#include "multirobotcore/mapf/grid_map.hpp"
#include "multirobotcore/mapf/instance.hpp"

#include <gtest/gtest.h>

#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace mapf = multirobotcore::mapf;
using mapf::Cell;
using mapf::Path;

TEST(GridMap, AsciiRoundTripWithBottomOrigin) {
  const std::vector<std::string> rows{"..@", "...", "#.."};
  const auto map = mapf::GridMap::from_ascii(rows);
  EXPECT_EQ(map.width(), 3);
  EXPECT_EQ(map.height(), 3);
  EXPECT_FALSE(map.is_free({2, 2}));  // top-right '@'
  EXPECT_FALSE(map.is_free({0, 0}));  // bottom-left '#'
  EXPECT_TRUE(map.is_free({1, 1}));
  EXPECT_EQ(map.num_free_cells(), 7U);
  EXPECT_EQ(map.to_ascii(), (std::vector<std::string>{"..@", "...", "@.."}));
  EXPECT_THROW((void)mapf::GridMap::from_ascii({"..", "..."}), std::invalid_argument);
  EXPECT_THROW((void)mapf::GridMap::from_ascii({".x"}), std::invalid_argument);
}

TEST(GridMap, ParsesMovingAiFormat) {
  std::istringstream in("type octile\nheight 2\nwidth 4\nmap\n..T.\r\n@...\n");
  const auto map = mapf::GridMap::parse_movingai(in);
  EXPECT_EQ(map.width(), 4);
  EXPECT_EQ(map.height(), 2);
  EXPECT_FALSE(map.is_free({2, 1}));
  EXPECT_FALSE(map.is_free({0, 0}));
  std::istringstream bad("type octile\nheight 3\nwidth 4\nmap\n....\n");
  EXPECT_THROW((void)mapf::GridMap::parse_movingai(bad), std::invalid_argument);
}

TEST(GridMap, NeighboursAndBfsDistances) {
  const auto map = mapf::GridMap::from_ascii({".....", ".@@@.", "....."});
  const auto n = map.neighbors({0, 1});
  EXPECT_EQ(n.size(), 2U);  // up and down only
  const auto d = map.distances_from({0, 0});
  EXPECT_EQ(d[static_cast<std::size_t>(map.index({4, 0}))], 4);
  EXPECT_EQ(d[static_cast<std::size_t>(map.index({2, 2}))], 4);
  EXPECT_EQ(d[static_cast<std::size_t>(map.index({1, 1}))], -1);
}

TEST(WorldFrame, CellCentresAndInverse) {
  const mapf::WorldFrame frame{{1.0, -2.0}, 0.5};
  const auto p = frame.to_world({3, 4});
  EXPECT_DOUBLE_EQ(p.x(), 1.0 + 0.5 * 3.5);
  EXPECT_DOUBLE_EQ(p.y(), -2.0 + 0.5 * 4.5);
  EXPECT_EQ(frame.to_cell(p), (Cell{3, 4}));
  const auto waypoints = mapf::to_waypoints({{0, 0}, {1, 0}, {1, 1}}, frame, 0.4);
  ASSERT_EQ(waypoints.size(), 3U);
  EXPECT_DOUBLE_EQ(waypoints[2].time, 0.8);
  EXPECT_TRUE(waypoints[2].position.isApprox(frame.to_world({1, 1})));
}

TEST(Paths, CostsIgnoreTrailingWaits) {
  const Path p{{0, 0}, {1, 0}, {1, 0}, {2, 0}, {2, 0}, {2, 0}};
  EXPECT_EQ(mapf::path_cost(p), 3U);
  EXPECT_EQ(mapf::trim_trailing_waits(p).size(), 4U);
  const Path leaves_and_returns{{0, 0}, {1, 0}, {0, 0}};
  EXPECT_EQ(mapf::path_cost(leaves_and_returns), 2U);
  EXPECT_EQ(mapf::sum_of_costs({p, leaves_and_returns}), 5U);
  EXPECT_EQ(mapf::makespan({p, leaves_and_returns}), 3U);
  EXPECT_EQ(mapf::position_at(p, 100), (Cell{2, 0}));
}

TEST(Paths, CheckPathsDetectsInvalidMoves) {
  const mapf::Instance inst{mapf::GridMap::from_ascii({"...."}), {{{0, 0}, {3, 0}}}};
  EXPECT_TRUE(mapf::check_paths(inst, {{{0, 0}, {1, 0}, {2, 0}, {3, 0}}}).valid);
  EXPECT_FALSE(mapf::check_paths(inst, {{{0, 0}, {2, 0}, {3, 0}}}).valid);  // jump
  EXPECT_FALSE(mapf::check_paths(inst, {{{1, 0}, {2, 0}, {3, 0}}}).valid);  // wrong start
  EXPECT_FALSE(mapf::check_paths(inst, {{{0, 0}, {1, 0}, {2, 0}}}).valid);  // wrong goal
  EXPECT_THROW(mapf::validate_instance({inst.map, {{{0, 0}, {3, 0}}, {{0, 0}, {2, 0}}}}),
               std::invalid_argument);
}

TEST(Conflicts, VertexConflict) {
  const std::vector<Path> paths{{{0, 0}, {1, 0}, {2, 0}}, {{2, 1}, {1, 1}, {1, 0}, {1, 1}}};
  // t = 1: agent 0 at (1,0), agent 1 at (1,1); t = 2: agent 0 at (2,0), agent 1 at (1,0).
  EXPECT_FALSE(mapf::first_conflict(paths).has_value());
  const std::vector<Path> colliding{{{0, 0}, {1, 0}, {2, 0}}, {{1, 1}, {1, 0}}};
  const auto c = mapf::first_conflict(colliding);
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->type, mapf::ConflictType::kVertex);
  EXPECT_EQ(c->time, 1);
  EXPECT_EQ(c->cell1, (Cell{1, 0}));
  EXPECT_EQ(c->agent1, 0U);
  EXPECT_EQ(c->agent2, 1U);
}

TEST(Conflicts, EdgeSwapConflict) {
  const std::vector<Path> paths{{{0, 0}, {1, 0}}, {{1, 0}, {0, 0}}};
  const auto c = mapf::first_conflict(paths);
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->type, mapf::ConflictType::kEdge);
  EXPECT_EQ(c->time, 0);
  EXPECT_EQ(c->cell1, (Cell{0, 0}));  // agent 0 moves (0,0) -> (1,0)
  EXPECT_EQ(c->cell2, (Cell{1, 0}));
  // Following (moving into a cell being vacated) is not a conflict.
  EXPECT_TRUE(mapf::is_collision_free({{{0, 0}, {1, 0}, {2, 0}}, {{1, 0}, {2, 0}, {3, 0}}}));
}

TEST(Conflicts, ParkedAgentsStillConflict) {
  // Agent 0 arrives at (1,0) at t = 1 and parks; agent 1 passes through at t = 3.
  const std::vector<Path> paths{{{0, 0}, {1, 0}}, {{1, 2}, {1, 1}, {1, 1}, {1, 0}, {1, 1}}};
  const auto all = mapf::find_conflicts(paths);
  ASSERT_EQ(all.size(), 1U);
  EXPECT_EQ(all[0].time, 3);
  EXPECT_EQ(mapf::count_conflicts(paths), 1U);
}

TEST(Generators, DeterministicAndConnected) {
  std::mt19937_64 a(5);
  std::mt19937_64 b(5);
  EXPECT_EQ(mapf::make_maze(15, 11, 0.1, a), mapf::make_maze(15, 11, 0.1, b));
  std::mt19937_64 rng(1);
  const auto maze = mapf::make_maze(21, 21, 0.1, rng);
  const auto component = mapf::largest_component(maze);
  EXPECT_EQ(component.size(), maze.num_free_cells());  // a maze is connected

  const auto wh = mapf::make_warehouse({});
  EXPECT_FALSE(wh.pickups.empty());
  EXPECT_EQ(mapf::largest_component(wh.map).size(), wh.map.num_free_cells());
  for (const auto& p : wh.pickups) {
    EXPECT_TRUE(wh.map.is_free(p));
  }
  std::mt19937_64 r(3);
  const auto inst = mapf::random_instance(wh.map, 12, wh.stations, wh.pickups, r);
  EXPECT_EQ(inst.agents.size(), 12U);
  EXPECT_NO_THROW(mapf::validate_instance(inst));
  EXPECT_THROW((void)mapf::random_instance(mapf::make_open_map(2, 2), 5, r), std::invalid_argument);
}
