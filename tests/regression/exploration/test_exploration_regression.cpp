// Regression: fixed exploration runs on the rooms scenario must keep their
// step counts and observation statistics.

#include "multirobotcore/exploration/exploration.hpp"

#include <gtest/gtest.h>

#include <filesystem>

namespace ex = multirobotcore::exploration;

TEST(ExplorationRegression, RoomsScenarioIsReproducible) {
  const auto map = ex::OccupancyGrid::load_ascii(
      std::filesystem::path(MRC_SOURCE_DIR) / "scenarios/exploration/rooms.txt", 0.5);
  struct Expected {
    ex::Strategy strategy;
    std::size_t steps;
    std::size_t observed;
    std::size_t redundant;
    std::size_t messages;
  };
  const Expected expected[] = {
      {ex::Strategy::kNearestFrontier, 97, 1563, 768, 349},
      {ex::Strategy::kHungarian, 127, 1564, 1194, 442},
      {ex::Strategy::kUtilityDiscount, 93, 1564, 738, 340},
  };
  for (const auto& e : expected) {
    ex::ExplorationConfig cfg;
    cfg.planner.strategy = e.strategy;
    cfg.sensor = {3.0, 180};
    ex::ExplorationSimulation sim(map, {{4, 4}, {6, 4}, {4, 6}}, cfg);
    const auto r = sim.run();
    EXPECT_TRUE(r.complete) << ex::to_string(e.strategy);
    EXPECT_EQ(r.steps, e.steps) << ex::to_string(e.strategy);
    EXPECT_EQ(r.observed_cells, e.observed) << ex::to_string(e.strategy);
    EXPECT_EQ(r.redundant_cells, e.redundant) << ex::to_string(e.strategy);
    EXPECT_EQ(r.messages, e.messages) << ex::to_string(e.strategy);
  }
}
