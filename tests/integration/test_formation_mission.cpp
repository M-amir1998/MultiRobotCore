// Formation -> passage -> reconfiguration, and formation -> leader loss ->
// expected degradation (no re-election) or recovery (re-election).

#include "multirobotcore/io/json.hpp"
#include "multirobotcore/scenarios/formation_mission.hpp"

#include <gtest/gtest.h>

#include <filesystem>

namespace sc = multirobotcore::scenarios;
namespace io = multirobotcore::io;

namespace {

sc::FormationMissionConfig mission_config() {
  const auto cfg = io::read_json_file(std::filesystem::path(MRC_SOURCE_DIR) /
                                      "configs/formation/formation.json");
  return sc::formation_mission_config_from_json(cfg.at("mission"));
}

}  // namespace

TEST(FormationMission, PassesNarrowPassageWithoutContacts) {
  const auto result = sc::run_formation_mission(mission_config());
  EXPECT_GT(result.path_progress, 0.95);
  EXPECT_FALSE(result.stalled);
  EXPECT_EQ(result.metrics.robot_collisions, 0U);
  EXPECT_EQ(result.metrics.obstacle_collisions, 0U);
  EXPECT_LT(result.mean_rmse, 0.2);
  EXPECT_LT(result.final_rmse, 0.05);
  bool contracted = false;
  bool restored = false;
  for (const auto& e : result.events) {
    contracted = contracted || e.label.find("contracting") != std::string::npos;
    restored = restored || e.label.find("restored") != std::string::npos;
  }
  EXPECT_TRUE(contracted);
  EXPECT_TRUE(restored);
}

TEST(FormationMission, LeaderLossWithoutReelectionStalls) {
  auto cfg = mission_config();
  cfg.leader_failure_time = 14.0;
  cfg.reelect_leader = false;
  const auto result = sc::run_formation_mission(cfg);
  EXPECT_LT(result.path_progress, 0.5);
  EXPECT_TRUE(result.stalled);
  EXPECT_EQ(result.final_leader, 0U);
}

TEST(FormationMission, LeaderReelectionRecoversTheMission) {
  auto cfg = mission_config();
  cfg.leader_failure_time = 14.0;
  cfg.reelect_leader = true;
  const auto result = sc::run_formation_mission(cfg);
  EXPECT_EQ(result.final_leader, 1U);  // lowest operational identifier
  EXPECT_GT(result.path_progress, 0.95);
  EXPECT_EQ(result.metrics.robot_collisions, 0U);
}

TEST(FormationMission, MessageLossDegradesGracefully) {
  auto cfg = mission_config();
  cfg.drop_probability = 0.2;
  const auto result = sc::run_formation_mission(cfg);
  EXPECT_GT(result.path_progress, 0.95);
  EXPECT_GT(result.network.dropped_loss, 0U);
  EXPECT_LT(result.mean_rmse, 0.5);
}
