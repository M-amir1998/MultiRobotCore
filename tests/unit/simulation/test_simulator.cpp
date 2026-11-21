#include "multirobotcore/communication/network.hpp"
#include "multirobotcore/simulation/events.hpp"
#include "multirobotcore/simulation/simulator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace sim = multirobotcore::simulation;
namespace models = multirobotcore::models;
using multirobotcore::Pose2;
using multirobotcore::Vec2;

namespace {

sim::Simulator make_sim(std::vector<Pose2> poses,
                        sim::Dynamics dynamics = sim::Dynamics::kSingleIntegrator,
                        sim::WorldConfig cfg = {}) {
  sim::RobotSpec spec;
  spec.dynamics = dynamics;
  spec.max_speed = 1.0;
  spec.radius = 0.2;
  std::vector<sim::RobotSpec> specs(poses.size(), spec);
  return sim::Simulator(cfg, specs, std::move(poses));
}

/// Proportional go-to-goal velocity command.
Vec2 go_to(const Vec2& p, const Vec2& goal) {
  return 1.5 * (goal - p);
}

}  // namespace

TEST(Simulator, SingleIntegratorExactAndSpeedLimited) {
  auto s = make_sim({Pose2{{0.0, 0.0}, 0.0}});
  const std::vector<Vec2> cmd{{0.5, 0.0}};
  for (int k = 0; k < 20; ++k) {
    s.step(cmd);
  }
  EXPECT_NEAR(s.time(), 1.0, 1e-12);
  EXPECT_TRUE(s.robot(0).position.isApprox(Vec2(0.5, 0.0)));
  const std::vector<Vec2> fast{{10.0, 0.0}};
  s.step(fast);
  EXPECT_NEAR(s.robot(0).velocity.norm(), 1.0, 1e-12);
  EXPECT_NEAR(s.metrics().distance_travelled[0], 0.55, 1e-12);
}

TEST(Simulator, DoubleIntegratorTracksVelocity) {
  auto s = make_sim({Pose2{{0.0, 0.0}, 0.0}}, sim::Dynamics::kDoubleIntegrator);
  const std::vector<Vec2> cmd{{0.0, 0.8}};
  for (int k = 0; k < 100; ++k) {
    s.step(cmd);
  }
  EXPECT_NEAR(s.robot(0).velocity.y(), 0.8, 1e-3);
}

TEST(Simulator, NonholonomicRobotsReachGoal) {
  for (const auto dyn : {sim::Dynamics::kUnicycle, sim::Dynamics::kDifferentialDrive}) {
    auto s = make_sim({Pose2{{0.0, 0.0}, 3.0}}, dyn);
    const Vec2 goal(3.0, 2.0);
    for (int k = 0; k < 400; ++k) {
      const std::vector<Vec2> cmd{go_to(s.robot(0).position, goal)};
      s.step(cmd);
    }
    EXPECT_LT((s.robot(0).position - goal).norm(), 0.2);
  }
}

TEST(Simulator, HeadOnRobotsCollideExactlyOnce) {
  auto s = make_sim({Pose2{{0.0, 0.0}, 0.0}, Pose2{{4.0, 0.0}, 0.0}});
  for (int k = 0; k < 120; ++k) {
    const std::vector<Vec2> cmd{{1.0, 0.0}, {-1.0, 0.0}};
    s.step(cmd);
  }
  EXPECT_EQ(s.metrics().robot_collisions, 1U);  // one contact interval
  EXPECT_LT(s.metrics().min_pair_clearance, 0.0);
  ASSERT_EQ(s.collision_events().size(), 1U);
  EXPECT_TRUE(s.collision_events()[0].other.has_value());
}

TEST(Simulator, BoundsAndObstaclesAreCountedAsContacts) {
  sim::WorldConfig cfg;
  cfg.bounds = multirobotcore::math::AABB{{-1.0, -1.0}, {1.0, 1.0}};
  cfg.obstacles.circles.push_back({{0.0, -3.0}, 0.5});
  auto s = make_sim({Pose2{{0.0, 0.0}, 0.0}}, sim::Dynamics::kSingleIntegrator, cfg);
  for (int k = 0; k < 60; ++k) {
    const std::vector<Vec2> cmd{{1.0, 0.0}};
    s.step(cmd);
  }
  EXPECT_NEAR(s.robot(0).position.x(), 0.8, 1e-12);  // 1.0 - radius
  EXPECT_EQ(s.metrics().obstacle_collisions, 1U);
  EXPECT_NEAR(s.metrics().min_obstacle_clearance, 3.0 - 0.5 - 0.2 - 0.0, 1.0);
}

TEST(Simulator, StoppedRobotsDoNotMoveAndStallIsDetected) {
  sim::WorldConfig cfg;
  cfg.stall.window = 1.0;
  auto s = make_sim({Pose2{{0.0, 0.0}, 0.0}, Pose2{{3.0, 0.0}, 0.0}},
                    sim::Dynamics::kSingleIntegrator, cfg);
  s.set_status(1, models::RobotStatus::kStopped);
  for (int k = 0; k < 30; ++k) {
    const std::vector<Vec2> cmd{{0.5, 0.0}, {1.0, 0.0}};
    s.step(cmd);
  }
  EXPECT_TRUE(s.robot(1).position.isApprox(Vec2(3.0, 0.0)));
  EXPECT_FALSE(s.stalled());  // robot 0 is still progressing
  for (int k = 0; k < 30; ++k) {
    const std::vector<Vec2> cmd{{0.0, 0.0}, {0.0, 0.0}};
    s.step(cmd);
  }
  EXPECT_TRUE(s.stalled());
  EXPECT_EQ(s.stalled_robots(), (std::vector<std::size_t>{0}));
}

TEST(Simulator, RunsAreDeterministicAndFramesRecorded) {
  sim::WorldConfig cfg;
  cfg.record_every = 5;
  auto run = [&] {
    auto s =
        make_sim({Pose2{{0.0, 0.0}, 0.0}, Pose2{{1.0, 1.0}, 1.0}}, sim::Dynamics::kUnicycle, cfg);
    for (int k = 0; k < 50; ++k) {
      const std::vector<Vec2> cmd{{0.3, 0.2}, {-0.4, 0.1}};
      s.step(cmd);
    }
    return s.frames();
  };
  const auto a = run();
  const auto b = run();
  ASSERT_EQ(a.size(), 11U);  // initial frame + 50 / 5
  for (std::size_t f = 0; f < a.size(); ++f) {
    for (std::size_t i = 0; i < 2; ++i) {
      EXPECT_EQ(a[f].poses[i].position, b[f].poses[i].position);
    }
  }
}

TEST(Simulator, RejectsInvalidCommands) {
  auto s = make_sim({Pose2{{0.0, 0.0}, 0.0}});
  const std::vector<Vec2> wrong_size{{0.0, 0.0}, {0.0, 0.0}};
  EXPECT_THROW(s.step(wrong_size), std::invalid_argument);
  const std::vector<Vec2> nan{{std::nan(""), 0.0}};
  EXPECT_THROW(s.step(nan), std::invalid_argument);
}

TEST(Events, PopDueOrdersByTimeThenInsertion) {
  sim::EventSchedule schedule;
  schedule.add({2.0, sim::EventType::kRobotStop, 1});
  schedule.add({1.0, sim::EventType::kLinkFailure, 0, 1});
  schedule.add({1.0, sim::EventType::kTaskArrival});
  EXPECT_TRUE(schedule.pop_due(0.5).empty());
  const auto due = schedule.pop_due(1.5);
  ASSERT_EQ(due.size(), 2U);
  EXPECT_EQ(due[0].type, sim::EventType::kLinkFailure);
  EXPECT_EQ(due[1].type, sim::EventType::kTaskArrival);
  EXPECT_EQ(schedule.pending(), 1U);
  EXPECT_EQ(sim::to_string(sim::EventType::kLeaderLoss), "leader_loss");
}

TEST(Events, ApplyAffectsSimulatorAndNetwork) {
  auto s = make_sim({Pose2{{0.0, 0.0}, 0.0}, Pose2{{1.0, 0.0}, 0.0}, Pose2{{2.0, 0.0}, 0.0}});
  multirobotcore::communication::NetworkSimulator net(3, {}, 1);
  EXPECT_TRUE(sim::apply_event({0.0, sim::EventType::kLeaderLoss, 0}, s, &net));
  EXPECT_EQ(s.robot(0).status, models::RobotStatus::kFailed);
  EXPECT_FALSE(net.node_available(0, 1.0));
  sim::ScheduledEvent link{0.0, sim::EventType::kLinkFailure, 1, 2};
  link.duration = 2.0;
  EXPECT_TRUE(sim::apply_event(link, s, &net));
  EXPECT_FALSE(net.link_available(1, 2, 1.0));
  EXPECT_TRUE(net.link_available(1, 2, 2.5));
  sim::ScheduledEvent obstacle{0.0, sim::EventType::kObstacleAppears};
  obstacle.position = Vec2(5.0, 5.0);
  obstacle.radius = 1.0;
  EXPECT_TRUE(sim::apply_event(obstacle, s, &net));
  EXPECT_EQ(s.obstacles().circles.size(), 1U);
  EXPECT_FALSE(sim::apply_event({0.0, sim::EventType::kTaskArrival}, s, &net));
}
