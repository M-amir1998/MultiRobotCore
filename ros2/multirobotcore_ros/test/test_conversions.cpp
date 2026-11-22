#include "multirobotcore_ros/conversions.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace mr = multirobotcore_ros;
using multirobotcore::Vec2;

TEST(Conversions, YawQuaternionRoundTrip) {
  for (double yaw = -3.1; yaw <= 3.1; yaw += 0.37) {
    EXPECT_NEAR(mr::quaternion_to_yaw(mr::yaw_to_quaternion(yaw)), yaw, 1e-12);
  }
}

TEST(Conversions, PoseTwistAndPoint) {
  const multirobotcore::Pose2 pose{{1.5, -2.0}, std::numbers::pi / 3.0};
  const auto back = mr::from_pose(mr::to_pose(pose));
  EXPECT_TRUE(back.position.isApprox(pose.position));
  EXPECT_NEAR(back.heading, pose.heading, 1e-12);
  EXPECT_TRUE(mr::from_twist(mr::to_twist(Vec2(0.3, -0.4))).isApprox(Vec2(0.3, -0.4)));
  EXPECT_TRUE(mr::from_point(mr::to_point(Vec2(7.0, 8.0))).isApprox(Vec2(7.0, 8.0)));
}

TEST(Conversions, TimedPathCarriesArrivalStamps) {
  const std::vector<std::pair<double, Vec2>> plan{
      {0.0, {0.5, 0.5}}, {1.0, {1.5, 0.5}}, {2.5, {1.5, 1.5}}};
  const auto path = mr::to_path(plan, "map", 100.0);
  ASSERT_EQ(path.poses.size(), 3U);
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_EQ(path.poses[2].header.stamp.sec, 102);
  EXPECT_NEAR(path.poses[2].header.stamp.nanosec * 1e-9, 0.5, 1e-6);
  EXPECT_DOUBLE_EQ(path.poses[1].pose.position.x, 1.5);
}

TEST(Conversions, RobotTopics) {
  EXPECT_EQ(mr::robot_topic(3, "cmd_vel"), "robot_3/cmd_vel");
}
