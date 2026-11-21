#pragma once

/// @file conversions.hpp
/// Conversions between MultiRobotCore value types and ROS 2 messages, plus
/// the parameter-to-configuration mapping shared by the nodes.
///
/// Frame convention: all robots live in one fixed planar world frame
/// ("map"); velocities on cmd_vel / cmd_vel_ref topics are planar world-frame
/// velocities of holonomic robots (linear.x, linear.y), as produced by the
/// MultiRobotCore controllers.

#include <multirobotcore/math/types.hpp>
#include <multirobotcore/scenarios/warehouse_mission.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace multirobotcore_ros {

using multirobotcore::Vec2;

inline geometry_msgs::msg::Point to_point(const Vec2& p) {
  geometry_msgs::msg::Point msg;
  msg.x = p.x();
  msg.y = p.y();
  msg.z = 0.0;
  return msg;
}

inline Vec2 from_point(const geometry_msgs::msg::Point& msg) { return {msg.x, msg.y}; }

/// Planar yaw as a quaternion about +z.
inline geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

inline double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline geometry_msgs::msg::Pose to_pose(const multirobotcore::Pose2& pose) {
  geometry_msgs::msg::Pose msg;
  msg.position = to_point(pose.position);
  msg.orientation = yaw_to_quaternion(pose.heading);
  return msg;
}

inline multirobotcore::Pose2 from_pose(const geometry_msgs::msg::Pose& msg) {
  return {from_point(msg.position), quaternion_to_yaw(msg.orientation)};
}

inline geometry_msgs::msg::Twist to_twist(const Vec2& v) {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = v.x();
  msg.linear.y = v.y();
  return msg;
}

inline Vec2 from_twist(const geometry_msgs::msg::Twist& msg) { return {msg.linear.x, msg.linear.y}; }

/// Timed plan (time [s] since the coordinator started, position) as a
/// nav_msgs/Path whose pose stamps carry the planned arrival times.
inline nav_msgs::msg::Path to_path(const std::vector<std::pair<double, Vec2>>& plan,
                                   const std::string& frame_id, double time_origin) {
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  for (const auto& [t, p] : plan) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    const double stamp = time_origin + t;
    pose.header.stamp.sec = static_cast<int32_t>(std::floor(stamp));
    pose.header.stamp.nanosec = static_cast<uint32_t>((stamp - std::floor(stamp)) * 1e9);
    pose.pose.position = to_point(p);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

/// Robot-scoped topic name, e.g. robot_topic(3, "cmd_vel") == "robot_3/cmd_vel".
inline std::string robot_topic(std::size_t robot, const std::string& name) {
  return "robot_" + std::to_string(robot) + "/" + name;
}

}  // namespace multirobotcore_ros
