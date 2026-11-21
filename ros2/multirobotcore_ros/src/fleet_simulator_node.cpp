// fleet_simulator: the MultiRobotCore deterministic simulator as a ROS 2 robot
// backend (a lightweight stand-in for Gazebo when testing coordination).
//
// Subscribes  robot_<i>/cmd_vel       geometry_msgs/Twist (planar world-frame velocity)
// Publishes   robot_<i>/odom          nav_msgs/Odometry
//             /fleet/poses            geometry_msgs/PoseArray (all robots)
//             /fleet/sim_status       std_msgs/String (JSON: time, contacts, clearance)
//             /clock                  rosgraph_msgs/Clock (simulation time)
//
// The simulator advances with a wall timer at real_time_factor / dt Hz, so
// the rest of the graph (use_sim_time:=true) runs faster than real time.

#include "multirobotcore_ros/conversions.hpp"
#include "multirobotcore_ros/parameters.hpp"

#include <multirobotcore/scenarios/warehouse_mission.hpp>
#include <multirobotcore/simulation/simulator.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <vector>

namespace multirobotcore_ros {
namespace {

namespace mrc = multirobotcore;

builtin_interfaces::msg::Time to_stamp(double seconds) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(std::floor(seconds));
  stamp.nanosec = static_cast<uint32_t>((seconds - std::floor(seconds)) * 1e9);
  return stamp;
}

class FleetSimulatorNode : public rclcpp::Node {
 public:
  FleetSimulatorNode() : rclcpp::Node("fleet_simulator") {
    config_ = declare_warehouse_parameters(*this);
    const double real_time_factor = declare_parameter<double>("real_time_factor", 1.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);
    publish_clock_ = declare_parameter<bool>("publish_clock", true);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    // The coordinator type is used only for the shared warehouse geometry.
    const mrc::scenarios::WarehouseCoordinator geometry(config_);
    mrc::simulation::WorldConfig world;
    world.dt = config_.dt;
    world.record = false;
    const auto& map = geometry.warehouse().map;
    world.bounds = mrc::math::AABB{mrc::Vec2::Zero(),
                                   mrc::Vec2(map.width() * config_.cell_size, map.height() * config_.cell_size)};
    world.obstacles = geometry.shelf_obstacles();
    mrc::simulation::RobotSpec spec;
    spec.radius = config_.robot_radius;
    spec.max_speed = config_.max_speed;
    spec.max_acceleration = config_.max_acceleration;
    const std::size_t n = config_.num_robots;
    sim_.emplace(world, std::vector<mrc::simulation::RobotSpec>(n, spec), geometry.home_poses());

    commands_.assign(n, mrc::Vec2::Zero());
    command_time_.assign(n, -1e9);
    for (std::size_t i = 0; i < n; ++i) {
      command_subs_.push_back(create_subscription<geometry_msgs::msg::Twist>(
          robot_topic(i, "cmd_vel"), 10, [this, i](const geometry_msgs::msg::Twist& msg) {
            commands_[i] = from_twist(msg);
            command_time_[i] = sim_->time();
          }));
      odom_pubs_.push_back(create_publisher<nav_msgs::msg::Odometry>(robot_topic(i, "odom"), 10));
    }
    poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/fleet/poses", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/fleet/sim_status", 10);
    if (publish_clock_) {
      clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    }
    const auto period = std::chrono::duration<double>(config_.dt / std::max(real_time_factor, 1e-3));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               [this] { step(); });
    RCLCPP_INFO(get_logger(), "simulating %zu robots at dt=%.3f s, real-time factor %.1f", n, config_.dt,
                real_time_factor);
  }

 private:
  void step() {
    const double t = sim_->time();
    std::vector<mrc::Vec2> u(commands_.size(), mrc::Vec2::Zero());
    for (std::size_t i = 0; i < u.size(); ++i) {
      if (t - command_time_[i] <= command_timeout_) {
        u[i] = commands_[i];
      }
    }
    sim_->step(u);
    const double now = sim_->time();
    const auto stamp = to_stamp(now);
    if (clock_pub_) {
      rosgraph_msgs::msg::Clock clock;
      clock.clock = stamp;
      clock_pub_->publish(clock);
    }
    geometry_msgs::msg::PoseArray poses;
    poses.header.stamp = stamp;
    poses.header.frame_id = frame_id_;
    for (std::size_t i = 0; i < sim_->num_robots(); ++i) {
      const auto& r = sim_->robot(i);
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = frame_id_;
      odom.child_frame_id = "robot_" + std::to_string(i) + "/base_link";
      odom.pose.pose = to_pose({r.position, r.heading});
      odom.twist.twist = to_twist(r.velocity);
      odom_pubs_[i]->publish(odom);
      poses.poses.push_back(odom.pose.pose);
    }
    poses_pub_->publish(poses);
    if (sim_->step_count() % 10 == 0) {
      const auto& m = sim_->metrics();
      std::ostringstream json;
      json << "{\"time\": " << now << ", \"robot_contacts\": " << m.robot_collisions
           << ", \"obstacle_contacts\": " << m.obstacle_collisions
           << ", \"min_pair_clearance\": " << (std::isfinite(m.min_pair_clearance) ? m.min_pair_clearance : -1.0)
           << "}";
      std_msgs::msg::String msg;
      msg.data = json.str();
      status_pub_->publish(msg);
    }
  }

  mrc::scenarios::WarehouseMissionConfig config_;
  std::optional<mrc::simulation::Simulator> sim_;
  std::vector<mrc::Vec2> commands_;
  std::vector<double> command_time_;
  double command_timeout_{0.5};
  bool publish_clock_{true};
  std::string frame_id_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> command_subs_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_pubs_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace multirobotcore_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multirobotcore_ros::FleetSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
