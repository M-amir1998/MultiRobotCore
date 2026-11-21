// orca_safety: generic reciprocal collision-avoidance filter for a fleet of
// holonomic robots, built on multirobotcore::avoidance::CollisionAvoider.
//
// Subscribes  robot_<i>/odom          nav_msgs/Odometry (position, velocity)
//             robot_<i>/cmd_vel_ref   geometry_msgs/Twist (preferred velocity)
// Publishes   robot_<i>/cmd_vel       geometry_msgs/Twist (safe velocity)
//
// Safe velocities are computed as soon as a fresh reference has arrived for
// every robot (event driven, one-step latency); a timer at control_rate is
// only a fallback that stops robots whose references became stale (older than
// reference_timeout). Shelves of the configured warehouse can be added as
// static obstacles (avoid_shelves).

#include "multirobotcore_ros/conversions.hpp"
#include "multirobotcore_ros/parameters.hpp"

#include <multirobotcore/avoidance/avoidance.hpp>
#include <multirobotcore/scenarios/warehouse_mission.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <vector>

namespace multirobotcore_ros {
namespace {

namespace mrc = multirobotcore;
namespace av = multirobotcore::avoidance;

class OrcaSafetyNode : public rclcpp::Node {
 public:
  OrcaSafetyNode() : rclcpp::Node("orca_safety") {
    const auto cfg = declare_warehouse_parameters(*this);
    const double rate = declare_parameter<double>("control_rate", 20.0);
    reference_timeout_ = declare_parameter<double>("reference_timeout", 0.5);
    const std::string method = declare_parameter<std::string>("method", "orca");

    av::AvoidanceConfig avoid;
    avoid.method = av::parse_method(method);
    avoid.time_horizon = cfg.orca_time_horizon;
    avoid.obstacle_time_horizon = 0.5 * cfg.orca_time_horizon;
    avoid.time_step = 1.0 / rate;
    avoid.neighbor_distance = declare_parameter<double>("neighbor_distance", 3.0);
    avoid.max_neighbors = static_cast<std::size_t>(declare_parameter<int64_t>("max_neighbors", 8));
    mrc::math::ObstacleSet obstacles;
    if (cfg.avoid_shelves) {
      obstacles = mrc::scenarios::WarehouseCoordinator(cfg).shelf_obstacles();
    }
    avoider_.emplace(avoid, obstacles);

    const std::size_t n = cfg.num_robots;
    agents_.assign(n, av::AgentState{mrc::Vec2::Zero(), mrc::Vec2::Zero(), cfg.robot_radius});
    preferred_.assign(n, mrc::Vec2::Zero());
    reference_time_.assign(n, -1e9);
    max_speeds_.assign(n, cfg.max_speed);
    have_odom_.assign(n, false);
    fresh_.assign(n, false);
    period_ = 1.0 / rate;
    for (std::size_t i = 0; i < n; ++i) {
      odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
          robot_topic(i, "odom"), 10, [this, i](const nav_msgs::msg::Odometry& msg) {
            agents_[i].position = from_point(msg.pose.pose.position);
            agents_[i].velocity = from_twist(msg.twist.twist);
            have_odom_[i] = true;
          }));
      ref_subs_.push_back(create_subscription<geometry_msgs::msg::Twist>(
          robot_topic(i, "cmd_vel_ref"), 10, [this, i](const geometry_msgs::msg::Twist& msg) {
            preferred_[i] = from_twist(msg);
            reference_time_[i] = this->now().seconds();
            fresh_[i] = true;
            if (std::all_of(fresh_.begin(), fresh_.end(), [](bool f) { return f; })) {
              control();
            }
          }));
      cmd_pubs_.push_back(create_publisher<geometry_msgs::msg::Twist>(robot_topic(i, "cmd_vel"), 10));
    }
    timer_ = rclcpp::create_timer(this, get_clock(),
                                  std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::duration<double>(1.0 / rate)),
                                  [this] {
                                    // Fallback only: run when no complete reference
                                    // set arrived for two periods.
                                    if (this->now().seconds() - last_control_ > 2.0 * period_) {
                                      control();
                                    }
                                  });
    RCLCPP_INFO(get_logger(), "%s safety filter for %zu robots", method.c_str(), n);
  }

 private:
  void control() {
    for (const bool ok : have_odom_) {
      if (!ok) {
        return;
      }
    }
    const double now = this->now().seconds();
    last_control_ = now;
    std::fill(fresh_.begin(), fresh_.end(), false);
    std::vector<mrc::Vec2> preferred = preferred_;
    for (std::size_t i = 0; i < preferred.size(); ++i) {
      if (now - reference_time_[i] > reference_timeout_) {
        preferred[i].setZero();
      }
    }
    const auto safe = avoider_->compute(agents_, preferred, max_speeds_);
    for (std::size_t i = 0; i < safe.size(); ++i) {
      cmd_pubs_[i]->publish(to_twist(safe[i]));
    }
  }

  std::optional<av::CollisionAvoider> avoider_;
  std::vector<av::AgentState> agents_;
  std::vector<mrc::Vec2> preferred_;
  std::vector<double> reference_time_;
  std::vector<double> max_speeds_;
  std::vector<bool> have_odom_;
  std::vector<bool> fresh_;
  double period_{0.05};
  double last_control_{-1e9};
  double reference_timeout_{0.5};
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> ref_subs_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_pubs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace multirobotcore_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multirobotcore_ros::OrcaSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
