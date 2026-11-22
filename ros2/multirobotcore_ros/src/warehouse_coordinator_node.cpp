// warehouse_coordinator: ROS 2 front end of multirobotcore::scenarios::
// WarehouseCoordinator (Hungarian task allocation + prioritized planning /
// CBS + trajectory tracking).
//
// Subscribes  /fleet/poses              geometry_msgs/PoseArray (one pose per robot)
//             /fleet/task_requests      geometry_msgs/PointStamped (pick near this point)
// Publishes   robot_<i>/cmd_vel_ref     geometry_msgs/Twist (preferred velocity)
//             robot_<i>/plan            nav_msgs/Path (timed MAPF plan, re-published on replans)
//             /fleet/markers            visualization_msgs/MarkerArray (robots, tasks, shelves)
//             /fleet/status             std_msgs/String (JSON summary)
//
// Optionally generates its own Poisson task stream (task_rate > 0) so the
// demo runs without an external order source.
//
// The control step is event driven: references are computed as soon as a new
// fleet pose message arrives, so the loop simulator -> coordinator -> safety
// filter -> simulator has a deterministic one-step latency instead of the
// variable delay of independent timers.

#include "multirobotcore_ros/conversions.hpp"
#include "multirobotcore_ros/parameters.hpp"

#include <multirobotcore/scenarios/warehouse_mission.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <vector>

namespace multirobotcore_ros {
namespace {

namespace mrc = multirobotcore;
namespace sc = multirobotcore::scenarios;

std_msgs::msg::ColorRGBA rgba(double r, double g, double b, double a = 1.0) {
  std_msgs::msg::ColorRGBA c;
  c.r = static_cast<float>(r);
  c.g = static_cast<float>(g);
  c.b = static_cast<float>(b);
  c.a = static_cast<float>(a);
  return c;
}

std_msgs::msg::ColorRGBA state_color(sc::WarehouseRobotState s) {
  switch (s) {
    case sc::WarehouseRobotState::kToPickup:
    case sc::WarehouseRobotState::kPicking:
      return rgba(0.165, 0.471, 0.839);  // blue
    case sc::WarehouseRobotState::kToDropoff:
    case sc::WarehouseRobotState::kDropping:
      return rgba(0.922, 0.408, 0.204);  // orange
    case sc::WarehouseRobotState::kIdle:
      break;
  }
  return rgba(0.6, 0.6, 0.58);
}

class WarehouseCoordinatorNode : public rclcpp::Node {
 public:
  WarehouseCoordinatorNode() : rclcpp::Node("warehouse_coordinator") {
    config_ = declare_warehouse_parameters(*this);
    task_rate_ = declare_parameter<double>("task_rate", 0.22);
    initial_tasks_ = static_cast<std::size_t>(declare_parameter<int64_t>("initial_tasks", 12));
    task_horizon_ = declare_parameter<double>("task_horizon", 85.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    coordinator_.emplace(config_);
    rng_.seed(config_.seed ^ 0xA5A5A5A5ULL);
    const std::size_t n = coordinator_->num_robots();
    positions_.assign(n, mrc::Vec2::Zero());

    poses_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
        "/fleet/poses", 10, [this](const geometry_msgs::msg::PoseArray& msg) {
          if (msg.poses.size() != positions_.size()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "expected %zu poses, got %zu",
                                 positions_.size(), msg.poses.size());
            return;
          }
          for (std::size_t i = 0; i < msg.poses.size(); ++i) {
            positions_[i] = from_point(msg.poses[i].position);
          }
          have_poses_ = true;
          control();
        });
    request_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/fleet/task_requests", 10, [this](const geometry_msgs::msg::PointStamped& msg) {
          external_requests_.push_back(from_point(msg.point));
        });
    for (std::size_t i = 0; i < n; ++i) {
      cmd_pubs_.push_back(
          create_publisher<geometry_msgs::msg::Twist>(robot_topic(i, "cmd_vel_ref"), 10));
      plan_pubs_.push_back(create_publisher<nav_msgs::msg::Path>(robot_topic(i, "plan"), 10));
    }
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/fleet/markers", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/fleet/status", 10);
  }

 private:
  double uniform01() { return static_cast<double>(rng_() >> 11U) * (1.0 / 9007199254740992.0); }

  void control() {
    if (!have_poses_) {
      return;
    }
    const double now = this->now().seconds();
    if (!start_) {
      start_ = now;
      for (std::size_t k = 0; k < initial_tasks_; ++k) {
        (void)coordinator_->request_task(0.0);
      }
      next_auto_ = task_rate_ > 0.0 ? -std::log(1.0 - uniform01()) / task_rate_ : -1.0;
    }
    const double t = now - *start_;
    while (task_rate_ > 0.0 && next_auto_ >= 0.0 && next_auto_ <= t && next_auto_ < task_horizon_) {
      (void)coordinator_->request_task(next_auto_);
      next_auto_ += -std::log(1.0 - uniform01()) / task_rate_;
    }
    for (const auto& p : external_requests_) {
      if (!coordinator_->request_task(t, p)) {
        RCLCPP_WARN(get_logger(), "task request rejected: every pickup already has an open task");
      }
    }
    external_requests_.clear();

    const auto preferred = coordinator_->update(t, positions_);
    for (std::size_t i = 0; i < preferred.size(); ++i) {
      cmd_pubs_[i]->publish(to_twist(preferred[i]));
    }
    const auto& stats = coordinator_->stats();
    if (stats.replans != published_replans_) {
      published_replans_ = stats.replans;
      for (std::size_t i = 0; i < plan_pubs_.size(); ++i) {
        auto path = to_path(coordinator_->timed_plan(i), frame_id_, *start_);
        path.header.stamp = this->now();
        plan_pubs_[i]->publish(path);
      }
    }
    if (t - last_visual_ >= 0.5) {
      last_visual_ = t;
      publish_markers();
      publish_status(t);
    }
  }

  void publish_markers() {
    visualization_msgs::msg::MarkerArray array;
    const auto stamp = this->now();
    const auto states = coordinator_->robot_states();
    for (std::size_t i = 0; i < positions_.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id_;
      m.header.stamp = stamp;
      m.ns = "robots";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.pose.position = to_point(positions_[i]);
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = 2.0 * config_.robot_radius;
      m.scale.z = 0.2;
      m.color = state_color(states[i]);
      array.markers.push_back(m);
    }
    visualization_msgs::msg::Marker tasks;
    tasks.header.frame_id = frame_id_;
    tasks.header.stamp = stamp;
    tasks.ns = "tasks";
    tasks.type = visualization_msgs::msg::Marker::CUBE_LIST;
    tasks.scale.x = tasks.scale.y = 0.4 * config_.cell_size;
    tasks.scale.z = 0.1;
    tasks.pose.orientation.w = 1.0;
    tasks.color = rgba(0.929, 0.631, 0.0);
    for (const auto& task : coordinator_->tasks()) {
      if (task.delivered_time < 0.0 && task.picked_time < 0.0) {
        tasks.points.push_back(to_point(coordinator_->frame().to_world(task.pickup)));
      }
    }
    array.markers.push_back(tasks);
    if (!shelves_sent_) {
      visualization_msgs::msg::Marker shelves;
      shelves.header.frame_id = frame_id_;
      shelves.header.stamp = stamp;
      shelves.ns = "shelves";
      shelves.type = visualization_msgs::msg::Marker::CUBE_LIST;
      shelves.scale.x = shelves.scale.y = 0.92 * config_.cell_size;
      shelves.scale.z = 0.5;
      shelves.pose.orientation.w = 1.0;
      shelves.color = rgba(0.86, 0.86, 0.84);
      for (const auto& box : coordinator_->shelf_obstacles().boxes) {
        shelves.points.push_back(to_point(box.center()));
      }
      array.markers.push_back(shelves);
      shelves_sent_ = true;
    }
    marker_pub_->publish(array);
  }

  void publish_status(double t) {
    const auto& stats = coordinator_->stats();
    std::ostringstream json;
    json << "{\"time\": " << t << ", \"tasks_requested\": " << coordinator_->tasks().size()
         << ", \"tasks_completed\": " << stats.tasks_completed << ", \"replans\": " << stats.replans
         << ", \"planning_failures\": " << stats.planning_failures
         << ", \"max_tracking_error\": " << stats.max_tracking_error << "}";
    std_msgs::msg::String msg;
    msg.data = json.str();
    status_pub_->publish(msg);
  }

  sc::WarehouseMissionConfig config_;
  std::optional<sc::WarehouseCoordinator> coordinator_;
  std::mt19937_64 rng_;
  std::vector<mrc::Vec2> positions_;
  std::vector<mrc::Vec2> external_requests_;
  bool have_poses_{false};
  bool shelves_sent_{false};
  std::optional<double> start_;
  double task_rate_{0.0};
  double task_horizon_{0.0};
  double next_auto_{-1.0};
  double last_visual_{-1e9};
  std::size_t initial_tasks_{0};
  std::size_t published_replans_{0};
  std::string frame_id_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr poses_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr request_sub_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_pubs_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> plan_pubs_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

}  // namespace
}  // namespace multirobotcore_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multirobotcore_ros::WarehouseCoordinatorNode>());
  rclcpp::shutdown();
  return 0;
}
