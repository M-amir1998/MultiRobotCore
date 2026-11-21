#pragma once

/// @file parameters.hpp
/// Declares the warehouse parameters shared by all nodes and maps them onto
/// multirobotcore::scenarios::WarehouseMissionConfig, so that the simulator,
/// coordinator and safety filter agree on the map and the fleet.

#include <multirobotcore/scenarios/warehouse_mission.hpp>

#include <rclcpp/rclcpp.hpp>

namespace multirobotcore_ros {

inline multirobotcore::scenarios::WarehouseMissionConfig declare_warehouse_parameters(rclcpp::Node& node) {
  multirobotcore::scenarios::WarehouseMissionConfig c;
  c.layout.shelf_block_columns = static_cast<int>(
      node.declare_parameter<int64_t>("layout.shelf_block_columns", c.layout.shelf_block_columns));
  c.layout.shelf_block_rows =
      static_cast<int>(node.declare_parameter<int64_t>("layout.shelf_block_rows", c.layout.shelf_block_rows));
  c.layout.shelf_length =
      static_cast<int>(node.declare_parameter<int64_t>("layout.shelf_length", c.layout.shelf_length));
  c.layout.shelf_depth =
      static_cast<int>(node.declare_parameter<int64_t>("layout.shelf_depth", c.layout.shelf_depth));
  c.layout.aisle_width =
      static_cast<int>(node.declare_parameter<int64_t>("layout.aisle_width", c.layout.aisle_width));
  c.layout.station_margin =
      static_cast<int>(node.declare_parameter<int64_t>("layout.station_margin", c.layout.station_margin));
  c.num_robots = static_cast<std::size_t>(node.declare_parameter<int64_t>("num_robots", 12));
  c.cell_size = node.declare_parameter<double>("cell_size", c.cell_size);
  c.step_duration = node.declare_parameter<double>("step_duration", c.step_duration);
  c.dt = node.declare_parameter<double>("dt", c.dt);
  c.seed = static_cast<std::uint64_t>(node.declare_parameter<int64_t>("seed", 7));
  c.service_steps = static_cast<std::size_t>(node.declare_parameter<int64_t>("service_steps", 2));
  c.robot_radius = node.declare_parameter<double>("robot_radius", 0.25);
  c.max_speed = node.declare_parameter<double>("max_speed", c.max_speed);
  c.max_acceleration = node.declare_parameter<double>("max_acceleration", c.max_acceleration);
  c.tracking_gain = node.declare_parameter<double>("tracking_gain", c.tracking_gain);
  c.orca_time_horizon = node.declare_parameter<double>("orca_time_horizon", 0.6);
  c.avoid_shelves = node.declare_parameter<bool>("avoid_shelves", true);
  c.planner = node.declare_parameter<std::string>("planner", c.planner);
  c.prioritized_attempts =
      static_cast<std::size_t>(node.declare_parameter<int64_t>("prioritized_attempts", 20));
  c.cbs_time_limit = node.declare_parameter<double>("cbs_time_limit", c.cbs_time_limit);
  return c;
}

}  // namespace multirobotcore_ros
