cmake_minimum_required(VERSION 3.22)
project(multirobotcore_ros LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
# The core library is ROS-independent and installed separately
# (see README.md): pass its prefix through CMAKE_PREFIX_PATH.
find_package(MultiRobotCore 0.1 CONFIG REQUIRED)

set(MRC_ROS_DEPS rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs rosgraph_msgs)

add_library(multirobotcore_ros_common INTERFACE)
target_include_directories(multirobotcore_ros_common
                           INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(
  multirobotcore_ros_common
  INTERFACE MultiRobotCore::scenarios MultiRobotCore::avoidance MultiRobotCore::simulation
            ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS})

function(mrc_ros_node name source)
  add_executable(${name} ${source})
  target_link_libraries(${name} PRIVATE multirobotcore_ros_common)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${name} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  # ament_target_dependencies is deprecated in newer distributions; link the
  # exported message/client targets directly.
  target_link_libraries(${name} PRIVATE rclcpp::rclcpp ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS}
                                        ${std_msgs_TARGETS} ${visualization_msgs_TARGETS}
                                        ${rosgraph_msgs_TARGETS})
  install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
endfunction()

mrc_ros_node(fleet_simulator src/fleet_simulator_node.cpp)
mrc_ros_node(warehouse_coordinator src/warehouse_coordinator_node.cpp)
mrc_ros_node(orca_safety src/orca_safety_node.cpp)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions multirobotcore_ros_common rclcpp::rclcpp)
  add_launch_test(test/test_warehouse_launch.py TIMEOUT 180)
endif()

ament_package()
