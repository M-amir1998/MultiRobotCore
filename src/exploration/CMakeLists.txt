mrc_add_module(
  exploration
  SOURCES occupancy_grid.cpp sensor.cpp frontier.cpp planner.cpp exploration.cpp
  PUBLIC_DEPS MultiRobotCore::core
  PRIVATE_DEPS MultiRobotCore::allocation)
