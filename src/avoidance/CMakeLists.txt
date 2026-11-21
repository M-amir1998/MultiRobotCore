mrc_add_module(
  avoidance
  SOURCES velocity_obstacle.cpp linear_program.cpp orca.cpp sampling.cpp avoidance.cpp
  PUBLIC_DEPS MultiRobotCore::core)
