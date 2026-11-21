mrc_add_module(
  simulation
  SOURCES simulator.cpp events.cpp
  PUBLIC_DEPS MultiRobotCore::core MultiRobotCore::communication)
