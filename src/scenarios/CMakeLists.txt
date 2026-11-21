# Integrated scenarios shared by demos, experiments and integration tests.
mrc_add_module(
  scenarios
  SOURCES formation_mission.cpp
  PUBLIC_DEPS MultiRobotCore::formation MultiRobotCore::consensus MultiRobotCore::simulation
              MultiRobotCore::communication MultiRobotCore::io)
