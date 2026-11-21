mrc_add_module(
  io
  SOURCES json.cpp
  PUBLIC_DEPS MultiRobotCore::simulation MultiRobotCore::communication MultiRobotCore::graph
              nlohmann_json::nlohmann_json)
