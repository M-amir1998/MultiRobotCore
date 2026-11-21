mrc_add_module(
  consensus
  SOURCES consensus.cpp distributed.cpp
  PUBLIC_DEPS MultiRobotCore::graph MultiRobotCore::communication)
