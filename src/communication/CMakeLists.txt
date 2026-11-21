mrc_add_module(
  communication
  SOURCES topology.cpp network.cpp
  PUBLIC_DEPS MultiRobotCore::graph)
