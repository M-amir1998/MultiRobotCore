mrc_add_module(
  graph
  SOURCES graph.cpp matrices.cpp connectivity.cpp topologies.cpp
  PUBLIC_DEPS MultiRobotCore::core)
