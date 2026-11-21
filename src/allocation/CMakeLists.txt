mrc_add_module(
  allocation
  SOURCES types.cpp hungarian.cpp auction.cpp dynamic.cpp sequential_auction.cpp
  PUBLIC_DEPS MultiRobotCore::core)
