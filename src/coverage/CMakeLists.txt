mrc_add_module(
  coverage
  SOURCES density.cpp voronoi.cpp quadrature.cpp coverage_control.cpp
  PUBLIC_DEPS MultiRobotCore::core)
