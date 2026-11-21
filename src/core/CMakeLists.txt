mrc_add_module(
  core
  SOURCES geometry.cpp obstacles.cpp convex_polygon.cpp robot_models.cpp robot_state.cpp
  PUBLIC_DEPS Eigen3::Eigen)
