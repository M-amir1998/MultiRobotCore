mrc_add_module(
  formation
  SOURCES shapes.cpp controllers.cpp metrics.cpp transition.cpp safety.cpp
  PUBLIC_DEPS MultiRobotCore::graph MultiRobotCore::core
  PRIVATE_DEPS MultiRobotCore::allocation)
