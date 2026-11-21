mrc_add_module(
  mapf
  SOURCES grid_map.cpp
          instance.cpp
          conflicts.cpp
          constraint_table.cpp
          space_time_astar.cpp
          planners.cpp
          conflict_based_search.cpp
          generators.cpp
  PUBLIC_DEPS MultiRobotCore::core)
