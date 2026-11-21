# Library modules in dependency order. Each module directory defines a target
# `MultiRobotCore::<module>` via mrc_add_module().
set(MULTIROBOTCORE_MODULES
    core
    graph
    communication
    consensus
    allocation
    formation
    coverage
    mapf
    avoidance
    exploration
    optimization
    control
    simulation
    io
    scenarios)

foreach(_module IN LISTS MULTIROBOTCORE_MODULES)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_module}/CMakeLists.txt")
    add_subdirectory(${_module})
  endif()
endforeach()

# Umbrella target linking every module: MultiRobotCore::multirobotcore
add_library(multirobotcore INTERFACE)
add_library(MultiRobotCore::multirobotcore ALIAS multirobotcore)
set_target_properties(multirobotcore PROPERTIES EXPORT_NAME multirobotcore)
get_property(_mrc_module_targets GLOBAL PROPERTY MULTIROBOTCORE_MODULE_TARGETS)
target_link_libraries(multirobotcore INTERFACE ${_mrc_module_targets})
