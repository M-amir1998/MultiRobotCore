# Demo executables (apps/demos/<name>.cpp -> mrc_demo_<name>) write
# deterministic JSON result files consumed by the Python renderers in
# scripts/render/.
#
# Experiment runners (apps/benchmarks/<name>.cpp -> mrc_experiment_<name>)
# sweep scenarios and write JSON tables consumed by scripts/benchmark/.
#
# A source file declares the library modules it needs with a comment line
#   // mrc-modules: mapf io
# and otherwise links the umbrella target MultiRobotCore::multirobotcore.

function(mrc_add_app_from_source prefix source)
  get_filename_component(_name "${source}" NAME_WE)
  file(STRINGS "${source}" _decl REGEX "^// mrc-modules:")
  set(_libs MultiRobotCore::multirobotcore)
  if(_decl)
    string(REGEX REPLACE "^// mrc-modules:[ ]*" "" _modules "${_decl}")
    separate_arguments(_modules)
    set(_libs "")
    foreach(_m IN LISTS _modules)
      list(APPEND _libs MultiRobotCore::${_m})
    endforeach()
  endif()
  mrc_add_app(
    ${prefix}_${_name}
    SOURCES "${source}"
    LIBS ${_libs})
  target_include_directories(${prefix}_${_name} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/common")
endfunction()

file(GLOB _mrc_demo_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/demos/*.cpp")
foreach(_src IN LISTS _mrc_demo_sources)
  mrc_add_app_from_source(mrc_demo "${_src}")
endforeach()

file(GLOB _mrc_experiment_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/*.cpp")
foreach(_src IN LISTS _mrc_experiment_sources)
  mrc_add_app_from_source(mrc_experiment "${_src}")
endforeach()
