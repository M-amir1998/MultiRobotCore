# Google Benchmark micro-benchmarks for the computational kernels.
#
#   benchmarks/bench_<name>.cpp -> mrc_bench_<name>
#
# A source declares the modules it needs with "// mrc-modules: <modules>",
# otherwise it links the umbrella target. Run, for example:
#   <build>/benchmarks/mrc_bench_mapf --benchmark_min_time=0.2s

file(GLOB _mrc_bench_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/bench_*.cpp")
foreach(_src IN LISTS _mrc_bench_sources)
  get_filename_component(_stem "${_src}" NAME_WE)
  string(REGEX REPLACE "^bench_" "" _name "${_stem}")
  file(STRINGS "${_src}" _decl REGEX "^// mrc-modules:")
  set(_libs MultiRobotCore::multirobotcore)
  if(_decl)
    string(REGEX REPLACE "^// mrc-modules:[ ]*" "" _modules "${_decl}")
    separate_arguments(_modules)
    set(_libs "")
    foreach(_m IN LISTS _modules)
      list(APPEND _libs MultiRobotCore::${_m})
    endforeach()
  endif()
  add_executable(mrc_bench_${_name} "${_src}")
  target_link_libraries(mrc_bench_${_name} PRIVATE ${_libs} benchmark::benchmark_main)
  mrc_set_project_warnings(mrc_bench_${_name})
endforeach()
