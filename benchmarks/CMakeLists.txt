# Google Benchmark micro-benchmarks for the computational kernels.
# Run: <build>/benchmarks/mrc_microbenchmarks --benchmark_min_time=0.2s

file(GLOB _mrc_bench_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
if(_mrc_bench_sources)
  add_executable(mrc_microbenchmarks ${_mrc_bench_sources})
  target_link_libraries(mrc_microbenchmarks PRIVATE MultiRobotCore::multirobotcore
                                                    benchmark::benchmark_main)
  mrc_set_project_warnings(mrc_microbenchmarks)
endif()
