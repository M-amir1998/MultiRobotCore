# Build options for MultiRobotCore.
#
# Tests, demos and benchmarks default to ON only when MultiRobotCore is the
# top-level project, so that consumers using add_subdirectory/FetchContent get a
# lean library build.

include_guard(GLOBAL)

set(_mrc_top_level OFF)
if(PROJECT_IS_TOP_LEVEL)
  set(_mrc_top_level ON)
endif()

option(MULTIROBOTCORE_BUILD_TESTS "Build the GoogleTest suites" ${_mrc_top_level})
option(MULTIROBOTCORE_BUILD_APPS "Build demo and experiment executables" ${_mrc_top_level})
option(MULTIROBOTCORE_BUILD_BENCHMARKS "Build Google Benchmark micro-benchmarks" ${_mrc_top_level})
option(MULTIROBOTCORE_INSTALL "Generate install and package-config rules" ON)
option(MULTIROBOTCORE_WARNINGS_AS_ERRORS "Treat project compiler warnings as errors" OFF)
option(MULTIROBOTCORE_ENABLE_CLANG_TIDY "Run clang-tidy as part of compilation" OFF)
option(MULTIROBOTCORE_WITH_OSQP "Build the MPC module against the OSQP QP solver" OFF)
set(MULTIROBOTCORE_SANITIZERS
    ""
    CACHE STRING "Semicolon-separated sanitizers to enable (address;undefined)")

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

if(MULTIROBOTCORE_ENABLE_CLANG_TIDY)
  find_program(MULTIROBOTCORE_CLANG_TIDY_EXE NAMES clang-tidy REQUIRED)
  set(CMAKE_CXX_CLANG_TIDY "${MULTIROBOTCORE_CLANG_TIDY_EXE}")
endif()
