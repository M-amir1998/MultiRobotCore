cmake_minimum_required(VERSION 3.24)

project(
  MultiRobotCore
  VERSION 0.1.0
  DESCRIPTION "Classical multi-robot coordination, planning, control, allocation and simulation library"
  HOMEPAGE_URL "https://github.com/M-amir1998/MultiRobotCore"
  LANGUAGES CXX)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(FATAL_ERROR "In-source builds are not supported. Use: cmake -S . -B build")
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

include(MultiRobotCoreOptions)
include(CompilerWarnings)
include(Sanitizers)
include(Dependencies)
include(MultiRobotCoreHelpers)

add_subdirectory(src)

if(MULTIROBOTCORE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(MULTIROBOTCORE_BUILD_APPS)
  add_subdirectory(apps)
endif()

if(MULTIROBOTCORE_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()

if(MULTIROBOTCORE_INSTALL)
  include(MultiRobotCoreInstall)
endif()
