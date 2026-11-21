# Third-party dependency resolution.
#
# Policy: use an installed package when one is available (offline builds,
# distribution packages, ROS 2 workspaces); otherwise fetch a pinned release
# archive verified by SHA-256. Archives are cached in
# MULTIROBOTCORE_DOWNLOAD_CACHE and shared by all build trees (each tree
# extracts and builds its own copy), so after the first download every
# configuration builds offline.

include_guard(GLOBAL)
include(FetchContent)

set(FETCHCONTENT_QUIET ON)
set(MULTIROBOTCORE_DOWNLOAD_CACHE
    "${PROJECT_SOURCE_DIR}/.deps/downloads"
    CACHE PATH "Directory caching downloaded dependency archives")

# --- Eigen (header-only linear algebra) --------------------------------------
find_package(Eigen3 3.4 QUIET NO_MODULE)
if(TARGET Eigen3::Eigen)
  set(MULTIROBOTCORE_EIGEN_BUNDLED FALSE)
  message(STATUS "MultiRobotCore: using installed Eigen ${Eigen3_VERSION}")
else()
  # Eigen's own CMake project configures documentation, BLAS probes and a large
  # test suite. Only the headers are required, so the archive is downloaded
  # without adding its CMake project and an imported interface target is
  # created instead.
  FetchContent_Declare(
    mrc_eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
    SOURCE_SUBDIR _headers_only_no_cmake_project
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  DOWNLOAD_DIR "${MULTIROBOTCORE_DOWNLOAD_CACHE}")
  FetchContent_MakeAvailable(mrc_eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
  set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                 "${mrc_eigen_SOURCE_DIR}")
  set(MULTIROBOTCORE_EIGEN_BUNDLED TRUE)
  set(MULTIROBOTCORE_EIGEN_SOURCE_DIR "${mrc_eigen_SOURCE_DIR}")
  message(STATUS "MultiRobotCore: using fetched Eigen 3.4.0")
endif()

# --- nlohmann/json (configuration and result files) --------------------------
set(JSON_Install
    ${MULTIROBOTCORE_INSTALL}
    CACHE BOOL "" FORCE)
set(JSON_BuildTests
    OFF
    CACHE BOOL "" FORCE)
FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
  URL_HASH SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  DOWNLOAD_DIR "${MULTIROBOTCORE_DOWNLOAD_CACHE}"
  FIND_PACKAGE_ARGS 3.11 CONFIG)
FetchContent_MakeAvailable(nlohmann_json)

# --- GoogleTest ---------------------------------------------------------------
if(MULTIROBOTCORE_BUILD_TESTS)
  set(gtest_force_shared_crt
      ON
      CACHE BOOL "" FORCE)
  set(INSTALL_GTEST
      OFF
      CACHE BOOL "" FORCE)
  set(BUILD_GMOCK
      OFF
      CACHE BOOL "" FORCE)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/releases/download/v1.17.0/googletest-1.17.0.tar.gz
    URL_HASH SHA256=65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  DOWNLOAD_DIR "${MULTIROBOTCORE_DOWNLOAD_CACHE}"
    FIND_PACKAGE_ARGS NAMES GTest)
  FetchContent_MakeAvailable(googletest)
  include(GoogleTest)
endif()

# --- Google Benchmark ---------------------------------------------------------
if(MULTIROBOTCORE_BUILD_BENCHMARKS)
  set(BENCHMARK_ENABLE_TESTING
      OFF
      CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_GTEST_TESTS
      OFF
      CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_INSTALL
      OFF
      CACHE BOOL "" FORCE)
  set(BENCHMARK_INSTALL_DOCS
      OFF
      CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_WERROR
      OFF
      CACHE BOOL "" FORCE)
  FetchContent_Declare(
    googlebenchmark
    URL https://github.com/google/benchmark/archive/refs/tags/v1.9.4.tar.gz
    URL_HASH SHA256=b334658edd35efcf06a99d9be21e4e93e092bd5f95074c1673d5c8705d95c104
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  DOWNLOAD_DIR "${MULTIROBOTCORE_DOWNLOAD_CACHE}"
    FIND_PACKAGE_ARGS NAMES benchmark)
  FetchContent_MakeAvailable(googlebenchmark)
endif()

# --- OSQP (optional QP solver for model predictive control) -------------------
if(MULTIROBOTCORE_WITH_OSQP)
  find_package(osqp QUIET CONFIG)
  if(NOT TARGET osqp::osqp AND NOT TARGET osqpstatic)
    set(OSQP_BUILD_SHARED_LIB
        OFF
        CACHE BOOL "" FORCE)
    set(OSQP_BUILD_DEMO_EXE
        OFF
        CACHE BOOL "" FORCE)
    set(OSQP_BUILD_UNITTESTS
        OFF
        CACHE BOOL "" FORCE)
    FetchContent_Declare(
      osqp
      URL https://github.com/osqp/osqp/archive/refs/tags/v1.0.0.tar.gz
      URL_HASH SHA256=dd6a1c2e7e921485697d5e7cdeeb043c712526c395b3700601f51d472a7d8e48
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  DOWNLOAD_DIR "${MULTIROBOTCORE_DOWNLOAD_CACHE}")
    FetchContent_MakeAvailable(osqp)
  endif()
endif()
