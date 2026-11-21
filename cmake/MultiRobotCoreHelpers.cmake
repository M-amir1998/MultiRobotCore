# Target helpers shared by library modules, tests, demos and benchmarks.

include_guard(GLOBAL)
include(GNUInstallDirs)

set_property(GLOBAL PROPERTY MULTIROBOTCORE_MODULE_TARGETS "")

# Copies the clang-cl ASan runtime next to an executable so that tests and
# demos run without modifying the global PATH.
function(mrc_copy_sanitizer_runtime target)
  if(WIN32 AND MULTIROBOTCORE_SANITIZER_RUNTIME_DIR)
    set(_dll "${MULTIROBOTCORE_SANITIZER_RUNTIME_DIR}/clang_rt.asan_dynamic-x86_64.dll")
    if(EXISTS "${_dll}")
      add_custom_command(
        TARGET ${target}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_dll}" "$<TARGET_FILE_DIR:${target}>"
        VERBATIM)
    endif()
  endif()
endfunction()

# mrc_add_module(<name> SOURCES ... [PUBLIC_DEPS ...] [PRIVATE_DEPS ...])
#
# Creates the library target `multirobotcore_<name>` exported as
# `MultiRobotCore::<name>`.
function(mrc_add_module name)
  cmake_parse_arguments(PARSE_ARGV 1 ARG "" "" "SOURCES;PUBLIC_DEPS;PRIVATE_DEPS")
  set(_target multirobotcore_${name})
  add_library(${_target} ${ARG_SOURCES})
  add_library(MultiRobotCore::${name} ALIAS ${_target})
  set_target_properties(${_target} PROPERTIES EXPORT_NAME ${name} POSITION_INDEPENDENT_CODE ON)
  target_include_directories(
    ${_target} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  target_compile_features(${_target} PUBLIC cxx_std_20)
  target_link_libraries(
    ${_target}
    PUBLIC ${ARG_PUBLIC_DEPS}
    PRIVATE ${ARG_PRIVATE_DEPS})
  if(MSVC)
    # Required by consumers as well: Eigen and <cmath> constants on MSVC.
    target_compile_definitions(${_target} PUBLIC _USE_MATH_DEFINES NOMINMAX)
  endif()
  mrc_set_project_warnings(${_target})
  set_property(GLOBAL APPEND PROPERTY MULTIROBOTCORE_MODULE_TARGETS ${_target})
endfunction()

# mrc_add_test_executable(<name> LABEL <label> SOURCES ... LIBS ...)
function(mrc_add_test_executable name)
  cmake_parse_arguments(PARSE_ARGV 1 ARG "" "LABEL" "SOURCES;LIBS")
  add_executable(${name} ${ARG_SOURCES})
  target_link_libraries(${name} PRIVATE ${ARG_LIBS} GTest::gtest_main)
  target_include_directories(${name} PRIVATE "${PROJECT_SOURCE_DIR}/tests/support")
  target_compile_definitions(${name} PRIVATE MRC_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  mrc_set_project_warnings(${name})
  mrc_copy_sanitizer_runtime(${name})
  gtest_discover_tests(
    ${name}
    DISCOVERY_MODE PRE_TEST
    DISCOVERY_TIMEOUT 60
    PROPERTIES LABELS ${ARG_LABEL} TIMEOUT 600)
endfunction()

# mrc_add_app(<name> SOURCES ... LIBS ...)
function(mrc_add_app name)
  cmake_parse_arguments(PARSE_ARGV 1 ARG "" "" "SOURCES;LIBS")
  add_executable(${name} ${ARG_SOURCES})
  target_link_libraries(${name} PRIVATE ${ARG_LIBS})
  target_compile_definitions(${name} PRIVATE MRC_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  mrc_set_project_warnings(${name})
  mrc_copy_sanitizer_runtime(${name})
endfunction()
