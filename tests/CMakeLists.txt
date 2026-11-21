# Test layout
#   unit/<module>/*.cpp        -> mrc_unit_<module>          (label: unit)
#   numerical/<module>/*.cpp   -> mrc_numerical_<module>     (label: numerical)
#   regression/<module>/*.cpp  -> mrc_regression_<module>    (label: regression)
#   integration/*.cpp          -> mrc_integration_tests      (label: integration)
#
# Per-module suites link only MultiRobotCore::<module> (and its dependencies);
# integration tests link the umbrella target. Run one category with
#   ctest --test-dir <build> -L <label>

foreach(_category IN ITEMS unit numerical regression)
  file(
    GLOB _mrc_dirs
    LIST_DIRECTORIES true
    CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${_category}/*")
  foreach(_dir IN LISTS _mrc_dirs)
    if(IS_DIRECTORY "${_dir}")
      get_filename_component(_module "${_dir}" NAME)
      file(GLOB _sources CONFIGURE_DEPENDS "${_dir}/*.cpp")
      if(_sources AND TARGET MultiRobotCore::${_module})
        mrc_add_test_executable(
          mrc_${_category}_${_module}
          LABEL ${_category}
          SOURCES ${_sources}
          LIBS MultiRobotCore::${_module})
      endif()
    endif()
  endforeach()
endforeach()

file(GLOB _mrc_integration_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/integration/*.cpp")
if(_mrc_integration_sources)
  mrc_add_test_executable(
    mrc_integration_tests
    LABEL integration
    SOURCES ${_mrc_integration_sources}
    LIBS MultiRobotCore::multirobotcore)
endif()
