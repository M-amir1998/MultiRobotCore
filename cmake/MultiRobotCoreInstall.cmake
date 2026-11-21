# Install rules and CMake package export:
#
#   find_package(MultiRobotCore CONFIG REQUIRED)
#   target_link_libraries(app PRIVATE MultiRobotCore::mapf)

include_guard(GLOBAL)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(MULTIROBOTCORE_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/MultiRobotCore")

get_property(_mrc_targets GLOBAL PROPERTY MULTIROBOTCORE_MODULE_TARGETS)

install(
  TARGETS ${_mrc_targets} multirobotcore
  EXPORT MultiRobotCoreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY "${PROJECT_SOURCE_DIR}/include/multirobotcore"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# When Eigen was fetched rather than found, ship its headers so that the
# installed package is self-contained.
set(MRC_BUNDLED_EIGEN_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}/multirobotcore/third_party/eigen3")
if(MULTIROBOTCORE_EIGEN_BUNDLED)
  install(DIRECTORY "${MULTIROBOTCORE_EIGEN_SOURCE_DIR}/Eigen"
          DESTINATION ${MRC_BUNDLED_EIGEN_INCLUDEDIR})
  install(FILES "${MULTIROBOTCORE_EIGEN_SOURCE_DIR}/COPYING.MPL2"
          DESTINATION ${MRC_BUNDLED_EIGEN_INCLUDEDIR})
endif()

install(
  EXPORT MultiRobotCoreTargets
  NAMESPACE MultiRobotCore::
  DESTINATION ${MULTIROBOTCORE_INSTALL_CMAKEDIR})

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/MultiRobotCoreConfig.cmake.in"
  "${PROJECT_BINARY_DIR}/MultiRobotCoreConfig.cmake"
  INSTALL_DESTINATION ${MULTIROBOTCORE_INSTALL_CMAKEDIR}
  PATH_VARS MRC_BUNDLED_EIGEN_INCLUDEDIR)

write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/MultiRobotCoreConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMinorVersion)

install(FILES "${PROJECT_BINARY_DIR}/MultiRobotCoreConfig.cmake"
              "${PROJECT_BINARY_DIR}/MultiRobotCoreConfigVersion.cmake"
        DESTINATION ${MULTIROBOTCORE_INSTALL_CMAKEDIR})
