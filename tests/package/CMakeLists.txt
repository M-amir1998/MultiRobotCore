# Downstream consumer used by scripts/ci/install_test.sh to verify the
# installed CMake package (exported targets, headers, dependencies).
cmake_minimum_required(VERSION 3.24)
project(multirobotcore_package_consumer LANGUAGES CXX)

find_package(MultiRobotCore 0.1 CONFIG REQUIRED)

add_executable(package_consumer main.cpp)
target_compile_features(package_consumer PRIVATE cxx_std_20)
target_link_libraries(package_consumer PRIVATE MultiRobotCore::mapf MultiRobotCore::allocation
                                               MultiRobotCore::consensus)
