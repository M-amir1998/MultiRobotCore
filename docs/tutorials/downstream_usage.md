# Using MultiRobotCore from another CMake project

## Install

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/.local/multirobotcore \
      -DMULTIROBOTCORE_BUILD_TESTS=OFF -DMULTIROBOTCORE_BUILD_APPS=OFF -DMULTIROBOTCORE_BUILD_BENCHMARKS=OFF
cmake --build build --parallel
cmake --install build
```

The install tree contains the headers, one static library per module, the
exported CMake targets, nlohmann/json's package files when it was fetched, and
Eigen's headers when Eigen was fetched (otherwise the system Eigen is used).

## Consume

```cmake
cmake_minimum_required(VERSION 3.24)
project(my_fleet LANGUAGES CXX)

find_package(MultiRobotCore 0.1 CONFIG REQUIRED)

add_executable(my_fleet main.cpp)
target_link_libraries(my_fleet PRIVATE MultiRobotCore::mapf MultiRobotCore::allocation)
```

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=$HOME/.local/multirobotcore
```

## Targets

| Target | Contents |
|---|---|
| `MultiRobotCore::core` | geometry, convex polygons, obstacles, robot models and state |
| `MultiRobotCore::graph` | graphs, Laplacians, connectivity, topologies |
| `MultiRobotCore::communication` | communication graphs, network simulator |
| `MultiRobotCore::consensus` | consensus, rendezvous, message passing, leader election |
| `MultiRobotCore::formation` | shapes, controllers, metrics, transitions |
| `MultiRobotCore::coverage` | Voronoi partition, densities, coverage control |
| `MultiRobotCore::allocation` | Hungarian, auction, dynamic allocation, SSI |
| `MultiRobotCore::mapf` | grids, space-time A*, prioritized planning, CBS |
| `MultiRobotCore::avoidance` | VO, RVO, ORCA |
| `MultiRobotCore::exploration` | occupancy grids, frontiers, exploration strategies |
| `MultiRobotCore::optimization` | distributed gradient descent, decentralised ADMM |
| `MultiRobotCore::simulation` | simulator, events |
| `MultiRobotCore::io` | JSON configuration and result helpers |
| `MultiRobotCore::scenarios` | formation mission, warehouse coordinator |
| `MultiRobotCore::multirobotcore` | everything |

`tests/package` is a complete downstream consumer; `scripts/ci/install_test.sh`
installs the library into a temporary prefix and builds it.
