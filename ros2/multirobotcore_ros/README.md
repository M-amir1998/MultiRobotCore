# multirobotcore_ros — optional ROS 2 adapter

ROS 2 nodes around the ROS-independent MultiRobotCore library. The core never
depends on ROS; this package consumes the installed core through
`find_package(MultiRobotCore)` and adds message conversions and nodes.

Verified on **ROS 2 Jazzy** (Ubuntu 24.04, GCC 13) with `colcon test`:
gtest conversion tests and an end-to-end `launch_testing` run in which the
three nodes deliver every task of a small fleet with zero robot or shelf
contacts.

## Nodes

| Executable | Role | Subscribes | Publishes |
|---|---|---|---|
| `fleet_simulator` | MultiRobotCore deterministic simulator as a robot backend (stand-in for Gazebo) | `robot_<i>/cmd_vel` (Twist) | `robot_<i>/odom` (Odometry), `/fleet/poses` (PoseArray), `/fleet/sim_status` (String, JSON), `/clock` |
| `warehouse_coordinator` | `scenarios::WarehouseCoordinator`: Hungarian allocation on grid travel costs, prioritized planning / CBS, trajectory tracking | `/fleet/poses`, `/fleet/task_requests` (PointStamped) | `robot_<i>/cmd_vel_ref` (Twist), `robot_<i>/plan` (Path with timed poses), `/fleet/markers` (MarkerArray), `/fleet/status` (String, JSON) |
| `orca_safety` | Generic reciprocal collision-avoidance filter (`avoidance::CollisionAvoider`, ORCA / RVO / VO) | `robot_<i>/odom`, `robot_<i>/cmd_vel_ref` | `robot_<i>/cmd_vel` |

Conventions: one fixed planar `map` frame; `cmd_vel*` carry planar
world-frame velocities of holonomic robots in `linear.x`, `linear.y`. The
safety filter is independent of the warehouse and can protect any fleet that
publishes odometry and reference velocities.

## Build

The core library is a plain CMake project; install it first, then build the
adapter with colcon:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S . -B build/ros2-core -DCMAKE_BUILD_TYPE=Release \
  -DMULTIROBOTCORE_BUILD_TESTS=OFF -DMULTIROBOTCORE_BUILD_APPS=OFF \
  -DMULTIROBOTCORE_BUILD_BENCHMARKS=OFF -DCMAKE_INSTALL_PREFIX=$PWD/build/ros2-core-install
cmake --build build/ros2-core --parallel 4 && cmake --install build/ros2-core
cd ros2 && colcon build --cmake-args -DCMAKE_PREFIX_PATH=$PWD/../build/ros2-core-install
```

`scripts/ci/ros2_build.sh` runs these steps plus `colcon test`;
`scripts/ci/docker_linux.sh ros2` runs them in a disposable `ros:jazzy-ros-base`
container (also on Windows hosts with Docker Desktop).

## Run

```bash
source ros2/install/setup.bash
ros2 launch multirobotcore_ros warehouse_demo.launch.py real_time_factor:=2.0
# request a pick close to a point (the coordinator picks the nearest free shelf access point)
ros2 topic pub --once /fleet/task_requests geometry_msgs/msg/PointStamped "{point: {x: 12.0, y: 6.0}}"
ros2 topic echo /fleet/status
```

Parameters live in `config/warehouse.yaml` (all under `/**`: ROS 2 gives
node-specific YAML entries precedence over wildcard entries regardless of
order, so a single namespace keeps launch-file overrides effective).

## Design notes

- **Event-driven control loop.** The coordinator computes references when a
  fleet pose message arrives; the safety filter computes as soon as it holds a
  fresh reference for every robot (a timer only stops robots whose references
  became stale). The loop simulator → coordinator → filter → simulator thus has
  a deterministic one-step latency. An earlier timer-driven version showed the
  cost of variable latency: tracking errors up to 1.4 m and shelf contacts.
- **Simulation time.** The simulator publishes `/clock` and can run faster
  than real time; the other nodes use `use_sim_time`.
- **Same algorithms as the core demos.** The coordinator wraps the same
  `WarehouseCoordinator` used by `mrc_demo_warehouse`, so ROS and offline runs
  exercise identical allocation, planning and tracking code.

## Limitations

- Holonomic velocity interface only; differential-drive robots would need the
  look-ahead controller of `models::track_velocity_lookahead` in a driver node.
- No TF broadcasting and no Nav2 integration (planned future adapters).
- The coordinator assumes all robots report poses in one message
  (`/fleet/poses`); per-robot pose aggregation is left to a deployment layer.
