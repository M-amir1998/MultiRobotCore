# ADR 0001 — ROS-independent core, optional ROS 2 adapter

**Status:** accepted

**Context.** The library must build, test, simulate and benchmark on a plain
CPU-only machine and be usable outside ROS (other middleware, offline
studies). Integration with ROS 2 is still valuable for deployment.

**Decision.** The core is a plain CMake/C++20 project with no ROS dependency.
ROS 2 support lives in `ros2/multirobotcore_ros`, an `ament_cmake` package
that consumes the installed core through `find_package(MultiRobotCore)`.
Algorithms expose plain-value interfaces; the ROS nodes only convert messages
and wrap `scenarios::WarehouseCoordinator`, `avoidance::CollisionAvoider` and
`simulation::Simulator`.

**Consequences.** ROS users build in two steps (install the core, then
`colcon build`), which is documented and scripted
(`scripts/ci/ros2_build.sh`). The ROS nodes and the offline demos run
identical algorithm code.
