# Simulation

Module: `MultiRobotCore::simulation` — headers `simulation/simulator.hpp`,
`simulation/events.hpp`.

## Purpose and scope

A lightweight, deterministic, CPU-only multi-robot simulator for algorithm
testing, benchmarking, integration and figures. It is not a physics engine and
does not replace Gazebo, Webots or Isaac Sim.

## Model

- **Robots** are discs with a radius, speed and acceleration limits. Each step
  the caller passes one desired planar velocity per robot — the common output
  of the coordination, formation, coverage and avoidance controllers — and the
  robot's dynamics model turns it into motion:
  - single integrator (optionally acceleration-limited),
  - double integrator with velocity tracking `a = sat(k (v_des - v))` and exact
    zero-order-hold integration,
  - unicycle and differential drive via look-ahead-point feedback
    linearisation and exact arc integration (wheel-speed saturation preserves
    curvature).
- **World**: optional rectangular bounds, static discs and boxes; obstacles can
  appear at runtime.
- **Status**: active, idle, finished, stopped (no motion, still communicates)
  and failed (no motion, no communication). Stopped and failed robots remain
  physical obstacles.

## Measurements

Collisions are detected and counted as **contact events** (a pair entering
contact counts once until it separates) but not resolved: overlapping robots
pass through each other, so a failure of the algorithm under test stays
visible. Metrics: robot–robot and robot–obstacle contact events, minimum pair
clearance, minimum obstacle clearance, distance travelled and control effort
per robot, and a stall flag (every active robot moved less than
`min_progress` metres over `window` seconds — a heuristic indicator, not a
formal deadlock proof).

## Determinism

Time is `step_count * dt`, never accumulated. Given a configuration, a seed
and a commit, every run is bit-for-bit reproducible on one platform
(`tests/unit/simulation` and the integration tests check this). Across
platforms, results that use the standard library's random distributions may
differ; the warehouse mission and MAPF generators use portable sampling.

## Events and failure injection

`EventSchedule` releases timed events in `(time, insertion)` order;
`apply_event` applies robot stop/failure/recovery, leader loss, link failures
with durations (through the network simulator) and appearing obstacles. Task
arrivals and cancellations are left to the scenario that owns the tasks.

## Recording

With `record = true` the simulator stores frames (time, poses, velocities,
status) every `record_every` steps; `io::frames_to_json` writes them in a
compact column-oriented form with rounded values, so identical runs produce
identical files.
