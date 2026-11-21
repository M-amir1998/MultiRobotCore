# Architecture

## Layers

```mermaid
flowchart TB
  subgraph Foundations
    core["core<br/>math, geometry, convex polygons,<br/>obstacles, robot models, robot state"]
    graph["graph<br/>graphs, Laplacians, connectivity"]
    comm["communication<br/>topologies, network simulator"]
  end
  subgraph Algorithms
    consensus["consensus<br/>consensus, rendezvous, leader election"]
    formation["formation<br/>shapes, controllers, transitions"]
    coverage["coverage<br/>Voronoi cells, Lloyd / CVT"]
    allocation["allocation<br/>Hungarian, auction, dynamic, SSI"]
    mapf["mapf<br/>space-time A*, PP, CBS"]
    avoidance["avoidance<br/>VO, RVO, ORCA"]
    exploration["exploration<br/>occupancy, frontiers"]
    optimization["optimization<br/>DGD, decentralised ADMM"]
  end
  subgraph Environment
    simulation["simulation<br/>deterministic simulator, events"]
    io["io<br/>JSON configs and results"]
  end
  scenarios["scenarios<br/>formation mission, warehouse coordinator"]
  apps["apps: demos + experiments"]
  ros2["ros2/multirobotcore_ros (optional)"]
  python["python/multirobotcore_viz<br/>renderers, plots, reports"]

  core --> graph --> comm
  graph --> consensus
  comm --> consensus
  graph --> formation
  allocation -.-> formation
  core --> coverage
  core --> allocation
  core --> mapf
  core --> avoidance
  allocation -.-> exploration
  graph --> optimization
  comm --> simulation
  simulation --> io
  consensus & formation & mapf & allocation & avoidance & simulation & io --> scenarios
  scenarios --> apps
  scenarios --> ros2
  apps -- "JSON results" --> python
```

Dashed arrows are private (implementation) dependencies. Every module is a
separate CMake target `MultiRobotCore::<module>`; `MultiRobotCore::multirobotcore`
links all of them.

## Design rules

- **Algorithms do not know the simulator.** Controllers and planners take
  plain values (positions, graphs, cost matrices, grids) and return plain
  values (velocities, assignments, paths). The simulator, the scenarios and
  the ROS 2 nodes consume these interfaces; nothing in an algorithm module
  depends on simulation, visualisation or middleware.
- **Value semantics.** Results are returned by value; no owning raw pointers,
  no global state, no singletons. Graphs, plans and assignments are regular
  types with equality.
- **Explicit randomness.** Every stochastic component takes a seeded
  `std::mt19937_64`. Scenario-level sampling that must be identical across
  compilers (warehouse tasks, MAPF generators) uses explicit transforms of the
  generator's output, because the standard distributions are
  implementation-defined.
- **Errors.** Invalid inputs (size mismatches, NaN costs, singular systems,
  impossible configurations) throw `std::invalid_argument` /
  `std::runtime_error`; algorithmic outcomes that are not errors (an
  unsolvable MAPF instance, a timed-out search, an infeasible ORCA program) are
  reported in result structs.
- **Linear algebra.** Eigen; linear systems are solved by factorisation
  (LDLT), never by explicit inversion.

## Units and time

SI units throughout: metres, seconds, radians, metres per second. Four time
notions are kept distinct:

| Notion | Where | Representation |
|---|---|---|
| Continuous simulation time | simulator, scenarios | `double` seconds = `step_count * dt` |
| Control step | controllers | `dt` [s] (default 0.05 s) |
| Communication round | network simulator, consensus, optimisation | integer counter, `round_period` [s] |
| MAPF timestep | `mapf` | integer step; one cell move or wait per step; converted to seconds with `step_duration` |

MAPF grids use integer cell coordinates (`Cell{x, y}`, `y = 0` at the bottom);
`WorldFrame` converts cells to metres.

## Data flow for figures and tables

```text
C++ demo / experiment (configs/*.json, seed)
        |  deterministic JSON result (results/, not committed)
        v
python/multirobotcore_viz + scripts/render, scripts/benchmark
        |
        v
docs/assets/*.gif, docs/assets/plots/*.png, docs/benchmarks/*.md
docs/assets/manifest.json records scenario, algorithm, seed, script, config and source commit
```

Python never re-implements an algorithm; it only reads results.

## Build and packaging

- CMake ≥ 3.24, C++20, presets for Debug/Release, warnings-as-errors CI
  builds and ASan/UBSan (clang++ / clang-cl).
- Dependencies: Eigen 3.4 and nlohmann/json (installed packages first,
  otherwise pinned SHA-256-verified archives cached in `.deps/downloads`);
  GoogleTest and Google Benchmark for tests and benchmarks.
- `cmake --install` exports all targets; `find_package(MultiRobotCore CONFIG)`
  works for downstream projects (checked by `tests/package`).

See also: [simulation](simulation.md), [ROS 2 adapter](ros2.md),
[architecture decisions](../adr/README.md).
