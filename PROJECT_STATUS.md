# Project status

State of MultiRobotCore 0.1.0 as of 2026-11-22. The per-algorithm matrix in
the README and in [docs/status.md](docs/status.md) is generated from
[configs/algorithms.yaml](configs/algorithms.yaml); this page records the
toolchain, the verification results and the known limitations.

## Summary

| Item | Value |
|---|---|
| Repository | <https://github.com/M-amir1998/MultiRobotCore> |
| Version | 0.1.0 ([CHANGELOG](CHANGELOG.md)) |
| License | MIT |
| C++ standard | C++20 |
| Build | CMake ≥ 3.24 with presets, Ninja |
| Compilers verified | MSVC 19.50 (Windows), clang-cl 20.1 (Windows, sanitizers), GCC 13.3 (Ubuntu 24.04), Clang 18.1 (Ubuntu 24.04, sanitizers) |
| Python | ≥ 3.11 (visualisation, reports, tests) |
| ROS 2 | Jazzy (optional adapter package) |

## Dependencies

| Dependency | Version | Use | Source |
|---|---|---|---|
| Eigen | 3.4.0 | linear algebra (required) | installed package, else pinned archive |
| nlohmann_json | 3.12.0 (installed ≥ 3.11 accepted) | configuration and results I/O (required) | installed package, else pinned archive |
| GoogleTest | 1.17.0 | tests | installed package, else pinned archive |
| Google Benchmark | 1.9.4 | micro-benchmarks | installed package, else pinned archive |
| OSQP | 1.0.0 | optional hook, unused in 0.1.0 | pinned archive, off by default |
| numpy, matplotlib, Pillow | see `pyproject.toml` | rendering and reports | pip |
| pytest, Ruff | see `pyproject.toml` (`dev` extra) | Python checks | pip |

Every fetched archive is pinned by SHA-256.

## Algorithms

| Area | Implemented |
|---|---|
| Graphs | weighted directed and undirected graphs with dynamic updates; adjacency, degree, Laplacian, normalised Laplacian, incidence, Metropolis and Perron matrices; connected components, Tarjan SCCs, spanning-tree roots, diameter, algebraic connectivity, Fiedler vector, Laplacian spectrum; complete, path, ring, star, directed ring/chain, grid and random topologies |
| Consensus | discrete-time (Perron matrix, ε < 1/d_max) and continuous-time (RK4) average consensus; weighted, leader-follower (pinning) and second-order consensus; rendezvous with an optional separation term; message-passing consensus over the network simulator; min-ID (FloodMin) leader election |
| Formation control | shape library; leader-follower, displacement-based (consensus) and distance-based controllers on Henneberg minimally rigid graphs; Procrustes-aligned formation metrics; assignment-aware transitions (minimum sum of squared distances, quintic blending); separation and obstacle terms; passage mission with multi-hop leader-state relay and leader re-election |
| Coverage | bounded Voronoi partition by half-plane clipping; uniform and Gaussian-mixture densities; cell quadrature; Lloyd iteration and gradient flow; limited-range variant (Cortés, Martínez & Bullo 2005) |
| Task allocation | Hungarian (shortest augmenting path, rectangular and infeasible pairs); Bertsekas auction (Gauss–Seidel and Jacobi); dynamic re-allocation with commitment and switching penalty; sequential single-item auctions; greedy baseline in the benchmarks |
| MAPF | 4-connected grid maps; space-time A* with constraint tables and goal parking; independent A*; prioritized planning with random restarts; Conflict-Based Search with cardinal-conflict prioritisation; map and instance generators; warehouse instances |
| Collision avoidance | sampling-based VO and RVO; ORCA (half-planes, incremental 2-D LP, 3-D fallback for infeasible programs); static obstacles; simulator adapter |
| Exploration | occupancy grid; ray-cast sensor; frontier detection and clustering; BFS distances and information gain; nearest-frontier, Hungarian and utility-discount assignment |
| Distributed optimisation | distributed gradient descent (Metropolis weights); decentralised consensus ADMM (Shi et al. 2014); bearing-only localisation example |
| Integration | warehouse coordinator (Hungarian on grid distances → prioritized planning with CBS fallback → timed waypoints → tracking → ORCA); ROS 2 Jazzy adapter (simulator backend, coordinator and ORCA safety nodes) |
| Not implemented | distributed MPC ([ADR 0008](docs/adr/0008-no-mpc-in-v0-1.md)) |

## Simulator and communication model

- **Simulator:** deterministic fixed-step simulator (time = step count × dt).
  Disc robots with single-integrator, double-integrator, unicycle or
  differential-drive models; speed, acceleration and yaw-rate limits. Contacts
  are detected and reported, not resolved. Stall detection; timed events for
  robot stop, failure and recovery, leader loss and link failures.
- **Communication:** fixed, disk, k-nearest, dropout and line-of-sight
  topologies; message-level network simulator with latency, jitter,
  independent loss, range limits, node and link outages, and traffic
  statistics.
- **Determinism:** bit-for-bit reproducible on one platform. The MAPF
  generators and the warehouse task generator use portable transforms of the
  raw `std::mt19937_64` output, so they match across compilers. Every planar
  sample draws its coordinates in an explicit order. The remaining components
  use the standard distributions, which can differ between standard libraries
  ([ADR 0007](docs/adr/0007-portable-sampling.md)).

## Benchmarks, motion graphics and README assets

- **Benchmark pages (8):** [consensus](docs/benchmarks/consensus.md),
  [formation](docs/benchmarks/formation.md),
  [allocation](docs/benchmarks/allocation.md), [MAPF](docs/benchmarks/mapf.md),
  [avoidance](docs/benchmarks/avoidance.md),
  [coverage](docs/benchmarks/coverage.md),
  [exploration](docs/benchmarks/exploration.md),
  [optimisation](docs/benchmarks/optimization.md). They are generated from
  the experiment executables by `scripts/benchmark/run_all.py`, plus Google
  Benchmark micro-benchmarks in `benchmarks/`. Runtimes come from one CPU-only
  laptop and are indicative.
- **Motion graphics:** 19 GIFs in `docs/assets/`, rendered headless from the
  JSON output of the C++ executables. The largest is
  `warehouse_hero.gif` at 4.2 MB, so every GIF is under the 5 MB budget. There
  are also 16 PNG plots. `docs/assets/manifest.json` records the scenario,
  algorithm, seed, configuration, script and source commit of each asset.
- **README assets:** a hero warehouse animation, a gallery of 8 animations,
  per-area figures, a benchmark plot and the generated status matrix.

## Verification

All results were measured at commit `a7bd7ad`, except the row marked †. That
row was measured at `4de5cf9`, which differs only by one `const auto` in the
simulator.

| Check | Environment | Result |
|---|---|---|
| Debug build, warnings as errors + tests | Windows, MSVC 19.50 | 217 / 217 passed |
| Release build, warnings as errors + tests | Windows, MSVC 19.50 | 217 / 217 passed |
| ASan + UBSan build + tests † | Windows, clang-cl 20.1 | 217 / 217 passed, no sanitizer reports |
| Debug build, warnings as errors + tests | Ubuntu 24.04 (Docker), GCC 13.3 | 217 / 217 passed |
| ASan + UBSan build + tests | Ubuntu 24.04 (Docker), Clang 18.1 | 217 / 217 passed, no sanitizer reports |
| ROS 2 adapter: colcon build + test | ROS 2 Jazzy (Docker), GCC 13.3 | 2 / 2 CTest targets, 8 results, 0 failures |
| Python tests | pytest | 8 / 8 passed |
| Python lint | Ruff 0.17 (`python`, `scripts`, `ros2`) | clean |
| Formatting | clang-format 20, `scripts/ci/check_format.sh` | 174 files clean |
| Static analysis | clang-tidy 20, `scripts/ci/run_clang_tidy.sh` | 51 sources, 0 findings |
| Demo smoke test | `scripts/ci/smoke_demos.sh` | 12 demos, valid JSON |
| Install + downstream `find_package` | `scripts/ci/install_test.sh` | passed |
| Secret scan | gitleaks | no leaks |

**Test count:** 217 C++ tests (194 unit, 7 numerical, 9 regression, 7
integration), 8 pytest tests, and the ROS 2 package tests. The ROS 2 tests are
a conversion unit-test executable and a launch test that runs the full node
graph, delivers 6 tasks without contacts and checks clean exit codes; colcon
reports 8 results. No test is skipped or disabled.

**Jenkins:** the [Jenkinsfile](Jenkinsfile) defines the pipeline (environment
validation, configure, debug build, unit, numerical, integration and MAPF
regression tests, release build, clang-format, clang-tidy, Ruff, pytest,
sanitizers, demo smoke tests, install test, ROS 2 adapter, benchmarks, asset
rendering). It has **not been run on a Jenkins controller**. Each stage calls
the scripts in `scripts/ci/`, and those were run locally with the results
above.

## Repository

- **Authors:** all commits are by the repository owner (M-amir1998).
- **Size:** about 31 MB packed, mostly the GIFs in `docs/assets/`.
- Build trees, dependency caches, raw frames and generated results are
  ignored (`.gitignore`).

## Known limitations

- **Consensus:** no guarantees for the stale-value policy under unbounded
  delays; rendezvous with separation is a heuristic.
- **Formation:** the distance-based law converges only locally and can settle
  in a reflected shape. Separation and obstacle terms carry no formal safety
  guarantee; clearances are measured, not certified.
- **Coverage:** local optima only; convex obstacle-free domains; robot size
  is not part of the objective.
- **Allocation:** the auction has no ε-scaling and no reverse auctions
  ([ADR 0005](docs/adr/0005-auction-without-epsilon-scaling.md)). SSI is a heuristic.
- **MAPF:** CBS times out on long single-lane corridor swaps (no
  symmetry-breaking constraints). There is no ECBS or other
  bounded-suboptimal variant. Prioritized planning is incomplete, with
  demonstrated failures. Grids are 4-connected with unit costs.
- **Avoidance:** VO and RVO are not collision-free. ORCA guarantees hold only
  for feasible programs. Symmetric configurations can be equilibria, and
  counterflow through doors can congest or deadlock.
- **Exploration:** centralised shared map, noise-free sensing, perfect
  localisation, occlusion-free gain estimate.
- **Distributed optimisation:** quadratic local costs only; synchronous,
  lossless rounds.
- **Warehouse:** holonomic robots, one global coordinator, full re-planning
  on events.
- **Simulator:** kinematic, not a physics engine; contacts are counted,
  not resolved.
- **Portability:** results that use standard-library distributions can differ
  between standard libraries.

## Future algorithms

- Distributed MPC with OSQP: centralised baseline, then a sequential
  distributed variant.
- Forward/reverse auctions with ε-scaling.
- ECBS, rolling-horizon MAPF and symmetry-breaking constraints for corridors.
- ORCA with polygon-edge obstacles; differential-drive ORCA.
- Asynchronous and lossy distributed optimisation.
- Nav2 and Gazebo adapters on top of the ROS 2 package.
