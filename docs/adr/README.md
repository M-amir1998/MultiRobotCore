# Architecture decision records

| ADR | Decision |
|---|---|
| [0001](0001-ros-independent-core.md) | ROS-independent C++20 core; ROS 2 as an optional adapter package |
| [0002](0002-dependency-policy.md) | Minimal dependencies: Eigen, nlohmann/json; installed packages first, pinned archives otherwise |
| [0003](0003-results-pipeline.md) | C++ is the source of truth; Python renders JSON results; no pybind11 |
| [0004](0004-voronoi-by-half-plane-clipping.md) | Bounded Voronoi cells by half-plane clipping instead of a general Voronoi library |
| [0005](0005-auction-without-epsilon-scaling.md) | Single-phase auction (no epsilon scaling) for the asymmetric formulation |
| [0006](0006-jenkins-only-ci.md) | Jenkins-only CI on a CPU-only agent; renders opt-in |
| [0007](0007-portable-sampling.md) | Portable sampling where results must match across compilers |
| [0008](0008-no-mpc-in-v0-1.md) | Distributed MPC deferred from v0.1 |
