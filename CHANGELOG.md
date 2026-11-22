# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

## [0.1.0] — 2026-11-22

First public release.

### Added

- **Foundations:** planar geometry, convex polygons with half-plane clipping,
  obstacle sets; single/double integrator, unicycle, differential-drive and
  bicycle models with exact discretisation; typed robot state.
- **Graphs:** weighted directed/undirected graphs with dynamic updates;
  adjacency, degree, Laplacian, normalised Laplacian, incidence, Metropolis and
  Perron matrices; components, Tarjan SCCs, spanning-tree roots, diameter,
  algebraic connectivity, Fiedler vector; standard topologies.
- **Communication:** fixed, disk, k-nearest, dropout and line-of-sight
  topologies; message-level network simulator (latency, jitter, loss, range,
  node/link outages, statistics).
- **Consensus:** discrete/continuous, weighted, leader-follower, second-order,
  rendezvous (with separation variant), message-passing consensus, min-ID
  leader election.
- **Formation:** shapes, Henneberg rigid graphs, leader-follower, consensus
  and distance-based controllers, metrics, assignment-aware transitions,
  safety terms; integrated passage mission with leader failure.
- **Coverage:** bounded Voronoi partitions, densities, quadrature, Lloyd and
  gradient coverage control, limited-range variant.
- **Task allocation:** Hungarian, auction (Gauss–Seidel/Jacobi), dynamic
  re-allocation, sequential single-item auctions.
- **MAPF:** grid maps, space-time A*, independent A*, prioritized planning,
  CBS with cardinal-conflict prioritisation, generators.
- **Collision avoidance:** velocity obstacles, sampling-based VO/RVO, ORCA.
- **Exploration:** occupancy grids, ray-cast sensing, frontiers, nearest,
  Hungarian and utility-discount strategies.
- **Distributed optimisation:** distributed gradient descent, decentralised
  consensus ADMM, bearing-only localisation example.
- **Simulation:** deterministic simulator, contact metrics, stall detection,
  event-based failure injection; portable sampling helpers
  (`math/random.hpp`) with an explicit draw order.
- **Integration:** warehouse coordinator and mission (allocation + MAPF +
  tracking + ORCA); optional ROS 2 Jazzy adapter with simulator, coordinator
  and ORCA safety nodes.
- **Tooling:** CMake presets, install/export, Jenkins pipeline, sanitizers,
  clang-tidy/clang-format, Ruff, pytest; Python visualisation package,
  reproducible renders, experiments and benchmark reports.

### Not included

- Distributed MPC (see `docs/adr/0008-no-mpc-in-v0-1.md`).
