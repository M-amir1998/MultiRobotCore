# ADR 0002 — Dependency policy

**Status:** accepted

**Context.** The library must build offline on a laptop, inside ROS
workspaces and on Windows without a package manager.

**Decision.** Runtime dependencies are Eigen 3.4 (linear algebra) and
nlohmann/json (configuration and result files). GoogleTest and Google
Benchmark are test-only. CMake first tries installed packages
(`find_package`), otherwise it downloads pinned release archives verified by
SHA-256 into `.deps/downloads`, shared by all build trees; each tree extracts
and builds its own copy (sharing one build directory across configurations
corrupted the GoogleTest build). No computational-geometry, graph or
optimisation library is used: the needed kernels are small and fully tested.

**Consequences.** After the first download every configuration builds
offline. The installed package ships Eigen headers when Eigen was fetched, so
downstream builds are self-contained.
