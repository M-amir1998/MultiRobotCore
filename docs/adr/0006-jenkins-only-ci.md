# ADR 0006 — Jenkins-only CI

**Status:** accepted

**Context.** Hosted CI minutes are unavailable; the reference machine is a
16 GB CPU-only laptop.

**Decision.** A declarative `Jenkinsfile` for one Linux agent with
`disableConcurrentBuilds()`, bounded parallelism (`BUILD_JOBS`, default 4),
warnings-as-errors Debug and Release builds, all test suites, clang-format,
clang-tidy, Ruff, pytest, ASan+UBSan, demo smoke tests and an install test.
Benchmarks, documentation renders (`RENDER_DOCS=false` by default) and the
ROS 2 adapter are opt-in parameters. There is no `.github/workflows`.

**Consequences.** Normal builds never re-render animations. The pipeline
scripts (`scripts/ci`, `scripts/setup`) also run locally and in the Docker
verification (`scripts/ci/docker_linux.sh`).
