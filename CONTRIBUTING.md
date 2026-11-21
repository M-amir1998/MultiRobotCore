# Contributing

Thank you for your interest in MultiRobotCore. The project values
mathematical correctness, reproducibility and honest reporting over feature
count.

## Workflow

1. Open an issue describing the algorithm or change, with references for any
   method you implement (no novelty claims for established methods).
2. Build and test locally:
   ```bash
   cmake --preset ci-debug && cmake --build --preset ci-debug && ctest --preset ci-debug
   cmake --preset ci-release && cmake --build --preset ci-release && ctest --preset ci-release
   ```
3. Keep the tree clean:
   ```bash
   scripts/ci/check_format.sh            # clang-format
   scripts/ci/run_clang_tidy.sh build/ci-debug
   .venv/bin/python -m ruff check python scripts && .venv/bin/python -m ruff format --check python scripts
   .venv/bin/python -m pytest
   python scripts/docs/generate_status.py --check
   ```
4. Open a pull request with a focused, conventional commit history
   (`feat(mapf): ...`, `fix(avoidance): ...`, `docs: ...`).

## Adding an algorithm

- Put the API in `include/multirobotcore/<module>/`, the implementation in
  `src/<module>/`; algorithms take and return plain values and must not depend
  on the simulator, visualisation or ROS.
- Tests in `tests/unit/<module>/` (properties, known instances, edge cases),
  plus `tests/numerical/<module>/` for accuracy and `tests/regression/<module>/`
  for small deterministic reference outputs. Validate optimality claims
  against an independent reference.
- Use explicit, seeded `std::mt19937_64` generators; no global random state.
- Add a demo in `apps/demos/` that writes JSON, a renderer in
  `scripts/render/`, an experiment in `apps/benchmarks/` when meaningful, a
  page in `docs/algorithms/` (purpose, formulation, mathematics, assumptions,
  algorithm, implementation notes, complexity, configuration, example,
  validation, benchmark, known limitations, references) and an entry in
  `configs/algorithms.yaml`.
- Document failure modes and limitations as carefully as successes.

## Coding standards

C++20, RAII, value semantics, `const` correctness, `std::span` / `std::optional`
where useful, no owning raw pointers, no global state, no unnecessary
inheritance. SI units; discrete MAPF units are documented separately. Solve
linear systems by factorisation, never by explicit inversion. Warnings are
errors in CI on MSVC, GCC and Clang.

## Assets

Animations and plots are generated from code
(`scripts/render/render_all.py`, `scripts/benchmark/run_all.py`). Do not commit
raw frames, videos, result dumps or build directories; keep README animations
below about 5 MB.
