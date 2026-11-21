# ADR 0003 — C++ is the source of truth; Python renders results

**Status:** accepted

**Context.** Animations and benchmark tables must reflect real algorithm
output, and reviewers must be able to regenerate them.

**Decision.** Demos and experiments are C++ executables that write
deterministic JSON (rounded values, ordered keys). Python
(`multirobotcore_viz`) only reads these files to render GIFs, plots and
markdown tables, and records provenance (scenario, algorithm, seed, script,
configuration, source commit) in `docs/assets/manifest.json`. pybind11
bindings were not added: the file interface already gives Python everything
it needs, keeps the C++ build free of Python and avoids a second API surface.

**Consequences.** Python never duplicates an algorithm. Rendering requires a
built tree (`python scripts/render/render_all.py --build-dir <build>`).
