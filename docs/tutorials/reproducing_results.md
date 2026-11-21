# Reproducing figures and benchmark tables

Every animation and table in the README and in `docs/` is generated from C++
output by scripts in this repository.

1. Build a Release tree: `cmake --preset ci-release && cmake --build --preset ci-release`.
2. Create the Python environment: `scripts/setup/bootstrap_python.sh`.
3. Render all animations:
   `.venv/bin/python scripts/render/render_all.py --build-dir build/ci-release`
   (single scripts: `--only warehouse,cbs`).
4. Re-run all experiments and regenerate `docs/benchmarks/*.md` and
   `docs/assets/plots/*.png`:
   `.venv/bin/python scripts/benchmark/run_all.py --build-dir build/ci-release`.

`docs/assets/manifest.json` records, for every asset, the scenario, the
algorithm, the seed, the configuration file, the generating script and the
commit it was rendered from. Configurations live in `configs/`; scenario maps
in `scenarios/`.

Determinism: given a configuration, a seed and a commit, results are
reproducible on the same platform. Components that use the standard library's
random distributions can differ between compilers (see
[ADR 0007](../adr/0007-portable-sampling.md)); the warehouse mission and the
MAPF generators are portable. Runtimes in benchmark tables are wall-clock
measurements on the machine named in each page and are indicative only.

Jenkins re-renders assets only when the `RENDER_DOCS` parameter is set.
