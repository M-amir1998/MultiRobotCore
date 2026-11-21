# Getting started

## Requirements

- CMake ≥ 3.24 and Ninja (or another generator)
- A C++20 compiler: GCC ≥ 12, Clang ≥ 15 or MSVC 19.3x (Visual Studio 2022/2026)
- Python ≥ 3.11 for renderers, reports and Python tests (optional)
- Network access for the first configure only (pinned dependency archives
  are cached in `.deps/downloads`); no GPU, no ROS

## Build and test

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build
```

or with presets:

```bash
cmake --preset ci-release          # warnings as errors
cmake --build --preset ci-release
ctest --preset ci-release
ctest --test-dir build/ci-release -L integration   # one category: unit, numerical, regression, integration
```

On Windows run the commands from a Visual Studio developer prompt (MSVC); the
`sanitize-windows` preset uses clang-cl with ASan and UBSan, `sanitize` uses
clang++ on Linux.

## Run a demo

Every demo writes a deterministic JSON result:

```bash
./build/apps/mrc_demo_warehouse            # -> results/warehouse.json
./build/apps/mrc_demo_formation --output /tmp/formation.json
./build/apps/mrc_demo_consensus --config configs/consensus/consensus.json
```

Demos: `consensus`, `formation`, `allocation`, `distributed_optimization`,
`mapf_conflicts`, `mapf_warehouse`, `avoidance_circle_swap`,
`avoidance_bottleneck`, `avoidance_velocity_space`, `coverage`, `exploration`,
`warehouse`.

## Render an animation

```bash
scripts/setup/bootstrap_python.sh          # creates .venv with the viz package
.venv/bin/python scripts/render/render_warehouse.py --build-dir build
.venv/bin/python scripts/render/render_all.py --build-dir build   # everything
```

## Run the benchmarks

```bash
./build/benchmarks/mrc_bench_mapf --benchmark_min_time=0.2s     # micro-benchmarks
.venv/bin/python scripts/benchmark/run_all.py --build-dir build # experiments + pages
```

## Use the library

```cpp
#include <multirobotcore/allocation/hungarian.hpp>
#include <multirobotcore/mapf/conflict_based_search.hpp>

namespace mrc = multirobotcore;

mrc::allocation::CostMatrix costs(2, 2);
costs << 4.0, 1.0,
         2.0, 5.0;
const auto assignment = mrc::allocation::solve_hungarian(costs);   // cost 3

const auto map = mrc::mapf::GridMap::from_ascii({"....", "@.@@"});
const mrc::mapf::Instance swap{map, {{{0, 1}, {3, 1}}, {{3, 1}, {0, 1}}}};
const auto plan = mrc::mapf::ConflictBasedSearch().solve(swap);    // one robot yields into the bay
```

Next: [downstream usage](downstream_usage.md), [architecture](../architecture/overview.md),
[ROS 2 adapter](../architecture/ros2.md).
