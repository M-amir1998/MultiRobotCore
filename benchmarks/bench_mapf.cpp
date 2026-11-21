// Micro-benchmarks for the MAPF kernels: space-time A*, conflict detection,
// prioritized planning and Conflict-Based Search.
//
// mrc-modules: mapf

#include "multirobotcore/mapf/conflict_based_search.hpp"
#include "multirobotcore/mapf/conflicts.hpp"
#include "multirobotcore/mapf/constraint_table.hpp"
#include "multirobotcore/mapf/generators.hpp"
#include "multirobotcore/mapf/planners.hpp"
#include "multirobotcore/mapf/space_time_astar.hpp"

#include <benchmark/benchmark.h>

#include <random>

namespace mapf = multirobotcore::mapf;

namespace {

void BM_SpaceTimeAStarOpen(benchmark::State& state) {
  const auto n = static_cast<int>(state.range(0));
  const auto map = mapf::make_open_map(n, n);
  const mapf::Cell start{0, 0};
  const mapf::Cell goal{n - 1, n - 1};
  const auto h = map.distances_from(goal);
  const mapf::ConstraintTable none(map);
  for (auto _ : state) {
    mapf::LowLevelStats stats;
    auto path = mapf::space_time_astar(map, start, goal, none, h, stats);
    benchmark::DoNotOptimize(path);
  }
}
BENCHMARK(BM_SpaceTimeAStarOpen)->Arg(32)->Arg(64)->Arg(128);

void BM_SpaceTimeAStarWithReservations(benchmark::State& state) {
  std::mt19937_64 rng(7);
  const auto map = mapf::make_open_map(48, 48);
  const auto instance =
      mapf::random_instance(map, static_cast<std::size_t>(state.range(0)) + 1, rng);
  const auto others = mapf::plan_prioritized(instance);
  mapf::ConstraintTable reservations(map);
  for (std::size_t a = 1; a < others.paths.size(); ++a) {
    reservations.reserve_path(others.paths[a]);
  }
  const auto& agent = instance.agents.front();
  const auto h = map.distances_from(agent.goal);
  for (auto _ : state) {
    mapf::LowLevelStats stats;
    auto path = mapf::space_time_astar(map, agent.start, agent.goal, reservations, h, stats);
    benchmark::DoNotOptimize(path);
  }
}
BENCHMARK(BM_SpaceTimeAStarWithReservations)->Arg(10)->Arg(40);

void BM_ConflictDetection(benchmark::State& state) {
  std::mt19937_64 rng(3);
  const auto instance = mapf::random_instance(mapf::make_open_map(32, 32),
                                              static_cast<std::size_t>(state.range(0)), rng);
  const auto paths = mapf::plan_independent(instance).paths;
  for (auto _ : state) {
    benchmark::DoNotOptimize(mapf::count_conflicts(paths));
  }
}
BENCHMARK(BM_ConflictDetection)->Arg(20)->Arg(80);

mapf::Instance warehouse_instance(std::size_t agents) {
  const auto wh = mapf::make_warehouse({});
  std::mt19937_64 rng(11);
  return mapf::random_instance(wh.map, agents, wh.stations, wh.pickups, rng);
}

void BM_PrioritizedWarehouse(benchmark::State& state) {
  const auto instance = warehouse_instance(static_cast<std::size_t>(state.range(0)));
  for (auto _ : state) {
    benchmark::DoNotOptimize(mapf::plan_prioritized(instance));
  }
}
BENCHMARK(BM_PrioritizedWarehouse)->Arg(10)->Arg(20)->Unit(benchmark::kMillisecond);

void BM_CbsWarehouse(benchmark::State& state) {
  const auto instance = warehouse_instance(static_cast<std::size_t>(state.range(0)));
  const mapf::ConflictBasedSearch cbs;
  for (auto _ : state) {
    benchmark::DoNotOptimize(cbs.solve(instance));
  }
}
BENCHMARK(BM_CbsWarehouse)->Arg(4)->Arg(8)->Arg(12)->Unit(benchmark::kMillisecond);

}  // namespace
