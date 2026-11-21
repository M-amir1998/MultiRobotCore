#include "multirobotcore/mapf/planners.hpp"

#include "multirobotcore/mapf/conflicts.hpp"
#include "multirobotcore/mapf/constraint_table.hpp"
#include "multirobotcore/mapf/space_time_astar.hpp"

#include "deterministic_random.hpp"

#include <algorithm>
#include <chrono>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>

namespace multirobotcore::mapf {
namespace {

double elapsed_ms(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
      .count();
}

void finalise_costs(PlannerResult& result) {
  result.sum_of_costs = sum_of_costs(result.paths);
  result.makespan = makespan(result.paths);
  result.conflicts = count_conflicts(result.paths);
}

std::vector<std::vector<int>> goal_heuristics(const Instance& instance) {
  std::vector<std::vector<int>> h;
  h.reserve(instance.agents.size());
  for (const Agent& a : instance.agents) {
    h.push_back(instance.map.distances_from(a.goal));
  }
  return h;
}

std::vector<std::size_t> initial_order(const Instance& instance,
                                       const std::vector<std::vector<int>>& h,
                                       const PrioritizedOptions& options, std::mt19937_64& rng) {
  const std::size_t n = instance.agents.size();
  std::vector<std::size_t> order(n);
  std::iota(order.begin(), order.end(), std::size_t{0});
  auto distance = [&](std::size_t a) {
    return h[a][static_cast<std::size_t>(instance.map.index(instance.agents[a].start))];
  };
  switch (options.order) {
    case PriorityOrder::kIndex:
      break;
    case PriorityOrder::kShortestFirst:
      std::stable_sort(order.begin(), order.end(),
                       [&](std::size_t a, std::size_t b) { return distance(a) < distance(b); });
      break;
    case PriorityOrder::kLongestFirst:
      std::stable_sort(order.begin(), order.end(),
                       [&](std::size_t a, std::size_t b) { return distance(a) > distance(b); });
      break;
    case PriorityOrder::kRandom:
      detail::shuffle(order, rng);
      break;
    case PriorityOrder::kCustom: {
      std::vector<std::size_t> sorted = options.custom_order;
      std::sort(sorted.begin(), sorted.end());
      if (sorted != order) {
        throw std::invalid_argument("plan_prioritized: custom order must be a permutation");
      }
      order = options.custom_order;
      break;
    }
  }
  return order;
}

}  // namespace

std::string_view to_string(PlannerStatus status) noexcept {
  switch (status) {
    case PlannerStatus::kSolved:
      return "solved";
    case PlannerStatus::kUnsafe:
      return "unsafe";
    case PlannerStatus::kFailed:
      return "failed";
    case PlannerStatus::kNoSolution:
      return "no_solution";
    case PlannerStatus::kTimeout:
      return "timeout";
    case PlannerStatus::kNodeLimit:
      return "node_limit";
  }
  return "unknown";
}

PlannerResult plan_independent(const Instance& instance) {
  validate_instance(instance);
  const auto start = std::chrono::steady_clock::now();
  PlannerResult result;
  const ConstraintTable none(instance.map);
  LowLevelStats stats;
  for (std::size_t a = 0; a < instance.agents.size(); ++a) {
    const auto h = instance.map.distances_from(instance.agents[a].goal);
    auto path = space_time_astar(instance.map, instance.agents[a].start, instance.agents[a].goal,
                                 none, h, stats);
    if (!path) {
      result.status = PlannerStatus::kNoSolution;
      result.message = "agent " + std::to_string(a) + " cannot reach its goal";
      result.paths.clear();
      result.low_level_expansions = stats.expansions;
      result.runtime_ms = elapsed_ms(start);
      return result;
    }
    result.paths.push_back(std::move(*path));
  }
  finalise_costs(result);
  result.status = result.conflicts == 0 ? PlannerStatus::kSolved : PlannerStatus::kUnsafe;
  result.low_level_expansions = stats.expansions;
  result.runtime_ms = elapsed_ms(start);
  return result;
}

PlannerResult plan_prioritized(const Instance& instance, const PrioritizedOptions& options) {
  validate_instance(instance);
  const auto start = std::chrono::steady_clock::now();
  const std::size_t n = instance.agents.size();
  const auto h = goal_heuristics(instance);
  std::mt19937_64 rng(options.seed);
  PlannerResult result;
  LowLevelStats stats;
  std::vector<std::size_t> order = initial_order(instance, h, options, rng);
  const std::size_t attempts = std::max<std::size_t>(options.max_attempts, 1);
  std::string last_failure;

  for (std::size_t attempt = 0; attempt < attempts; ++attempt) {
    if (attempt > 0) {
      detail::shuffle(order, rng);
    }
    ConstraintTable reservations(instance.map);
    std::vector<Path> paths(n);
    bool ok = true;
    for (const std::size_t a : order) {
      auto path = space_time_astar(instance.map, instance.agents[a].start, instance.agents[a].goal,
                                   reservations, h[a], stats);
      if (!path) {
        ok = false;
        last_failure = "agent " + std::to_string(a) + " found no path given higher priorities";
        break;
      }
      reservations.reserve_path(*path);
      paths[a] = std::move(*path);
    }
    result.attempts = attempt + 1;
    if (ok) {
      result.paths = std::move(paths);
      result.order = order;
      finalise_costs(result);
      result.status = result.conflicts == 0 ? PlannerStatus::kSolved : PlannerStatus::kUnsafe;
      result.low_level_expansions = stats.expansions;
      result.runtime_ms = elapsed_ms(start);
      return result;
    }
  }
  result.status = PlannerStatus::kFailed;
  result.order = order;
  result.message = last_failure;
  result.low_level_expansions = stats.expansions;
  result.runtime_ms = elapsed_ms(start);
  return result;
}

}  // namespace multirobotcore::mapf
