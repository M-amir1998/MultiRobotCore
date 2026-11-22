#include "multirobotcore/exploration/planner.hpp"

#include "multirobotcore/allocation/hungarian.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace multirobotcore::exploration {
namespace {

struct Candidate {
  GridCell cell;
  std::size_t cluster;
  double gain;
};

/// Nearest reachable candidate for one robot (ties: larger gain, then index).
std::optional<std::size_t> nearest(const std::vector<int>& dist, const OccupancyGrid& map,
                                   const std::vector<Candidate>& candidates) {
  std::optional<std::size_t> best;
  for (std::size_t t = 0; t < candidates.size(); ++t) {
    const int d = dist[map.index(candidates[t].cell)];
    if (d == kUnreachable) {
      continue;
    }
    if (!best) {
      best = t;
      continue;
    }
    const int db = dist[map.index(candidates[*best].cell)];
    if (d < db || (d == db && candidates[t].gain > candidates[*best].gain)) {
      best = t;
    }
  }
  return best;
}

}  // namespace

std::string_view to_string(Strategy strategy) noexcept {
  switch (strategy) {
    case Strategy::kNearestFrontier:
      return "nearest_frontier";
    case Strategy::kHungarian:
      return "hungarian";
    case Strategy::kUtilityDiscount:
      return "utility_discount";
  }
  return "unknown";
}

std::optional<Strategy> strategy_from_string(std::string_view name) noexcept {
  for (const Strategy s :
       {Strategy::kNearestFrontier, Strategy::kHungarian, Strategy::kUtilityDiscount}) {
    if (to_string(s) == name) {
      return s;
    }
  }
  return std::nullopt;
}

TargetAssignment assign_targets(const OccupancyGrid& map, std::span<const GridCell> robots,
                                std::span<const FrontierCluster> clusters,
                                const PlannerConfig& config, double sensor_radius_cells) {
  const std::size_t n = robots.size();
  TargetAssignment out;
  out.targets.assign(n, std::nullopt);
  out.clusters.assign(n, std::nullopt);
  if (n == 0 || clusters.empty()) {
    return out;
  }
  std::vector<Candidate> candidates;
  candidates.reserve(clusters.size());
  for (std::size_t k = 0; k < clusters.size(); ++k) {
    const GridCell c = clusters[k].representative;
    candidates.push_back(
        {c, k, static_cast<double>(information_gain(map, c, sensor_radius_cells))});
  }
  std::vector<std::vector<int>> dist;
  dist.reserve(n);
  for (const GridCell& r : robots) {
    dist.push_back(bfs_distances(map, r));
  }
  auto assign = [&](std::size_t robot, std::size_t t) {
    out.targets[robot] = candidates[t].cell;
    out.clusters[robot] = candidates[t].cluster;
  };

  switch (config.strategy) {
    case Strategy::kNearestFrontier: {
      for (std::size_t i = 0; i < n; ++i) {
        if (const auto t = nearest(dist[i], map, candidates)) {
          assign(i, *t);
        }
      }
      break;
    }
    case Strategy::kHungarian: {
      allocation::CostMatrix costs(static_cast<Eigen::Index>(n),
                                   static_cast<Eigen::Index>(candidates.size()));
      for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t t = 0; t < candidates.size(); ++t) {
          const int d = dist[i][map.index(candidates[t].cell)];
          costs(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(t)) =
              d == kUnreachable
                  ? allocation::kInfeasible
                  : config.cost_weight * d - config.information_weight * candidates[t].gain;
        }
      }
      const allocation::Assignment a = allocation::solve_hungarian(costs);
      for (std::size_t i = 0; i < n; ++i) {
        if (a.robot_to_task[i]) {
          assign(i, *a.robot_to_task[i]);
        } else if (const auto t = nearest(dist[i], map, candidates)) {
          assign(i, *t);  // more robots than frontiers: share the closest one
        }
      }
      break;
    }
    case Strategy::kUtilityDiscount: {
      double max_gain = 0.0;
      double max_cost = 0.0;
      for (const auto& candidate : candidates) {
        max_gain = std::max(max_gain, candidate.gain);
        for (std::size_t i = 0; i < n; ++i) {
          max_cost = std::max(max_cost, static_cast<double>(dist[i][map.index(candidate.cell)]));
        }
      }
      std::vector<double> utility(candidates.size(), 1.0);
      if (max_gain > 0.0) {
        for (std::size_t t = 0; t < candidates.size(); ++t) {
          utility[t] = candidates[t].gain / max_gain;
        }
      }
      const double cost_scale = max_cost > 0.0 ? 1.0 / max_cost : 1.0;
      std::vector<char> done(n, 0);
      for (std::size_t round = 0; round < n; ++round) {
        double best_value = -std::numeric_limits<double>::infinity();
        std::optional<std::pair<std::size_t, std::size_t>> best;
        for (std::size_t i = 0; i < n; ++i) {
          if (done[i] != 0) {
            continue;
          }
          for (std::size_t t = 0; t < candidates.size(); ++t) {
            const int d = dist[i][map.index(candidates[t].cell)];
            if (d == kUnreachable) {
              continue;
            }
            const double value = utility[t] - config.cost_weight * cost_scale * d;
            if (value > best_value) {
              best_value = value;
              best = std::pair{i, t};
            }
          }
        }
        if (!best) {
          break;  // remaining robots cannot reach any frontier
        }
        const auto [robot, t] = *best;
        assign(robot, t);
        done[robot] = 1;
        for (std::size_t u = 0; u < candidates.size(); ++u) {
          const double d = std::hypot(candidates[u].cell.x - candidates[t].cell.x,
                                      candidates[u].cell.y - candidates[t].cell.y);
          utility[u] -= std::max(0.0, 1.0 - d / config.discount_radius);
        }
      }
      break;
    }
  }
  return out;
}

}  // namespace multirobotcore::exploration
