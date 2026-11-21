#include "multirobotcore/exploration/frontier.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>

namespace multirobotcore::exploration {
namespace {

constexpr std::array<GridCell, 4> kFour{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
constexpr std::array<GridCell, 8> kEight{
    {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};

GridCell add(const GridCell& a, const GridCell& b) noexcept {
  return {a.x + b.x, a.y + b.y};
}

void require_in_bounds(const OccupancyGrid& map, const GridCell& c, const char* what) {
  if (!map.in_bounds(c)) {
    throw std::out_of_range(std::string(what) + ": cell out of bounds");
  }
}

}  // namespace

bool is_frontier(const OccupancyGrid& map, const GridCell& cell) noexcept {
  if (!map.in_bounds(cell) || map.at(cell) != Occupancy::kFree) {
    return false;
  }
  return std::any_of(kFour.begin(), kFour.end(), [&](const GridCell& d) {
    const GridCell n = add(cell, d);
    return map.in_bounds(n) && map.at(n) == Occupancy::kUnknown;
  });
}

std::vector<GridCell> detect_frontiers(const OccupancyGrid& map) {
  std::vector<GridCell> out;
  for (std::size_t i = 0; i < map.size(); ++i) {
    const GridCell c = map.cell(i);
    if (is_frontier(map, c)) {
      out.push_back(c);
    }
  }
  return out;
}

std::vector<FrontierCluster> cluster_frontiers(const OccupancyGrid& map,
                                               std::span<const GridCell> frontiers,
                                               std::size_t min_size) {
  std::vector<char> is_member(map.size(), 0);
  for (const GridCell& c : frontiers) {
    require_in_bounds(map, c, "cluster_frontiers");
    is_member[map.index(c)] = 1;
  }
  std::vector<char> visited(map.size(), 0);
  std::vector<FrontierCluster> clusters;
  for (std::size_t i = 0; i < map.size(); ++i) {
    if (is_member[i] == 0 || visited[i] != 0) {
      continue;
    }
    FrontierCluster cluster;
    std::deque<GridCell> queue{map.cell(i)};
    visited[i] = 1;
    while (!queue.empty()) {
      const GridCell c = queue.front();
      queue.pop_front();
      cluster.cells.push_back(c);
      for (const GridCell& d : kEight) {
        const GridCell n = add(c, d);
        if (map.in_bounds(n) && is_member[map.index(n)] != 0 && visited[map.index(n)] == 0) {
          visited[map.index(n)] = 1;
          queue.push_back(n);
        }
      }
    }
    if (cluster.cells.size() < min_size) {
      continue;
    }
    std::sort(cluster.cells.begin(), cluster.cells.end(),
              [&](const GridCell& a, const GridCell& b) { return map.index(a) < map.index(b); });
    Vec2 sum = Vec2::Zero();
    for (const GridCell& c : cluster.cells) {
      sum += Vec2(c.x, c.y);
    }
    cluster.centroid = sum / static_cast<double>(cluster.cells.size());
    double best = std::numeric_limits<double>::infinity();
    for (const GridCell& c : cluster.cells) {
      const double d = (Vec2(c.x, c.y) - cluster.centroid).squaredNorm();
      if (d < best) {
        best = d;
        cluster.representative = c;
      }
    }
    clusters.push_back(std::move(cluster));
  }
  return clusters;
}

std::vector<int> bfs_distances(const OccupancyGrid& map, const GridCell& start) {
  require_in_bounds(map, start, "bfs_distances");
  std::vector<int> dist(map.size(), kUnreachable);
  std::deque<GridCell> queue{start};
  dist[map.index(start)] = 0;
  while (!queue.empty()) {
    const GridCell c = queue.front();
    queue.pop_front();
    for (const GridCell& d : kFour) {
      const GridCell n = add(c, d);
      if (map.in_bounds(n) && map.at(n) == Occupancy::kFree && dist[map.index(n)] == kUnreachable) {
        dist[map.index(n)] = dist[map.index(c)] + 1;
        queue.push_back(n);
      }
    }
  }
  return dist;
}

std::vector<GridCell> shortest_path(const OccupancyGrid& map, const GridCell& start,
                                    const GridCell& goal) {
  require_in_bounds(map, start, "shortest_path");
  require_in_bounds(map, goal, "shortest_path");
  constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> parent(map.size(), kNone);
  std::vector<char> visited(map.size(), 0);
  std::deque<GridCell> queue{start};
  visited[map.index(start)] = 1;
  bool found = start == goal;
  while (!queue.empty() && !found) {
    const GridCell c = queue.front();
    queue.pop_front();
    for (const GridCell& d : kFour) {
      const GridCell n = add(c, d);
      if (map.in_bounds(n) && map.at(n) == Occupancy::kFree && visited[map.index(n)] == 0) {
        visited[map.index(n)] = 1;
        parent[map.index(n)] = map.index(c);
        if (n == goal) {
          found = true;
          break;
        }
        queue.push_back(n);
      }
    }
  }
  if (!found) {
    return {};
  }
  std::vector<GridCell> path{goal};
  std::size_t current = map.index(goal);
  while (current != map.index(start)) {
    current = parent[current];
    path.push_back(map.cell(current));
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::size_t information_gain(const OccupancyGrid& map, const GridCell& target,
                             double radius_cells) {
  const int r = static_cast<int>(std::ceil(radius_cells));
  const double r2 = radius_cells * radius_cells;
  std::size_t gain = 0;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (static_cast<double>(dx * dx + dy * dy) > r2) {
        continue;
      }
      const GridCell c{target.x + dx, target.y + dy};
      if (map.in_bounds(c) && map.at(c) == Occupancy::kUnknown) {
        ++gain;
      }
    }
  }
  return gain;
}

}  // namespace multirobotcore::exploration
