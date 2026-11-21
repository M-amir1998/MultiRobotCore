#include "multirobotcore/mapf/constraint_table.hpp"

#include <algorithm>
#include <stdexcept>

namespace multirobotcore::mapf {

namespace detail {

std::size_t TimedMoveHash::operator()(const TimedMove& m) const noexcept {
  std::uint64_t h = static_cast<std::uint32_t>(m.from);
  h = h * 0x9E3779B97F4A7C15ULL ^ static_cast<std::uint32_t>(m.to);
  h = h * 0x9E3779B97F4A7C15ULL ^ static_cast<std::uint32_t>(m.t);
  return static_cast<std::size_t>(h ^ (h >> 29U));
}

}  // namespace detail

ConstraintTable::ConstraintTable(const GridMap& map) : map_(&map) {}

void ConstraintTable::add_vertex(Cell cell, int t) {
  if (t < 0) {
    throw std::invalid_argument("ConstraintTable: negative time");
  }
  const int idx = map_->index(cell);
  vertex_.insert(detail::vertex_key(idx, t));
  auto [it, inserted] = latest_vertex_.try_emplace(idx, t);
  if (!inserted) {
    it->second = std::max(it->second, t);
  }
  max_time_ = std::max(max_time_, t);
}

void ConstraintTable::add_edge(Cell from, Cell to, int t) {
  if (t < 0) {
    throw std::invalid_argument("ConstraintTable: negative time");
  }
  edge_.insert(detail::TimedMove{map_->index(from), map_->index(to), t});
  max_time_ = std::max(max_time_, t);
}

void ConstraintTable::add_permanent(Cell cell, int t) {
  if (t < 0) {
    throw std::invalid_argument("ConstraintTable: negative time");
  }
  const int idx = map_->index(cell);
  auto [it, inserted] = permanent_.try_emplace(idx, t);
  if (!inserted) {
    it->second = std::min(it->second, t);
  }
  max_time_ = std::max(max_time_, t);
}

void ConstraintTable::reserve_path(const Path& path) {
  if (path.empty()) {
    return;
  }
  const auto last = static_cast<int>(path.size()) - 1;
  for (int t = 0; t <= last; ++t) {
    add_vertex(path[static_cast<std::size_t>(t)], t);
  }
  for (int t = 0; t < last; ++t) {
    const Cell from = path[static_cast<std::size_t>(t)];
    const Cell to = path[static_cast<std::size_t>(t) + 1];
    if (from != to) {
      add_edge(to, from, t);  // forbid swapping through this agent
    }
  }
  add_permanent(path.back(), last);
}

bool ConstraintTable::is_vertex_blocked(Cell cell, int t) const {
  const int idx = map_->index(cell);
  if (vertex_.contains(detail::vertex_key(idx, t))) {
    return true;
  }
  const auto it = permanent_.find(idx);
  return it != permanent_.end() && t >= it->second;
}

bool ConstraintTable::is_edge_blocked(Cell from, Cell to, int t) const {
  return edge_.contains(detail::TimedMove{map_->index(from), map_->index(to), t});
}

int ConstraintTable::latest_vertex_block(Cell cell) const {
  const auto it = latest_vertex_.find(map_->index(cell));
  return it == latest_vertex_.end() ? -1 : it->second;
}

bool ConstraintTable::is_permanently_blocked(Cell cell) const {
  return permanent_.contains(map_->index(cell));
}

ConflictAvoidanceTable::ConflictAvoidanceTable(const GridMap& map, const std::vector<Path>& paths,
                                               std::size_t skip_agent)
    : map_(&map) {
  for (std::size_t a = 0; a < paths.size(); ++a) {
    if (a == skip_agent || paths[a].empty()) {
      continue;
    }
    const Path& p = paths[a];
    const auto last = static_cast<int>(p.size()) - 1;
    for (int t = 0; t <= last; ++t) {
      ++occupancy_[detail::vertex_key(map.index(p[static_cast<std::size_t>(t)]), t)];
    }
    for (int t = 0; t < last; ++t) {
      const Cell from = p[static_cast<std::size_t>(t)];
      const Cell to = p[static_cast<std::size_t>(t) + 1];
      if (from != to) {
        ++moves_[detail::TimedMove{map.index(from), map.index(to), t}];
      }
    }
    parked_[map.index(p.back())].push_back(last);
  }
}

int ConflictAvoidanceTable::conflicts(Cell from, Cell to, int t) const {
  int count = 0;
  const int to_idx = map_->index(to);
  if (const auto it = occupancy_.find(detail::vertex_key(to_idx, t + 1)); it != occupancy_.end()) {
    count += it->second;
  }
  if (const auto it = parked_.find(to_idx); it != parked_.end()) {
    for (const int start : it->second) {
      if (t + 1 > start) {  // occupancy covers t + 1 == start
        ++count;
      }
    }
  }
  if (from != to) {
    const auto it = moves_.find(detail::TimedMove{to_idx, map_->index(from), t});
    if (it != moves_.end()) {
      count += it->second;
    }
  }
  return count;
}

}  // namespace multirobotcore::mapf
