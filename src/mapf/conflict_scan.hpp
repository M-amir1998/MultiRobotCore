#pragma once

// Internal: generic conflict scan over paths accessed through a getter, so
// that callers holding paths by pointer (CBS) avoid copies.

#include "multirobotcore/mapf/conflicts.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace multirobotcore::mapf::detail {

[[nodiscard]] inline std::uint64_t cell_key(Cell c) noexcept {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(c.x)) << 32U) |
         static_cast<std::uint64_t>(static_cast<std::uint32_t>(c.y));
}

/// Calls on_conflict(conflict) for every pairwise conflict in time order;
/// stops early when on_conflict returns false.
template <typename GetPath, typename OnConflict>
void scan_conflicts(std::size_t num_agents, GetPath&& get_path, OnConflict&& on_conflict) {
  std::size_t horizon = 0;
  for (std::size_t a = 0; a < num_agents; ++a) {
    horizon = std::max(horizon, get_path(a).size());
  }
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> occupants;
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> moves;
  occupants.reserve(num_agents * 2);
  moves.reserve(num_agents * 2);
  for (std::size_t t = 0; t < horizon; ++t) {
    occupants.clear();
    for (std::size_t i = 0; i < num_agents; ++i) {
      const Cell c = position_at(get_path(i), t);
      auto& list = occupants[cell_key(c)];
      for (const std::size_t j : list) {
        if (!on_conflict(Conflict{ConflictType::kVertex, j, i, c, c, static_cast<int>(t)})) {
          return;
        }
      }
      list.push_back(i);
    }
    if (t + 1 >= horizon) {
      break;
    }
    moves.clear();
    for (std::size_t i = 0; i < num_agents; ++i) {
      const Cell from = position_at(get_path(i), t);
      const Cell to = position_at(get_path(i), t + 1);
      if (from == to) {
        continue;
      }
      const std::uint64_t reverse = cell_key(to) * 1000003ULL ^ cell_key(from);
      if (const auto it = moves.find(reverse); it != moves.end()) {
        for (const std::size_t j : it->second) {
          // Guard against hash aliasing: confirm the swap explicitly.
          if (position_at(get_path(j), t) == to && position_at(get_path(j), t + 1) == from) {
            if (!on_conflict(Conflict{ConflictType::kEdge, j, i, to, from, static_cast<int>(t)})) {
              return;
            }
          }
        }
      }
      moves[cell_key(from) * 1000003ULL ^ cell_key(to)].push_back(i);
    }
  }
}

}  // namespace multirobotcore::mapf::detail
