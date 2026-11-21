#pragma once

/// @file conflicts.hpp
/// Inter-agent conflicts in the classical MAPF model (Stern et al. 2019):
///
///  - vertex conflict  <a1, a2, v, t>: both agents occupy cell v at time t;
///  - edge (swap) conflict <a1, a2, u, v, t>: a1 moves u -> v while a2 moves
///    v -> u between t and t + 1.
///
/// Agents park at their final cell after their path ends, so an agent that
/// has arrived still conflicts with others passing through its goal.

#include "multirobotcore/mapf/instance.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace multirobotcore::mapf {

enum class ConflictType { kVertex, kEdge };

struct Conflict {
  ConflictType type{ConflictType::kVertex};
  std::size_t agent1{0};
  std::size_t agent2{0};
  Cell cell1;   ///< vertex: the shared cell; edge: agent1's origin u
  Cell cell2;   ///< vertex: unused (= cell1); edge: agent1's destination v
  int time{0};  ///< vertex: t; edge: departure time t

  friend bool operator==(const Conflict&, const Conflict&) = default;
};

/// The earliest conflict (by time; vertex conflicts at t before edge
/// conflicts departing at t; then by agent indices). nullopt if collision-free.
[[nodiscard]] std::optional<Conflict> first_conflict(const std::vector<Path>& paths);

/// All pairwise conflicts, ordered as in first_conflict().
[[nodiscard]] std::vector<Conflict> find_conflicts(const std::vector<Path>& paths);

/// Number of pairwise conflicts (find_conflicts().size(), without allocation of
/// the conflict list).
[[nodiscard]] std::size_t count_conflicts(const std::vector<Path>& paths);

/// True when no two paths conflict.
[[nodiscard]] bool is_collision_free(const std::vector<Path>& paths);

}  // namespace multirobotcore::mapf
