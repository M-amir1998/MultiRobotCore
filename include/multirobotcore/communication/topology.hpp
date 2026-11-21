#pragma once

/// @file topology.hpp
/// Communication-graph generation from robot positions.
///
/// Models: fixed topology, distance-based (disk) graphs, k-nearest-neighbour
/// graphs, independent random edge dropout and optional line-of-sight
/// obstruction. Failed robots are isolated. These are abstractions for
/// testing the robustness of distributed algorithms, not radio models.

#include "multirobotcore/graph/graph.hpp"
#include "multirobotcore/math/obstacles.hpp"
#include "multirobotcore/math/types.hpp"

#include <cstddef>
#include <optional>
#include <random>
#include <span>
#include <vector>

namespace multirobotcore::communication {

enum class TopologyModel {
  kFixed,     ///< user-provided graph, independent of positions
  kDisk,      ///< edge {i, j} iff ||p_i - p_j|| <= radius
  kKNearest,  ///< each robot listens to its k nearest robots
};

struct TopologyConfig {
  TopologyModel model{TopologyModel::kDisk};
  double radius{5.0};               ///< disk model range [m]
  std::size_t k{3};                 ///< k-nearest model
  bool symmetric{true};             ///< k-nearest: symmetrise into an undirected graph
  double dropout_probability{0.0};  ///< independent per-edge removal probability
  bool line_of_sight{false};        ///< drop edges blocked by obstacles
};

/// Undirected unit-weight disk graph.
[[nodiscard]] graph::Graph disk_graph(std::span<const Vec2> positions, double radius);

/// k-nearest-neighbour graph. Directed (j -> i for the k nearest j of i) unless
/// `symmetric`, in which case the undirected union is returned. Ties are broken
/// by robot index for determinism.
[[nodiscard]] graph::Graph knn_graph(std::span<const Vec2> positions, std::size_t k,
                                     bool symmetric);

/// Removes each edge independently with probability p (undirected edges are
/// removed as a unit).
[[nodiscard]] graph::Graph apply_dropout(const graph::Graph& graph, double probability,
                                         std::mt19937_64& rng);

/// Removes edges whose line of sight intersects an obstacle.
[[nodiscard]] graph::Graph filter_line_of_sight(const graph::Graph& graph,
                                                std::span<const Vec2> positions,
                                                const math::ObstacleSet& obstacles);

class CommunicationModel {
 public:
  explicit CommunicationModel(TopologyConfig config, std::optional<graph::Graph> fixed_graph = {});

  /// Builds the communication graph for the current positions. Inactive robots
  /// (active[i] == false) are isolated. `obstacles` is used only when
  /// line-of-sight filtering is enabled.
  [[nodiscard]] graph::Graph build(std::span<const Vec2> positions, std::mt19937_64& rng,
                                   std::span<const bool> active = {},
                                   const math::ObstacleSet* obstacles = nullptr) const;

  [[nodiscard]] const TopologyConfig& config() const noexcept { return config_; }

 private:
  TopologyConfig config_;
  std::optional<graph::Graph> fixed_;
};

}  // namespace multirobotcore::communication
