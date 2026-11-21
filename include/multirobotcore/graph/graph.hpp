#pragma once

/// @file graph.hpp
/// Weighted directed/undirected graph for communication, sensing and
/// formation topologies.
///
/// Convention (Ren & Beard 2005; Olfati-Saber, Fax & Murray 2007): a directed
/// edge j -> i means that information flows from node j to node i, i.e. node i
/// receives (measures) the state of node j. The adjacency matrix therefore has
/// A(i, j) = w(j -> i) > 0, and node i's in-neighbours are the nodes it listens
/// to. An undirected edge {i, j} is stored as the pair j -> i and i -> j.
///
/// Node indices are dense, 0..N-1. Self-loops are not allowed and weights must
/// be finite and strictly positive.

#include <cstddef>
#include <optional>
#include <vector>

namespace multirobotcore::graph {

using NodeId = std::size_t;

enum class Directedness { kUndirected, kDirected };

/// Directed edge from `source` to `target` carrying information source -> target.
struct Edge {
  NodeId source{0};
  NodeId target{0};
  double weight{1.0};

  friend bool operator==(const Edge&, const Edge&) = default;
};

/// Adjacent node with the weight of the connecting edge.
struct Neighbor {
  NodeId id{0};
  double weight{1.0};

  friend bool operator==(const Neighbor&, const Neighbor&) = default;
};

class Graph {
 public:
  explicit Graph(std::size_t num_nodes = 0, Directedness directedness = Directedness::kUndirected);

  [[nodiscard]] static Graph undirected(std::size_t num_nodes) {
    return Graph(num_nodes, Directedness::kUndirected);
  }
  [[nodiscard]] static Graph directed(std::size_t num_nodes) {
    return Graph(num_nodes, Directedness::kDirected);
  }

  [[nodiscard]] std::size_t num_nodes() const noexcept { return in_.size(); }

  /// Number of edges. Undirected edges are counted once.
  [[nodiscard]] std::size_t num_edges() const noexcept;

  [[nodiscard]] bool is_directed() const noexcept {
    return directedness_ == Directedness::kDirected;
  }
  [[nodiscard]] Directedness directedness() const noexcept { return directedness_; }

  /// Appends an isolated node and returns its index.
  NodeId add_node();

  /// Inserts or updates the edge source -> target (and the reverse edge for
  /// undirected graphs). Throws std::invalid_argument for self-loops,
  /// out-of-range nodes or non-positive/non-finite weights.
  void add_edge(NodeId source, NodeId target, double weight = 1.0);

  /// Removes source -> target (and the reverse edge for undirected graphs).
  /// Returns true when an edge was removed.
  bool remove_edge(NodeId source, NodeId target);

  /// Removes every edge incident to `node` (e.g. after a robot failure).
  void isolate_node(NodeId node);

  void clear_edges() noexcept;

  [[nodiscard]] bool has_edge(NodeId source, NodeId target) const;
  [[nodiscard]] std::optional<double> weight(NodeId source, NodeId target) const;

  /// Nodes whose information node `node` receives (sorted by id).
  [[nodiscard]] const std::vector<Neighbor>& in_neighbors(NodeId node) const;

  /// Nodes that receive information from `node` (sorted by id).
  [[nodiscard]] const std::vector<Neighbor>& out_neighbors(NodeId node) const;

  /// Ids of the in-neighbours of `node` (the neighbour set N_i used by
  /// distributed protocols). Equal to the adjacent set for undirected graphs.
  [[nodiscard]] std::vector<NodeId> neighbor_ids(NodeId node) const;

  /// Weighted in-degree d_i = sum_j a_ij.
  [[nodiscard]] double in_degree(NodeId node) const;
  /// Weighted out-degree.
  [[nodiscard]] double out_degree(NodeId node) const;

  /// All edges, sorted by (source, target). For undirected graphs each edge is
  /// reported once with source < target.
  [[nodiscard]] std::vector<Edge> edges() const;

  /// True when every edge has a reverse edge of equal weight.
  [[nodiscard]] bool is_symmetric() const;

  friend bool operator==(const Graph&, const Graph&) = default;

 private:
  void check_node(NodeId node) const;
  static bool upsert(std::vector<Neighbor>& list, NodeId id, double weight);
  static bool erase(std::vector<Neighbor>& list, NodeId id);
  void add_directed(NodeId source, NodeId target, double weight);
  bool remove_directed(NodeId source, NodeId target);

  Directedness directedness_{Directedness::kUndirected};
  std::vector<std::vector<Neighbor>> in_;   ///< in_[i]: edges j -> i
  std::vector<std::vector<Neighbor>> out_;  ///< out_[i]: edges i -> j
};

}  // namespace multirobotcore::graph
