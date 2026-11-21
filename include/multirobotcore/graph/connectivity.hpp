#pragma once

/// @file connectivity.hpp
/// Connectivity analyses that determine whether distributed protocols can
/// converge (e.g. consensus requires a directed spanning tree).

#include "multirobotcore/graph/graph.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace multirobotcore::graph {

inline constexpr std::size_t kUnreachable = std::numeric_limits<std::size_t>::max();

/// Connected components (weakly connected for directed graphs). Each component
/// is sorted; components are ordered by their smallest node id.
[[nodiscard]] std::vector<std::vector<NodeId>> connected_components(const Graph& graph);

/// Strongly connected components (Tarjan 1972, iterative), ordered by smallest
/// node id with sorted members.
[[nodiscard]] std::vector<std::vector<NodeId>> strongly_connected_components(const Graph& graph);

/// Undirected: connected. Directed: strongly connected. Empty graphs and single
/// nodes are connected.
[[nodiscard]] bool is_connected(const Graph& graph);

/// Nodes from which every other node can be reached along the direction of
/// information flow, i.e. roots of directed spanning trees.
[[nodiscard]] std::vector<NodeId> spanning_tree_roots(const Graph& graph);

/// True when the graph contains a directed spanning tree. This is the
/// necessary and sufficient topological condition for first-order consensus
/// on a fixed digraph (Ren & Beard 2005).
[[nodiscard]] bool has_spanning_tree(const Graph& graph);

/// Hop distances from `source` following edge directions; kUnreachable for
/// nodes that cannot be reached.
[[nodiscard]] std::vector<std::size_t> hop_distances(const Graph& graph, NodeId source);

/// Hop diameter (longest shortest path). nullopt when some pair is unreachable.
[[nodiscard]] std::optional<std::size_t> diameter(const Graph& graph);

/// Algebraic connectivity lambda_2(L) of an undirected graph (Fiedler 1973):
/// positive if and only if the graph is connected; governs the convergence
/// rate of continuous-time consensus. Throws for directed graphs.
[[nodiscard]] double algebraic_connectivity(const Graph& graph);

/// Unit eigenvector associated with lambda_2(L) of an undirected graph.
[[nodiscard]] Eigen::VectorXd fiedler_vector(const Graph& graph);

/// Ascending eigenvalues of the Laplacian of an undirected graph.
[[nodiscard]] Eigen::VectorXd laplacian_spectrum(const Graph& graph);

}  // namespace multirobotcore::graph
