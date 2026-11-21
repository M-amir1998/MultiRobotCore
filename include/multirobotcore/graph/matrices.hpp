#pragma once

/// @file matrices.hpp
/// Algebraic graph representations (Mesbahi & Egerstedt 2010, ch. 2).
///
/// With the information-flow convention of graph.hpp:
///   A(i, j) = w(j -> i)              adjacency
///   D       = diag(sum_j A(i, j))    in-degree matrix
///   L       = D - A                  (in-degree) graph Laplacian, L 1 = 0
///
/// For undirected graphs L is symmetric positive semidefinite and
/// L = B W B^T where B is an incidence matrix of any orientation.

#include "multirobotcore/graph/graph.hpp"

#include <Eigen/Core>

namespace multirobotcore::graph {

[[nodiscard]] Eigen::MatrixXd adjacency_matrix(const Graph& graph);

[[nodiscard]] Eigen::MatrixXd degree_matrix(const Graph& graph);

[[nodiscard]] Eigen::MatrixXd laplacian_matrix(const Graph& graph);

/// Symmetric normalised Laplacian I - D^{-1/2} A D^{-1/2} of an undirected
/// graph. Rows/columns of isolated nodes are zero. Throws std::invalid_argument
/// for directed graphs.
[[nodiscard]] Eigen::MatrixXd normalized_laplacian(const Graph& graph);

/// Oriented incidence matrix B (num_nodes x num_edges), with columns ordered as
/// Graph::edges(): B(target, e) = +1 and B(source, e) = -1.
[[nodiscard]] Eigen::MatrixXd incidence_matrix(const Graph& graph);

/// Diagonal matrix of edge weights ordered as Graph::edges().
[[nodiscard]] Eigen::MatrixXd edge_weight_matrix(const Graph& graph);

/// Metropolis-Hastings weights (Xiao, Boyd & Lall 2005) of an undirected
/// graph: W(i, j) = 1 / (1 + max(deg_i, deg_j)) for edges, W(i, i) = 1 -
/// sum_j W(i, j). W is symmetric and doubly stochastic and needs only
/// neighbour degrees, so it can be built without central coordination.
/// Unweighted degrees (neighbour counts) are used.
[[nodiscard]] Eigen::MatrixXd metropolis_weights(const Graph& graph);

/// Perron matrix P = I - epsilon L of the discrete-time consensus iteration.
/// It is row stochastic and non-negative when 0 < epsilon <= 1 / max_i d_i.
[[nodiscard]] Eigen::MatrixXd perron_matrix(const Graph& graph, double epsilon);

/// Maximum weighted in-degree max_i d_i.
[[nodiscard]] double max_in_degree(const Graph& graph);

}  // namespace multirobotcore::graph
