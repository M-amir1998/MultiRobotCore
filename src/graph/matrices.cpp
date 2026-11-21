#include "multirobotcore/graph/matrices.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace multirobotcore::graph {
namespace {

Eigen::Index idx(std::size_t i) {
  return static_cast<Eigen::Index>(i);
}

}  // namespace

Eigen::MatrixXd adjacency_matrix(const Graph& graph) {
  const auto n = idx(graph.num_nodes());
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(n, n);
  for (NodeId i = 0; i < graph.num_nodes(); ++i) {
    for (const Neighbor& j : graph.in_neighbors(i)) {
      a(idx(i), idx(j.id)) = j.weight;
    }
  }
  return a;
}

Eigen::MatrixXd degree_matrix(const Graph& graph) {
  const auto n = idx(graph.num_nodes());
  Eigen::MatrixXd d = Eigen::MatrixXd::Zero(n, n);
  for (NodeId i = 0; i < graph.num_nodes(); ++i) {
    d(idx(i), idx(i)) = graph.in_degree(i);
  }
  return d;
}

Eigen::MatrixXd laplacian_matrix(const Graph& graph) {
  return degree_matrix(graph) - adjacency_matrix(graph);
}

Eigen::MatrixXd normalized_laplacian(const Graph& graph) {
  if (graph.is_directed()) {
    throw std::invalid_argument("normalized_laplacian: graph must be undirected");
  }
  const auto n = idx(graph.num_nodes());
  Eigen::MatrixXd l = Eigen::MatrixXd::Zero(n, n);
  for (NodeId i = 0; i < graph.num_nodes(); ++i) {
    const double di = graph.in_degree(i);
    if (di <= 0.0) {
      continue;
    }
    l(idx(i), idx(i)) = 1.0;
    for (const Neighbor& j : graph.in_neighbors(i)) {
      l(idx(i), idx(j.id)) = -j.weight / std::sqrt(di * graph.in_degree(j.id));
    }
  }
  return l;
}

Eigen::MatrixXd incidence_matrix(const Graph& graph) {
  const auto edges = graph.edges();
  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(idx(graph.num_nodes()), idx(edges.size()));
  for (std::size_t e = 0; e < edges.size(); ++e) {
    b(idx(edges[e].target), idx(e)) = 1.0;
    b(idx(edges[e].source), idx(e)) = -1.0;
  }
  return b;
}

Eigen::MatrixXd edge_weight_matrix(const Graph& graph) {
  const auto edges = graph.edges();
  Eigen::MatrixXd w = Eigen::MatrixXd::Zero(idx(edges.size()), idx(edges.size()));
  for (std::size_t e = 0; e < edges.size(); ++e) {
    w(idx(e), idx(e)) = edges[e].weight;
  }
  return w;
}

Eigen::MatrixXd metropolis_weights(const Graph& graph) {
  if (graph.is_directed()) {
    throw std::invalid_argument("metropolis_weights: graph must be undirected");
  }
  const auto n = idx(graph.num_nodes());
  Eigen::MatrixXd w = Eigen::MatrixXd::Zero(n, n);
  for (const Edge& e : graph.edges()) {
    const auto di = static_cast<double>(graph.in_neighbors(e.source).size());
    const auto dj = static_cast<double>(graph.in_neighbors(e.target).size());
    const double value = 1.0 / (1.0 + std::max(di, dj));
    w(idx(e.source), idx(e.target)) = value;
    w(idx(e.target), idx(e.source)) = value;
  }
  for (Eigen::Index i = 0; i < n; ++i) {
    w(i, i) = 1.0 - w.row(i).sum();
  }
  return w;
}

Eigen::MatrixXd perron_matrix(const Graph& graph, double epsilon) {
  if (!(epsilon > 0.0)) {
    throw std::invalid_argument("perron_matrix: epsilon must be positive");
  }
  const auto n = idx(graph.num_nodes());
  return Eigen::MatrixXd::Identity(n, n) - epsilon * laplacian_matrix(graph);
}

double max_in_degree(const Graph& graph) {
  double result = 0.0;
  for (NodeId i = 0; i < graph.num_nodes(); ++i) {
    result = std::max(result, graph.in_degree(i));
  }
  return result;
}

}  // namespace multirobotcore::graph
