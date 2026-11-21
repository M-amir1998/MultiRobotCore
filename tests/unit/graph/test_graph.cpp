#include "multirobotcore/graph/connectivity.hpp"
#include "multirobotcore/graph/graph.hpp"
#include "multirobotcore/graph/matrices.hpp"
#include "multirobotcore/graph/topologies.hpp"

#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <numbers>
#include <random>
#include <stdexcept>

namespace g = multirobotcore::graph;

TEST(Graph, UndirectedEdgesAreStoredBothWays) {
  g::Graph graph = g::Graph::undirected(4);
  graph.add_edge(0, 1, 2.0);
  graph.add_edge(2, 1);
  EXPECT_EQ(graph.num_edges(), 2U);
  EXPECT_TRUE(graph.has_edge(1, 0));
  EXPECT_DOUBLE_EQ(*graph.weight(1, 0), 2.0);
  EXPECT_EQ(graph.neighbor_ids(1), (std::vector<g::NodeId>{0, 2}));
  EXPECT_DOUBLE_EQ(graph.in_degree(1), 3.0);
  EXPECT_TRUE(graph.is_symmetric());
  const auto edges = graph.edges();
  ASSERT_EQ(edges.size(), 2U);
  EXPECT_EQ(edges[0], (g::Edge{0, 1, 2.0}));
  EXPECT_EQ(edges[1], (g::Edge{1, 2, 1.0}));
}

TEST(Graph, DirectedEdgesFollowInformationFlow) {
  g::Graph graph = g::Graph::directed(3);
  graph.add_edge(0, 1);  // node 1 listens to node 0
  EXPECT_TRUE(graph.has_edge(0, 1));
  EXPECT_FALSE(graph.has_edge(1, 0));
  EXPECT_EQ(graph.neighbor_ids(1), (std::vector<g::NodeId>{0}));
  EXPECT_TRUE(graph.neighbor_ids(0).empty());
  const Eigen::MatrixXd a = g::adjacency_matrix(graph);
  EXPECT_DOUBLE_EQ(a(1, 0), 1.0);
  EXPECT_DOUBLE_EQ(a(0, 1), 0.0);
  EXPECT_FALSE(graph.is_symmetric());
}

TEST(Graph, DynamicUpdates) {
  g::Graph graph = g::complete_graph(4);
  EXPECT_EQ(graph.num_edges(), 6U);
  graph.add_edge(0, 1, 5.0);  // update, not insert
  EXPECT_EQ(graph.num_edges(), 6U);
  EXPECT_DOUBLE_EQ(*graph.weight(1, 0), 5.0);
  EXPECT_TRUE(graph.remove_edge(1, 0));
  EXPECT_FALSE(graph.remove_edge(1, 0));
  EXPECT_EQ(graph.num_edges(), 5U);
  graph.isolate_node(2);
  EXPECT_EQ(graph.num_edges(), 2U);
  EXPECT_TRUE(graph.in_neighbors(2).empty());
  EXPECT_FALSE(graph.has_edge(3, 2));
  const g::NodeId added = graph.add_node();
  EXPECT_EQ(added, 4U);
  graph.add_edge(4, 0);
  EXPECT_TRUE(graph.has_edge(0, 4));
  graph.clear_edges();
  EXPECT_EQ(graph.num_edges(), 0U);
}

TEST(Graph, RejectsInvalidEdges) {
  g::Graph graph = g::Graph::undirected(3);
  EXPECT_THROW(graph.add_edge(1, 1), std::invalid_argument);
  EXPECT_THROW(graph.add_edge(0, 1, 0.0), std::invalid_argument);
  EXPECT_THROW(graph.add_edge(0, 1, -1.0), std::invalid_argument);
  EXPECT_THROW(graph.add_edge(0, 1, std::nan("")), std::invalid_argument);
  EXPECT_THROW(graph.add_edge(0, 3), std::out_of_range);
  EXPECT_THROW((void)graph.in_neighbors(7), std::out_of_range);
}

TEST(Laplacian, UndirectedIsSymmetricPsdWithZeroRowSums) {
  std::mt19937_64 rng(11);
  for (int trial = 0; trial < 20; ++trial) {
    g::Graph graph = g::random_graph(9, 0.35, rng);
    // Random weights.
    std::uniform_real_distribution<double> w(0.1, 3.0);
    for (const auto& e : graph.edges()) {
      graph.add_edge(e.source, e.target, w(rng));
    }
    const Eigen::MatrixXd l = g::laplacian_matrix(graph);
    EXPECT_TRUE(l.isApprox(l.transpose()));
    EXPECT_LT(l.rowwise().sum().cwiseAbs().maxCoeff(), 1e-12);
    const Eigen::VectorXd spectrum = g::laplacian_spectrum(graph);
    EXPECT_GT(spectrum.minCoeff(), -1e-10);
    // L = B W B^T for an arbitrary orientation.
    const Eigen::MatrixXd b = g::incidence_matrix(graph);
    const Eigen::MatrixXd bwbt = b * g::edge_weight_matrix(graph) * b.transpose();
    EXPECT_TRUE(bwbt.isApprox(l, 1e-12) || (l.norm() == 0.0 && bwbt.norm() == 0.0));
    // Multiplicity of the zero eigenvalue equals the number of components.
    const auto zero_count = (spectrum.array().abs() < 1e-9).count();
    EXPECT_EQ(static_cast<std::size_t>(zero_count), g::connected_components(graph).size());
  }
}

TEST(Laplacian, DirectedRowSumsAreZero) {
  g::Graph graph = g::directed_ring(5);
  graph.add_edge(0, 3, 2.5);
  const Eigen::MatrixXd l = g::laplacian_matrix(graph);
  EXPECT_LT(l.rowwise().sum().cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_DOUBLE_EQ(l(3, 3), 3.5);  // in-degree of node 3: from 2 (1.0) and 0 (2.5)
  EXPECT_DOUBLE_EQ(l(3, 0), -2.5);
}

TEST(Laplacian, NormalizedSpectrumInZeroTwo) {
  const g::Graph graph = g::ring_graph(7);
  const Eigen::MatrixXd ln = g::normalized_laplacian(graph);
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(ln);
  EXPECT_GT(solver.eigenvalues().minCoeff(), -1e-12);
  EXPECT_LT(solver.eigenvalues().maxCoeff(), 2.0 + 1e-12);
  EXPECT_THROW((void)g::normalized_laplacian(g::directed_ring(3)), std::invalid_argument);
}

TEST(Laplacian, ClosedFormAlgebraicConnectivity) {
  const double pi = std::numbers::pi;
  for (std::size_t n = 3; n <= 12; ++n) {
    const auto dn = static_cast<double>(n);
    EXPECT_NEAR(g::algebraic_connectivity(g::complete_graph(n)), dn, 1e-9);
    EXPECT_NEAR(g::algebraic_connectivity(g::path_graph(n)), 2.0 * (1.0 - std::cos(pi / dn)), 1e-9);
    EXPECT_NEAR(g::algebraic_connectivity(g::ring_graph(n)), 2.0 * (1.0 - std::cos(2.0 * pi / dn)),
                1e-9);
    EXPECT_NEAR(g::algebraic_connectivity(g::star_graph(n)), 1.0, 1e-9);
  }
  g::Graph disconnected = g::path_graph(6);
  disconnected.remove_edge(2, 3);
  EXPECT_NEAR(g::algebraic_connectivity(disconnected), 0.0, 1e-12);
}

TEST(Laplacian, FiedlerVectorSeparatesBarbell) {
  // Two triangles joined by a single bridge 2-3.
  g::Graph graph = g::Graph::undirected(6);
  for (auto [a, b] : {std::pair{0, 1}, {1, 2}, {0, 2}, {3, 4}, {4, 5}, {3, 5}, {2, 3}}) {
    graph.add_edge(static_cast<g::NodeId>(a), static_cast<g::NodeId>(b));
  }
  const Eigen::VectorXd v = g::fiedler_vector(graph);
  EXPECT_GT(v(0) * v(1), 0.0);
  EXPECT_GT(v(4) * v(5), 0.0);
  EXPECT_LT(v(0) * v(5), 0.0);
  EXPECT_NEAR(v.sum(), 0.0, 1e-9);  // orthogonal to the all-ones vector
}

TEST(Matrices, MetropolisWeightsAreDoublyStochastic) {
  std::mt19937_64 rng(3);
  const g::Graph graph = g::random_graph(12, 0.3, rng);
  const Eigen::MatrixXd w = g::metropolis_weights(graph);
  EXPECT_TRUE(w.isApprox(w.transpose()));
  EXPECT_LT((w.rowwise().sum().array() - 1.0).abs().maxCoeff(), 1e-12);
  EXPECT_GE(w.minCoeff(), 0.0);
}

TEST(Matrices, PerronMatrixIsRowStochastic) {
  const g::Graph graph = g::star_graph(6);
  const double eps = 1.0 / g::max_in_degree(graph);
  const Eigen::MatrixXd p = g::perron_matrix(graph, eps);
  EXPECT_LT((p.rowwise().sum().array() - 1.0).abs().maxCoeff(), 1e-12);
  EXPECT_GE(p.minCoeff(), -1e-15);
  EXPECT_THROW((void)g::perron_matrix(graph, 0.0), std::invalid_argument);
}

TEST(Connectivity, ComponentsAndDiameter) {
  EXPECT_EQ(g::diameter(g::path_graph(6)), 5U);
  EXPECT_EQ(g::diameter(g::ring_graph(7)), 3U);
  EXPECT_EQ(g::diameter(g::star_graph(9)), 2U);
  EXPECT_EQ(g::diameter(g::complete_graph(5)), 1U);
  EXPECT_EQ(g::diameter(g::grid_graph(3, 4)), 5U);

  g::Graph graph = g::Graph::undirected(5);
  graph.add_edge(0, 4);
  graph.add_edge(1, 2);
  const auto components = g::connected_components(graph);
  ASSERT_EQ(components.size(), 3U);
  EXPECT_EQ(components[0], (std::vector<g::NodeId>{0, 4}));
  EXPECT_EQ(components[1], (std::vector<g::NodeId>{1, 2}));
  EXPECT_EQ(components[2], (std::vector<g::NodeId>{3}));
  EXPECT_FALSE(g::is_connected(graph));
  EXPECT_FALSE(g::diameter(graph).has_value());
}

TEST(Connectivity, DirectedSpanningTreesAndStrongComponents) {
  const g::Graph chain = g::directed_chain(4);
  EXPECT_EQ(g::strongly_connected_components(chain).size(), 4U);
  EXPECT_EQ(g::spanning_tree_roots(chain), (std::vector<g::NodeId>{0}));
  EXPECT_TRUE(g::has_spanning_tree(chain));
  EXPECT_FALSE(g::is_connected(chain));

  const g::Graph ring = g::directed_ring(5);
  EXPECT_TRUE(g::is_connected(ring));
  EXPECT_EQ(g::spanning_tree_roots(ring).size(), 5U);

  // Two independent sources (0 and 1) both feeding node 2: no spanning tree.
  g::Graph two_sources = g::Graph::directed(3);
  two_sources.add_edge(0, 2);
  two_sources.add_edge(1, 2);
  EXPECT_FALSE(g::has_spanning_tree(two_sources));

  // SCCs of a graph with a cycle {1, 2, 3} plus a tail.
  g::Graph mixed = g::Graph::directed(5);
  mixed.add_edge(0, 1);
  mixed.add_edge(1, 2);
  mixed.add_edge(2, 3);
  mixed.add_edge(3, 1);
  mixed.add_edge(3, 4);
  const auto sccs = g::strongly_connected_components(mixed);
  ASSERT_EQ(sccs.size(), 3U);
  EXPECT_EQ(sccs[1], (std::vector<g::NodeId>{1, 2, 3}));
}

TEST(Connectivity, RandomGraphInvariants) {
  std::mt19937_64 rng(2024);
  for (int trial = 0; trial < 30; ++trial) {
    const g::Graph graph = g::random_graph(10, 0.25, rng);
    const bool connected = g::is_connected(graph);
    EXPECT_EQ(connected, g::algebraic_connectivity(graph) > 1e-9);
    EXPECT_EQ(connected, g::diameter(graph).has_value());
    EXPECT_EQ(connected, g::has_spanning_tree(graph));
    std::size_t total = 0;
    for (const auto& c : g::connected_components(graph)) {
      total += c.size();
    }
    EXPECT_EQ(total, graph.num_nodes());
  }
}
