#include "multirobotcore/consensus/consensus.hpp"
#include "multirobotcore/consensus/distributed.hpp"
#include "multirobotcore/graph/connectivity.hpp"
#include "multirobotcore/graph/topologies.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

namespace c = multirobotcore::consensus;
namespace g = multirobotcore::graph;
namespace comm = multirobotcore::communication;

namespace {

c::StateMatrix random_state(std::size_t n, Eigen::Index dims, std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> u(-5.0, 5.0);
  c::StateMatrix x(static_cast<Eigen::Index>(n), dims);
  for (Eigen::Index i = 0; i < x.rows(); ++i) {
    for (Eigen::Index k = 0; k < dims; ++k) {
      x(i, k) = u(rng);
    }
  }
  return x;
}

/// Euclidean norm of the disagreement vector X - 1 mean(X).
double deviation_norm(const c::StateMatrix& x) {
  return (x.rowwise() - c::average(x)).norm();
}

}  // namespace

TEST(Consensus, ConnectedGraphConvergesToAverage) {
  const g::Graph graph = g::ring_graph(8);
  const c::StateMatrix x0 = random_state(8, 2, 1);
  const double eps = 0.9 * c::max_convex_step(graph);
  const auto run = c::run_consensus(graph, x0, {eps, 5000, 1e-9, true});
  ASSERT_TRUE(run.converged);
  for (const auto& x : run.history) {
    EXPECT_TRUE(c::average(x).isApprox(c::average(x0), 1e-12));  // invariant at every step
  }
  for (Eigen::Index i = 0; i < 8; ++i) {
    EXPECT_TRUE(run.final_state.row(i).isApprox(c::average(x0), 1e-8));
  }
}

TEST(Consensus, BoundaryStepOscillatesOnBipartiteRegularGraphs) {
  // Even cycle: lambda_max(L) = 4 = 2 d_max, so eps = 1/d_max gives an
  // eigenvalue -1 of P = I - eps L and the alternating mode never decays.
  const g::Graph graph = g::ring_graph(8);
  c::StateMatrix x0(8, 1);
  x0 << 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0;
  const auto run = c::run_consensus(graph, x0, {c::max_convex_step(graph), 500, 1e-9, false});
  EXPECT_FALSE(run.converged);
  EXPECT_NEAR(c::disagreement(run.final_state), 1.0, 1e-12);
  EXPECT_NEAR(c::discrete_contraction_factor(graph, c::max_convex_step(graph)), 1.0, 1e-12);
}

TEST(Consensus, DisagreementContractsAtSpectralRate) {
  std::mt19937_64 rng(5);
  for (int trial = 0; trial < 10; ++trial) {
    g::Graph graph = g::random_graph(10, 0.4, rng);
    if (!g::is_connected(graph)) {
      continue;
    }
    const double eps = 0.9 * c::max_convex_step(graph);
    const double rho = c::discrete_contraction_factor(graph, eps);
    ASSERT_LT(rho, 1.0);
    c::StateMatrix x = random_state(10, 1, static_cast<std::uint64_t>(trial));
    for (int k = 0; k < 30; ++k) {
      const double before = deviation_norm(x);
      x = c::consensus_step(graph, x, eps);
      EXPECT_LE(deviation_norm(x), rho * before + 1e-12);
    }
  }
}

TEST(Consensus, DisconnectedComponentsAgreeIndependently) {
  g::Graph graph = g::Graph::undirected(6);
  graph.add_edge(0, 1);
  graph.add_edge(1, 2);
  graph.add_edge(3, 4);
  graph.add_edge(4, 5);
  c::StateMatrix x0(6, 1);
  x0 << 0.0, 3.0, 6.0, 10.0, 20.0, 30.0;
  const auto run = c::run_consensus(graph, x0, {0.4, 2000, 1e-10, false});
  EXPECT_FALSE(run.converged);  // no global agreement
  for (const auto& component : g::connected_components(graph)) {
    EXPECT_LT(c::disagreement(run.final_state, component), 1e-8);
  }
  EXPECT_NEAR(run.final_state(0, 0), 3.0, 1e-8);
  EXPECT_NEAR(run.final_state(5, 0), 20.0, 1e-8);
}

TEST(Consensus, DirectedChainFollowsRoot) {
  const g::Graph chain = g::directed_chain(5);
  c::StateMatrix x0(5, 1);
  x0 << 7.0, -1.0, 2.0, 4.0, 9.0;
  const auto run = c::run_consensus(chain, x0, {0.5, 5000, 1e-9, false});
  ASSERT_TRUE(run.converged);
  EXPECT_NEAR(run.final_state(4, 0), 7.0, 1e-8);  // root value, not the average
  EXPECT_DOUBLE_EQ(run.final_state(0, 0), 7.0);   // the root never moves
}

TEST(Consensus, NoSpanningTreeMeansNoAgreement) {
  g::Graph graph = g::Graph::directed(3);
  graph.add_edge(0, 2);
  graph.add_edge(1, 2);
  ASSERT_FALSE(g::has_spanning_tree(graph));
  c::StateMatrix x0(3, 1);
  x0 << 0.0, 10.0, 3.0;
  const auto run = c::run_consensus(graph, x0, {0.5, 3000, 1e-9, false});
  EXPECT_FALSE(run.converged);
  EXPECT_NEAR(run.final_state(2, 0), 5.0, 1e-8);  // compromise between the two sources
}

TEST(Consensus, ConvexHullInvariantForConvexSteps) {
  std::mt19937_64 rng(77);
  for (int trial = 0; trial < 20; ++trial) {
    const g::Graph graph = g::random_graph(12, 0.3, rng);
    c::StateMatrix x = random_state(12, 1, static_cast<std::uint64_t>(100 + trial));
    const double eps = std::min(1.0, c::max_convex_step(graph));
    for (int k = 0; k < 20; ++k) {
      const double lo = x.minCoeff();
      const double hi = x.maxCoeff();
      x = c::consensus_step(graph, x, eps);
      EXPECT_GE(x.minCoeff(), lo - 1e-12);
      EXPECT_LE(x.maxCoeff(), hi + 1e-12);
    }
  }
}

TEST(Consensus, ContinuousTimeMatchesExponentialBound) {
  const g::Graph graph = g::path_graph(6);
  const double lambda2 = g::algebraic_connectivity(graph);
  c::StateMatrix x = random_state(6, 1, 9);
  const c::StateMatrix x0 = x;
  const double dt = 0.01;
  for (int k = 0; k < 300; ++k) {
    x = c::consensus_flow_rk4(graph, x, dt);
  }
  EXPECT_TRUE(c::average(x).isApprox(c::average(x0), 1e-12));
  EXPECT_LE(deviation_norm(x), std::exp(-lambda2 * 3.0) * deviation_norm(x0) * (1.0 + 1e-6));
}

TEST(Consensus, LeaderFollowerTracksReference) {
  const g::Graph graph = g::path_graph(5);
  std::vector<double> pinning(5, 0.0);
  pinning[0] = 1.0;  // only agent 0 observes the leader
  Eigen::RowVectorXd reference(2);
  reference << 3.0, -2.0;
  c::StateMatrix x = random_state(5, 2, 4);
  for (int k = 0; k < 4000; ++k) {
    x = c::leader_follower_step(graph, x, pinning, reference, 0.3);
  }
  for (Eigen::Index i = 0; i < 5; ++i) {
    EXPECT_TRUE(x.row(i).isApprox(reference, 1e-8));
  }
}

TEST(Consensus, LeaderFollowerUnreachableFollowerDoesNotTrack) {
  g::Graph graph = g::path_graph(4);
  graph.isolate_node(3);
  std::vector<double> pinning{1.0, 0.0, 0.0, 0.0};
  Eigen::RowVectorXd reference(1);
  reference << 10.0;
  c::StateMatrix x = c::StateMatrix::Zero(4, 1);
  for (int k = 0; k < 2000; ++k) {
    x = c::leader_follower_step(graph, x, pinning, reference, 0.3);
  }
  EXPECT_NEAR(x(2, 0), 10.0, 1e-6);
  EXPECT_DOUBLE_EQ(x(3, 0), 0.0);
}

TEST(Consensus, SecondOrderReachesCommonVelocity) {
  const g::Graph graph = g::ring_graph(6);
  c::SecondOrderState s{random_state(6, 2, 1), random_state(6, 2, 2)};
  const Eigen::RowVectorXd v_avg = c::average(s.velocity);
  for (int k = 0; k < 4000; ++k) {
    s = c::second_order_step(graph, s, 1.5, 0.01);
  }
  EXPECT_TRUE(c::average(s.velocity).isApprox(v_avg, 1e-9));
  EXPECT_LT(c::disagreement(s.velocity), 1e-6);
  EXPECT_LT(c::disagreement(s.position), 1e-5);
}

TEST(Consensus, RendezvousPreservesCentroidOnUndirectedGraphs) {
  const g::Graph graph = g::complete_graph(4);
  const std::vector<Eigen::Vector2d> p{{0.0, 0.0}, {4.0, 0.0}, {4.0, 3.0}, {1.0, 5.0}};
  const auto u = c::rendezvous_velocities(graph, p, 0.5, 100.0);
  Eigen::Vector2d sum = Eigen::Vector2d::Zero();
  for (const auto& v : u) {
    sum += v;
  }
  EXPECT_LT(sum.norm(), 1e-12);
  const auto saturated = c::rendezvous_velocities(graph, p, 0.5, 0.1);
  for (const auto& v : saturated) {
    EXPECT_LE(v.norm(), 0.1 + 1e-12);
  }
}

TEST(DistributedConsensus, LosslessNetworkMatchesCentralisedIteration) {
  const g::Graph graph = g::ring_graph(7);
  const c::StateMatrix x0 = random_state(7, 1, 3);
  c::DistributedConsensus dc(graph, x0, {0.3, 0.1, c::MissingPolicy::kSkip, 1});
  comm::NetworkSimulator net(7, {}, 1);
  c::StateMatrix reference = x0;
  for (int k = 0; k < 50; ++k) {
    dc.round(net);
    reference = c::consensus_step(graph, reference, 0.3);
    EXPECT_TRUE(dc.state().isApprox(reference, 1e-12));
  }
  EXPECT_EQ(net.stats().rounds, 50U);
  EXPECT_EQ(net.stats().messages_sent, 50U * 14U);
}

TEST(DistributedConsensus, ConvergesUnderMessageLoss) {
  const g::Graph graph = g::ring_graph(8);
  const c::StateMatrix x0 = random_state(8, 1, 8);
  for (const auto policy : {c::MissingPolicy::kSkip, c::MissingPolicy::kUseLastReceived}) {
    comm::ChannelConfig channel;
    channel.drop_probability = 0.2;
    comm::NetworkSimulator net(8, channel, 42);
    c::DistributedConsensus dc(graph, x0, {0.3, 0.1, policy, 1});
    for (int k = 0; k < 600; ++k) {
      dc.round(net);
    }
    EXPECT_LT(c::disagreement(dc.state()), 1e-6);
    // Agreement value stays inside the initial range.
    EXPECT_GE(dc.state().minCoeff(), x0.minCoeff() - 1e-9);
    EXPECT_LE(dc.state().maxCoeff(), x0.maxCoeff() + 1e-9);
    EXPECT_GT(net.stats().dropped_loss, 0U);
  }
}

TEST(LeaderElection, FailedLeaderIsReplacedByMinimumOperationalId) {
  const g::Graph graph = g::path_graph(6);
  std::vector<bool> operational(6, true);
  operational[0] = false;  // the original leader failed
  comm::NetworkSimulator net(6, {}, 1);
  const auto result = c::elect_leader_min_id(graph, operational, net, 6);
  EXPECT_TRUE(result.agreed);
  EXPECT_FALSE(result.leader[0].has_value());
  for (std::size_t i = 1; i < 6; ++i) {
    EXPECT_EQ(result.leader[i], 1U);
  }
}

TEST(LeaderElection, TooFewRoundsOrPartitionsAreReported) {
  const g::Graph graph = g::path_graph(6);
  const std::vector<bool> operational(6, true);
  comm::NetworkSimulator net(6, {}, 1);
  EXPECT_FALSE(c::elect_leader_min_id(graph, operational, net, 2).agreed);

  g::Graph split = g::path_graph(6);
  split.remove_edge(2, 3);
  comm::NetworkSimulator net2(6, {}, 1);
  const auto result = c::elect_leader_min_id(split, operational, net2, 6);
  EXPECT_TRUE(result.agreed);  // agreement within each component
  EXPECT_EQ(result.leader[5], 3U);
  EXPECT_EQ(result.leader[1], 0U);
}
