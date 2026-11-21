#include "multirobotcore/communication/network.hpp"
#include "multirobotcore/communication/topology.hpp"
#include "multirobotcore/graph/connectivity.hpp"

#include <gtest/gtest.h>

#include <random>
#include <stdexcept>
#include <vector>

namespace comm = multirobotcore::communication;
namespace g = multirobotcore::graph;
using multirobotcore::Vec2;

namespace {

std::vector<Vec2> line_positions(std::size_t n, double spacing) {
  std::vector<Vec2> p;
  for (std::size_t i = 0; i < n; ++i) {
    p.emplace_back(spacing * static_cast<double>(i), 0.0);
  }
  return p;
}

}  // namespace

TEST(Topology, DiskGraphUsesRadius) {
  const auto p = line_positions(5, 1.0);
  const g::Graph graph = comm::disk_graph(p, 1.0);
  EXPECT_EQ(graph.num_edges(), 4U);  // only consecutive robots
  EXPECT_TRUE(g::is_connected(graph));
  EXPECT_EQ(comm::disk_graph(p, 0.5).num_edges(), 0U);
  EXPECT_EQ(comm::disk_graph(p, 10.0).num_edges(), 10U);
}

TEST(Topology, KNearestDirectedAndSymmetric) {
  const std::vector<Vec2> p{{0.0, 0.0}, {1.0, 0.0}, {3.0, 0.0}, {10.0, 0.0}};
  const g::Graph directed = comm::knn_graph(p, 1, false);
  ASSERT_TRUE(directed.is_directed());
  // Each node listens to exactly one neighbour.
  for (g::NodeId i = 0; i < 4; ++i) {
    EXPECT_EQ(directed.in_neighbors(i).size(), 1U);
  }
  EXPECT_TRUE(directed.has_edge(1, 0));  // 0 listens to 1
  EXPECT_TRUE(directed.has_edge(2, 3));  // 3 listens to 2
  const g::Graph sym = comm::knn_graph(p, 1, true);
  EXPECT_FALSE(sym.is_directed());
  EXPECT_TRUE(sym.has_edge(3, 2));
}

TEST(Topology, DropoutIsSeededAndBounded) {
  const g::Graph full = comm::disk_graph(line_positions(20, 1.0), 100.0);
  std::mt19937_64 a(5);
  std::mt19937_64 b(5);
  const g::Graph da = comm::apply_dropout(full, 0.3, a);
  const g::Graph db = comm::apply_dropout(full, 0.3, b);
  EXPECT_EQ(da, db);
  EXPECT_LT(da.num_edges(), full.num_edges());
  std::mt19937_64 c(1);
  EXPECT_EQ(comm::apply_dropout(full, 0.0, c).num_edges(), full.num_edges());
  EXPECT_EQ(comm::apply_dropout(full, 1.0, c).num_edges(), 0U);
  EXPECT_THROW((void)comm::apply_dropout(full, 1.5, c), std::invalid_argument);
}

TEST(Topology, LineOfSightRemovesBlockedEdges) {
  const std::vector<Vec2> p{{0.0, 0.0}, {4.0, 0.0}, {0.0, 4.0}};
  multirobotcore::math::ObstacleSet obstacles;
  obstacles.boxes.push_back({{1.5, -1.0}, {2.5, 1.0}});  // wall between 0 and 1
  comm::TopologyConfig cfg;
  cfg.radius = 10.0;
  cfg.line_of_sight = true;
  const comm::CommunicationModel model(cfg);
  std::mt19937_64 rng(1);
  const g::Graph graph = model.build(p, rng, {}, &obstacles);
  EXPECT_FALSE(graph.has_edge(0, 1));
  EXPECT_TRUE(graph.has_edge(0, 2));
  EXPECT_TRUE(graph.has_edge(1, 2));
}

TEST(Topology, InactiveRobotsAreIsolated) {
  const auto p = line_positions(4, 1.0);
  comm::TopologyConfig cfg;
  cfg.radius = 1.5;
  const comm::CommunicationModel model(cfg);
  std::mt19937_64 rng(1);
  const bool active[] = {true, false, true, true};
  const g::Graph graph = model.build(p, rng, active);
  EXPECT_TRUE(graph.in_neighbors(1).empty());
  EXPECT_FALSE(g::is_connected(graph));
}

TEST(Topology, FixedModelRequiresGraph) {
  comm::TopologyConfig cfg;
  cfg.model = comm::TopologyModel::kFixed;
  EXPECT_THROW(comm::CommunicationModel{cfg}, std::invalid_argument);
}

TEST(Network, LatencyOrdersDelivery) {
  comm::ChannelConfig cfg;
  cfg.latency = 0.1;
  comm::NetworkSimulator net(3, cfg, 1);
  EXPECT_TRUE(net.send(0, 1, 7, {1.0, 2.0}, 0.0));
  EXPECT_TRUE(net.send(2, 1, 7, {3.0}, 0.05));
  EXPECT_TRUE(net.deliver(0.09).empty());
  const auto first = net.deliver(0.1);
  ASSERT_EQ(first.size(), 1U);
  EXPECT_EQ(first[0].sender, 0U);
  EXPECT_EQ(first[0].payload, (std::vector<double>{1.0, 2.0}));
  EXPECT_DOUBLE_EQ(first[0].delivery_time, 0.1);
  const auto inboxes = net.deliver_inboxes(1.0);
  ASSERT_EQ(inboxes[1].size(), 1U);
  EXPECT_EQ(inboxes[1][0].sender, 2U);
  EXPECT_EQ(net.stats().messages_delivered, 2U);
  EXPECT_EQ(net.stats().bytes_sent, net.message_bytes(2) + net.message_bytes(1));
}

TEST(Network, RandomLossMatchesProbabilityAndIsReproducible) {
  comm::ChannelConfig cfg;
  cfg.drop_probability = 0.2;
  comm::NetworkSimulator a(2, cfg, 99);
  comm::NetworkSimulator b(2, cfg, 99);
  std::vector<bool> outcome_a;
  std::vector<bool> outcome_b;
  for (int i = 0; i < 5000; ++i) {
    outcome_a.push_back(a.send(0, 1, 0, {}, 0.0));
    outcome_b.push_back(b.send(0, 1, 0, {}, 0.0));
  }
  EXPECT_EQ(outcome_a, outcome_b);
  const double loss = static_cast<double>(a.stats().dropped_loss) / 5000.0;
  EXPECT_NEAR(loss, 0.2, 0.03);
}

TEST(Network, OutagesAndRange) {
  comm::ChannelConfig cfg;
  cfg.max_range = 2.0;
  comm::NetworkSimulator net(3, cfg, 3);
  const std::vector<Vec2> p{{0.0, 0.0}, {1.0, 0.0}, {5.0, 0.0}};
  net.set_positions(p);
  net.add_link_outage({0, 1, 1.0, 2.0});
  net.add_node_outage({1, 3.0, 4.0});
  EXPECT_TRUE(net.send(0, 1, 0, {}, 0.5));
  EXPECT_FALSE(net.send(1, 0, 0, {}, 1.5));  // link outage (bidirectional)
  EXPECT_TRUE(net.send(0, 1, 0, {}, 2.0));   // outage interval is half-open
  EXPECT_FALSE(net.send(0, 1, 0, {}, 3.5));  // receiver down
  EXPECT_FALSE(net.send(0, 2, 0, {}, 0.0));  // out of range
  EXPECT_EQ(net.stats().dropped_outage, 2U);
  EXPECT_EQ(net.stats().dropped_range, 1U);
  EXPECT_FALSE(net.link_available(0, 2, 0.0));
  EXPECT_TRUE(net.link_available(0, 1, 5.0));
}

TEST(Network, ReceiverDownAtDeliveryDropsMessage) {
  comm::ChannelConfig cfg;
  cfg.latency = 1.0;
  comm::NetworkSimulator net(2, cfg, 3);
  net.add_node_outage({1, 0.5, 2.0});
  EXPECT_TRUE(net.send(0, 1, 0, {}, 0.0));
  EXPECT_TRUE(net.deliver(5.0).empty());
  EXPECT_EQ(net.stats().dropped_outage, 1U);
  EXPECT_DOUBLE_EQ(net.stats().delivery_ratio(), 0.0);
}
