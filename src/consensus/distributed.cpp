#include "multirobotcore/consensus/distributed.hpp"

#include "multirobotcore/graph/connectivity.hpp"

#include <algorithm>
#include <map>
#include <stdexcept>
#include <utility>

namespace multirobotcore::consensus {
namespace {

Eigen::Index idx(std::size_t i) {
  return static_cast<Eigen::Index>(i);
}

std::vector<double> row_to_vector(const Eigen::RowVectorXd& row) {
  return {row.data(), row.data() + row.size()};
}

}  // namespace

DistributedConsensus::DistributedConsensus(graph::Graph graph, StateMatrix initial,
                                           DistributedConsensusConfig config)
    : graph_(std::move(graph)), x_(std::move(initial)), config_(config) {
  if (static_cast<std::size_t>(x_.rows()) != graph_.num_nodes()) {
    throw std::invalid_argument("DistributedConsensus: state rows must equal graph size");
  }
  if (!(config_.epsilon > 0.0) || !(config_.round_period > 0.0)) {
    throw std::invalid_argument("DistributedConsensus: epsilon and round period must be positive");
  }
  active_.assign(graph_.num_nodes(), true);
  last_.assign(graph_.num_nodes(), {});
  for (graph::NodeId i = 0; i < graph_.num_nodes(); ++i) {
    last_[i].assign(graph_.in_neighbors(i).size(), std::nullopt);
  }
}

double DistributedConsensus::time() const noexcept {
  return static_cast<double>(round_) * config_.round_period;
}

void DistributedConsensus::set_graph(graph::Graph graph) {
  if (graph.num_nodes() != graph_.num_nodes()) {
    throw std::invalid_argument("DistributedConsensus::set_graph: size mismatch");
  }
  // Carry over the stale values of neighbours that remain neighbours.
  std::vector<std::vector<std::optional<Eigen::RowVectorXd>>> last(graph.num_nodes());
  for (graph::NodeId i = 0; i < graph.num_nodes(); ++i) {
    const auto& new_nbrs = graph.in_neighbors(i);
    const auto& old_nbrs = graph_.in_neighbors(i);
    last[i].assign(new_nbrs.size(), std::nullopt);
    for (std::size_t k = 0; k < new_nbrs.size(); ++k) {
      for (std::size_t m = 0; m < old_nbrs.size(); ++m) {
        if (old_nbrs[m].id == new_nbrs[k].id) {
          last[i][k] = last_[i][m];
        }
      }
    }
  }
  graph_ = std::move(graph);
  last_ = std::move(last);
}

void DistributedConsensus::set_active(graph::NodeId agent, bool active) {
  active_.at(agent) = active;
}

const StateMatrix& DistributedConsensus::round(communication::NetworkSimulator& network) {
  if (network.num_nodes() != graph_.num_nodes()) {
    throw std::invalid_argument("DistributedConsensus::round: network size mismatch");
  }
  const double t = time();
  // 1. Broadcast: agent i sends to every agent that listens to it.
  for (graph::NodeId i = 0; i < graph_.num_nodes(); ++i) {
    if (!active_[i]) {
      continue;
    }
    const std::vector<double> payload = row_to_vector(x_.row(idx(i)));
    for (const graph::Neighbor& listener : graph_.out_neighbors(i)) {
      network.send(i, listener.id, config_.topic, payload, t);
    }
  }
  // 2. Collect everything delivered before the next round; keep the most
  //    recently sent value per (receiver, sender).
  const auto inboxes = network.deliver_inboxes(t + config_.round_period);
  StateMatrix next = x_;
  for (graph::NodeId i = 0; i < graph_.num_nodes(); ++i) {
    if (!active_[i]) {
      continue;
    }
    std::map<graph::NodeId, std::pair<double, const std::vector<double>*>> latest;
    for (const auto& m : inboxes[i]) {
      if (m.topic != config_.topic) {
        continue;
      }
      auto it = latest.find(m.sender);
      if (it == latest.end() || m.send_time >= it->second.first) {
        latest[m.sender] = {m.send_time, &m.payload};
      }
    }
    Eigen::RowVectorXd dx = Eigen::RowVectorXd::Zero(x_.cols());
    const auto& nbrs = graph_.in_neighbors(i);
    for (std::size_t k = 0; k < nbrs.size(); ++k) {
      const auto it = latest.find(nbrs[k].id);
      if (it != latest.end()) {
        const auto& v = *it->second.second;
        last_[i][k] =
            Eigen::Map<const Eigen::RowVectorXd>(v.data(), static_cast<Eigen::Index>(v.size()));
      } else if (config_.missing == MissingPolicy::kSkip) {
        continue;
      }
      if (last_[i][k]) {
        dx += nbrs[k].weight * (*last_[i][k] - x_.row(idx(i)));
      }
    }
    next.row(idx(i)) += config_.epsilon * dx;
  }
  x_ = std::move(next);
  network.mark_round();
  ++round_;
  return x_;
}

LeaderElectionResult elect_leader_min_id(const graph::Graph& graph,
                                         const std::vector<bool>& operational,
                                         communication::NetworkSimulator& network,
                                         std::size_t max_rounds, double start_time,
                                         double round_period) {
  const std::size_t n = graph.num_nodes();
  if (operational.size() != n || network.num_nodes() != n) {
    throw std::invalid_argument("elect_leader_min_id: size mismatch");
  }
  constexpr std::uint32_t kTopic = 0xE1EC7;
  LeaderElectionResult result;
  result.leader.assign(n, std::nullopt);
  for (graph::NodeId i = 0; i < n; ++i) {
    if (operational[i]) {
      result.leader[i] = i;
    }
  }
  for (std::size_t r = 0; r < max_rounds; ++r) {
    const double t = start_time + static_cast<double>(r) * round_period;
    for (graph::NodeId i = 0; i < n; ++i) {
      if (!operational[i]) {
        continue;
      }
      const std::vector<double> payload{static_cast<double>(*result.leader[i])};
      for (const graph::Neighbor& listener : graph.out_neighbors(i)) {
        network.send(i, listener.id, kTopic, payload, t);
      }
    }
    const auto inboxes = network.deliver_inboxes(t + round_period);
    for (graph::NodeId i = 0; i < n; ++i) {
      if (!operational[i]) {
        continue;
      }
      for (const auto& m : inboxes[i]) {
        if (m.topic == kTopic && !m.payload.empty()) {
          const auto candidate = static_cast<graph::NodeId>(m.payload.front());
          result.leader[i] = std::min(*result.leader[i], candidate);
        }
      }
    }
    network.mark_round();
    ++result.rounds;
  }
  // Agreement check per connected component of the operational subgraph.
  graph::Graph alive = graph;
  for (graph::NodeId i = 0; i < n; ++i) {
    if (!operational[i]) {
      alive.isolate_node(i);
    }
  }
  result.agreed = true;
  for (const auto& component : graph::connected_components(alive)) {
    std::optional<graph::NodeId> expected;
    for (const graph::NodeId i : component) {
      if (operational[i]) {
        expected = expected ? std::min(*expected, i) : i;
      }
    }
    for (const graph::NodeId i : component) {
      if (operational[i] && result.leader[i] != expected) {
        result.agreed = false;
      }
    }
  }
  return result;
}

}  // namespace multirobotcore::consensus
