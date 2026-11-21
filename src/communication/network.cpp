#include "multirobotcore/communication/network.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace multirobotcore::communication {

NetworkSimulator::NetworkSimulator(std::size_t num_nodes, ChannelConfig config, std::uint64_t seed)
    : num_nodes_(num_nodes), config_(config), rng_(seed) {
  if (config_.latency < 0.0 || config_.jitter < 0.0) {
    throw std::invalid_argument("NetworkSimulator: latency and jitter must be non-negative");
  }
  if (config_.drop_probability < 0.0 || config_.drop_probability > 1.0) {
    throw std::invalid_argument("NetworkSimulator: drop probability must be in [0, 1]");
  }
}

void NetworkSimulator::check_node(NodeId node) const {
  if (node >= num_nodes_) {
    throw std::out_of_range("NetworkSimulator: node " + std::to_string(node) + " out of range");
  }
}

void NetworkSimulator::set_positions(std::span<const Vec2> positions) {
  if (positions.size() != num_nodes_) {
    throw std::invalid_argument("NetworkSimulator::set_positions: size mismatch");
  }
  positions_.assign(positions.begin(), positions.end());
}

void NetworkSimulator::add_link_outage(const LinkOutage& outage) {
  check_node(outage.a);
  check_node(outage.b);
  link_outages_.push_back(outage);
}

void NetworkSimulator::add_node_outage(const NodeOutage& outage) {
  check_node(outage.node);
  node_outages_.push_back(outage);
}

bool NetworkSimulator::node_available(NodeId node, double time) const {
  check_node(node);
  return std::none_of(node_outages_.begin(), node_outages_.end(), [&](const NodeOutage& o) {
    return o.node == node && time >= o.start && time < o.end;
  });
}

bool NetworkSimulator::link_available(NodeId sender, NodeId receiver, double time) const {
  if (!node_available(sender, time) || !node_available(receiver, time)) {
    return false;
  }
  const bool link_down =
      std::any_of(link_outages_.begin(), link_outages_.end(), [&](const LinkOutage& o) {
        const bool same = (o.a == sender && o.b == receiver) || (o.a == receiver && o.b == sender);
        return same && time >= o.start && time < o.end;
      });
  if (link_down) {
    return false;
  }
  if (std::isfinite(config_.max_range)) {
    if (positions_.size() != num_nodes_) {
      throw std::logic_error("NetworkSimulator: positions required for range-limited channel");
    }
    return (positions_[sender] - positions_[receiver]).norm() <= config_.max_range;
  }
  return true;
}

bool NetworkSimulator::send(NodeId sender, NodeId receiver, std::uint32_t topic,
                            std::vector<double> payload, double now) {
  check_node(sender);
  check_node(receiver);
  ++stats_.messages_sent;
  stats_.bytes_sent += message_bytes(payload.size());

  // The loss and jitter draws are consumed for every send attempt, so the
  // random stream does not depend on deterministic outages or range drops.
  std::bernoulli_distribution lose(config_.drop_probability);
  const bool lost = lose(rng_);
  std::uniform_real_distribution<double> jitter(0.0, config_.jitter);
  const double delay = config_.latency + (config_.jitter > 0.0 ? jitter(rng_) : 0.0);

  if (!link_available(sender, receiver, now)) {
    const bool out_of_range =
        node_available(sender, now) && node_available(receiver, now) &&
        std::isfinite(config_.max_range) &&
        (positions_[sender] - positions_[receiver]).norm() > config_.max_range;
    if (out_of_range) {
      ++stats_.dropped_range;
    } else {
      ++stats_.dropped_outage;
    }
    return false;
  }
  if (lost) {
    ++stats_.dropped_loss;
    return false;
  }
  pending_.push(
      Message{sender, receiver, topic, std::move(payload), now, now + delay, next_sequence_++});
  return true;
}

std::size_t NetworkSimulator::broadcast(NodeId sender, std::span<const NodeId> receivers,
                                        std::uint32_t topic, const std::vector<double>& payload,
                                        double now) {
  std::size_t accepted = 0;
  for (const NodeId r : receivers) {
    if (send(sender, r, topic, payload, now)) {
      ++accepted;
    }
  }
  return accepted;
}

std::vector<Message> NetworkSimulator::deliver(double now) {
  std::vector<Message> out;
  while (!pending_.empty() && pending_.top().delivery_time <= now) {
    Message m = pending_.top();
    pending_.pop();
    if (!node_available(m.receiver, m.delivery_time)) {
      ++stats_.dropped_outage;
      continue;
    }
    ++stats_.messages_delivered;
    stats_.bytes_delivered += message_bytes(m.payload.size());
    out.push_back(std::move(m));
  }
  return out;
}

std::vector<std::vector<Message>> NetworkSimulator::deliver_inboxes(double now) {
  std::vector<std::vector<Message>> inboxes(num_nodes_);
  for (Message& m : deliver(now)) {
    inboxes[m.receiver].push_back(std::move(m));
  }
  return inboxes;
}

}  // namespace multirobotcore::communication
