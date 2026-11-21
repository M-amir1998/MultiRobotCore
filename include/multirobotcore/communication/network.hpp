#pragma once

/// @file network.hpp
/// Lightweight deterministic message-passing simulator.
///
/// Messages carry a sender, receiver, topic and a numeric payload. Each send
/// is subject to (in order): node outages, link outages, range limits and
/// independent random loss; surviving messages are delivered after a fixed
/// latency plus uniform jitter. All randomness comes from an explicitly seeded
/// generator, so a run is reproducible from (configuration, seed, call order).
///
/// Time is continuous simulation time [s]. A "communication round" is an
/// application-level synchronous exchange, counted via mark_round().

#include "multirobotcore/graph/graph.hpp"
#include "multirobotcore/math/types.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <random>
#include <span>
#include <vector>

namespace multirobotcore::communication {

using graph::NodeId;

struct Message {
  NodeId sender{0};
  NodeId receiver{0};
  std::uint32_t topic{0};
  std::vector<double> payload;
  double send_time{0.0};      ///< [s]
  double delivery_time{0.0};  ///< [s]
  std::uint64_t sequence{0};  ///< global send order (deterministic tie-break)
};

struct ChannelConfig {
  double latency{0.0};           ///< fixed one-way delay [s]
  double jitter{0.0};            ///< additional uniform delay in [0, jitter] [s]
  double drop_probability{0.0};  ///< independent loss probability per message
  double max_range{std::numeric_limits<double>::infinity()};  ///< [m]
  std::size_t header_bytes{24};  ///< per-message overhead for byte estimates
};

/// Link {a, b} unavailable in both directions for start <= t < end.
struct LinkOutage {
  NodeId a{0};
  NodeId b{0};
  double start{0.0};
  double end{std::numeric_limits<double>::infinity()};
};

/// Node cannot send or receive for start <= t < end.
struct NodeOutage {
  NodeId node{0};
  double start{0.0};
  double end{std::numeric_limits<double>::infinity()};
};

struct NetworkStats {
  std::size_t messages_sent{0};  ///< send attempts
  std::size_t messages_delivered{0};
  std::size_t dropped_loss{0};  ///< random loss
  std::size_t dropped_range{0};
  std::size_t dropped_outage{0};  ///< node or link outage
  std::size_t bytes_sent{0};
  std::size_t bytes_delivered{0};
  std::size_t rounds{0};

  [[nodiscard]] std::size_t dropped() const noexcept {
    return dropped_loss + dropped_range + dropped_outage;
  }
  [[nodiscard]] double delivery_ratio() const noexcept {
    return messages_sent == 0
               ? 1.0
               : static_cast<double>(messages_delivered) / static_cast<double>(messages_sent);
  }
};

class NetworkSimulator {
 public:
  NetworkSimulator(std::size_t num_nodes, ChannelConfig config, std::uint64_t seed);

  [[nodiscard]] std::size_t num_nodes() const noexcept { return num_nodes_; }
  [[nodiscard]] const ChannelConfig& config() const noexcept { return config_; }

  /// Positions used for range checks (required when max_range is finite).
  void set_positions(std::span<const Vec2> positions);

  void add_link_outage(const LinkOutage& outage);
  void add_node_outage(const NodeOutage& outage);

  /// Deterministic availability (outages and range), excluding random loss.
  [[nodiscard]] bool link_available(NodeId sender, NodeId receiver, double time) const;
  [[nodiscard]] bool node_available(NodeId node, double time) const;

  /// Sends one message. Returns false if it was dropped at send time.
  bool send(NodeId sender, NodeId receiver, std::uint32_t topic, std::vector<double> payload,
            double now);

  /// Sends the same payload to every receiver; returns the number accepted.
  std::size_t broadcast(NodeId sender, std::span<const NodeId> receivers, std::uint32_t topic,
                        const std::vector<double>& payload, double now);

  /// Removes and returns every message with delivery_time <= now, ordered by
  /// (delivery_time, sequence). Messages whose receiver is down at delivery
  /// time are dropped.
  [[nodiscard]] std::vector<Message> deliver(double now);

  /// Like deliver(), grouped into one inbox per receiver.
  [[nodiscard]] std::vector<std::vector<Message>> deliver_inboxes(double now);

  void mark_round() noexcept { ++stats_.rounds; }

  [[nodiscard]] std::size_t in_flight() const noexcept { return pending_.size(); }
  [[nodiscard]] const NetworkStats& stats() const noexcept { return stats_; }

  [[nodiscard]] std::size_t message_bytes(std::size_t payload_size) const noexcept {
    return config_.header_bytes + payload_size * sizeof(double);
  }

 private:
  struct Later {
    bool operator()(const Message& a, const Message& b) const noexcept {
      if (a.delivery_time != b.delivery_time) {
        return a.delivery_time > b.delivery_time;
      }
      return a.sequence > b.sequence;
    }
  };

  void check_node(NodeId node) const;

  std::size_t num_nodes_;
  ChannelConfig config_;
  std::mt19937_64 rng_;
  std::vector<Vec2> positions_;
  std::vector<LinkOutage> link_outages_;
  std::vector<NodeOutage> node_outages_;
  std::priority_queue<Message, std::vector<Message>, Later> pending_;
  std::uint64_t next_sequence_{0};
  NetworkStats stats_;
};

}  // namespace multirobotcore::communication
