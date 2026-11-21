#pragma once

/// @file distributed.hpp
/// Message-passing implementations over communication::NetworkSimulator:
/// asynchronous-information consensus under loss/latency and deterministic
/// leader election.

#include "multirobotcore/communication/network.hpp"
#include "multirobotcore/consensus/consensus.hpp"
#include "multirobotcore/graph/graph.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace multirobotcore::consensus {

/// How an agent treats a neighbour whose message did not arrive this round.
enum class MissingPolicy {
  kSkip,             ///< omit the neighbour term this round
  kUseLastReceived,  ///< reuse the most recent value received (stale information)
};

struct DistributedConsensusConfig {
  double epsilon{0.1};       ///< step size (should satisfy eps <= 1 / d_max)
  double round_period{0.1};  ///< time between synchronous rounds [s]
  MissingPolicy missing{MissingPolicy::kSkip};
  std::uint32_t topic{1};
};

/// Synchronous-round consensus where every exchange goes through the network
/// simulator. Each round t_k = k * round_period:
///   1. every operational agent broadcasts x_i to its in-neighbours' senders
///      (i.e. to every agent that listens to it);
///   2. messages delivered by t_k + round_period are applied:
///        x_i <- x_i + eps sum_{j received} a_ij (x_j - x_i).
/// With losses the effective graph differs per round and is generally not
/// balanced, so the average is not preserved exactly; agreement still occurs
/// when the union of the effective graphs is jointly connected over bounded
/// intervals (Jadbabaie, Lin & Morse 2003; Ren & Beard 2005). Latencies longer
/// than round_period behave like lost messages under kSkip.
class DistributedConsensus {
 public:
  DistributedConsensus(graph::Graph graph, StateMatrix initial, DistributedConsensusConfig config);

  /// Executes one round using `network`; returns the new state.
  const StateMatrix& round(communication::NetworkSimulator& network);

  /// Replaces the communication graph (topology changes, link failures).
  void set_graph(graph::Graph graph);

  /// Marks an agent as failed: it stops sending and updating.
  void set_active(graph::NodeId agent, bool active);

  [[nodiscard]] const StateMatrix& state() const noexcept { return x_; }
  [[nodiscard]] std::size_t rounds() const noexcept { return round_; }
  [[nodiscard]] double time() const noexcept;

 private:
  graph::Graph graph_;
  StateMatrix x_;
  DistributedConsensusConfig config_;
  std::vector<bool> active_;
  // last_[i][k]: last value agent i received from its k-th in-neighbour.
  std::vector<std::vector<std::optional<Eigen::RowVectorXd>>> last_;
  std::size_t round_{0};
};

struct LeaderElectionResult {
  /// Leader each agent believes in (nullopt for failed agents).
  std::vector<std::optional<graph::NodeId>> leader;
  std::size_t rounds{0};
  bool agreed{false};  ///< all operational agents in each component agree
};

/// Minimum-identifier flooding election (FloodMin; Lynch 1996, ch. 4):
/// every operational agent repeatedly broadcasts the smallest identifier it
/// has seen. After diameter rounds of reliable communication each connected
/// component agrees on its smallest operational identifier. Runs for
/// `max_rounds` rounds (use at least the graph diameter; more rounds add
/// robustness to message loss).
[[nodiscard]] LeaderElectionResult elect_leader_min_id(const graph::Graph& graph,
                                                       const std::vector<bool>& operational,
                                                       communication::NetworkSimulator& network,
                                                       std::size_t max_rounds,
                                                       double start_time = 0.0,
                                                       double round_period = 0.1);

}  // namespace multirobotcore::consensus
