#pragma once

/// @file consensus.hpp
/// First- and second-order consensus protocols on (possibly weighted,
/// directed) communication graphs.
///
/// States are stored row-wise: X is N x d, row i is agent i's d-dimensional
/// state (d = 1 for scalar agreement, d = 2 for planar rendezvous).
///
/// Continuous time (Olfati-Saber & Murray 2004):
///     x_i' = sum_j a_ij (x_j - x_i)         <=>   X' = -L X
/// Discrete time (Olfati-Saber, Fax & Murray 2007):
///     X(k+1) = (I - eps L) X(k) = P X(k),   0 < eps <= 1 / max_i d_i
///
/// Guarantees (fixed graph): agreement iff the graph has a directed spanning
/// tree (Ren & Beard 2005); for undirected/balanced graphs the agreement value
/// is the initial average and the disagreement decays like exp(-lambda_2 t)
/// (continuous) or (1 - eps lambda_2)^k with eps <= 1/d_max (discrete).

#include "multirobotcore/graph/graph.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <span>
#include <vector>

namespace multirobotcore::consensus {

using StateMatrix = Eigen::MatrixXd;

/// Largest step 1 / max_i d_i for which P = I - eps L is non-negative and row
/// stochastic (each update is then a convex combination of neighbour states).
/// Returns +infinity for edgeless graphs.
///
/// Convergence on a connected graph requires the strict inequality
/// eps < 1 / max_i d_i (Olfati-Saber, Fax & Murray 2007): at equality P can
/// have the eigenvalue -1 (e.g. even cycles and other bipartite regular
/// graphs), and the iteration then oscillates forever.
[[nodiscard]] double max_convex_step(const graph::Graph& graph);

/// One discrete-time step X <- X - eps L X.
[[nodiscard]] StateMatrix consensus_step(const graph::Graph& graph, const StateMatrix& x,
                                         double epsilon);

/// One RK4 step of X' = -L X over dt.
[[nodiscard]] StateMatrix consensus_flow_rk4(const graph::Graph& graph, const StateMatrix& x,
                                             double dt);

/// Leader-follower (pinning) consensus, one discrete step:
///   x_i <- x_i + eps [ sum_j a_ij (x_j - x_i) + b_i (r - x_i) ]
/// where r is the leader/reference state and b_i >= 0 the pinning gains
/// (b_i > 0 for agents that observe the leader). All agents converge to a
/// constant r iff every agent is reachable from the pinned set. Requires
/// eps <= 1 / max_i (d_i + b_i) for a convex update.
[[nodiscard]] StateMatrix leader_follower_step(const graph::Graph& graph, const StateMatrix& x,
                                               std::span<const double> pinning_gains,
                                               const Eigen::RowVectorXd& reference, double epsilon);

/// Second-order consensus for double integrators (Ren 2008):
///   p_i'' = sum_j a_ij [ (p_j - p_i) + gamma (v_j - v_i) ]
struct SecondOrderState {
  StateMatrix position;
  StateMatrix velocity;
};

/// One RK4 step of the second-order protocol.
[[nodiscard]] SecondOrderState second_order_step(const graph::Graph& graph,
                                                 const SecondOrderState& state, double gamma,
                                                 double dt);

/// Mean of the rows of X.
[[nodiscard]] Eigen::RowVectorXd average(const StateMatrix& x);

/// Disagreement max_i ||x_i - mean(X)||.
[[nodiscard]] double disagreement(const StateMatrix& x);

/// Disagreement restricted to `members` (e.g. one connected component).
[[nodiscard]] double disagreement(const StateMatrix& x, std::span<const graph::NodeId> members);

/// Rate bound for discrete consensus on an undirected graph:
/// max(|1 - eps lambda_2|, |1 - eps lambda_N|). The disagreement vector norm
/// contracts at least by this factor per step.
[[nodiscard]] double discrete_contraction_factor(const graph::Graph& graph, double epsilon);

/// Rendezvous (spatial consensus) velocity commands for single-integrator
/// robots: u_i = gain * sum_j a_ij (p_j - p_i), saturated to max_speed.
[[nodiscard]] std::vector<Eigen::Vector2d> rendezvous_velocities(
    const graph::Graph& graph, std::span<const Eigen::Vector2d> positions, double gain,
    double max_speed);

/// Practical rendezvous for physical (disc-shaped) robots: the consensus
/// cohesion of rendezvous_velocities() saturated to max_speed, plus a
/// short-range barrier repulsion (unbounded at contact, so it dominates there)
///   + k_rep * sum_{j : d_ij < d_s} (1/d_ij - 1/d_s) (p_i - p_j) / d_ij
/// for every robot j closer than the safety distance d_s (all robots, not only
/// communication neighbours: proximity is assumed to be sensed). The
/// equilibria are compact clusters rather than exact point agreement. This is
/// an engineering variant in the spirit of cohesion/separation flocking
/// (Olfati-Saber 2006), without its convergence guarantees.
[[nodiscard]] std::vector<Eigen::Vector2d> rendezvous_with_separation_velocities(
    const graph::Graph& graph, std::span<const Eigen::Vector2d> positions, double gain,
    double safety_distance, double repulsion_gain, double max_speed);

struct ConsensusRunConfig {
  double epsilon{0.1};
  std::size_t max_iterations{1000};
  double tolerance{1e-6};  ///< stop when disagreement <= tolerance
  bool record_history{true};
};

struct ConsensusRun {
  std::vector<StateMatrix> history;  ///< X(0), X(1), ... if recorded
  std::vector<double> disagreement;  ///< per iteration, including k = 0
  std::size_t iterations{0};
  bool converged{false};
  StateMatrix final_state;
};

/// Runs synchronous discrete consensus on a fixed graph until convergence.
[[nodiscard]] ConsensusRun run_consensus(const graph::Graph& graph, const StateMatrix& x0,
                                         const ConsensusRunConfig& config);

}  // namespace multirobotcore::consensus
