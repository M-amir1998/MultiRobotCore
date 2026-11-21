#include "multirobotcore/consensus/consensus.hpp"

#include "multirobotcore/graph/connectivity.hpp"
#include "multirobotcore/graph/matrices.hpp"
#include "multirobotcore/math/numerics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace multirobotcore::consensus {
namespace {

Eigen::Index idx(std::size_t i) {
  return static_cast<Eigen::Index>(i);
}

void check_rows(const graph::Graph& graph, const StateMatrix& x, const char* what) {
  if (static_cast<std::size_t>(x.rows()) != graph.num_nodes()) {
    throw std::invalid_argument(std::string(what) + ": state rows must equal graph size");
  }
}

/// Computes -L X, i.e. row i = sum_j a_ij (x_j - x_i), using neighbour lists.
StateMatrix negative_laplacian_apply(const graph::Graph& graph, const StateMatrix& x) {
  StateMatrix out = StateMatrix::Zero(x.rows(), x.cols());
  for (graph::NodeId i = 0; i < graph.num_nodes(); ++i) {
    for (const graph::Neighbor& j : graph.in_neighbors(i)) {
      out.row(idx(i)) += j.weight * (x.row(idx(j.id)) - x.row(idx(i)));
    }
  }
  return out;
}

}  // namespace

double max_convex_step(const graph::Graph& graph) {
  const double d = graph::max_in_degree(graph);
  return d > 0.0 ? 1.0 / d : std::numeric_limits<double>::infinity();
}

StateMatrix consensus_step(const graph::Graph& graph, const StateMatrix& x, double epsilon) {
  check_rows(graph, x, "consensus_step");
  if (!(epsilon > 0.0)) {
    throw std::invalid_argument("consensus_step: epsilon must be positive");
  }
  return x + epsilon * negative_laplacian_apply(graph, x);
}

StateMatrix consensus_flow_rk4(const graph::Graph& graph, const StateMatrix& x, double dt) {
  check_rows(graph, x, "consensus_flow_rk4");
  const StateMatrix k1 = negative_laplacian_apply(graph, x);
  const StateMatrix k2 = negative_laplacian_apply(graph, x + 0.5 * dt * k1);
  const StateMatrix k3 = negative_laplacian_apply(graph, x + 0.5 * dt * k2);
  const StateMatrix k4 = negative_laplacian_apply(graph, x + dt * k3);
  return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

StateMatrix leader_follower_step(const graph::Graph& graph, const StateMatrix& x,
                                 std::span<const double> pinning_gains,
                                 const Eigen::RowVectorXd& reference, double epsilon) {
  check_rows(graph, x, "leader_follower_step");
  if (pinning_gains.size() != graph.num_nodes()) {
    throw std::invalid_argument("leader_follower_step: one pinning gain per agent required");
  }
  if (reference.size() != x.cols()) {
    throw std::invalid_argument("leader_follower_step: reference dimension mismatch");
  }
  StateMatrix dx = negative_laplacian_apply(graph, x);
  for (graph::NodeId i = 0; i < graph.num_nodes(); ++i) {
    if (pinning_gains[i] < 0.0) {
      throw std::invalid_argument("leader_follower_step: pinning gains must be non-negative");
    }
    dx.row(idx(i)) += pinning_gains[i] * (reference - x.row(idx(i)));
  }
  return x + epsilon * dx;
}

SecondOrderState second_order_step(const graph::Graph& graph, const SecondOrderState& state,
                                   double gamma, double dt) {
  check_rows(graph, state.position, "second_order_step");
  check_rows(graph, state.velocity, "second_order_step");
  auto accel = [&](const StateMatrix& p, const StateMatrix& v) {
    return StateMatrix(negative_laplacian_apply(graph, p) +
                       gamma * negative_laplacian_apply(graph, v));
  };
  const StateMatrix& p = state.position;
  const StateMatrix& v = state.velocity;
  const StateMatrix k1p = v;
  const StateMatrix k1v = accel(p, v);
  const StateMatrix k2p = v + 0.5 * dt * k1v;
  const StateMatrix k2v = accel(p + 0.5 * dt * k1p, v + 0.5 * dt * k1v);
  const StateMatrix k3p = v + 0.5 * dt * k2v;
  const StateMatrix k3v = accel(p + 0.5 * dt * k2p, v + 0.5 * dt * k2v);
  const StateMatrix k4p = v + dt * k3v;
  const StateMatrix k4v = accel(p + dt * k3p, v + dt * k3v);
  return SecondOrderState{p + (dt / 6.0) * (k1p + 2.0 * k2p + 2.0 * k3p + k4p),
                          v + (dt / 6.0) * (k1v + 2.0 * k2v + 2.0 * k3v + k4v)};
}

Eigen::RowVectorXd average(const StateMatrix& x) {
  if (x.rows() == 0) {
    return Eigen::RowVectorXd::Zero(x.cols());
  }
  return x.colwise().mean();
}

double disagreement(const StateMatrix& x) {
  if (x.rows() == 0) {
    return 0.0;
  }
  const Eigen::RowVectorXd mean = average(x);
  return (x.rowwise() - mean).rowwise().norm().maxCoeff();
}

double disagreement(const StateMatrix& x, std::span<const graph::NodeId> members) {
  if (members.empty()) {
    return 0.0;
  }
  Eigen::RowVectorXd mean = Eigen::RowVectorXd::Zero(x.cols());
  for (const graph::NodeId i : members) {
    mean += x.row(idx(i));
  }
  mean /= static_cast<double>(members.size());
  double worst = 0.0;
  for (const graph::NodeId i : members) {
    worst = std::max(worst, (x.row(idx(i)) - mean).norm());
  }
  return worst;
}

double discrete_contraction_factor(const graph::Graph& graph, double epsilon) {
  if (graph.num_nodes() < 2) {
    return 0.0;
  }
  const Eigen::VectorXd s = graph::laplacian_spectrum(graph);
  return std::max(std::abs(1.0 - epsilon * s(1)), std::abs(1.0 - epsilon * s(s.size() - 1)));
}

std::vector<Eigen::Vector2d> rendezvous_velocities(const graph::Graph& graph,
                                                   std::span<const Eigen::Vector2d> positions,
                                                   double gain, double max_speed) {
  if (positions.size() != graph.num_nodes()) {
    throw std::invalid_argument("rendezvous_velocities: one position per agent required");
  }
  std::vector<Eigen::Vector2d> u(positions.size(), Eigen::Vector2d::Zero());
  for (graph::NodeId i = 0; i < graph.num_nodes(); ++i) {
    for (const graph::Neighbor& j : graph.in_neighbors(i)) {
      u[i] += j.weight * (positions[j.id] - positions[i]);
    }
    u[i] = math::clamp_norm(gain * u[i], max_speed);
  }
  return u;
}

std::vector<Eigen::Vector2d> rendezvous_with_separation_velocities(
    const graph::Graph& graph, std::span<const Eigen::Vector2d> positions, double gain,
    double safety_distance, double repulsion_gain, double max_speed) {
  if (positions.size() != graph.num_nodes()) {
    throw std::invalid_argument("rendezvous_with_separation_velocities: size mismatch");
  }
  if (!(safety_distance > 0.0) || repulsion_gain < 0.0) {
    throw std::invalid_argument(
        "rendezvous_with_separation_velocities: invalid safety distance or repulsion gain");
  }
  std::vector<Eigen::Vector2d> u(positions.size(), Eigen::Vector2d::Zero());
  for (graph::NodeId i = 0; i < graph.num_nodes(); ++i) {
    // Cohesion is the unmodified consensus term, saturated to max_speed before
    // the barrier is added: the barrier is unbounded as d_ij -> 0, so it
    // dominates at contact regardless of how many neighbours attract.
    Eigen::Vector2d attraction = Eigen::Vector2d::Zero();
    for (const graph::Neighbor& j : graph.in_neighbors(i)) {
      attraction += j.weight * (positions[j.id] - positions[i]);
    }
    attraction = math::clamp_norm(gain * attraction, max_speed);
    Eigen::Vector2d repulsion = Eigen::Vector2d::Zero();
    for (std::size_t j = 0; j < positions.size(); ++j) {
      if (j == i) {
        continue;
      }
      const Eigen::Vector2d d = positions[i] - positions[j];
      const double dist = std::max(d.norm(), 1e-6);
      if (dist < safety_distance) {
        repulsion += repulsion_gain * (1.0 / dist - 1.0 / safety_distance) * d / dist;
      }
    }
    u[i] = math::clamp_norm(attraction + repulsion, max_speed);
  }
  return u;
}

ConsensusRun run_consensus(const graph::Graph& graph, const StateMatrix& x0,
                           const ConsensusRunConfig& config) {
  check_rows(graph, x0, "run_consensus");
  ConsensusRun run;
  StateMatrix x = x0;
  run.disagreement.push_back(disagreement(x));
  if (config.record_history) {
    run.history.push_back(x);
  }
  while (run.iterations < config.max_iterations && run.disagreement.back() > config.tolerance) {
    x = consensus_step(graph, x, config.epsilon);
    ++run.iterations;
    run.disagreement.push_back(disagreement(x));
    if (config.record_history) {
      run.history.push_back(x);
    }
  }
  run.converged = run.disagreement.back() <= config.tolerance;
  run.final_state = x;
  return run;
}

}  // namespace multirobotcore::consensus
