#include "multirobotcore/formation/controllers.hpp"

#include "multirobotcore/formation/shapes.hpp"
#include "multirobotcore/math/numerics.hpp"

#include <stdexcept>

namespace multirobotcore::formation {

std::vector<Vec2> leader_follower_velocities(const Pose2& leader, const Vec2& leader_velocity,
                                             std::size_t leader_index,
                                             std::span<const Vec2> positions,
                                             std::span<const Vec2> offsets_from_leader,
                                             const LeaderFollowerConfig& config) {
  if (positions.size() != offsets_from_leader.size()) {
    throw std::invalid_argument("leader_follower_velocities: one offset per robot required");
  }
  if (leader_index >= positions.size()) {
    throw std::out_of_range("leader_follower_velocities: leader index out of range");
  }
  std::vector<Vec2> u(positions.size(), Vec2::Zero());
  const Vec2 feedforward = config.feedforward ? leader_velocity : Vec2::Zero();
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (i == leader_index) {
      continue;
    }
    const Vec2 slot = to_world(leader, offsets_from_leader[i]);
    u[i] = math::clamp_norm(feedforward + config.gain * (slot - positions[i]), config.max_speed);
  }
  return u;
}

Vec2 consensus_formation_velocity(std::size_t robot, const graph::Graph& graph,
                                  std::span<const Vec2> positions, std::span<const Vec2> offsets,
                                  const Pose2& reference, const Vec2& reference_velocity,
                                  double pinning, const ConsensusFormationConfig& config) {
  const std::size_t n = graph.num_nodes();
  if (positions.size() != n || offsets.size() != n) {
    throw std::invalid_argument("consensus_formation_velocity: size mismatch");
  }
  if (robot >= n) {
    throw std::out_of_range("consensus_formation_velocity: robot index out of range");
  }
  Vec2 coupling = Vec2::Zero();
  for (const graph::Neighbor& j : graph.in_neighbors(robot)) {
    const Vec2 desired = math::rotate(offsets[j.id] - offsets[robot], reference.heading);
    coupling += j.weight * ((positions[j.id] - positions[robot]) - desired);
  }
  Vec2 command = reference_velocity + config.gain * coupling;
  if (pinning > 0.0) {
    command +=
        pinning * config.pinning_gain * (to_world(reference, offsets[robot]) - positions[robot]);
  }
  return math::clamp_norm(command, config.max_speed);
}

std::vector<Vec2> consensus_formation_velocities(
    const graph::Graph& graph, std::span<const Vec2> positions, std::span<const Vec2> offsets,
    const Pose2& reference, const Vec2& reference_velocity, std::span<const double> pinning,
    const ConsensusFormationConfig& config) {
  const std::size_t n = graph.num_nodes();
  if (!pinning.empty() && pinning.size() != n) {
    throw std::invalid_argument("consensus_formation_velocities: pinning size mismatch");
  }
  std::vector<Vec2> u(n, Vec2::Zero());
  for (graph::NodeId i = 0; i < n; ++i) {
    u[i] = consensus_formation_velocity(i, graph, positions, offsets, reference, reference_velocity,
                                        pinning.empty() ? 0.0 : pinning[i], config);
  }
  return u;
}

std::vector<Vec2> distance_formation_velocities(const graph::Graph& graph,
                                                std::span<const Vec2> positions,
                                                std::span<const Vec2> offsets,
                                                const DistanceFormationConfig& config) {
  const std::size_t n = graph.num_nodes();
  if (positions.size() != n || offsets.size() != n) {
    throw std::invalid_argument("distance_formation_velocities: size mismatch");
  }
  std::vector<Vec2> u(n, Vec2::Zero());
  for (graph::NodeId i = 0; i < n; ++i) {
    Vec2 command = Vec2::Zero();
    for (const graph::Neighbor& j : graph.in_neighbors(i)) {
      const double desired_sq = (offsets[i] - offsets[j.id]).squaredNorm();
      const Vec2 rel = positions[j.id] - positions[i];
      command += (rel.squaredNorm() - desired_sq) * rel;
    }
    u[i] = math::clamp_norm(config.gain * command, config.max_speed);
  }
  return u;
}

}  // namespace multirobotcore::formation
