#pragma once

/// @file controllers.hpp
/// Formation controllers for single-integrator robots (velocity commands).
///
/// Leader-follower (Desai, Ostrowski & Kumar 2001; Oh, Park & Ahn 2015):
///   u_i = v_L + k (p_L + R(theta_L) d_i - p_i)
/// Displacement-based consensus formation (Ren & Beard 2008; Oh, Park &
/// Ahn 2015):
///   u_i = v_r + k sum_j a_ij [(p_j - p_i) - R (d_j - d_i)]
///             + k_p b_i (p_r + R d_i - p_i)
/// Distance-based gradient formation (Krick, Broucke & Francis 2009):
///   u_i = k sum_{j in N_i} (||p_i - p_j||^2 - d_ij^2) (p_j - p_i)
///
/// Guarantees (single integrators, no saturation): leader-follower converges
/// exponentially to the slots; the consensus law converges to the formation up
/// to a translation on a connected undirected graph and to the exact slots
/// when every robot is reachable from a pinned robot; the distance law is only
/// locally asymptotically stable for infinitesimally rigid frameworks and can
/// converge to flipped or incorrect equilibria from poor initial conditions.

#include "multirobotcore/graph/graph.hpp"
#include "multirobotcore/math/types.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace multirobotcore::formation {

struct LeaderFollowerConfig {
  double gain{1.2};        ///< k [1/s]
  double max_speed{1.0};   ///< [m/s]
  bool feedforward{true};  ///< add the leader velocity
};

/// Velocity commands for every robot. `offsets_from_leader[i]` is robot i's
/// slot in the leader frame (the leader's own entry is ignored and its command
/// is zero: the leader is driven externally).
[[nodiscard]] std::vector<Vec2> leader_follower_velocities(
    const Pose2& leader, const Vec2& leader_velocity, std::size_t leader_index,
    std::span<const Vec2> positions, std::span<const Vec2> offsets_from_leader,
    const LeaderFollowerConfig& config);

struct ConsensusFormationConfig {
  double gain{0.8};          ///< coupling gain k [1/s]
  double pinning_gain{0.8};  ///< reference tracking gain k_p [1/s]
  double max_speed{1.0};     ///< [m/s]
};

/// Displacement-based consensus formation command of a single robot, for
/// settings where each robot holds its own estimate of the reference pose
/// (e.g. received over a lossy network). `positions` may be that robot's
/// (noisy) estimates; the command is not saturated when max_speed is
/// infinite.
[[nodiscard]] Vec2 consensus_formation_velocity(std::size_t robot, const graph::Graph& graph,
                                                std::span<const Vec2> positions,
                                                std::span<const Vec2> offsets,
                                                const Pose2& reference,
                                                const Vec2& reference_velocity, double pinning,
                                                const ConsensusFormationConfig& config);

/// Displacement-based consensus formation. `pinning[i] > 0` for robots that
/// observe the reference pose; an empty span means no pinning (formation up to
/// translation).
[[nodiscard]] std::vector<Vec2> consensus_formation_velocities(
    const graph::Graph& graph, std::span<const Vec2> positions, std::span<const Vec2> offsets,
    const Pose2& reference, const Vec2& reference_velocity, std::span<const double> pinning,
    const ConsensusFormationConfig& config);

struct DistanceFormationConfig {
  double gain{0.4};       ///< k [1/(m^2 s)]
  double max_speed{1.0};  ///< [m/s]
};

/// Gradient distance-based formation using desired distances ||d_i - d_j||
/// on the edges of `graph` (typically henneberg_graph(offsets)).
[[nodiscard]] std::vector<Vec2> distance_formation_velocities(
    const graph::Graph& graph, std::span<const Vec2> positions, std::span<const Vec2> offsets,
    const DistanceFormationConfig& config);

}  // namespace multirobotcore::formation
