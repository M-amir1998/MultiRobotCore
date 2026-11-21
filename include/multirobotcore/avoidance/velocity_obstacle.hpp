#pragma once

/// @file velocity_obstacle.hpp
/// Velocity obstacles for disc-shaped agents (Fiorini & Shiller 1998).
///
/// Notation: agent A at p_A with velocity v_A and radius r_A, neighbour B at
/// p_B with v_B and r_B. With relative position p = p_B - p_A, combined radius
/// R = r_A + r_B and relative velocity v = v_A - v_B, A and B collide within
/// the time horizon tau iff
///
///     exists t in [0, tau] :  || p - t v || < R .
///
/// The set of such relative velocities is the truncated velocity obstacle
/// VO^tau(p, R): a cone with apex at the origin whose legs are tangent to the
/// disc D(p / tau, R / tau), cut off by that disc. VO^tau is convex. In A's
/// absolute velocity space the obstacle is v_B + VO^tau (VO) or, for the
/// reciprocal velocity obstacle of van den Berg, Lin & Manocha (2008),
/// (v_A + v_B) / 2 + VO^(2 tau): A's new velocity v' is in the RVO iff
/// 2 v' - v_A lies in the VO.

#include "multirobotcore/math/types.hpp"

#include <limits>

namespace multirobotcore::avoidance {

/// Disc-shaped agent as seen by a velocity-space planner.
struct AgentState {
  Vec2 position{Vec2::Zero()};  ///< [m]
  Vec2 velocity{Vec2::Zero()};  ///< [m/s]
  double radius{0.2};           ///< [m]
};

inline constexpr double kNever = std::numeric_limits<double>::infinity();

/// Earliest t >= 0 with ||p - t v|| <= R (p = p_B - p_A, v = v_A - v_B).
/// Returns 0 when the discs already overlap and +infinity when they never
/// touch (including when they move apart).
[[nodiscard]] double time_to_collision(const Vec2& relative_position, const Vec2& relative_velocity,
                                       double combined_radius) noexcept;

/// Like time_to_collision(), but for overlapping discs returns +infinity when
/// the relative velocity separates them (v . p < 0) and 0 otherwise, so that
/// escaping velocities are not penalised.
[[nodiscard]] double effective_time_to_collision(const Vec2& relative_position,
                                                 const Vec2& relative_velocity,
                                                 double combined_radius) noexcept;

/// Minimum of ||p - t v|| over t in [0, tau].
[[nodiscard]] double min_distance_within_horizon(const Vec2& relative_position,
                                                 const Vec2& relative_velocity,
                                                 double time_horizon) noexcept;

enum class VOMembership { kOutside, kBoundary, kInside };

/// Membership of the relative velocity v in VO^tau(p, R). The boundary band is
/// |min_t ||p - t v|| - R| <= tolerance (tolerance in metres).
[[nodiscard]] VOMembership classify_relative_velocity(const Vec2& relative_position,
                                                      const Vec2& relative_velocity,
                                                      double combined_radius, double time_horizon,
                                                      double tolerance = 1e-9) noexcept;

/// Geometry of a (reciprocal) velocity obstacle in A's absolute velocity
/// space: apex + VO^h(p, R) with h = time_horizon.
struct VelocityObstacleCone {
  Vec2 apex{Vec2::Zero()};               ///< v_B (VO) or (v_A + v_B) / 2 (RVO) [m/s]
  Vec2 relative_position{Vec2::Zero()};  ///< p = p_B - p_A [m]
  double combined_radius{0.0};           ///< R [m]
  double time_horizon{1.0};              ///< h [s]
  Vec2 cutoff_center{Vec2::Zero()};      ///< apex + p / h
  double cutoff_radius{0.0};             ///< R / h
  Vec2 left_leg{Vec2::Zero()};           ///< unit direction of the counter-clockwise leg
  Vec2 right_leg{Vec2::Zero()};          ///< unit direction of the clockwise leg
  bool overlapping{false};  ///< discs overlap: every velocity collides (legs undefined)

  /// Membership of an absolute velocity of A.
  [[nodiscard]] VOMembership classify(const Vec2& velocity, double tolerance = 1e-9) const noexcept;
  [[nodiscard]] bool contains(const Vec2& velocity) const noexcept {
    return classify(velocity) == VOMembership::kInside;
  }
};

/// VO^tau_{A|B} with apex v_B.
[[nodiscard]] VelocityObstacleCone velocity_obstacle(const AgentState& self,
                                                     const AgentState& other,
                                                     double time_horizon) noexcept;

/// Truncated RVO_{A|B}: apex (v_A + v_B) / 2 and horizon 2 tau.
[[nodiscard]] VelocityObstacleCone reciprocal_velocity_obstacle(const AgentState& self,
                                                                const AgentState& other,
                                                                double time_horizon) noexcept;

}  // namespace multirobotcore::avoidance
