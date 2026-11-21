#pragma once

/// @file sampling.hpp
/// Sampling-based velocity selection with velocity obstacles (VO) or
/// reciprocal velocity obstacles (RVO, van den Berg, Lin & Manocha 2008).
///
/// Candidate velocities (the preferred velocity, zero, and a deterministic
/// polar grid inside the speed disc) are scored with the penalty of the RVO
/// paper,
///
///     penalty(v) = w / t_c(v) + || v_pref - v ||,
///
/// where t_c(v) is the earliest predicted collision time over all neighbours
/// within the time horizon (+infinity if none). Under VO the neighbour is
/// assumed to keep its velocity (relative velocity v - v_B); under RVO both
/// agents are assumed to share the avoidance effort (relative velocity
/// 2 v - v_A - v_B). Static obstacles always use VO semantics.

#include "multirobotcore/avoidance/velocity_obstacle.hpp"

#include <cstddef>
#include <span>

namespace multirobotcore::avoidance {

enum class SamplingRule { kVelocityObstacle, kReciprocalVelocityObstacle };

struct SamplingParams {
  double time_horizon{2.0};           ///< collisions later than this are ignored [s]
  double obstacle_time_horizon{1.0};  ///< horizon for static obstacles [s]
  std::size_t speed_rings{6};         ///< concentric speed levels
  std::size_t angle_samples{36};      ///< directions per ring
  double penalty_weight{1.0};         ///< w [m]
};

struct SampledVelocity {
  Vec2 velocity{Vec2::Zero()};
  double penalty{0.0};
  double time_to_collision{kNever};  ///< of the selected velocity [s]
  bool admissible{true};             ///< outside every (R)VO within the horizon
};

[[nodiscard]] SampledVelocity sample_velocity(const AgentState& self, const Vec2& preferred,
                                              double max_speed,
                                              std::span<const AgentState> neighbors,
                                              std::span<const AgentState> static_obstacles,
                                              SamplingRule rule, const SamplingParams& params);

}  // namespace multirobotcore::avoidance
