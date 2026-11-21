#pragma once

/// @file orca.hpp
/// Optimal Reciprocal Collision Avoidance (van den Berg, Guy, Lin & Manocha,
/// "Reciprocal n-body collision avoidance", Robotics Research, STAR 70, 2011).
///
/// For agent A and neighbour B, let v = v_A - v_B be the current relative
/// velocity and u the smallest change that moves v onto the boundary of
/// VO^tau(p, R), with n the outward boundary normal at v + u. Then
///
///     ORCA^tau_{A|B} = { v' : (v' - (v_A + rho u)) . n >= 0 }
///
/// with responsibility rho = 1/2 for reciprocating agents and rho = 1 for
/// non-reciprocal (static or failed) obstacles. If both agents choose
/// velocities inside their ORCA half-planes, their relative velocity lies
/// outside VO^tau, i.e. they are collision-free for at least tau seconds
/// (the half-plane supports the convex set VO^tau at v + u).
///
/// When the discs already overlap, the VO is built with the time step dt
/// instead of tau so that the agents separate within one step.

#include "multirobotcore/avoidance/linear_program.hpp"
#include "multirobotcore/avoidance/velocity_obstacle.hpp"

#include <span>
#include <vector>

namespace multirobotcore::avoidance {

struct OrcaConstraint {
  HalfPlane half_plane;   ///< ORCA^tau_{A|B} in A's velocity space
  Vec2 u{Vec2::Zero()};   ///< smallest relative-velocity change to the VO boundary
  bool colliding{false};  ///< discs already overlap (dt-based construction)
};

/// Builds ORCA^tau_{A|B}. `time_step` is used when the agents overlap.
[[nodiscard]] OrcaConstraint orca_constraint(const AgentState& self, const AgentState& other,
                                             double time_horizon, double time_step,
                                             double responsibility = 0.5);

struct OrcaParams {
  double time_horizon{2.0};           ///< tau for agents [s]
  double obstacle_time_horizon{1.0};  ///< tau for static obstacles [s]
  double time_step{0.05};             ///< control period, used for overlapping discs [s]
};

struct OrcaVelocity {
  Vec2 velocity{Vec2::Zero()};
  bool feasible{true};                 ///< false when the 3D fallback program was used
  double max_violation{0.0};           ///< [m/s]
  std::vector<HalfPlane> constraints;  ///< obstacle constraints first, then agents
};

/// New velocity for `self`: the speed-limited velocity closest to `preferred`
/// inside every ORCA half-plane. Entries of `static_obstacles` are
/// non-reciprocating (obstacle discs, stopped robots; their velocity is used
/// as given, normally zero) and give hard constraints with full
/// responsibility and the obstacle time horizon. Reciprocating neighbours give
/// soft constraints with half responsibility. If the program is infeasible,
/// the velocity minimising the largest soft violation subject to the hard
/// constraints is returned.
[[nodiscard]] OrcaVelocity orca_velocity(const AgentState& self, const Vec2& preferred,
                                         double max_speed, std::span<const AgentState> neighbors,
                                         std::span<const AgentState> static_obstacles,
                                         const OrcaParams& params);

}  // namespace multirobotcore::avoidance
