#pragma once

/// @file linear_program.hpp
/// Low-dimensional linear programs in velocity space used by ORCA
/// (van den Berg, Guy, Lin & Manocha 2011, section 5.2).
///
/// Problem: minimise ||v - v_pref|| subject to half-plane constraints
/// n_i . v >= c_i and the speed limit ||v|| <= V. It is solved by the
/// incremental algorithm for 2D linear programming (de Berg et al.,
/// Computational Geometry, ch. 4) adapted to the circular speed constraint:
/// whenever the current optimum violates constraint i, the new optimum lies on
/// the boundary line of i and is found by a 1D program on that line.
///
/// When the constraints are jointly infeasible, the velocity that minimises
/// the maximum violation of the soft constraints, subject to the hard
/// constraints and the speed limit, is computed instead (the "3D" program of
/// the ORCA paper), solved incrementally by projecting the problem onto the
/// boundary of the newly violated constraint.

#include "multirobotcore/math/numerics.hpp"
#include "multirobotcore/math/types.hpp"

#include <cstddef>
#include <span>

namespace multirobotcore::avoidance {

/// Closed half-plane { v : normal . v >= offset } with a unit normal.
struct HalfPlane {
  Vec2 normal{1.0, 0.0};
  double offset{0.0};

  /// Amount by which v violates the constraint (<= 0 when satisfied) [m/s].
  [[nodiscard]] double violation(const Vec2& v) const noexcept { return offset - normal.dot(v); }
  /// Closest point of the boundary line to the origin.
  [[nodiscard]] Vec2 point() const noexcept { return offset * normal; }
  /// Unit direction of the boundary line (normal rotated by +90 degrees).
  [[nodiscard]] Vec2 direction() const noexcept { return math::perp(normal); }
};

/// Half-plane whose boundary passes through `point`, oriented along `normal`
/// (normalised internally; must be non-zero).
[[nodiscard]] HalfPlane half_plane_through(const Vec2& point, const Vec2& normal);

/// Result of the 2D program. fail_index == constraints.size() on success;
/// otherwise the first constraint that could not be satisfied together with
/// the previous ones (velocity then holds the optimum for the prefix).
struct Lp2Result {
  Vec2 velocity{Vec2::Zero()};
  std::size_t fail_index{0};
};

/// Incremental 2D program. With `direction_objective` the target is a unit
/// direction to maximise instead of a point to approach.
[[nodiscard]] Lp2Result solve_lp2(std::span<const HalfPlane> constraints, const Vec2& target,
                                  double max_speed, bool direction_objective = false);

struct VelocityLpResult {
  Vec2 velocity{Vec2::Zero()};
  bool feasible{true};        ///< all constraints satisfied together with the speed limit
  double max_violation{0.0};  ///< largest soft-constraint violation [m/s] (0 when feasible)
};

/// Solves the velocity program. The first `num_hard` constraints are kept
/// hard when the problem is infeasible; if they conflict with the speed limit
/// themselves, all constraints are treated as soft.
[[nodiscard]] VelocityLpResult solve_velocity_lp(std::span<const HalfPlane> constraints,
                                                 std::size_t num_hard, const Vec2& preferred,
                                                 double max_speed);

}  // namespace multirobotcore::avoidance
