#include "multirobotcore/avoidance/linear_program.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace multirobotcore::avoidance {
namespace {

constexpr double kParallel = 1e-9;  // |n_j . d| below this: lines are parallel
constexpr double kSlack = 1e-12;    // feasibility tolerance [m/s]

/// 1D program on the boundary line of constraints[i], subject to constraints
/// [0, i) and the speed disc. Returns false when that set is empty.
bool solve_on_line(std::span<const HalfPlane> constraints, std::size_t i, double max_speed,
                   const Vec2& target, bool direction_objective, Vec2& result) {
  const HalfPlane& h = constraints[i];
  const Vec2 q = h.point();
  const Vec2 d = h.direction();
  // q is orthogonal to d, so ||q + t d||^2 = ||q||^2 + t^2.
  const double disc = max_speed * max_speed - q.squaredNorm();
  if (disc < 0.0) {
    return false;  // the line misses the speed disc
  }
  const double half_chord = std::sqrt(disc);
  double t_lo = -half_chord;
  double t_hi = half_chord;
  for (std::size_t j = 0; j < i; ++j) {
    const HalfPlane& c = constraints[j];
    // c.normal . (q + t d) >= c.offset  <=>  denom * t >= numer
    const double denom = c.normal.dot(d);
    const double numer = c.offset - c.normal.dot(q);
    if (std::abs(denom) <= kParallel) {
      if (numer > kSlack) {
        return false;  // parallel and the whole line violates c
      }
      continue;
    }
    const double t = numer / denom;
    if (denom > 0.0) {
      t_lo = std::max(t_lo, t);
    } else {
      t_hi = std::min(t_hi, t);
    }
    if (t_lo > t_hi + kSlack) {
      return false;
    }
  }
  double t = 0.0;
  if (direction_objective) {
    t = target.dot(d) > 0.0 ? t_hi : t_lo;
  } else {
    t = std::clamp(d.dot(target - q), t_lo, std::max(t_lo, t_hi));
  }
  result = q + t * d;
  return true;
}

Vec2 initial_solution(const Vec2& target, double max_speed, bool direction_objective) {
  if (direction_objective) {
    return math::safe_normalized(target) * max_speed;
  }
  if (target.squaredNorm() > max_speed * max_speed) {
    return math::safe_normalized(target) * max_speed;
  }
  return target;
}

/// Minimises the largest violation of the soft constraints [num_hard, m)
/// subject to the hard constraints [0, num_hard) and the speed disc, starting
/// from `v`, which is optimal for the prefix [0, begin).
Vec2 minimise_max_violation(std::span<const HalfPlane> constraints, std::size_t num_hard,
                            std::size_t begin, double max_speed, Vec2 v) {
  double distance = 0.0;
  std::vector<HalfPlane> projected;
  for (std::size_t i = begin; i < constraints.size(); ++i) {
    const HalfPlane& ci = constraints[i];
    if (ci.violation(v) <= distance) {
      continue;
    }
    // The new optimum equalises the violation of i with the largest previous
    // soft violation: keep the region where violation_j <= violation_i,
    // i.e. (n_j - n_i) . v >= c_j - c_i, then push as far as possible along n_i.
    projected.assign(constraints.begin(),
                     constraints.begin() + static_cast<std::ptrdiff_t>(num_hard));
    for (std::size_t j = num_hard; j < i; ++j) {
      const HalfPlane& cj = constraints[j];
      const Vec2 normal = cj.normal - ci.normal;
      const double norm = normal.norm();
      if (norm <= kParallel) {
        continue;  // same orientation: violation_j - violation_i is constant
      }
      projected.push_back({normal / norm, (cj.offset - ci.offset) / norm});
    }
    const Vec2 previous = v;
    const Lp2Result r = solve_lp2(projected, ci.normal, max_speed, true);
    // Failure can only stem from round-off; keep the previous solution then.
    v = r.fail_index < projected.size() ? previous : r.velocity;
    distance = ci.violation(v);
  }
  return v;
}

}  // namespace

HalfPlane half_plane_through(const Vec2& point, const Vec2& normal) {
  const double n = normal.norm();
  if (!(n > 0.0)) {
    throw std::invalid_argument("half_plane_through: normal must be non-zero");
  }
  const Vec2 unit = normal / n;
  return HalfPlane{unit, unit.dot(point)};
}

Lp2Result solve_lp2(std::span<const HalfPlane> constraints, const Vec2& target, double max_speed,
                    bool direction_objective) {
  Vec2 v = initial_solution(target, max_speed, direction_objective);
  for (std::size_t i = 0; i < constraints.size(); ++i) {
    if (constraints[i].violation(v) <= kSlack) {
      continue;
    }
    const Vec2 previous = v;
    if (!solve_on_line(constraints, i, max_speed, target, direction_objective, v)) {
      return Lp2Result{previous, i};
    }
  }
  return Lp2Result{v, constraints.size()};
}

VelocityLpResult solve_velocity_lp(std::span<const HalfPlane> constraints, std::size_t num_hard,
                                   const Vec2& preferred, double max_speed) {
  if (num_hard > constraints.size()) {
    throw std::invalid_argument("solve_velocity_lp: num_hard exceeds the number of constraints");
  }
  if (!(max_speed >= 0.0)) {
    throw std::invalid_argument("solve_velocity_lp: max_speed must be non-negative");
  }
  const Lp2Result first = solve_lp2(constraints, preferred, max_speed, false);
  if (first.fail_index == constraints.size()) {
    return VelocityLpResult{first.velocity, true, 0.0};
  }
  // Hard constraints that already conflict with the speed limit cannot be
  // kept hard: treat everything as soft.
  const std::size_t hard = first.fail_index < num_hard ? 0 : num_hard;
  const Vec2 v =
      minimise_max_violation(constraints, hard, first.fail_index, max_speed, first.velocity);
  double worst = 0.0;
  for (std::size_t i = hard; i < constraints.size(); ++i) {
    worst = std::max(worst, constraints[i].violation(v));
  }
  return VelocityLpResult{v, false, worst};
}

}  // namespace multirobotcore::avoidance
