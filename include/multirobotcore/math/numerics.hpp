#pragma once

/// @file numerics.hpp
/// Small numerical helpers used across modules.

#include "multirobotcore/math/types.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace multirobotcore::math {

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kTwoPi = 2.0 * std::numbers::pi;

/// Wraps an angle to the half-open interval (-pi, pi].
[[nodiscard]] inline double wrap_angle(double angle) noexcept {
  double wrapped = std::remainder(angle, kTwoPi);
  if (wrapped <= -kPi) {
    wrapped += kTwoPi;
  }
  return wrapped;
}

/// z-component of the 3D cross product of two planar vectors.
[[nodiscard]] inline double cross(const Vec2& a, const Vec2& b) noexcept {
  return a.x() * b.y() - a.y() * b.x();
}

/// Rotates a vector by +90 degrees.
[[nodiscard]] inline Vec2 perp(const Vec2& v) noexcept {
  return {-v.y(), v.x()};
}

/// Rotates a vector by `angle` radians counter-clockwise.
[[nodiscard]] inline Vec2 rotate(const Vec2& v, double angle) noexcept {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {c * v.x() - s * v.y(), s * v.x() + c * v.y()};
}

/// Returns v scaled down (direction preserved) so that ||v|| <= max_norm.
[[nodiscard]] inline Vec2 clamp_norm(const Vec2& v, double max_norm) noexcept {
  const double n = v.norm();
  if (n > max_norm && n > 0.0) {
    return v * (max_norm / n);
  }
  return v;
}

/// Unit vector along v, or zero when ||v|| is below `eps`.
[[nodiscard]] inline Vec2 safe_normalized(const Vec2& v, double eps = 1e-12) noexcept {
  const double n = v.norm();
  return n > eps ? Vec2(v / n) : Vec2::Zero();
}

/// Quintic smoothstep s -> 6s^5 - 15s^4 + 10s^3 on [0, 1] (clamped outside).
/// Zero first and second derivative at both ends: used for smooth transitions.
[[nodiscard]] inline double smoothstep5(double s) noexcept {
  const double t = std::clamp(s, 0.0, 1.0);
  return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

[[nodiscard]] inline bool is_finite(const Vec2& v) noexcept {
  return std::isfinite(v.x()) && std::isfinite(v.y());
}

}  // namespace multirobotcore::math
