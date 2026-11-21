#pragma once

/// @file geometry.hpp
/// Planar geometric primitives and exact distance/intersection queries.

#include "multirobotcore/math/types.hpp"

#include <optional>

namespace multirobotcore::math {

/// Closed disc.
struct Circle {
  Vec2 center{Vec2::Zero()};
  double radius{0.0};
};

/// Axis-aligned box [min, max] (closed).
struct AABB {
  Vec2 min{Vec2::Zero()};
  Vec2 max{Vec2::Zero()};

  [[nodiscard]] double width() const noexcept { return max.x() - min.x(); }
  [[nodiscard]] double height() const noexcept { return max.y() - min.y(); }
  [[nodiscard]] Vec2 center() const noexcept { return 0.5 * (min + max); }
  [[nodiscard]] bool contains(const Vec2& p, double tolerance = 0.0) const noexcept;
  /// Box grown by `margin` on every side (shrunk for negative margins).
  [[nodiscard]] AABB inflated(double margin) const noexcept;
};

/// Closed line segment from a to b.
struct Segment {
  Vec2 a{Vec2::Zero()};
  Vec2 b{Vec2::Zero()};
};

/// Closest point on a segment to p.
[[nodiscard]] Vec2 closest_point(const Segment& segment, const Vec2& p) noexcept;

/// Euclidean distance from p to a segment.
[[nodiscard]] double distance(const Segment& segment, const Vec2& p) noexcept;

/// Closest point of a (solid) box to p; equals p when p is inside.
[[nodiscard]] Vec2 closest_point(const AABB& box, const Vec2& p) noexcept;

/// Distance from p to a solid box (zero inside).
[[nodiscard]] double distance(const AABB& box, const Vec2& p) noexcept;

/// True when the segment intersects the closed disc.
[[nodiscard]] bool intersects(const Segment& segment, const Circle& circle) noexcept;

/// True when the segment intersects the closed box (Liang-Barsky clipping).
[[nodiscard]] bool intersects(const Segment& segment, const AABB& box) noexcept;

/// True when two discs overlap (touching counts as overlapping).
[[nodiscard]] bool intersects(const Circle& a, const Circle& b) noexcept;

/// True when a disc overlaps a solid box.
[[nodiscard]] bool intersects(const Circle& circle, const AABB& box) noexcept;

/// Smallest t >= 0 with ||origin + t*direction - c|| = r, i.e. the first time a
/// ray enters the disc. Returns 0 when the origin is already inside. `direction`
/// need not be normalised; returns nullopt when the ray misses.
[[nodiscard]] std::optional<double> ray_intersection(const Vec2& origin, const Vec2& direction,
                                                     const Circle& circle) noexcept;

/// Smallest t >= 0 at which the ray origin + t*direction enters a solid box.
[[nodiscard]] std::optional<double> ray_intersection(const Vec2& origin, const Vec2& direction,
                                                     const AABB& box) noexcept;

}  // namespace multirobotcore::math
