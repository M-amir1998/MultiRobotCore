#pragma once

/// @file convex_polygon.hpp
/// Convex polygon with exact half-plane clipping. This is the geometric kernel
/// behind bounded Voronoi cells: a cell is the domain clipped by the bisector
/// half-planes of every other generator.

#include "multirobotcore/math/geometry.hpp"
#include "multirobotcore/math/types.hpp"

#include <vector>

namespace multirobotcore::math {

class ConvexPolygon {
 public:
  ConvexPolygon() = default;

  /// Constructs from vertices of a convex polygon in either orientation.
  /// Vertices are stored counter-clockwise. Throws std::invalid_argument when
  /// the input is not convex or contains non-finite values.
  explicit ConvexPolygon(std::vector<Vec2> vertices);

  [[nodiscard]] static ConvexPolygon from_box(const AABB& box);

  /// Counter-clockwise vertex list.
  [[nodiscard]] const std::vector<Vec2>& vertices() const noexcept { return vertices_; }
  [[nodiscard]] std::size_t size() const noexcept { return vertices_.size(); }

  /// True when the polygon has no interior (fewer than three vertices).
  [[nodiscard]] bool empty() const noexcept { return vertices_.size() < 3; }

  /// Enclosed area [m^2] (zero for empty polygons).
  [[nodiscard]] double area() const noexcept;

  /// Area centroid. For degenerate polygons returns the vertex average.
  [[nodiscard]] Vec2 centroid() const;

  /// True when p is inside or on the boundary (within `tolerance`).
  [[nodiscard]] bool contains(const Vec2& p, double tolerance = 1e-12) const noexcept;

  [[nodiscard]] AABB bounding_box() const;

  /// Intersection with the half-plane { x : normal . x <= offset }.
  [[nodiscard]] ConvexPolygon clipped(const Vec2& normal, double offset) const;

 private:
  struct Unchecked {};
  ConvexPolygon(std::vector<Vec2> ccw_vertices, Unchecked /*tag*/) noexcept;

  std::vector<Vec2> vertices_;
};

}  // namespace multirobotcore::math
