#pragma once

/// @file voronoi.hpp
/// Bounded Voronoi partition of a convex domain.
///
/// The cell of generator p_i is the domain intersected with the bisector
/// half-planes { q : (p_j - p_i) . q <= (||p_j||^2 - ||p_i||^2) / 2 } of every
/// other generator j. Each cell is computed by exact convex half-plane
/// clipping (math::ConvexPolygon::clipped), which avoids a general Voronoi
/// construction and its degenerate cases. Candidates are processed in order of
/// increasing distance and clipping stops once ||p_j - p_i|| exceeds twice the
/// largest vertex distance of the current cell: no farther bisector can cut
/// the cell, so the result is exact.
///
/// Coincident generators (closer than `kCoincidenceTolerance`) share the same
/// region; the lowest index owns it and the others receive an empty cell. A
/// controller moving the owner breaks the coincidence on the next step.

#include "multirobotcore/math/convex_polygon.hpp"
#include "multirobotcore/math/types.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace multirobotcore::coverage {

inline constexpr double kCoincidenceTolerance = 1e-9;  ///< [m]

struct VoronoiCell {
  std::size_t generator{0};
  math::ConvexPolygon polygon;  ///< empty for dominated coincident generators
  /// Generators whose bisector contributes an edge of the cell (Delaunay
  /// neighbours restricted to the domain), sorted.
  std::vector<std::size_t> neighbors;
};

/// Cell of generator `index`, clipped only by the generators listed in
/// `candidates` (all others when empty). Restricting the candidates gives the
/// limited-range cell used by distributed coverage, which is a superset of the
/// exact cell.
[[nodiscard]] VoronoiCell voronoi_cell(std::size_t index, std::span<const Vec2> generators,
                                       const math::ConvexPolygon& domain,
                                       std::span<const std::size_t> candidates = {});

/// Full bounded partition, one cell per generator. Generators outside the
/// domain are allowed (their cells may be empty).
[[nodiscard]] std::vector<VoronoiCell> bounded_voronoi(std::span<const Vec2> generators,
                                                       const math::ConvexPolygon& domain);

/// Largest distance from `center` to a vertex of `polygon` (0 when empty).
[[nodiscard]] double max_vertex_distance(const math::ConvexPolygon& polygon, const Vec2& center);

/// Intersection of `polygon` with the disc B(center, radius), where the disc is
/// represented by its inscribed regular polygon with `sides` edges (area error
/// 1 - sides sin(2 pi / sides) / (2 pi), about 0.64 % for 32 sides).
[[nodiscard]] math::ConvexPolygon clip_to_disc(const math::ConvexPolygon& polygon,
                                               const Vec2& center, double radius, int sides = 32);

}  // namespace multirobotcore::coverage
