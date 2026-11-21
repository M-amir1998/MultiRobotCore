#pragma once

/// @file quadrature.hpp
/// Deterministic integration of a density over convex polygons.
///
/// The polygon is fan-triangulated from its first vertex; each triangle is
/// split into s^2 congruent sub-triangles (s = `subdivisions`), and each
/// sub-triangle is integrated with the symmetric 3-point rule at barycentric
/// coordinates (2/3, 1/6, 1/6), which is exact for polynomials of degree 2.
/// Consequently mass, centroid and second moments are exact (up to round-off)
/// for uniform densities, and converge with mesh refinement for smooth
/// densities (error O(h^3) per unit area for C^3 densities, h ~ 1/s).

#include "multirobotcore/coverage/density.hpp"
#include "multirobotcore/math/convex_polygon.hpp"
#include "multirobotcore/math/types.hpp"

namespace multirobotcore::coverage {

/// Integrals of one cell with respect to its generator p.
struct CellIntegrals {
  double mass{0.0};             ///< M = int_V phi
  Vec2 centroid{Vec2::Zero()};  ///< C = (1/M) int_V q phi (p when M = 0)
  double cost{0.0};             ///< H_V(p) = int_V ||q - p||^2 phi
};

/// Integrates mass, centroid and the coverage cost about `generator`.
[[nodiscard]] CellIntegrals integrate_cell(const math::ConvexPolygon& cell, const Vec2& generator,
                                           const DensityFunction& density, int subdivisions = 4);

}  // namespace multirobotcore::coverage
