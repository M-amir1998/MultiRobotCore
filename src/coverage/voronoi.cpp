#include "multirobotcore/coverage/voronoi.hpp"

#include "multirobotcore/math/numerics.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace multirobotcore::coverage {
namespace {

double scale_of(const math::ConvexPolygon& domain, const Vec2& p) {
  double scale = std::max(1.0, p.cwiseAbs().maxCoeff());
  for (const Vec2& v : domain.vertices()) {
    scale = std::max(scale, v.cwiseAbs().maxCoeff());
  }
  return scale;
}

/// True when an edge of `cell` lies on the line { q : n . q = c }.
bool has_edge_on_line(const math::ConvexPolygon& cell, const Vec2& n, double c, double tolerance) {
  const auto& v = cell.vertices();
  const std::size_t m = v.size();
  const double n_norm = n.norm();
  for (std::size_t k = 0; k < m; ++k) {
    const Vec2& a = v[k];
    const Vec2& b = v[(k + 1) % m];
    if ((b - a).norm() <= tolerance) {
      continue;
    }
    if (std::abs(n.dot(a) - c) <= tolerance * n_norm &&
        std::abs(n.dot(b) - c) <= tolerance * n_norm) {
      return true;
    }
  }
  return false;
}

}  // namespace

double max_vertex_distance(const math::ConvexPolygon& polygon, const Vec2& center) {
  double best = 0.0;
  for (const Vec2& v : polygon.vertices()) {
    best = std::max(best, (v - center).norm());
  }
  return best;
}

math::ConvexPolygon clip_to_disc(const math::ConvexPolygon& polygon, const Vec2& center,
                                 double radius, int sides) {
  if (!(radius > 0.0) || sides < 3) {
    throw std::invalid_argument("clip_to_disc: radius must be positive and sides >= 3");
  }
  // Edges of the inscribed regular polygon (vertices at angles 2 pi k / sides)
  // have outward normals at angles (2k + 1) pi / sides and lie at distance
  // radius * cos(pi / sides) from the centre.
  const double pi = math::kPi;
  const double apothem = radius * std::cos(pi / sides);
  math::ConvexPolygon out = polygon;
  for (int k = 0; k < sides && !out.empty(); ++k) {
    const double angle = (2.0 * k + 1.0) * pi / sides;
    const Vec2 normal(std::cos(angle), std::sin(angle));
    out = out.clipped(normal, normal.dot(center) + apothem);
  }
  return out;
}

VoronoiCell voronoi_cell(std::size_t index, std::span<const Vec2> generators,
                         const math::ConvexPolygon& domain,
                         std::span<const std::size_t> candidates) {
  if (index >= generators.size()) {
    throw std::out_of_range("voronoi_cell: generator index out of range");
  }
  const Vec2& p = generators[index];
  VoronoiCell cell{index, {}, {}};

  std::vector<std::size_t> others;
  if (candidates.empty()) {
    others.resize(generators.size());
    std::iota(others.begin(), others.end(), std::size_t{0});
  } else {
    others.assign(candidates.begin(), candidates.end());
    std::sort(others.begin(), others.end());
    others.erase(std::unique(others.begin(), others.end()), others.end());
  }
  others.erase(std::remove(others.begin(), others.end(), index), others.end());
  for (const std::size_t j : others) {
    if (j >= generators.size()) {
      throw std::out_of_range("voronoi_cell: candidate index out of range");
    }
    // A coincident generator with a lower index owns the shared region.
    if (j < index && (generators[j] - p).norm() <= kCoincidenceTolerance) {
      return cell;
    }
  }
  std::stable_sort(others.begin(), others.end(), [&](std::size_t a, std::size_t b) {
    return (generators[a] - p).squaredNorm() < (generators[b] - p).squaredNorm();
  });

  math::ConvexPolygon polygon = domain;
  double reach = max_vertex_distance(polygon, p);
  std::vector<std::size_t> clipped_by;
  for (const std::size_t j : others) {
    const Vec2& q = generators[j];
    const double d = (q - p).norm();
    if (d <= kCoincidenceTolerance) {
      continue;  // dominated coincident generator: ignored
    }
    if (d > 2.0 * reach) {
      break;  // bisector lies beyond every vertex: no further cut possible
    }
    const Vec2 normal = q - p;
    polygon = polygon.clipped(normal, normal.dot(0.5 * (p + q)));
    clipped_by.push_back(j);
    if (polygon.empty()) {
      break;
    }
    reach = max_vertex_distance(polygon, p);
  }

  if (!polygon.empty()) {
    const double tolerance = 1e-9 * scale_of(domain, p);
    for (const std::size_t j : clipped_by) {
      const Vec2& q = generators[j];
      const Vec2 normal = q - p;
      if (has_edge_on_line(polygon, normal, normal.dot(0.5 * (p + q)), tolerance)) {
        cell.neighbors.push_back(j);
      }
    }
    std::sort(cell.neighbors.begin(), cell.neighbors.end());
  }
  cell.polygon = std::move(polygon);
  return cell;
}

std::vector<VoronoiCell> bounded_voronoi(std::span<const Vec2> generators,
                                         const math::ConvexPolygon& domain) {
  std::vector<VoronoiCell> cells;
  cells.reserve(generators.size());
  for (std::size_t i = 0; i < generators.size(); ++i) {
    cells.push_back(voronoi_cell(i, generators, domain));
  }
  return cells;
}

}  // namespace multirobotcore::coverage
