#include "multirobotcore/math/convex_polygon.hpp"

#include "multirobotcore/math/numerics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace multirobotcore::math {
namespace {

double signed_area(const std::vector<Vec2>& v) noexcept {
  double twice_area = 0.0;
  const std::size_t n = v.size();
  for (std::size_t i = 0; i < n; ++i) {
    twice_area += cross(v[i], v[(i + 1) % n]);
  }
  return 0.5 * twice_area;
}

double characteristic_length(const std::vector<Vec2>& v) noexcept {
  double scale = 0.0;
  for (const Vec2& p : v) {
    scale = std::max(scale, p.cwiseAbs().maxCoeff());
  }
  return std::max(scale, 1.0);
}

/// Removes consecutive (cyclic) vertices closer than `eps`.
std::vector<Vec2> remove_duplicates(std::vector<Vec2> v, double eps) {
  std::vector<Vec2> out;
  out.reserve(v.size());
  for (const Vec2& p : v) {
    if (out.empty() || (p - out.back()).norm() > eps) {
      out.push_back(p);
    }
  }
  while (out.size() > 1 && (out.front() - out.back()).norm() <= eps) {
    out.pop_back();
  }
  return out;
}

}  // namespace

ConvexPolygon::ConvexPolygon(std::vector<Vec2> ccw_vertices, Unchecked /*tag*/) noexcept
    : vertices_(std::move(ccw_vertices)) {}

ConvexPolygon::ConvexPolygon(std::vector<Vec2> vertices) {
  for (const Vec2& p : vertices) {
    if (!is_finite(p)) {
      throw std::invalid_argument("ConvexPolygon: non-finite vertex");
    }
  }
  const double eps = 1e-12 * characteristic_length(vertices);
  vertices = remove_duplicates(std::move(vertices), eps);
  if (vertices.size() >= 3 && signed_area(vertices) < 0.0) {
    std::reverse(vertices.begin(), vertices.end());
  }
  const std::size_t n = vertices.size();
  if (n >= 3) {
    const double scale = characteristic_length(vertices);
    for (std::size_t i = 0; i < n; ++i) {
      const Vec2& a = vertices[i];
      const Vec2& b = vertices[(i + 1) % n];
      const Vec2& c = vertices[(i + 2) % n];
      if (cross(b - a, c - b) < -1e-9 * scale * scale) {
        throw std::invalid_argument("ConvexPolygon: vertices do not form a convex polygon");
      }
    }
  }
  vertices_ = std::move(vertices);
}

ConvexPolygon ConvexPolygon::from_box(const AABB& box) {
  return ConvexPolygon({box.min, {box.max.x(), box.min.y()}, box.max, {box.min.x(), box.max.y()}});
}

double ConvexPolygon::area() const noexcept {
  return empty() ? 0.0 : std::abs(signed_area(vertices_));
}

Vec2 ConvexPolygon::centroid() const {
  if (vertices_.empty()) {
    throw std::logic_error("ConvexPolygon::centroid: polygon has no vertices");
  }
  const std::size_t n = vertices_.size();
  double twice_area = 0.0;
  Vec2 accum = Vec2::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2& p = vertices_[i];
    const Vec2& q = vertices_[(i + 1) % n];
    const double w = cross(p, q);
    twice_area += w;
    accum += w * (p + q);
  }
  const double scale = characteristic_length(vertices_);
  if (std::abs(twice_area) <= 1e-14 * scale * scale) {
    Vec2 mean = Vec2::Zero();
    for (const Vec2& p : vertices_) {
      mean += p;
    }
    return mean / static_cast<double>(n);
  }
  return accum / (3.0 * twice_area);
}

bool ConvexPolygon::contains(const Vec2& p, double tolerance) const noexcept {
  if (empty()) {
    return false;
  }
  const std::size_t n = vertices_.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2& a = vertices_[i];
    const Vec2& b = vertices_[(i + 1) % n];
    const Vec2 edge = b - a;
    const double len = edge.norm();
    if (len <= 0.0) {
      continue;
    }
    // Signed distance of p to the left of edge a->b (inside is positive).
    if (cross(edge, p - a) / len < -tolerance) {
      return false;
    }
  }
  return true;
}

AABB ConvexPolygon::bounding_box() const {
  if (vertices_.empty()) {
    throw std::logic_error("ConvexPolygon::bounding_box: polygon has no vertices");
  }
  AABB box{vertices_.front(), vertices_.front()};
  for (const Vec2& p : vertices_) {
    box.min = box.min.cwiseMin(p);
    box.max = box.max.cwiseMax(p);
  }
  return box;
}

ConvexPolygon ConvexPolygon::clipped(const Vec2& normal, double offset) const {
  // Sutherland-Hodgman clipping against a single half-plane. Clipping a convex
  // polygon by a half-plane yields a convex polygon with the same orientation.
  if (vertices_.empty()) {
    return {};
  }
  const std::size_t n = vertices_.size();
  std::vector<Vec2> out;
  out.reserve(n + 1);
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2& current = vertices_[i];
    const Vec2& next = vertices_[(i + 1) % n];
    const double d_current = normal.dot(current) - offset;
    const double d_next = normal.dot(next) - offset;
    const bool current_inside = d_current <= 0.0;
    const bool next_inside = d_next <= 0.0;
    if (current_inside) {
      out.push_back(current);
    }
    if (current_inside != next_inside) {
      const double t = d_current / (d_current - d_next);
      out.push_back(current + t * (next - current));
    }
  }
  const double eps = 1e-12 * characteristic_length(vertices_);
  out = remove_duplicates(std::move(out), eps);
  if (out.size() < 3) {
    return {};
  }
  return ConvexPolygon(std::move(out), Unchecked{});
}

}  // namespace multirobotcore::math
