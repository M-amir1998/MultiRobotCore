#include "multirobotcore/math/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace multirobotcore::math {

bool AABB::contains(const Vec2& p, double tolerance) const noexcept {
  return p.x() >= min.x() - tolerance && p.x() <= max.x() + tolerance &&
         p.y() >= min.y() - tolerance && p.y() <= max.y() + tolerance;
}

AABB AABB::inflated(double margin) const noexcept {
  const Vec2 m(margin, margin);
  return AABB{min - m, max + m};
}

Vec2 closest_point(const Segment& segment, const Vec2& p) noexcept {
  const Vec2 d = segment.b - segment.a;
  const double length_sq = d.squaredNorm();
  if (length_sq <= 0.0) {
    return segment.a;
  }
  const double t = std::clamp((p - segment.a).dot(d) / length_sq, 0.0, 1.0);
  return segment.a + t * d;
}

double distance(const Segment& segment, const Vec2& p) noexcept {
  return (closest_point(segment, p) - p).norm();
}

Vec2 closest_point(const AABB& box, const Vec2& p) noexcept {
  return p.cwiseMax(box.min).cwiseMin(box.max);
}

double distance(const AABB& box, const Vec2& p) noexcept {
  return (closest_point(box, p) - p).norm();
}

bool intersects(const Segment& segment, const Circle& circle) noexcept {
  return distance(segment, circle.center) <= circle.radius;
}

bool intersects(const Segment& segment, const AABB& box) noexcept {
  // Liang-Barsky parametric clipping of a + t (b - a), t in [0, 1].
  double t0 = 0.0;
  double t1 = 1.0;
  const Vec2 d = segment.b - segment.a;
  for (int axis = 0; axis < 2; ++axis) {
    const double p_vals[2] = {-d[axis], d[axis]};
    const double q_vals[2] = {segment.a[axis] - box.min[axis], box.max[axis] - segment.a[axis]};
    for (int k = 0; k < 2; ++k) {
      const double p = p_vals[k];
      const double q = q_vals[k];
      if (p == 0.0) {
        if (q < 0.0) {
          return false;  // parallel and outside this slab
        }
        continue;
      }
      const double r = q / p;
      if (p < 0.0) {
        t0 = std::max(t0, r);
      } else {
        t1 = std::min(t1, r);
      }
      if (t0 > t1) {
        return false;
      }
    }
  }
  return true;
}

bool intersects(const Circle& a, const Circle& b) noexcept {
  const double r = a.radius + b.radius;
  return (a.center - b.center).squaredNorm() <= r * r;
}

bool intersects(const Circle& circle, const AABB& box) noexcept {
  return distance(box, circle.center) <= circle.radius;
}

std::optional<double> ray_intersection(const Vec2& origin, const Vec2& direction,
                                       const Circle& circle) noexcept {
  const Vec2 f = origin - circle.center;
  const double c = f.squaredNorm() - circle.radius * circle.radius;
  if (c <= 0.0) {
    return 0.0;  // origin inside or on the disc
  }
  const double a = direction.squaredNorm();
  if (a <= 0.0) {
    return std::nullopt;
  }
  const double b = 2.0 * f.dot(direction);
  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0) {
    return std::nullopt;
  }
  // Origin is outside, so both roots share a sign (product c/a > 0).
  const double t = (-b - std::sqrt(discriminant)) / (2.0 * a);
  if (t < 0.0) {
    return std::nullopt;
  }
  return t;
}

std::optional<double> ray_intersection(const Vec2& origin, const Vec2& direction,
                                       const AABB& box) noexcept {
  if (box.contains(origin)) {
    return 0.0;
  }
  double t_min = -std::numeric_limits<double>::infinity();
  double t_max = std::numeric_limits<double>::infinity();
  for (int axis = 0; axis < 2; ++axis) {
    if (std::abs(direction[axis]) < 1e-15) {
      if (origin[axis] < box.min[axis] || origin[axis] > box.max[axis]) {
        return std::nullopt;
      }
      continue;
    }
    double t1 = (box.min[axis] - origin[axis]) / direction[axis];
    double t2 = (box.max[axis] - origin[axis]) / direction[axis];
    if (t1 > t2) {
      std::swap(t1, t2);
    }
    t_min = std::max(t_min, t1);
    t_max = std::min(t_max, t2);
  }
  if (t_max < std::max(t_min, 0.0)) {
    return std::nullopt;
  }
  return std::max(t_min, 0.0);
}

}  // namespace multirobotcore::math
