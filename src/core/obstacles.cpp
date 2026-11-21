#include "multirobotcore/math/obstacles.hpp"

#include <algorithm>

namespace multirobotcore::math {

bool ObstacleSet::blocks(const Segment& segment) const noexcept {
  return std::any_of(circles.begin(), circles.end(),
                     [&](const Circle& c) { return intersects(segment, c); }) ||
         std::any_of(boxes.begin(), boxes.end(),
                     [&](const AABB& b) { return intersects(segment, b); });
}

bool ObstacleSet::collides(const Circle& body) const noexcept {
  return std::any_of(circles.begin(), circles.end(),
                     [&](const Circle& c) { return intersects(body, c); }) ||
         std::any_of(boxes.begin(), boxes.end(),
                     [&](const AABB& b) { return intersects(body, b); });
}

double ObstacleSet::distance(const Vec2& p) const noexcept {
  double best = std::numeric_limits<double>::infinity();
  for (const Circle& c : circles) {
    best = std::min(best, std::max(0.0, (p - c.center).norm() - c.radius));
  }
  for (const AABB& b : boxes) {
    best = std::min(best, math::distance(b, p));
  }
  return best;
}

Vec2 ObstacleSet::closest_point(const Vec2& p) const noexcept {
  double best = std::numeric_limits<double>::infinity();
  Vec2 result = p;
  for (const Circle& c : circles) {
    const Vec2 d = p - c.center;
    const double n = d.norm();
    const double dist = std::max(0.0, n - c.radius);
    if (dist < best) {
      best = dist;
      result = n <= c.radius ? p : Vec2(c.center + d * (c.radius / n));
    }
  }
  for (const AABB& b : boxes) {
    const Vec2 q = math::closest_point(b, p);
    const double dist = (q - p).norm();
    if (dist < best) {
      best = dist;
      result = q;
    }
  }
  return result;
}

}  // namespace multirobotcore::math
