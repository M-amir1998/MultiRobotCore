#include "multirobotcore/math/convex_polygon.hpp"
#include "multirobotcore/math/geometry.hpp"
#include "multirobotcore/math/numerics.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <stdexcept>

namespace mrc = multirobotcore;
using mrc::Vec2;
using mrc::math::AABB;
using mrc::math::Circle;
using mrc::math::ConvexPolygon;
using mrc::math::Segment;

TEST(Numerics, WrapAngleStaysInHalfOpenInterval) {
  EXPECT_NEAR(mrc::math::wrap_angle(3.0 * mrc::math::kPi), mrc::math::kPi, 1e-12);
  EXPECT_NEAR(mrc::math::wrap_angle(-mrc::math::kPi), mrc::math::kPi, 1e-12);
  EXPECT_NEAR(mrc::math::wrap_angle(0.25), 0.25, 1e-15);
  for (double a = -20.0; a <= 20.0; a += 0.37) {
    const double w = mrc::math::wrap_angle(a);
    EXPECT_GT(w, -mrc::math::kPi);
    EXPECT_LE(w, mrc::math::kPi);
    EXPECT_NEAR(std::cos(w), std::cos(a), 1e-12);
    EXPECT_NEAR(std::sin(w), std::sin(a), 1e-12);
  }
}

TEST(Numerics, ClampNormPreservesDirection) {
  const Vec2 v(3.0, 4.0);
  const Vec2 c = mrc::math::clamp_norm(v, 1.0);
  EXPECT_NEAR(c.norm(), 1.0, 1e-12);
  EXPECT_NEAR(c.x() / c.y(), 0.75, 1e-12);
  EXPECT_EQ(mrc::math::clamp_norm(v, 10.0), v);
}

TEST(Numerics, SmoothstepEndpointsAndMonotonicity) {
  EXPECT_DOUBLE_EQ(mrc::math::smoothstep5(0.0), 0.0);
  EXPECT_DOUBLE_EQ(mrc::math::smoothstep5(1.0), 1.0);
  EXPECT_DOUBLE_EQ(mrc::math::smoothstep5(0.5), 0.5);
  double prev = 0.0;
  for (double s = 0.0; s <= 1.0; s += 0.01) {
    const double v = mrc::math::smoothstep5(s);
    EXPECT_GE(v, prev - 1e-15);
    prev = v;
  }
}

TEST(Geometry, SegmentClosestPointAndDistance) {
  const Segment s{{0.0, 0.0}, {2.0, 0.0}};
  EXPECT_TRUE(mrc::math::closest_point(s, {1.0, 1.0}).isApprox(Vec2(1.0, 0.0)));
  EXPECT_TRUE(mrc::math::closest_point(s, {-1.0, 1.0}).isApprox(Vec2(0.0, 0.0)));
  EXPECT_NEAR(mrc::math::distance(s, {3.0, 0.0}), 1.0, 1e-12);
  const Segment degenerate{{1.0, 1.0}, {1.0, 1.0}};
  EXPECT_NEAR(mrc::math::distance(degenerate, {4.0, 5.0}), 5.0, 1e-12);
}

TEST(Geometry, SegmentIntersections) {
  const Circle c{{0.0, 0.0}, 1.0};
  EXPECT_TRUE(mrc::math::intersects(Segment{{-2.0, 0.5}, {2.0, 0.5}}, c));
  EXPECT_TRUE(mrc::math::intersects(Segment{{-2.0, 1.0}, {2.0, 1.0}}, c));  // tangent
  EXPECT_FALSE(mrc::math::intersects(Segment{{-2.0, 1.1}, {2.0, 1.1}}, c));
  const AABB box{{0.0, 0.0}, {1.0, 1.0}};
  EXPECT_TRUE(mrc::math::intersects(Segment{{-1.0, 0.5}, {2.0, 0.5}}, box));
  EXPECT_TRUE(mrc::math::intersects(Segment{{0.2, 0.2}, {0.3, 0.3}}, box));  // fully inside
  EXPECT_FALSE(mrc::math::intersects(Segment{{-1.0, 2.0}, {2.0, 1.5}}, box));
  EXPECT_FALSE(mrc::math::intersects(Segment{{2.0, 0.0}, {2.0, 1.0}}, box));  // parallel outside
}

TEST(Geometry, RayIntersections) {
  const Circle c{{5.0, 0.0}, 1.0};
  const auto t = mrc::math::ray_intersection({0.0, 0.0}, {1.0, 0.0}, c);
  ASSERT_TRUE(t.has_value());
  EXPECT_NEAR(*t, 4.0, 1e-12);
  EXPECT_FALSE(mrc::math::ray_intersection({0.0, 0.0}, {-1.0, 0.0}, c).has_value());
  EXPECT_NEAR(*mrc::math::ray_intersection({5.0, 0.5}, {1.0, 0.0}, c), 0.0, 0.0);

  const AABB box{{2.0, -1.0}, {3.0, 1.0}};
  const auto tb = mrc::math::ray_intersection({0.0, 0.0}, {2.0, 0.0}, box);
  ASSERT_TRUE(tb.has_value());
  EXPECT_NEAR(*tb, 1.0, 1e-12);
  EXPECT_FALSE(mrc::math::ray_intersection({0.0, 0.0}, {0.0, 1.0}, box).has_value());
}

TEST(ConvexPolygon, AreaCentroidAndOrientation) {
  const ConvexPolygon square = ConvexPolygon::from_box({{0.0, 0.0}, {2.0, 2.0}});
  EXPECT_NEAR(square.area(), 4.0, 1e-12);
  EXPECT_TRUE(square.centroid().isApprox(Vec2(1.0, 1.0)));
  // Clockwise input is reoriented.
  const ConvexPolygon cw({{0.0, 0.0}, {0.0, 1.0}, {1.0, 0.0}});
  EXPECT_NEAR(cw.area(), 0.5, 1e-12);
  EXPECT_TRUE(cw.centroid().isApprox(Vec2(1.0 / 3.0, 1.0 / 3.0)));
}

TEST(ConvexPolygon, RejectsNonConvexInput) {
  EXPECT_THROW(ConvexPolygon({{0.0, 0.0}, {2.0, 0.0}, {1.0, 0.5}, {2.0, 2.0}, {0.0, 2.0}}),
               std::invalid_argument);
}

TEST(ConvexPolygon, HalfPlaneClipping) {
  const ConvexPolygon square = ConvexPolygon::from_box({{0.0, 0.0}, {2.0, 2.0}});
  // Keep x <= 1.
  const ConvexPolygon left = square.clipped({1.0, 0.0}, 1.0);
  EXPECT_NEAR(left.area(), 2.0, 1e-12);
  EXPECT_TRUE(left.centroid().isApprox(Vec2(0.5, 1.0)));
  // Diagonal cut keeps the lower-left triangle: x + y <= 2.
  const ConvexPolygon tri = square.clipped({1.0, 1.0}, 2.0);
  EXPECT_NEAR(tri.area(), 2.0, 1e-12);
  // Clipping away everything yields an empty polygon.
  EXPECT_TRUE(square.clipped({1.0, 0.0}, -1.0).empty());
  // A non-cutting half-plane leaves the polygon unchanged.
  EXPECT_NEAR(square.clipped({1.0, 0.0}, 5.0).area(), 4.0, 1e-12);
}

TEST(ConvexPolygon, RandomClippingPreservesConvexityAndContainment) {
  std::mt19937_64 rng(7);
  std::uniform_real_distribution<double> angle(0.0, 2.0 * mrc::math::kPi);
  std::uniform_real_distribution<double> off(-1.0, 3.0);
  ConvexPolygon poly = ConvexPolygon::from_box({{0.0, 0.0}, {4.0, 4.0}});
  double previous_area = poly.area();
  for (int i = 0; i < 6 && !poly.empty(); ++i) {
    const double a = angle(rng);
    const Vec2 n(std::cos(a), std::sin(a));
    const double c = n.dot(Vec2(2.0, 2.0)) + off(rng) * 0.3;
    poly = poly.clipped(n, c);
    EXPECT_LE(poly.area(), previous_area + 1e-12);
    previous_area = poly.area();
    for (const Vec2& v : poly.vertices()) {
      EXPECT_LE(n.dot(v), c + 1e-9);
    }
    if (!poly.empty()) {
      EXPECT_TRUE(poly.contains(poly.centroid()));
      // Reconstructing through the validating constructor must succeed.
      EXPECT_NO_THROW(ConvexPolygon(poly.vertices()));
    }
  }
}
