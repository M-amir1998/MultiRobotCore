#include "multirobotcore/avoidance/velocity_obstacle.hpp"
#include "multirobotcore/math/numerics.hpp"
#include "multirobotcore/math/random.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

namespace av = multirobotcore::avoidance;
using multirobotcore::Vec2;

TEST(TimeToCollision, KnownCases) {
  // Gap of 4 m closing at 1 m/s.
  EXPECT_NEAR(av::time_to_collision({5.0, 0.0}, {1.0, 0.0}, 1.0), 4.0, 1e-12);
  // Moving apart, static and passing by never collide.
  EXPECT_EQ(av::time_to_collision({5.0, 0.0}, {-1.0, 0.0}, 1.0), av::kNever);
  EXPECT_EQ(av::time_to_collision({5.0, 0.0}, {0.0, 0.0}, 1.0), av::kNever);
  EXPECT_EQ(av::time_to_collision({5.0, 0.0}, {0.0, 1.0}, 1.0), av::kNever);
  // Grazing pass at lateral offset 0.6 < R = 1.
  EXPECT_NEAR(av::time_to_collision({5.0, 0.6}, {1.0, 0.0}, 1.0), 5.0 - 0.8, 1e-12);
  // Overlapping discs collide now.
  EXPECT_EQ(av::time_to_collision({0.5, 0.0}, {-1.0, 0.0}, 1.0), 0.0);
}

TEST(TimeToCollision, EffectiveValueLetsOverlappingDiscsEscape) {
  EXPECT_EQ(av::effective_time_to_collision({0.5, 0.0}, {-1.0, 0.0}, 1.0), av::kNever);
  EXPECT_EQ(av::effective_time_to_collision({0.5, 0.0}, {1.0, 0.0}, 1.0), 0.0);
}

TEST(VelocityObstacle, InsideOutsideBoundaryAndTruncation) {
  const Vec2 p(5.0, 0.0);
  const double r = 1.0;
  const double tau = 10.0;
  EXPECT_EQ(av::classify_relative_velocity(p, {1.0, 0.0}, r, tau), av::VOMembership::kInside);
  EXPECT_EQ(av::classify_relative_velocity(p, {0.0, 1.0}, r, tau), av::VOMembership::kOutside);
  EXPECT_EQ(av::classify_relative_velocity(p, {-1.0, 0.0}, r, tau), av::VOMembership::kOutside);
  // Velocity along a tangent leg: minimum distance exactly R.
  const double a = std::asin(r / p.norm());
  EXPECT_EQ(av::classify_relative_velocity(p, {std::cos(a), std::sin(a)}, r, tau, 1e-9),
            av::VOMembership::kBoundary);
  EXPECT_EQ(
      av::classify_relative_velocity(p, 1.2 * Vec2(std::cos(a + 0.01), std::sin(a + 0.01)), r, tau),
      av::VOMembership::kOutside);
  // Truncation: 0.3 m/s covers 3 m in 10 s (no contact); 0.45 m/s covers 4.5 m.
  EXPECT_EQ(av::classify_relative_velocity(p, {0.3, 0.0}, r, tau), av::VOMembership::kOutside);
  EXPECT_EQ(av::classify_relative_velocity(p, {0.45, 0.0}, r, tau), av::VOMembership::kInside);
  // Exactly reaching the disc at t = tau is the boundary.
  EXPECT_EQ(av::classify_relative_velocity(p, {0.4, 0.0}, r, tau, 1e-9),
            av::VOMembership::kBoundary);
}

TEST(VelocityObstacle, ConeGeometryIsTangentToCutoffDisc) {
  const av::AgentState a{{1.0, 2.0}, {0.3, 0.1}, 0.4};
  const av::AgentState b{{4.0, 6.0}, {-0.2, 0.5}, 0.6};
  const double tau = 2.0;
  const auto vo = av::velocity_obstacle(a, b, tau);
  ASSERT_FALSE(vo.overlapping);
  EXPECT_TRUE(vo.apex.isApprox(b.velocity));
  EXPECT_NEAR(vo.left_leg.norm(), 1.0, 1e-12);
  EXPECT_NEAR(vo.right_leg.norm(), 1.0, 1e-12);
  EXPECT_NEAR(vo.cutoff_radius, 1.0 / tau, 1e-12);
  // Distance from the cut-off centre to each leg line equals the cut-off radius.
  const Vec2 c = vo.cutoff_center - vo.apex;
  for (const Vec2& leg : {vo.left_leg, vo.right_leg}) {
    EXPECT_NEAR(std::abs(multirobotcore::math::cross(leg, c)), vo.cutoff_radius, 1e-12);
  }
  // The left leg is counter-clockwise of the centre line.
  EXPECT_GT(multirobotcore::math::cross(c, vo.left_leg), 0.0);
  EXPECT_LT(multirobotcore::math::cross(c, vo.right_leg), 0.0);
  // Points along the legs (beyond the tangent point) lie on the boundary.
  EXPECT_EQ(vo.classify(vo.apex + 3.0 * vo.left_leg, 1e-9), av::VOMembership::kBoundary);
  EXPECT_TRUE(vo.contains(vo.cutoff_center + 0.01 * c));
}

TEST(VelocityObstacle, ReciprocalConeHalvesTheEffort) {
  const av::AgentState a{{0.0, 0.0}, {1.0, 0.0}, 0.5};
  const av::AgentState b{{4.0, 0.0}, {-1.0, 0.0}, 0.5};
  const auto rvo = av::reciprocal_velocity_obstacle(a, b, 2.0);
  EXPECT_TRUE(rvo.apex.isApprox(Vec2(0.0, 0.0)));
  EXPECT_DOUBLE_EQ(rvo.time_horizon, 4.0);
  // v' is in the RVO iff 2 v' - v_A is in the VO of B.
  const auto vo = av::velocity_obstacle(a, b, 2.0);
  std::mt19937_64 rng(3);
  std::uniform_real_distribution<double> u(-2.0, 2.0);
  for (int k = 0; k < 2000; ++k) {
    const Vec2 v = multirobotcore::math::sample_vec2(rng, u);
    const auto in_rvo = rvo.classify(v, 1e-12);
    const auto in_vo = vo.classify(2.0 * v - a.velocity, 1e-12);
    if (in_rvo != av::VOMembership::kBoundary && in_vo != av::VOMembership::kBoundary) {
      EXPECT_EQ(in_rvo, in_vo);
    }
  }
}

TEST(VelocityObstacle, MembershipAgreesWithCollisionTime) {
  std::mt19937_64 rng(17);
  std::uniform_real_distribution<double> pos(-6.0, 6.0);
  std::uniform_real_distribution<double> vel(-3.0, 3.0);
  const double r = 1.0;
  const double tau = 2.5;
  int inside = 0;
  for (int k = 0; k < 5000; ++k) {
    const Vec2 p = multirobotcore::math::sample_vec2(rng, pos);
    if (p.norm() <= r) {
      continue;
    }
    const Vec2 v = multirobotcore::math::sample_vec2(rng, vel);
    const auto m = av::classify_relative_velocity(p, v, r, tau, 1e-9);
    const double tc = av::time_to_collision(p, v, r);
    if (m == av::VOMembership::kInside) {
      ++inside;
      EXPECT_LT(tc, tau);
    } else if (m == av::VOMembership::kOutside) {
      EXPECT_GT(tc, tau);
    }
  }
  EXPECT_GT(inside, 100);
}

TEST(VelocityObstacle, OverlappingDiscsCoverTheWholePlane) {
  const av::AgentState a{{0.0, 0.0}, {0.0, 0.0}, 0.5};
  const av::AgentState b{{0.6, 0.0}, {0.0, 0.0}, 0.5};
  const auto vo = av::velocity_obstacle(a, b, 2.0);
  EXPECT_TRUE(vo.overlapping);
  EXPECT_TRUE(vo.contains({-3.0, 0.0}));
}
