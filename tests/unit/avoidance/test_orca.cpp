#include "multirobotcore/avoidance/orca.hpp"
#include "multirobotcore/avoidance/velocity_obstacle.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

namespace av = multirobotcore::avoidance;
using multirobotcore::Vec2;

TEST(Orca, HeadOnConstraintsAreReciprocal) {
  const av::AgentState a{{0.0, 0.0}, {1.0, 0.0}, 0.5};
  const av::AgentState b{{4.0, 0.0}, {-1.0, 0.0}, 0.5};
  const double tau = 2.0;
  const auto ca = av::orca_constraint(a, b, tau, 0.1);
  const auto cb = av::orca_constraint(b, a, tau, 0.1);
  EXPECT_FALSE(ca.colliding);
  EXPECT_TRUE(ca.half_plane.normal.isApprox(-cb.half_plane.normal, 1e-12));
  EXPECT_TRUE(ca.u.isApprox(-cb.u, 1e-12));
  // On a collision course the current velocity is excluded, and the boundary
  // passes through v_A + u / 2.
  EXPECT_GT(ca.half_plane.violation(a.velocity), 0.0);
  EXPECT_NEAR(ca.half_plane.violation(a.velocity + 0.5 * ca.u), 0.0, 1e-12);
  // u moves the relative velocity exactly onto the VO boundary.
  const Vec2 rel = a.velocity - b.velocity + ca.u;
  EXPECT_EQ(av::classify_relative_velocity(b.position - a.position, rel, 1.0, tau, 1e-9),
            av::VOMembership::kBoundary);
}

TEST(Orca, CurrentVelocityAllowedWhenNoCollisionIsPredicted) {
  const av::AgentState a{{0.0, 0.0}, {1.0, 0.0}, 0.5};
  const av::AgentState b{{0.0, 5.0}, {1.0, 0.0}, 0.5};  // parallel motion
  const auto c = av::orca_constraint(a, b, 2.0, 0.1);
  EXPECT_LE(c.half_plane.violation(a.velocity), 0.0);
}

TEST(Orca, ReciprocalChoicesAreCollisionFreeForTheHorizon) {
  // Theorem (van den Berg et al. 2011): if v_A' in ORCA_A|B and v_B' in
  // ORCA_B|A then v_A' - v_B' is outside VO^tau.
  std::mt19937_64 rng(123);
  std::uniform_real_distribution<double> pos(-4.0, 4.0);
  std::uniform_real_distribution<double> vel(-1.5, 1.5);
  std::uniform_real_distribution<double> rad(0.2, 0.6);
  const double tau = 2.0;
  int checked = 0;
  for (int trial = 0; trial < 400; ++trial) {
    const av::AgentState a{{pos(rng), pos(rng)}, {vel(rng), vel(rng)}, rad(rng)};
    const av::AgentState b{{pos(rng), pos(rng)}, {vel(rng), vel(rng)}, rad(rng)};
    const double radius = a.radius + b.radius;
    if ((b.position - a.position).norm() <= radius) {
      continue;
    }
    const auto ca = av::orca_constraint(a, b, tau, 0.1);
    const auto cb = av::orca_constraint(b, a, tau, 0.1);
    for (int k = 0; k < 30; ++k) {
      const Vec2 va(vel(rng) * 2.0, vel(rng) * 2.0);
      const Vec2 vb(vel(rng) * 2.0, vel(rng) * 2.0);
      if (ca.half_plane.violation(va) > 0.0 || cb.half_plane.violation(vb) > 0.0) {
        continue;
      }
      ++checked;
      EXPECT_NE(av::classify_relative_velocity(b.position - a.position, va - vb, radius, tau, 1e-9),
                av::VOMembership::kInside)
          << "trial " << trial;
    }
  }
  EXPECT_GT(checked, 1000);
}

TEST(Orca, OverlappingAgentsArePushedApart) {
  const av::AgentState a{{0.0, 0.0}, {0.0, 0.0}, 0.5};
  const av::AgentState b{{0.8, 0.0}, {0.0, 0.0}, 0.5};
  const auto c = av::orca_constraint(a, b, 2.0, 0.1);
  EXPECT_TRUE(c.colliding);
  EXPECT_TRUE(c.half_plane.normal.isApprox(Vec2(-1.0, 0.0), 1e-12));
  // Required separation speed: half of the 0.2 m overlap per 0.1 s step.
  EXPECT_NEAR(c.half_plane.offset, 1.0, 1e-12);
}

TEST(Orca, StaticObstacleTakesFullResponsibility) {
  const av::AgentState a{{0.0, 0.0}, {1.0, 0.0}, 0.5};
  const av::AgentState wall{{3.0, 0.0}, {0.0, 0.0}, 0.5};
  const auto half = av::orca_constraint(a, wall, 4.0, 0.1, 0.5);
  const auto full = av::orca_constraint(a, wall, 4.0, 0.1, 1.0);
  EXPECT_NEAR(full.half_plane.violation(a.velocity + full.u), 0.0, 1e-12);
  EXPECT_GT(full.half_plane.offset, half.half_plane.offset);
}

TEST(Orca, VelocitySatisfiesAllConstraintsWhenFeasible) {
  const av::AgentState self{{0.0, 0.0}, {1.0, 0.0}, 0.3};
  const std::vector<av::AgentState> neighbors{{{2.0, 0.2}, {-1.0, 0.0}, 0.3},
                                              {{1.5, -1.5}, {0.0, 1.0}, 0.3},
                                              {{3.0, 2.0}, {-0.5, -0.5}, 0.3}};
  const std::vector<av::AgentState> obstacles{{{0.0, 1.2}, {0.0, 0.0}, 0.4}};
  const auto out = av::orca_velocity(self, {1.0, 0.0}, 1.2, neighbors, obstacles, {2.0, 1.0, 0.1});
  ASSERT_TRUE(out.feasible);
  ASSERT_EQ(out.constraints.size(), 4U);
  for (const auto& h : out.constraints) {
    EXPECT_LE(h.violation(out.velocity), 1e-9);
  }
  EXPECT_LE(out.velocity.norm(), 1.2 + 1e-9);
}

TEST(Orca, NoNeighboursReturnsPreferredVelocity) {
  const av::AgentState self{{0.0, 0.0}, {0.0, 0.0}, 0.3};
  const auto out = av::orca_velocity(self, {0.5, 0.5}, 1.0, {}, {}, {});
  EXPECT_TRUE(out.velocity.isApprox(Vec2(0.5, 0.5)));
}

TEST(Orca, InvalidParametersThrow) {
  const av::AgentState a{};
  const av::AgentState b{{1.0, 0.0}, {0.0, 0.0}, 0.2};
  EXPECT_THROW((void)av::orca_constraint(a, b, 0.0, 0.1), std::invalid_argument);
  EXPECT_THROW((void)av::orca_constraint(a, b, 1.0, 0.0), std::invalid_argument);
}
