#include "multirobotcore/avoidance/linear_program.hpp"
#include "multirobotcore/math/random.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace av = multirobotcore::avoidance;
using multirobotcore::Vec2;

namespace {

constexpr double kTwoPi = 6.283185307179586;

av::HalfPlane random_half_plane_containing(const Vec2& point, std::mt19937_64& rng) {
  std::uniform_real_distribution<double> angle(0.0, kTwoPi);
  std::uniform_real_distribution<double> margin(0.0, 0.4);
  const double a = angle(rng);
  const Vec2 n(std::cos(a), std::sin(a));
  return av::HalfPlane{n, n.dot(point) - margin(rng)};
}

/// Dense grid over the speed disc.
std::vector<Vec2> grid(double radius, double step) {
  std::vector<Vec2> g;
  for (double x = -radius; x <= radius + 1e-12; x += step) {
    for (double y = -radius; y <= radius + 1e-12; y += step) {
      if (x * x + y * y <= radius * radius) {
        g.emplace_back(x, y);
      }
    }
  }
  return g;
}

double max_violation(const std::vector<av::HalfPlane>& c, std::size_t from, const Vec2& v) {
  double worst = -std::numeric_limits<double>::infinity();
  for (std::size_t i = from; i < c.size(); ++i) {
    worst = std::max(worst, c[i].violation(v));
  }
  return worst;
}

}  // namespace

TEST(VelocityLp, UnconstrainedReturnsClampedPreference) {
  const auto r = av::solve_velocity_lp({}, 0, {3.0, 4.0}, 1.0);
  EXPECT_TRUE(r.feasible);
  EXPECT_TRUE(r.velocity.isApprox(Vec2(0.6, 0.8)));
  const auto inside = av::solve_velocity_lp({}, 0, {0.1, 0.2}, 1.0);
  EXPECT_TRUE(inside.velocity.isApprox(Vec2(0.1, 0.2)));
}

TEST(VelocityLp, ProjectsOntoViolatedHalfPlane) {
  // Keep x <= 0.2 (normal (-1, 0), offset -0.2); preference (1, 0.5).
  const std::vector<av::HalfPlane> c{{{-1.0, 0.0}, -0.2}};
  const auto r = av::solve_velocity_lp(c, 0, {1.0, 0.5}, 2.0);
  EXPECT_TRUE(r.feasible);
  EXPECT_NEAR(r.velocity.x(), 0.2, 1e-12);
  EXPECT_NEAR(r.velocity.y(), 0.5, 1e-12);
}

TEST(VelocityLp, HalfPlaneThroughPoint) {
  const auto h = av::half_plane_through({1.0, 1.0}, {0.0, 2.0});
  EXPECT_NEAR(h.offset, 1.0, 1e-12);
  EXPECT_NEAR(h.violation({5.0, 1.0}), 0.0, 1e-12);
  EXPECT_LT(h.violation({0.0, 2.0}), 0.0);
}

TEST(VelocityLp, MatchesBruteForceOnRandomFeasibleProblems) {
  std::mt19937_64 rng(42);
  std::uniform_real_distribution<double> u(-1.4, 1.4);
  std::uniform_int_distribution<int> count(1, 7);
  const double speed = 1.0;
  const double step = 0.01;
  const auto samples = grid(speed, step);
  for (int trial = 0; trial < 120; ++trial) {
    Vec2 interior = multirobotcore::math::sample_vec2(rng, u);
    interior = interior.norm() > 0.9 ? Vec2(interior.normalized() * 0.9) : interior;
    std::vector<av::HalfPlane> c;
    const int m = count(rng);
    for (int k = 0; k < m; ++k) {
      c.push_back(random_half_plane_containing(interior, rng));
    }
    const Vec2 pref = multirobotcore::math::sample_vec2(rng, u);
    const auto r = av::solve_velocity_lp(c, 0, pref, speed);
    ASSERT_TRUE(r.feasible) << "trial " << trial;
    EXPECT_LE(r.velocity.norm(), speed + 1e-9);
    EXPECT_LE(max_violation(c, 0, r.velocity), 1e-9);
    const double lp_cost = (r.velocity - pref).norm();
    double best = std::numeric_limits<double>::infinity();
    for (const Vec2& s : samples) {
      if (max_violation(c, 0, s) <= 0.0) {
        best = std::min(best, (s - pref).norm());
      }
    }
    // No feasible sample beats the LP optimum; the grid gets within its resolution.
    EXPECT_GE(best, lp_cost - 1e-9) << "trial " << trial;
    EXPECT_LE(lp_cost, best + step) << "trial " << trial;
  }
}

TEST(VelocityLp, InfeasibleProblemMinimisesLargestViolation) {
  // x >= 0.5 and x <= -0.5 cannot hold together: optimum x = 0, violation 0.5.
  const std::vector<av::HalfPlane> c{{{1.0, 0.0}, 0.5}, {{-1.0, 0.0}, 0.5}};
  const auto r = av::solve_velocity_lp(c, 0, {0.0, 0.3}, 1.0);
  EXPECT_FALSE(r.feasible);
  EXPECT_NEAR(r.velocity.x(), 0.0, 1e-9);
  EXPECT_NEAR(r.max_violation, 0.5, 1e-9);
}

TEST(VelocityLp, InfeasibleFallbackMatchesBruteForce) {
  std::mt19937_64 rng(7);
  std::uniform_real_distribution<double> angle(0.0, kTwoPi);
  std::uniform_real_distribution<double> offset(0.2, 0.9);
  const double speed = 1.0;
  const double step = 0.01;
  const auto samples = grid(speed, step);
  int infeasible = 0;
  for (int trial = 0; trial < 80; ++trial) {
    // Half-planes whose normals point outwards in all directions and whose
    // boundaries are far from the origin: usually jointly infeasible.
    std::vector<av::HalfPlane> c;
    for (int k = 0; k < 4; ++k) {
      const double a = angle(rng);
      c.push_back({{std::cos(a), std::sin(a)}, offset(rng)});
    }
    const auto r = av::solve_velocity_lp(c, 0, {0.0, 0.0}, speed);
    if (r.feasible) {
      continue;
    }
    ++infeasible;
    double best = std::numeric_limits<double>::infinity();
    for (const Vec2& s : samples) {
      best = std::min(best, max_violation(c, 0, s));
    }
    EXPECT_LE(r.velocity.norm(), speed + 1e-9);
    EXPECT_GE(r.max_violation, best - step) << "trial " << trial;
    EXPECT_LE(r.max_violation, best + 1e-6) << "trial " << trial;
  }
  EXPECT_GT(infeasible, 10);
}

TEST(VelocityLp, HardConstraintsStayHard) {
  // Hard: x >= 0.6. Soft: x <= -0.5 (offset 0.5 on normal (-1, 0)).
  const std::vector<av::HalfPlane> c{{{1.0, 0.0}, 0.6}, {{-1.0, 0.0}, 0.5}};
  const auto r = av::solve_velocity_lp(c, 1, {0.0, 0.0}, 1.0);
  EXPECT_FALSE(r.feasible);
  EXPECT_GE(r.velocity.x(), 0.6 - 1e-9);
  EXPECT_NEAR(r.max_violation, 1.1, 1e-9);
}

TEST(VelocityLp, HardConstraintOutsideSpeedDiscBecomesSoft) {
  const std::vector<av::HalfPlane> c{{{1.0, 0.0}, 2.0}};
  const auto r = av::solve_velocity_lp(c, 1, {0.0, 0.0}, 1.0);
  EXPECT_FALSE(r.feasible);
  EXPECT_NEAR(r.velocity.x(), 1.0, 1e-9);
  EXPECT_NEAR(r.max_violation, 1.0, 1e-9);
}

TEST(VelocityLp, RejectsInvalidArguments) {
  const std::vector<av::HalfPlane> c{{{1.0, 0.0}, 0.0}};
  EXPECT_THROW((void)av::solve_velocity_lp(c, 2, {0.0, 0.0}, 1.0), std::invalid_argument);
  EXPECT_THROW((void)av::solve_velocity_lp(c, 0, {0.0, 0.0}, -1.0), std::invalid_argument);
  EXPECT_THROW((void)av::half_plane_through({0.0, 0.0}, {0.0, 0.0}), std::invalid_argument);
}
