#include "multirobotcore/math/numerics.hpp"
#include "multirobotcore/models/robot_models.hpp"
#include "multirobotcore/models/robot_state.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace mrc = multirobotcore;
namespace models = multirobotcore::models;
using mrc::Vec2;

TEST(SingleIntegrator, Step) {
  EXPECT_TRUE(
      models::step_single_integrator({1.0, 2.0}, {0.5, -1.0}, 2.0).isApprox(Vec2(2.0, 0.0)));
}

TEST(DoubleIntegrator, ExactDiscretisationMatchesClosedForm) {
  models::DoubleIntegratorState s{{0.0, 0.0}, {1.0, 0.0}};
  const Vec2 a(0.0, 2.0);
  const double dt = 0.01;
  for (int k = 0; k < 100; ++k) {
    s = models::step_double_integrator(s, a, dt);
  }
  // t = 1: p = v0 t + a t^2 / 2
  EXPECT_NEAR(s.position.x(), 1.0, 1e-12);
  EXPECT_NEAR(s.position.y(), 1.0, 1e-12);
  EXPECT_NEAR(s.velocity.y(), 2.0, 1e-12);
}

TEST(Unicycle, FullCircleReturnsToStart) {
  models::UnicycleState s{{0.0, 0.0}, 0.0};
  const models::UnicycleInput u{1.0, 1.0};  // radius 1, period 2 pi
  const int steps = 100;
  const double dt = 2.0 * mrc::math::kPi / steps;
  double max_radius_error = 0.0;
  for (int k = 0; k < steps; ++k) {
    s = models::step_unicycle(s, u, dt);
    // The trajectory is a circle of radius 1 centred at (0, 1).
    max_radius_error =
        std::max(max_radius_error, std::abs((s.position - Vec2(0.0, 1.0)).norm() - 1.0));
  }
  EXPECT_LT(max_radius_error, 1e-12);
  EXPECT_NEAR(s.position.norm(), 0.0, 1e-9);
  EXPECT_NEAR(mrc::math::wrap_angle(s.heading), 0.0, 1e-9);
}

TEST(Unicycle, StraightLineWhenYawRateIsZero) {
  const models::UnicycleState s{{1.0, 1.0}, mrc::math::kPi / 2.0};
  const auto n = models::step_unicycle(s, {2.0, 0.0}, 0.5);
  EXPECT_TRUE(n.position.isApprox(Vec2(1.0, 2.0)));
}

TEST(Unicycle, LookaheadTrackingMovesOffAxisPointAsCommanded) {
  const models::UnicycleState s{{0.0, 0.0}, 0.3};
  const double l = 0.2;
  const Vec2 desired(0.4, -0.7);
  const auto u =
      models::track_velocity_lookahead(s, desired, l, models::UnicycleLimits{10.0, 10.0, true});
  const double c = std::cos(s.heading);
  const double sn = std::sin(s.heading);
  const Vec2 q_dot(c * u.linear - l * sn * u.angular, sn * u.linear + l * c * u.angular);
  EXPECT_TRUE(q_dot.isApprox(desired, 1e-12));
}

TEST(DifferentialDrive, KinematicsRoundTripAndSaturation) {
  const models::DifferentialDriveParams p{0.05, 0.3, 10.0};
  const models::UnicycleInput body{0.3, 0.8};
  const auto w = models::body_to_wheels(body, p);
  const auto back = models::wheels_to_body(w, p);
  EXPECT_NEAR(back.linear, body.linear, 1e-12);
  EXPECT_NEAR(back.angular, body.angular, 1e-12);

  const models::WheelSpeeds fast{20.0, 40.0};
  const auto sat = models::saturate(fast, p);
  EXPECT_NEAR(sat.right, 10.0, 1e-12);
  EXPECT_NEAR(sat.left, 5.0, 1e-12);  // curvature preserved
}

TEST(Bicycle, SteeringIsSaturated) {
  const models::BicycleParams params{0.5, 0.4};
  const models::UnicycleState s{{0.0, 0.0}, 0.0};
  const auto a = models::step_bicycle(s, {1.0, 1.5}, params, 0.1);
  const auto b = models::step_bicycle(s, {1.0, 0.4}, params, 0.1);
  EXPECT_NEAR(a.heading, b.heading, 1e-15);
  EXPECT_NEAR(a.heading, std::tan(0.4) / 0.5 * 0.1, 1e-12);
}

TEST(RobotState, StatusStrings) {
  EXPECT_EQ(models::to_string(models::RobotStatus::kFailed), "failed");
  EXPECT_TRUE(models::is_operational(models::RobotStatus::kActive));
  EXPECT_FALSE(models::is_operational(models::RobotStatus::kStopped));
}
