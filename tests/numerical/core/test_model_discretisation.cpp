// Exact zero-order-hold discretisations versus a high-resolution RK4
// integration of the continuous models.

#include "multirobotcore/math/numerics.hpp"
#include "multirobotcore/models/robot_models.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace models = multirobotcore::models;
using multirobotcore::Vec2;

namespace {

/// RK4 for the unicycle x' = v cos(th), y' = v sin(th), th' = w.
std::array<double, 3> rk4_unicycle(std::array<double, 3> s, double v, double w, double t,
                                   int steps) {
  const double h = t / steps;
  auto f = [&](const std::array<double, 3>& z) {
    return std::array<double, 3>{v * std::cos(z[2]), v * std::sin(z[2]), w};
  };
  for (int k = 0; k < steps; ++k) {
    const auto k1 = f(s);
    const auto k2 = f({s[0] + 0.5 * h * k1[0], s[1] + 0.5 * h * k1[1], s[2] + 0.5 * h * k1[2]});
    const auto k3 = f({s[0] + 0.5 * h * k2[0], s[1] + 0.5 * h * k2[1], s[2] + 0.5 * h * k2[2]});
    const auto k4 = f({s[0] + h * k3[0], s[1] + h * k3[1], s[2] + h * k3[2]});
    for (int i = 0; i < 3; ++i) {
      s[static_cast<std::size_t>(i)] +=
          h / 6.0 *
          (k1[static_cast<std::size_t>(i)] + 2 * k2[static_cast<std::size_t>(i)] +
           2 * k3[static_cast<std::size_t>(i)] + k4[static_cast<std::size_t>(i)]);
    }
  }
  return s;
}

}  // namespace

TEST(ModelDiscretisation, UnicycleExactStepMatchesFineRk4) {
  for (const double w : {-1.3, -0.2, 0.0, 1e-10, 0.7, 2.5}) {
    const models::UnicycleState start{{0.4, -1.2}, 0.3};
    const double v = 0.9;
    const double dt = 0.8;
    const auto exact = models::step_unicycle(start, {v, w}, dt);
    const auto ref =
        rk4_unicycle({start.position.x(), start.position.y(), start.heading}, v, w, dt, 4000);
    EXPECT_NEAR(exact.position.x(), ref[0], 1e-11) << "w = " << w;
    EXPECT_NEAR(exact.position.y(), ref[1], 1e-11) << "w = " << w;
    EXPECT_NEAR(std::remainder(exact.heading - ref[2], 2 * multirobotcore::math::kPi), 0.0, 1e-12);
  }
}

TEST(ModelDiscretisation, StepIndependentForConstantInputs) {
  // Exact discretisations compose: one step of 2 dt equals two steps of dt.
  const models::UnicycleState s{{1.0, 2.0}, -0.4};
  const auto once = models::step_unicycle(s, {1.1, 0.9}, 0.6);
  const auto twice =
      models::step_unicycle(models::step_unicycle(s, {1.1, 0.9}, 0.3), {1.1, 0.9}, 0.3);
  EXPECT_LT((once.position - twice.position).norm(), 1e-13);

  const models::DoubleIntegratorState d{{0.0, 0.0}, {0.3, -0.2}};
  const Vec2 a(0.5, 1.5);
  const auto d_once = models::step_double_integrator(d, a, 1.0);
  auto d_many = d;
  for (int k = 0; k < 100; ++k) {
    d_many = models::step_double_integrator(d_many, a, 0.01);
  }
  EXPECT_LT((d_once.position - d_many.position).norm(), 1e-12);
  EXPECT_LT((d_once.velocity - d_many.velocity).norm(), 1e-12);
}

TEST(ModelDiscretisation, BicycleTurningRadiusMatchesGeometry) {
  // Constant steering: circle of radius L / tan(delta) about the rear axle.
  const models::BicycleParams params{0.8, 0.6};
  const double delta = 0.35;
  const double radius = params.wheelbase / std::tan(delta);
  models::UnicycleState s{{0.0, 0.0}, 0.0};
  const Vec2 centre(0.0, radius);
  double worst = 0.0;
  for (int k = 0; k < 500; ++k) {
    s = models::step_bicycle(s, {1.0, delta}, params, 0.02);
    worst = std::max(worst, std::abs((s.position - centre).norm() - radius));
  }
  EXPECT_LT(worst, 1e-11);
}
