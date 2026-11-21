// Regression: a fixed Lloyd run must keep producing the same configuration.
// Values were recorded from the reference implementation; the tolerance
// allows for last-bit differences between math libraries.

#include "multirobotcore/coverage/coverage_control.hpp"

#include <gtest/gtest.h>

#include <random>
#include <vector>

namespace cov = multirobotcore::coverage;
using multirobotcore::Vec2;

TEST(CoverageRegression, LloydRunIsReproducible) {
  std::mt19937_64 rng(2024);
  std::uniform_real_distribution<double> u(0.0, 10.0);
  std::vector<Vec2> p;
  for (int i = 0; i < 8; ++i) {
    p.emplace_back(u(rng), u(rng));
  }
  const auto density =
      cov::DensityFunction::gaussian_mixture({{{7.0, 7.0}, 1.5, 4.0}, {{2.5, 3.0}, 1.0, 2.0}}, 0.2);
  const cov::CoverageController controller(
      multirobotcore::math::ConvexPolygon::from_box({{0.0, 0.0}, {10.0, 10.0}}), density, {});
  for (int k = 0; k < 25; ++k) {
    p = controller.lloyd_step(p);
  }
  const std::vector<Vec2> expected{
      {8.367413581755, 6.252718347929}, {2.077759676851, 5.238156825222},
      {1.836681256148, 2.398221107944}, {4.207632122540, 8.270356316923},
      {7.865606142118, 2.692716937200}, {6.047489962398, 6.106795122563},
      {7.426717705678, 8.358376675795}, {4.013258544057, 2.715839717009},
  };
  ASSERT_EQ(expected.size(), p.size());
  for (std::size_t i = 0; i < p.size(); ++i) {
    EXPECT_NEAR(p[i].x(), expected[i].x(), 1e-6) << i;
    EXPECT_NEAR(p[i].y(), expected[i].y(), 1e-6) << i;
  }
  EXPECT_NEAR(controller.evaluate(p).cost, 141.385535076822, 1e-6);
}
