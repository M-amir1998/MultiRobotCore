// Numerical accuracy of the coverage quadrature against closed-form integrals
// of a Gaussian density over a rectangle.

#include "multirobotcore/coverage/density.hpp"
#include "multirobotcore/coverage/quadrature.hpp"
#include "multirobotcore/math/convex_polygon.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <vector>

namespace cov = multirobotcore::coverage;
using multirobotcore::Vec2;

namespace {

// int_a^b exp(-(x - m)^2 / (2 s^2)) dx
double gauss_integral(double a, double b, double m, double s) {
  const double k = s * std::sqrt(std::numbers::pi / 2.0);
  return k * (std::erf((b - m) / (s * std::sqrt(2.0))) - std::erf((a - m) / (s * std::sqrt(2.0))));
}

// int_a^b x exp(-(x - m)^2 / (2 s^2)) dx
double gauss_first_moment(double a, double b, double m, double s) {
  const double e =
      s * s *
      (std::exp(-(a - m) * (a - m) / (2 * s * s)) - std::exp(-(b - m) * (b - m) / (2 * s * s)));
  return m * gauss_integral(a, b, m, s) + e;
}

}  // namespace

TEST(QuadratureConvergence, GaussianMassAndCentroidConvergeAtLeastQuadratically) {
  const double x0 = 0.0, x1 = 3.0, y0 = 0.0, y1 = 2.0;
  const Vec2 mean(1.1, 0.7);
  const double sigma = 0.6;
  const double weight = 2.0;
  const double floor = 0.3;
  const auto density = cov::DensityFunction::gaussian_mixture({{mean, sigma, weight}}, floor);
  const auto box = multirobotcore::math::ConvexPolygon::from_box({{x0, y0}, {x1, y1}});

  const double gx = gauss_integral(x0, x1, mean.x(), sigma);
  const double gy = gauss_integral(y0, y1, mean.y(), sigma);
  const double area = (x1 - x0) * (y1 - y0);
  const double exact_mass = floor * area + weight * gx * gy;
  const double exact_mx =
      floor * area * 0.5 * (x0 + x1) + weight * gauss_first_moment(x0, x1, mean.x(), sigma) * gy;
  const double exact_my =
      floor * area * 0.5 * (y0 + y1) + weight * gx * gauss_first_moment(y0, y1, mean.y(), sigma);
  const Vec2 exact_centroid(exact_mx / exact_mass, exact_my / exact_mass);

  std::vector<double> mass_error;
  std::vector<double> centroid_error;
  for (const int s : {2, 4, 8, 16, 32}) {
    const auto r = cov::integrate_cell(box, Vec2::Zero(), density, s);
    mass_error.push_back(std::abs(r.mass - exact_mass) / exact_mass);
    centroid_error.push_back((r.centroid - exact_centroid).norm());
  }
  // Errors decrease monotonically and with observed order >= 2 per halving of h.
  for (std::size_t k = 1; k < mass_error.size(); ++k) {
    EXPECT_LT(mass_error[k], mass_error[k - 1]);
    EXPECT_LT(centroid_error[k], centroid_error[k - 1]);
  }
  for (std::size_t k = 1; k + 1 < mass_error.size(); ++k) {
    EXPECT_GT(std::log2(mass_error[k] / mass_error[k + 1]), 1.9) << "refinement step " << k;
  }
  EXPECT_LT(mass_error.back(), 1e-6);
  EXPECT_LT(centroid_error.back(), 1e-6);
}

TEST(QuadratureConvergence, UniformDensityIsExactForSecondMoments) {
  // Triangle with vertices (0,0), (2,0), (0,1): area 1, centroid (2/3, 1/3),
  // polar moment about the origin  int (x^2 + y^2) dA = 5/6 (exact).
  const multirobotcore::math::ConvexPolygon tri({{0.0, 0.0}, {2.0, 0.0}, {0.0, 1.0}});
  const auto r = cov::integrate_cell(tri, Vec2::Zero(), cov::DensityFunction::uniform(1.0), 1);
  EXPECT_NEAR(r.mass, 1.0, 1e-14);
  EXPECT_NEAR(r.centroid.x(), 2.0 / 3.0, 1e-14);
  EXPECT_NEAR(r.centroid.y(), 1.0 / 3.0, 1e-14);
  EXPECT_NEAR(r.cost, 5.0 / 6.0, 1e-13);
}
