#include "multirobotcore/coverage/coverage_control.hpp"
#include "multirobotcore/coverage/density.hpp"
#include "multirobotcore/coverage/quadrature.hpp"
#include "multirobotcore/coverage/voronoi.hpp"
#include "multirobotcore/math/numerics.hpp"
#include "multirobotcore/math/random.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

namespace cov = multirobotcore::coverage;
namespace math = multirobotcore::math;
using multirobotcore::Vec2;

namespace {

math::ConvexPolygon square(double side) {
  return math::ConvexPolygon::from_box({{0.0, 0.0}, {side, side}});
}

std::vector<Vec2> random_points(std::size_t n, double side, std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> u(0.0, side);
  std::vector<Vec2> p;
  for (std::size_t i = 0; i < n; ++i) {
    p.push_back(math::sample_vec2(rng, u));
  }
  return p;
}

cov::DensityFunction two_bumps() {
  return cov::DensityFunction::gaussian_mixture({{{7.5, 7.0}, 1.2, 6.0}, {{2.5, 2.0}, 0.8, 2.0}},
                                                0.2);
}

}  // namespace

TEST(Voronoi, PartitionCoversDomainAndMatchesNearestGenerator) {
  const auto domain = square(10.0);
  for (std::uint64_t seed = 1; seed <= 5; ++seed) {
    const auto p = random_points(15, 10.0, seed);
    const auto cells = cov::bounded_voronoi(p, domain);
    double area = 0.0;
    for (const auto& c : cells) {
      area += c.polygon.area();
    }
    EXPECT_NEAR(area, 100.0, 1e-9);
    // Ownership: every sample lies in the cell of its nearest generator.
    const auto samples = random_points(400, 10.0, 100 + seed);
    for (const Vec2& q : samples) {
      std::size_t owner = 0;
      for (std::size_t i = 1; i < p.size(); ++i) {
        if ((q - p[i]).squaredNorm() < (q - p[owner]).squaredNorm()) {
          owner = i;
        }
      }
      EXPECT_TRUE(cells[owner].polygon.contains(q, 1e-9));
    }
  }
}

TEST(Voronoi, NeighboursAreSymmetricAndShareEdges) {
  const auto p = random_points(20, 10.0, 9);
  const auto cells = cov::bounded_voronoi(p, square(10.0));
  for (const auto& c : cells) {
    for (const std::size_t j : c.neighbors) {
      const auto& back = cells[j].neighbors;
      EXPECT_TRUE(std::binary_search(back.begin(), back.end(), c.generator))
          << c.generator << " <-> " << j;
    }
  }
  // Two generators split the square into two halves and are neighbours.
  const std::vector<Vec2> two{{2.0, 5.0}, {8.0, 5.0}};
  const auto halves = cov::bounded_voronoi(two, square(10.0));
  EXPECT_NEAR(halves[0].polygon.area(), 50.0, 1e-12);
  EXPECT_EQ(halves[0].neighbors, (std::vector<std::size_t>{1}));
}

TEST(Voronoi, EdgeCases) {
  const auto domain = square(4.0);
  // Single robot: the cell is the whole domain.
  const std::vector<Vec2> one{{1.0, 1.0}};
  EXPECT_NEAR(cov::bounded_voronoi(one, domain)[0].polygon.area(), 16.0, 1e-12);
  // Coincident robots: the lower index owns the region.
  const std::vector<Vec2> same{{1.0, 1.0}, {1.0, 1.0}, {3.0, 3.0}};
  const auto cells = cov::bounded_voronoi(same, domain);
  EXPECT_FALSE(cells[0].polygon.empty());
  EXPECT_TRUE(cells[1].polygon.empty());
  EXPECT_NEAR(cells[0].polygon.area() + cells[2].polygon.area(), 16.0, 1e-12);
  // Robot on the boundary.
  const std::vector<Vec2> boundary{{0.0, 2.0}, {3.0, 2.0}};
  const auto b = cov::bounded_voronoi(boundary, domain);
  EXPECT_TRUE(b[0].polygon.contains({0.0, 2.0}, 1e-12));
  EXPECT_NEAR(b[0].polygon.area(), 6.0, 1e-12);
  EXPECT_THROW((void)cov::voronoi_cell(5, boundary, domain), std::out_of_range);
}

TEST(Quadrature, ExactForUniformDensity) {
  const auto uniform = cov::DensityFunction::uniform(1.0);
  const auto unit = square(1.0);
  const auto sq = cov::integrate_cell(unit, {0.0, 0.0}, uniform, 1);
  EXPECT_NEAR(sq.mass, 1.0, 1e-14);
  EXPECT_TRUE(sq.centroid.isApprox(Vec2(0.5, 0.5), 1e-14));
  EXPECT_NEAR(sq.cost, 2.0 / 3.0, 1e-14);  // int (x^2 + y^2) over the unit square

  const math::ConvexPolygon tri({{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}});
  const Vec2 c(1.0 / 3.0, 1.0 / 3.0);
  const auto t = cov::integrate_cell(tri, c, uniform, 1);
  EXPECT_NEAR(t.mass, 0.5, 1e-14);
  EXPECT_TRUE(t.centroid.isApprox(c, 1e-14));
  // Polar moment about the centroid: A (a^2 + b^2 + c^2) / 36 = 0.5 * 4 / 36.
  EXPECT_NEAR(t.cost, 1.0 / 18.0, 1e-14);
}

TEST(Quadrature, ConvergesForGaussianDensity) {
  const double pi = math::kPi;
  const auto gauss = cov::DensityFunction::gaussian_mixture({{{0.0, 0.0}, 1.0, 1.0}}, 0.0);
  const auto domain = math::ConvexPolygon::from_box({{-7.0, -7.0}, {7.0, 7.0}});
  const double exact_mass = 2.0 * pi;          // truncation at 7 sigma is negligible
  const double exact_second = 2.0 * 2.0 * pi;  // int ||q||^2 phi = 2 sigma^2 * mass
  double previous_error = 1e9;
  for (const int s : {2, 4, 8, 16}) {
    const auto r = cov::integrate_cell(domain, {0.0, 0.0}, gauss, s);
    const double error = std::abs(r.mass - exact_mass) / exact_mass;
    EXPECT_LT(error, previous_error);
    previous_error = error;
    EXPECT_LT(r.centroid.norm(), 1e-12);  // symmetric quadrature
    if (s == 16) {
      EXPECT_LT(error, 1e-5);
      EXPECT_NEAR(r.cost / exact_second, 1.0, 1e-4);
    }
  }
}

TEST(Quadrature, EmptyCellAndInvalidSubdivisions) {
  const auto r =
      cov::integrate_cell(math::ConvexPolygon{}, {1.0, 2.0}, cov::DensityFunction::uniform());
  EXPECT_DOUBLE_EQ(r.mass, 0.0);
  EXPECT_TRUE(r.centroid.isApprox(Vec2(1.0, 2.0)));
  EXPECT_THROW(
      (void)cov::integrate_cell(square(1.0), {0.0, 0.0}, cov::DensityFunction::uniform(), 0),
      std::invalid_argument);
}

TEST(Density, ValidationAndEvaluation) {
  EXPECT_THROW((void)cov::DensityFunction::uniform(0.0), std::invalid_argument);
  EXPECT_THROW((void)cov::DensityFunction::gaussian_mixture({{{0.0, 0.0}, 0.0, 1.0}}, 1.0),
               std::invalid_argument);
  EXPECT_THROW((void)cov::DensityFunction::gaussian_mixture({}, 0.0), std::invalid_argument);
  const auto d = cov::DensityFunction::gaussian_mixture({{{1.0, 1.0}, 0.5, 3.0}}, 0.5);
  EXPECT_NEAR(d({1.0, 1.0}), 3.5, 1e-12);
  EXPECT_NEAR(d({100.0, 100.0}), 0.5, 1e-12);
  EXPECT_TRUE(cov::DensityFunction::uniform(2.0).is_uniform());
}

TEST(CoverageControl, LloydIterationIsMonotoneAndConverges) {
  const cov::CoverageController controller(square(10.0), two_bumps(), {1.0, 1.0, 6, {}});
  auto p = random_points(10, 10.0, 21);
  double cost = controller.evaluate(p).cost;
  double error = 0.0;
  for (int k = 0; k < 300; ++k) {
    cov::CoverageState state;
    p = controller.lloyd_step(p, &state);
    error = state.max_centroid_error;
    const double next = controller.evaluate(p).cost;
    EXPECT_LE(next, cost * (1.0 + 1e-9) + 1e-12) << "iteration " << k;
    cost = next;
  }
  EXPECT_LT(error, 1e-2);
}

TEST(CoverageControl, GradientFlowDecreasesCost) {
  const cov::CoverageController controller(square(10.0), two_bumps(), {1.0, 2.0, 4, {}});
  auto p = random_points(8, 10.0, 4);
  const double initial = controller.evaluate(p).cost;
  for (int k = 0; k < 200; ++k) {
    const auto u = controller.velocities(p);
    for (std::size_t i = 0; i < p.size(); ++i) {
      EXPECT_LE(u[i].norm(), 2.0 + 1e-12);
      p[i] += 0.05 * u[i];
    }
  }
  EXPECT_LT(controller.evaluate(p).cost, 0.6 * initial);
}

TEST(CoverageControl, DensityAttractsRobots) {
  const auto density = cov::DensityFunction::gaussian_mixture({{{8.0, 5.0}, 1.0, 20.0}}, 0.1);
  const cov::CoverageController controller(square(10.0), density, {1.0, 1.0, 4, {}});
  auto p = random_points(12, 10.0, 33);
  for (int k = 0; k < 100; ++k) {
    p = controller.lloyd_step(p);
  }
  const auto right = std::count_if(p.begin(), p.end(), [](const Vec2& q) { return q.x() > 5.0; });
  EXPECT_GE(right, 9);
}

TEST(CoverageControl, SingleRobotMovesToDomainCentroid) {
  const cov::CoverageController controller(square(6.0), cov::DensityFunction::uniform(), {});
  const std::vector<Vec2> p{{0.5, 5.0}};
  const auto next = controller.lloyd_step(p);
  EXPECT_TRUE(next[0].isApprox(Vec2(3.0, 3.0), 1e-12));
}

TEST(CoverageControl, CoincidentRobotsSeparateUnderGradientFlow) {
  const cov::CoverageController controller(square(4.0), cov::DensityFunction::uniform(),
                                           {1.0, 1.0, 4, {}});
  std::vector<Vec2> p{{1.0, 1.0}, {1.0, 1.0}, {3.0, 3.0}};
  for (int k = 0; k < 400; ++k) {
    const auto u = controller.velocities(p);
    for (std::size_t i = 0; i < p.size(); ++i) {
      ASSERT_TRUE(math::is_finite(u[i]));
      p[i] += 0.05 * u[i];
    }
  }
  EXPECT_GT((p[0] - p[1]).norm(), 0.5);
  for (const auto& c : controller.evaluate(p).cells) {
    EXPECT_FALSE(c.polygon.empty());
  }
}

TEST(CoverageControl, LimitedRangeUsesRLimitedCells) {
  const auto domain = square(10.0);
  const auto p = random_points(25, 10.0, 77);
  const cov::CoverageController exact(domain, cov::DensityFunction::uniform(), {});
  const cov::CoverageController wide(domain, cov::DensityFunction::uniform(), {1.0, 1.0, 4, 30.0});
  const cov::CoverageController narrow(domain, cov::DensityFunction::uniform(), {1.0, 1.0, 4, 2.0});
  const auto e = exact.evaluate(p);
  const auto w = wide.controller_state(p);
  const auto s = narrow.controller_state(p);
  std::size_t certified = 0;
  for (std::size_t i = 0; i < p.size(); ++i) {
    // Large radius: every cell certified and identical to the Voronoi cell.
    EXPECT_NEAR(w.cells[i].polygon.area(), e.cells[i].polygon.area(), 1e-9);
    EXPECT_TRUE(w.certified_exact[i]);
    // Small radius: the r-limited cell equals the exact Voronoi cell clipped
    // to B(p_i, R/2), and is contained in the Voronoi cell.
    const auto reference = s.certified_exact[i] ? e.cells[i].polygon
                                                : cov::clip_to_disc(e.cells[i].polygon, p[i], 1.0);
    EXPECT_NEAR(s.cells[i].polygon.area(), reference.area(), 1e-9);
    EXPECT_LE(s.cells[i].polygon.area(), e.cells[i].polygon.area() + 1e-9);
    if (s.certified_exact[i]) {
      ++certified;
    }
  }
  EXPECT_LT(certified, p.size());
  EXPECT_NEAR(s.cost, e.cost, 1e-9);  // reported cost always uses the exact partition
  EXPECT_GT(w.communicating_pairs, s.communicating_pairs);
}

TEST(CoverageControl, IsolatedRobotsDoNotCollapseUnderLimitedRange) {
  // With a radius smaller than the spacing every robot is isolated; it must
  // stay near its own region instead of jumping to the global centroid.
  const cov::CoverageController controller(square(20.0), cov::DensityFunction::uniform(),
                                           {1.0, 1.0, 4, 1.0});
  auto p = random_points(10, 20.0, 5);
  const auto start = p;
  for (int k = 0; k < 20; ++k) {
    p = controller.lloyd_step(p);
  }
  for (std::size_t i = 0; i < p.size(); ++i) {
    EXPECT_LT((p[i] - start[i]).norm(), 1.0);
  }
}

TEST(Voronoi, ClipToDiscAreaMatchesInscribedPolygon) {
  const auto big = square(10.0);
  const auto disc = cov::clip_to_disc(big, {5.0, 5.0}, 2.0, 32);
  const double expected = 0.5 * 32 * 4.0 * std::sin(2.0 * math::kPi / 32);
  EXPECT_NEAR(disc.area(), expected, 1e-9);
  EXPECT_NEAR(disc.area() / (math::kPi * 4.0), 1.0, 0.0065);
  EXPECT_THROW((void)cov::clip_to_disc(big, {5.0, 5.0}, 0.0), std::invalid_argument);
}

TEST(CoverageMetrics, SingleRobotCentredInSquare) {
  const std::vector<Vec2> p{{0.5, 0.5}};
  const auto m = cov::evaluate_metrics(p, square(1.0), cov::DensityFunction::uniform(), 100);
  EXPECT_LE(m.max_uncovered_distance, std::sqrt(0.5) + 1e-12);
  EXPECT_GT(m.max_uncovered_distance, 0.69);
  // Mean distance from the centre of a unit square is about 0.3826.
  EXPECT_NEAR(m.mean_distance, 0.3826, 2e-3);
  EXPECT_NEAR(m.cost, 1.0 / 6.0, 1e-12);
  EXPECT_THROW((void)cov::evaluate_metrics({}, square(1.0), cov::DensityFunction::uniform()),
               std::invalid_argument);
}
