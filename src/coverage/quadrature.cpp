#include "multirobotcore/coverage/quadrature.hpp"

#include "multirobotcore/math/numerics.hpp"

#include <cmath>
#include <stdexcept>

namespace multirobotcore::coverage {
namespace {

struct Accumulator {
  double mass{0.0};
  Vec2 first{Vec2::Zero()};
  double cost{0.0};
};

/// Symmetric 3-point rule (exact for quadratics) on triangle (x, y, z).
void integrate_subtriangle(const Vec2& x, const Vec2& y, const Vec2& z, double area,
                           const Vec2& generator, const DensityFunction& density,
                           Accumulator& acc) {
  constexpr double kMajor = 2.0 / 3.0;
  constexpr double kMinor = 1.0 / 6.0;
  const Vec2 points[3] = {kMajor * x + kMinor * y + kMinor * z,
                          kMinor * x + kMajor * y + kMinor * z,
                          kMinor * x + kMinor * y + kMajor * z};
  const double w = area / 3.0;
  for (const Vec2& q : points) {
    const double f = w * density(q);
    acc.mass += f;
    acc.first += f * q;
    acc.cost += f * (q - generator).squaredNorm();
  }
}

void integrate_triangle(const Vec2& a, const Vec2& b, const Vec2& c, int s, const Vec2& generator,
                        const DensityFunction& density, Accumulator& acc) {
  const double area = 0.5 * std::abs(math::cross(b - a, c - a));
  if (area <= 0.0) {
    return;
  }
  const auto sd = static_cast<double>(s);
  const double sub_area = area / (sd * sd);
  const Vec2 e1 = (b - a) / sd;
  const Vec2 e2 = (c - a) / sd;
  for (int i = 0; i < s; ++i) {
    for (int j = 0; j + i < s; ++j) {
      const Vec2 p0 = a + static_cast<double>(i) * e1 + static_cast<double>(j) * e2;
      integrate_subtriangle(p0, p0 + e1, p0 + e2, sub_area, generator, density, acc);
      if (i + j <= s - 2) {
        integrate_subtriangle(p0 + e1, p0 + e1 + e2, p0 + e2, sub_area, generator, density, acc);
      }
    }
  }
}

}  // namespace

CellIntegrals integrate_cell(const math::ConvexPolygon& cell, const Vec2& generator,
                             const DensityFunction& density, int subdivisions) {
  if (subdivisions < 1) {
    throw std::invalid_argument("integrate_cell: subdivisions must be at least 1");
  }
  CellIntegrals out;
  out.centroid = generator;
  if (cell.empty()) {
    return out;
  }
  Accumulator acc;
  const auto& v = cell.vertices();
  for (std::size_t k = 1; k + 1 < v.size(); ++k) {
    integrate_triangle(v[0], v[k], v[k + 1], subdivisions, generator, density, acc);
  }
  out.mass = acc.mass;
  out.cost = acc.cost;
  if (acc.mass > 0.0) {
    out.centroid = acc.first / acc.mass;
  }
  return out;
}

}  // namespace multirobotcore::coverage
