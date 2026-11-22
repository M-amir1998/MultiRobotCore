#include "multirobotcore/coverage/density.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace multirobotcore::coverage {

DensityFunction::DensityFunction(std::vector<GaussianBump> bumps, double floor)
    : bumps_(std::move(bumps)), floor_(floor) {}

DensityFunction DensityFunction::uniform(double value) {
  if (!(value > 0.0) || !std::isfinite(value)) {
    throw std::invalid_argument("DensityFunction::uniform: value must be finite and positive");
  }
  return {{}, value};
}

DensityFunction DensityFunction::gaussian_mixture(std::vector<GaussianBump> bumps, double floor) {
  if (!(floor >= 0.0) || !std::isfinite(floor)) {
    throw std::invalid_argument("DensityFunction: floor must be finite and non-negative");
  }
  bool positive_somewhere = floor > 0.0;
  for (const GaussianBump& b : bumps) {
    if (!(b.sigma > 0.0) || !(b.weight >= 0.0) || !std::isfinite(b.sigma) ||
        !std::isfinite(b.weight) || !std::isfinite(b.mean.x()) || !std::isfinite(b.mean.y())) {
      throw std::invalid_argument("DensityFunction: invalid Gaussian bump");
    }
    positive_somewhere = positive_somewhere || b.weight > 0.0;
  }
  if (!positive_somewhere) {
    throw std::invalid_argument("DensityFunction: density is identically zero");
  }
  return {std::move(bumps), floor};
}

double DensityFunction::operator()(const Vec2& q) const noexcept {
  double value = floor_;
  for (const GaussianBump& b : bumps_) {
    value += b.weight * std::exp(-(q - b.mean).squaredNorm() / (2.0 * b.sigma * b.sigma));
  }
  return value;
}

}  // namespace multirobotcore::coverage
