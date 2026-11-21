#pragma once

/// @file density.hpp
/// Importance (density) functions phi : Q -> [0, inf) over the workspace.
///
/// phi(q) = floor + sum_k w_k exp(-||q - mu_k||^2 / (2 sigma_k^2))
///
/// A uniform density is the special case without bumps. Higher density marks
/// regions that deserve more sensing effort; centroidal Voronoi
/// configurations place more robots there.

#include "multirobotcore/math/types.hpp"

#include <vector>

namespace multirobotcore::coverage {

/// Isotropic Gaussian bump.
struct GaussianBump {
  Vec2 mean{Vec2::Zero()};  ///< [m]
  double sigma{1.0};        ///< standard deviation [m]
  double weight{1.0};       ///< peak height above the floor
};

class DensityFunction {
 public:
  /// Constant density phi(q) = value (> 0).
  [[nodiscard]] static DensityFunction uniform(double value = 1.0);

  /// Gaussian mixture on top of a constant floor (>= 0). Throws
  /// std::invalid_argument for non-positive sigma, negative weights or a
  /// density that is identically zero.
  [[nodiscard]] static DensityFunction gaussian_mixture(std::vector<GaussianBump> bumps,
                                                        double floor);

  [[nodiscard]] double operator()(const Vec2& q) const noexcept;

  [[nodiscard]] bool is_uniform() const noexcept { return bumps_.empty(); }
  [[nodiscard]] double floor() const noexcept { return floor_; }
  [[nodiscard]] const std::vector<GaussianBump>& bumps() const noexcept { return bumps_; }

 private:
  DensityFunction(std::vector<GaussianBump> bumps, double floor);

  std::vector<GaussianBump> bumps_;
  double floor_{1.0};
};

}  // namespace multirobotcore::coverage
