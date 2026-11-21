#pragma once

/// @file coverage_control.hpp
/// Voronoi-based coverage control (Cortes, Martinez, Karatas & Bullo 2004).
///
/// Locational cost of robot positions P over a convex domain Q:
///   H(P) = sum_i int_{V_i(P)} ||q - p_i||^2 phi(q) dq
/// Its gradient is dH/dp_i = 2 M_i (p_i - C_i), with M_i and C_i the mass and
/// centroid of the Voronoi cell V_i. Critical points are centroidal Voronoi
/// configurations (p_i = C_i).
///
///  - Continuous gradient flow:  p_i' = -k (p_i - C_i), saturated to v_max.
///  - Lloyd iteration:           p_i <- C_i (Lloyd 1982), monotone in H.
///  - Limited-range (distributed) variant (Cortes, Martinez & Bullo 2005):
///    robot i uses its r-limited cell, the Voronoi cell intersected with the
///    disc B(p_i, R/2), which it computes exactly from the robots within
///    communication radius R, and follows the gradient of the limited-range
///    locational cost. The r-limited cell equals the Voronoi cell whenever the
///    cell lies inside B(p_i, R/2), a condition the robot checks locally
///    (CoverageState::certified_exact). The disc is represented by its
///    inscribed 32-gon (clip_to_disc).

#include "multirobotcore/coverage/density.hpp"
#include "multirobotcore/coverage/quadrature.hpp"
#include "multirobotcore/coverage/voronoi.hpp"
#include "multirobotcore/math/convex_polygon.hpp"
#include "multirobotcore/math/types.hpp"

#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <vector>

namespace multirobotcore::coverage {

struct CoverageConfig {
  double gain{1.0};       ///< k [1/s]
  double max_speed{1.0};  ///< [m/s]
  int quadrature_subdivisions{4};
  /// Communication radius for the limited-range variant [m]; nullopt uses
  /// the exact (centralised) partition.
  std::optional<double> communication_radius;
};

/// Partition and integrals for one configuration.
struct CoverageState {
  std::vector<VoronoiCell> cells;
  std::vector<CellIntegrals> integrals;
  double cost{0.0};                ///< H(P) using the exact partition
  double max_centroid_error{0.0};  ///< max_i ||p_i - C_i|| [m]
  /// Limited-range variant only: per robot, true when the Voronoi cell lies
  /// inside B(p_i, R/2), so that the r-limited cell is the exact cell.
  std::vector<bool> certified_exact;
  /// Limited-range variant only: number of robot pairs within range (each
  /// pair exchanges positions once per control step).
  std::size_t communicating_pairs{0};
};

class CoverageController {
 public:
  CoverageController(math::ConvexPolygon domain, DensityFunction density, CoverageConfig config);

  /// Exact partition, integrals and cost (centralised evaluation).
  [[nodiscard]] CoverageState evaluate(std::span<const Vec2> positions) const;

  /// Partition used by the controller: exact, or r-limited cells when a
  /// communication radius is configured (neighbour lists then refer to the
  /// local Voronoi cell before disc clipping). The reported cost is always the
  /// exact H(P).
  [[nodiscard]] CoverageState controller_state(std::span<const Vec2> positions) const;

  /// Gradient-flow velocity commands u_i = sat(-k (p_i - C_i)).
  [[nodiscard]] std::vector<Vec2> velocities(std::span<const Vec2> positions,
                                             CoverageState* state = nullptr) const;

  /// One Lloyd step: every robot jumps to its cell centroid.
  [[nodiscard]] std::vector<Vec2> lloyd_step(std::span<const Vec2> positions,
                                             CoverageState* state = nullptr) const;

  [[nodiscard]] const math::ConvexPolygon& domain() const noexcept { return domain_; }
  [[nodiscard]] const DensityFunction& density() const noexcept { return density_; }
  [[nodiscard]] const CoverageConfig& config() const noexcept { return config_; }

 private:
  [[nodiscard]] CoverageState build_state(std::span<const Vec2> positions, bool limited) const;

  math::ConvexPolygon domain_;
  DensityFunction density_;
  CoverageConfig config_;
};

/// Exact locational cost H(P).
[[nodiscard]] double coverage_cost(std::span<const Vec2> positions,
                                   const math::ConvexPolygon& domain,
                                   const DensityFunction& density, int subdivisions = 4);

struct CoverageMetrics {
  double cost{0.0};                    ///< H(P) [density * m^4]
  double mean_distance{0.0};           ///< density-weighted mean distance to the nearest robot [m]
  double max_uncovered_distance{0.0};  ///< max over samples of the nearest-robot distance [m]
  double max_centroid_error{0.0};      ///< [m]
  double min_separation{std::numeric_limits<double>::infinity()};  ///< centre distance [m]
};

/// Metrics evaluated on a regular sample grid with `samples_per_axis` points
/// per bounding-box axis (points outside the domain are discarded).
[[nodiscard]] CoverageMetrics evaluate_metrics(std::span<const Vec2> positions,
                                               const math::ConvexPolygon& domain,
                                               const DensityFunction& density,
                                               int samples_per_axis = 120, int subdivisions = 4);

}  // namespace multirobotcore::coverage
