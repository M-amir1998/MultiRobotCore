#include "multirobotcore/coverage/coverage_control.hpp"

#include "multirobotcore/math/numerics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace multirobotcore::coverage {
namespace {

void require_positions(std::span<const Vec2> positions, const char* what) {
  if (positions.empty()) {
    throw std::invalid_argument(std::string(what) + ": at least one robot required");
  }
  for (const Vec2& p : positions) {
    if (!math::is_finite(p)) {
      throw std::invalid_argument(std::string(what) + ": non-finite position");
    }
  }
}

}  // namespace

CoverageController::CoverageController(math::ConvexPolygon domain, DensityFunction density,
                                       CoverageConfig config)
    : domain_(std::move(domain)), density_(std::move(density)), config_(config) {
  if (domain_.empty()) {
    throw std::invalid_argument("CoverageController: domain must have positive area");
  }
  if (!(config_.gain > 0.0) || !(config_.max_speed > 0.0) || config_.quadrature_subdivisions < 1) {
    throw std::invalid_argument("CoverageController: invalid configuration");
  }
  if (config_.communication_radius && !(*config_.communication_radius > 0.0)) {
    throw std::invalid_argument("CoverageController: communication radius must be positive");
  }
}

CoverageState CoverageController::build_state(std::span<const Vec2> positions, bool limited) const {
  require_positions(positions, "CoverageController");
  const std::size_t n = positions.size();
  CoverageState state;
  if (limited) {
    const double r = *config_.communication_radius;
    const double half = 0.5 * r;
    state.cells.reserve(n);
    state.certified_exact.assign(n, false);
    std::vector<std::size_t> in_range;
    for (std::size_t i = 0; i < n; ++i) {
      in_range.clear();
      for (std::size_t j = 0; j < n; ++j) {
        if (j != i && (positions[j] - positions[i]).norm() <= r) {
          in_range.push_back(j);
          if (j > i) {
            ++state.communicating_pairs;
          }
        }
      }
      // Local Voronoi cell from the robots in range (an empty candidate list
      // would mean "all generators", so an isolated robot starts from the
      // whole domain). Every bisector that can cut B(p_i, R/2) belongs to a
      // robot within R, so the r-limited cell (local cell intersected with
      // B(p_i, R/2)) is exact.
      VoronoiCell local = in_range.empty() ? VoronoiCell{i, domain_, {}}
                                           : voronoi_cell(i, positions, domain_, in_range);
      state.certified_exact[i] = max_vertex_distance(local.polygon, positions[i]) <= half;
      if (!state.certified_exact[i] && !local.polygon.empty()) {
        local.polygon = clip_to_disc(local.polygon, positions[i], half);
      }
      state.cells.push_back(std::move(local));
    }
    state.cost = coverage_cost(positions, domain_, density_, config_.quadrature_subdivisions);
  } else {
    state.cells = bounded_voronoi(positions, domain_);
  }
  state.integrals.reserve(n);
  double exact_cost = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    state.integrals.push_back(integrate_cell(state.cells[i].polygon, positions[i], density_,
                                             config_.quadrature_subdivisions));
    exact_cost += state.integrals.back().cost;
    state.max_centroid_error =
        std::max(state.max_centroid_error, (positions[i] - state.integrals.back().centroid).norm());
  }
  if (!limited) {
    state.cost = exact_cost;
  }
  return state;
}

CoverageState CoverageController::evaluate(std::span<const Vec2> positions) const {
  return build_state(positions, false);
}

CoverageState CoverageController::controller_state(std::span<const Vec2> positions) const {
  return build_state(positions, config_.communication_radius.has_value());
}

std::vector<Vec2> CoverageController::velocities(std::span<const Vec2> positions,
                                                 CoverageState* state) const {
  CoverageState s = controller_state(positions);
  std::vector<Vec2> u;
  u.reserve(positions.size());
  for (std::size_t i = 0; i < positions.size(); ++i) {
    u.push_back(math::clamp_norm(-config_.gain * (positions[i] - s.integrals[i].centroid),
                                 config_.max_speed));
  }
  if (state != nullptr) {
    *state = std::move(s);
  }
  return u;
}

std::vector<Vec2> CoverageController::lloyd_step(std::span<const Vec2> positions,
                                                 CoverageState* state) const {
  CoverageState s = controller_state(positions);
  std::vector<Vec2> next;
  next.reserve(positions.size());
  for (const CellIntegrals& c : s.integrals) {
    next.push_back(c.centroid);
  }
  if (state != nullptr) {
    *state = std::move(s);
  }
  return next;
}

double coverage_cost(std::span<const Vec2> positions, const math::ConvexPolygon& domain,
                     const DensityFunction& density, int subdivisions) {
  require_positions(positions, "coverage_cost");
  double cost = 0.0;
  for (const VoronoiCell& cell : bounded_voronoi(positions, domain)) {
    cost += integrate_cell(cell.polygon, positions[cell.generator], density, subdivisions).cost;
  }
  return cost;
}

CoverageMetrics evaluate_metrics(std::span<const Vec2> positions, const math::ConvexPolygon& domain,
                                 const DensityFunction& density, int samples_per_axis,
                                 int subdivisions) {
  require_positions(positions, "evaluate_metrics");
  if (samples_per_axis < 2) {
    throw std::invalid_argument("evaluate_metrics: samples_per_axis must be at least 2");
  }
  CoverageMetrics m;
  const CoverageController controller(domain, density, CoverageConfig{1.0, 1.0, subdivisions, {}});
  const CoverageState state = controller.evaluate(positions);
  m.cost = state.cost;
  m.max_centroid_error = state.max_centroid_error;

  const math::AABB box = domain.bounding_box();
  const double dx = box.width() / samples_per_axis;
  const double dy = box.height() / samples_per_axis;
  double weighted_distance = 0.0;
  double total_weight = 0.0;
  for (int a = 0; a < samples_per_axis; ++a) {
    for (int b = 0; b < samples_per_axis; ++b) {
      const Vec2 q(box.min.x() + (a + 0.5) * dx, box.min.y() + (b + 0.5) * dy);
      if (!domain.contains(q)) {
        continue;
      }
      double nearest = std::numeric_limits<double>::infinity();
      for (const Vec2& p : positions) {
        nearest = std::min(nearest, (q - p).norm());
      }
      const double w = density(q);
      weighted_distance += w * nearest;
      total_weight += w;
      m.max_uncovered_distance = std::max(m.max_uncovered_distance, nearest);
    }
  }
  m.mean_distance = total_weight > 0.0 ? weighted_distance / total_weight : 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    for (std::size_t j = i + 1; j < positions.size(); ++j) {
      m.min_separation = std::min(m.min_separation, (positions[i] - positions[j]).norm());
    }
  }
  return m;
}

}  // namespace multirobotcore::coverage
