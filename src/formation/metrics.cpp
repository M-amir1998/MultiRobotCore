#include "multirobotcore/formation/metrics.hpp"

#include "multirobotcore/formation/shapes.hpp"
#include "multirobotcore/math/numerics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace multirobotcore::formation {
namespace {

Vec2 mean_of(std::span<const Vec2> v) {
  Vec2 m = Vec2::Zero();
  for (const Vec2& x : v) {
    m += x;
  }
  return v.empty() ? m : Vec2(m / static_cast<double>(v.size()));
}

}  // namespace

FormationError formation_error(std::span<const Vec2> positions, std::span<const Vec2> offsets,
                               Alignment alignment, const Pose2& reference) {
  if (positions.size() != offsets.size()) {
    throw std::invalid_argument("formation_error: one offset per robot required");
  }
  FormationError result;
  if (positions.empty()) {
    return result;
  }
  double theta = 0.0;
  Vec2 translation = Vec2::Zero();
  switch (alignment) {
    case Alignment::kReference:
      theta = reference.heading;
      translation = reference.position;
      break;
    case Alignment::kTranslation:
      translation = mean_of(positions) - mean_of(offsets);
      break;
    case Alignment::kRigid: {
      // 2-D Procrustes: maximise sum_i p~_i . R(theta) d~_i over theta.
      const Vec2 pm = mean_of(positions);
      const Vec2 dm = mean_of(offsets);
      double s_cos = 0.0;
      double s_sin = 0.0;
      for (std::size_t i = 0; i < positions.size(); ++i) {
        const Vec2 p = positions[i] - pm;
        const Vec2 d = offsets[i] - dm;
        s_cos += p.dot(d);
        s_sin += math::cross(d, p);
      }
      theta = std::atan2(s_sin, s_cos);
      translation = pm - math::rotate(dm, theta);
      break;
    }
  }
  double sum_sq = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    const double e = (positions[i] - (translation + math::rotate(offsets[i], theta))).norm();
    sum_sq += e * e;
    result.max_error = std::max(result.max_error, e);
  }
  result.rmse = std::sqrt(sum_sq / static_cast<double>(positions.size()));
  result.translation = translation;
  result.rotation = theta;
  return result;
}

Vec2 relative_position_error(const Vec2& p_i, const Vec2& p_j, const Vec2& d_i, const Vec2& d_j,
                             double theta) noexcept {
  return (p_j - p_i) - math::rotate(d_j - d_i, theta);
}

DistanceError distance_error(const graph::Graph& graph, std::span<const Vec2> positions,
                             std::span<const Vec2> offsets) {
  if (positions.size() != graph.num_nodes() || offsets.size() != graph.num_nodes()) {
    throw std::invalid_argument("distance_error: size mismatch");
  }
  DistanceError result;
  const auto edges = graph.edges();
  if (edges.empty()) {
    return result;
  }
  double sum_sq = 0.0;
  for (const graph::Edge& e : edges) {
    const double err = std::abs((positions[e.source] - positions[e.target]).norm() -
                                (offsets[e.source] - offsets[e.target]).norm());
    sum_sq += err * err;
    result.max = std::max(result.max, err);
  }
  result.rms = std::sqrt(sum_sq / static_cast<double>(edges.size()));
  return result;
}

double min_separation(std::span<const Vec2> positions) noexcept {
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < positions.size(); ++i) {
    for (std::size_t j = i + 1; j < positions.size(); ++j) {
      best = std::min(best, (positions[i] - positions[j]).norm());
    }
  }
  return best;
}

}  // namespace multirobotcore::formation
