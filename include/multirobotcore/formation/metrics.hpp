#pragma once

/// @file metrics.hpp
/// Formation error measures.

#include "multirobotcore/graph/graph.hpp"
#include "multirobotcore/math/types.hpp"

#include <span>

namespace multirobotcore::formation {

/// How the desired shape is aligned with the robots before measuring error.
enum class Alignment {
  kReference,    ///< use the given reference pose (absolute slot error)
  kTranslation,  ///< best translation (shape and heading must match)
  kRigid,        ///< best translation and rotation (2-D Kabsch/Procrustes)
};

struct FormationError {
  double rmse{0.0};       ///< sqrt(mean_i ||e_i||^2) [m]
  double max_error{0.0};  ///< max_i ||e_i|| [m]
  Vec2 translation{Vec2::Zero()};
  double rotation{0.0};  ///< [rad]
};

/// Slot errors e_i = p_i - (t + R(theta) d_i) after the requested alignment.
/// `reference` is used only with Alignment::kReference.
[[nodiscard]] FormationError formation_error(std::span<const Vec2> positions,
                                             std::span<const Vec2> offsets, Alignment alignment,
                                             const Pose2& reference = {});

/// Relative-position error of one robot pair:
/// (p_j - p_i) - R(theta) (d_j - d_i).
[[nodiscard]] Vec2 relative_position_error(const Vec2& p_i, const Vec2& p_j, const Vec2& d_i,
                                           const Vec2& d_j, double theta) noexcept;

struct DistanceError {
  double rms{0.0};  ///< [m]
  double max{0.0};  ///< [m]
};

/// | ||p_i - p_j|| - ||d_i - d_j|| | over the edges of `graph`.
[[nodiscard]] DistanceError distance_error(const graph::Graph& graph,
                                           std::span<const Vec2> positions,
                                           std::span<const Vec2> offsets);

/// Minimum pairwise centre distance [m] (infinity for fewer than two robots).
[[nodiscard]] double min_separation(std::span<const Vec2> positions) noexcept;

}  // namespace multirobotcore::formation
