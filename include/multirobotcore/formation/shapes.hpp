#pragma once

/// @file shapes.hpp
/// Desired formation geometry.
///
/// A formation is a list of offsets d_i expressed in the formation frame,
/// whose +x axis points in the direction of travel. A reference pose
/// (p_ref, theta_ref) maps offsets to world slots p_i* = p_ref + R(theta_ref) d_i.
/// Shapes produced here are centred (sum_i d_i = 0) so that the reference is
/// the formation centroid; relative_to_slot() re-expresses them relative to a
/// leader slot for leader-follower control.

#include "multirobotcore/graph/graph.hpp"
#include "multirobotcore/math/types.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace multirobotcore::formation {

enum class Shape : std::uint8_t {
  kLine,      ///< abreast, perpendicular to the direction of travel
  kColumn,    ///< single file along the direction of travel
  kTriangle,  ///< rows of 1, 2, 3, ... robots with the apex in front
  kSquare,    ///< ceil(sqrt(n)) x ceil(sqrt(n)) block
  kCircle,    ///< regular polygon with neighbour spacing `spacing`
  kV,         ///< apex in front, two trailing arms
};

[[nodiscard]] std::string_view to_string(Shape shape) noexcept;
[[nodiscard]] std::optional<Shape> shape_from_string(std::string_view name) noexcept;

/// Offsets of `n` robots in `shape` with nearest-neighbour spacing `spacing`
/// [m], centred on the formation centroid.
[[nodiscard]] std::vector<Vec2> make_shape(Shape shape, std::size_t n, double spacing);

/// V formation with a configurable half-angle between each arm and the
/// direction of travel [rad].
[[nodiscard]] std::vector<Vec2> make_v_shape(std::size_t n, double spacing, double half_angle);

/// Offsets translated so that their centroid is the origin.
[[nodiscard]] std::vector<Vec2> centered(std::vector<Vec2> offsets);

/// Offsets re-expressed relative to `slot` (that slot becomes the origin).
[[nodiscard]] std::vector<Vec2> relative_to_slot(std::span<const Vec2> offsets, std::size_t slot);

/// World position p_ref + R(theta_ref) d.
[[nodiscard]] Vec2 to_world(const Pose2& reference, const Vec2& offset) noexcept;
[[nodiscard]] std::vector<Vec2> to_world(const Pose2& reference, std::span<const Vec2> offsets);

/// Minimally rigid formation graph (2n - 3 edges) by Henneberg type-I vertex
/// additions: robot k >= 2 is connected to the closest earlier robot and to
/// the closest earlier robot that is not collinear with those two. The
/// framework is then infinitesimally rigid whenever such anchors exist;
/// collinear shapes (line, column) remain flexible and cannot be stabilised
/// by distance constraints alone.
[[nodiscard]] graph::Graph henneberg_graph(std::span<const Vec2> offsets);

/// Desired inter-robot distances ||d_i - d_j|| for every edge of `graph`,
/// ordered as graph.edges().
[[nodiscard]] std::vector<double> desired_distances(const graph::Graph& graph,
                                                    std::span<const Vec2> offsets);

}  // namespace multirobotcore::formation
