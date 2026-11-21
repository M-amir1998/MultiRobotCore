#pragma once

/// @file json.hpp
/// JSON conversion for configuration files and deterministic result files.
///
/// Result files are consumed by the Python renderers and regression tests.
/// Keys keep insertion order (nlohmann::ordered_json) and floating-point
/// values are rounded to a fixed number of decimals, so that identical runs
/// produce byte-identical files and diffs stay small.

#include "multirobotcore/communication/network.hpp"
#include "multirobotcore/graph/graph.hpp"
#include "multirobotcore/math/geometry.hpp"
#include "multirobotcore/math/obstacles.hpp"
#include "multirobotcore/math/types.hpp"
#include "multirobotcore/simulation/simulator.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <span>
#include <vector>

namespace multirobotcore::io {

using Json = nlohmann::ordered_json;

/// Default number of decimals for positions [m] and times [s] in result files.
inline constexpr int kDefaultDecimals = 4;

[[nodiscard]] double round_to(double value, int decimals = kDefaultDecimals);

[[nodiscard]] Json to_json(const Vec2& v, int decimals = kDefaultDecimals);
[[nodiscard]] Json to_json(const Pose2& pose, int decimals = kDefaultDecimals);
[[nodiscard]] Json to_json(std::span<const Vec2> points, int decimals = kDefaultDecimals);
[[nodiscard]] Json to_json(const math::Circle& circle, int decimals = kDefaultDecimals);
[[nodiscard]] Json to_json(const math::AABB& box, int decimals = kDefaultDecimals);
[[nodiscard]] Json to_json(const math::ObstacleSet& obstacles, int decimals = kDefaultDecimals);

/// {"directed": bool, "num_nodes": n, "edges": [[source, target, weight], ...]}
[[nodiscard]] Json to_json(const graph::Graph& graph);

[[nodiscard]] Json to_json(const communication::NetworkStats& stats);
[[nodiscard]] Json to_json(const simulation::SimulationMetrics& metrics);

/// Compact column-oriented trajectory:
/// {"t": [...], "positions": [[[x, y], ...], ...], "headings": [[...], ...],
///  "status": [["active", ...], ...]}
[[nodiscard]] Json frames_to_json(const std::vector<simulation::Frame>& frames,
                                  int decimals = kDefaultDecimals);

[[nodiscard]] Vec2 vec2_from_json(const Json& value);
[[nodiscard]] std::vector<Vec2> vec2_list_from_json(const Json& value);

/// Reads a JSON file (comments are not allowed). Throws std::runtime_error.
[[nodiscard]] Json read_json_file(const std::filesystem::path& path);

/// Writes a JSON file, creating parent directories. indent < 0 writes compact
/// output. Throws std::runtime_error on failure.
void write_json_file(const std::filesystem::path& path, const Json& value, int indent = -1);

}  // namespace multirobotcore::io
