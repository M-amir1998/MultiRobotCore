#pragma once

/// @file occupancy_grid.hpp
/// Ternary occupancy grid (unknown / free / occupied) used both as the
/// ground-truth environment and as the robots' shared belief map.
///
/// Cell (x, y) covers [origin + (x, y) * resolution, origin + (x + 1, y + 1) *
/// resolution). ASCII maps list the top row first, so row r of the text is
/// y = height - 1 - r and the map is not mirrored when plotted.

#include "multirobotcore/math/types.hpp"

#include <compare>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace multirobotcore::exploration {

enum class Occupancy : std::uint8_t { kUnknown = 0, kFree = 1, kOccupied = 2 };

/// Integer grid coordinates.
struct GridCell {
  int x{0};
  int y{0};

  friend auto operator<=>(const GridCell&, const GridCell&) = default;
};

class OccupancyGrid {
 public:
  OccupancyGrid() = default;
  OccupancyGrid(int width, int height, double resolution = 1.0, Vec2 origin = Vec2::Zero(),
                Occupancy fill = Occupancy::kUnknown);

  /// Parses rows of '#' (occupied) and '.' (free). Throws on ragged rows or
  /// other characters.
  [[nodiscard]] static OccupancyGrid from_ascii(const std::vector<std::string>& rows,
                                                double resolution = 1.0,
                                                Vec2 origin = Vec2::Zero());

  /// Loads an ASCII map file (blank lines and lines starting with ';' are
  /// ignored).
  [[nodiscard]] static OccupancyGrid load_ascii(const std::filesystem::path& path,
                                                double resolution = 1.0,
                                                Vec2 origin = Vec2::Zero());

  [[nodiscard]] int width() const noexcept { return width_; }
  [[nodiscard]] int height() const noexcept { return height_; }
  [[nodiscard]] double resolution() const noexcept { return resolution_; }
  [[nodiscard]] const Vec2& origin() const noexcept { return origin_; }
  [[nodiscard]] std::size_t size() const noexcept { return cells_.size(); }

  [[nodiscard]] bool in_bounds(const GridCell& c) const noexcept {
    return c.x >= 0 && c.y >= 0 && c.x < width_ && c.y < height_;
  }
  [[nodiscard]] std::size_t index(const GridCell& c) const noexcept {
    return static_cast<std::size_t>(c.y) * static_cast<std::size_t>(width_) +
           static_cast<std::size_t>(c.x);
  }
  [[nodiscard]] GridCell cell(std::size_t index) const noexcept {
    return GridCell{static_cast<int>(index % static_cast<std::size_t>(width_)),
                    static_cast<int>(index / static_cast<std::size_t>(width_))};
  }

  /// Occupancy of an in-bounds cell (out-of-bounds cells read as occupied).
  [[nodiscard]] Occupancy at(const GridCell& c) const noexcept {
    return in_bounds(c) ? cells_[index(c)] : Occupancy::kOccupied;
  }
  void set(const GridCell& c, Occupancy value);

  [[nodiscard]] GridCell world_to_cell(const Vec2& p) const noexcept;
  [[nodiscard]] Vec2 cell_center(const GridCell& c) const noexcept;

  [[nodiscard]] std::size_t count(Occupancy value) const noexcept;
  [[nodiscard]] const std::vector<Occupancy>& data() const noexcept { return cells_; }

 private:
  int width_{0};
  int height_{0};
  double resolution_{1.0};
  Vec2 origin_{Vec2::Zero()};
  std::vector<Occupancy> cells_;
};

}  // namespace multirobotcore::exploration
