#pragma once

/// @file grid_map.hpp
/// 4-connected occupancy grid for multi-agent path finding.
///
/// Discrete units: MAPF works in grid cells and integer timesteps. One
/// timestep is one move to a 4-neighbour or one wait. Conversion to SI units
/// (metres, seconds) happens only through WorldFrame and to_waypoints().
///
/// Cells are addressed as (x, y) with x the column and y the row, y = 0 at the
/// bottom, so that grid and world axes agree. Text formats (ASCII and MovingAI
/// .map) list the top row first; loaders flip the rows accordingly.

#include "multirobotcore/math/types.hpp"

#include <compare>
#include <cstddef>
#include <filesystem>
#include <istream>
#include <ostream>
#include <string>
#include <vector>

namespace multirobotcore::mapf {

struct Cell {
  int x{0};
  int y{0};

  friend auto operator<=>(const Cell&, const Cell&) = default;
  friend std::ostream& operator<<(std::ostream& os, const Cell& c) {
    return os << '(' << c.x << ", " << c.y << ')';
  }
};

class GridMap {
 public:
  GridMap() = default;
  /// All-free width x height grid.
  GridMap(int width, int height);

  /// Parses rows of characters, top row first. '.', 'G', 'S', 'E' and ' '
  /// are free; '@', 'O', 'T', 'W' and '#' are blocked. Rows must have equal
  /// length. Throws std::invalid_argument otherwise.
  [[nodiscard]] static GridMap from_ascii(const std::vector<std::string>& rows);

  /// Parses the MovingAI benchmark format (Sturtevant 2012):
  ///   type octile / height H / width W / map / H rows of W characters.
  [[nodiscard]] static GridMap parse_movingai(std::istream& in);
  [[nodiscard]] static GridMap load_movingai(const std::filesystem::path& path);

  [[nodiscard]] int width() const noexcept { return width_; }
  [[nodiscard]] int height() const noexcept { return height_; }
  [[nodiscard]] std::size_t num_cells() const noexcept { return blocked_.size(); }
  [[nodiscard]] std::size_t num_free_cells() const noexcept;

  [[nodiscard]] bool in_bounds(Cell c) const noexcept {
    return c.x >= 0 && c.y >= 0 && c.x < width_ && c.y < height_;
  }
  [[nodiscard]] bool is_free(Cell c) const noexcept {
    return in_bounds(c) && blocked_[static_cast<std::size_t>(index(c))] == 0;
  }
  void set_blocked(Cell c, bool blocked);

  /// Linear index y * width + x (no bounds check).
  [[nodiscard]] int index(Cell c) const noexcept { return c.y * width_ + c.x; }
  [[nodiscard]] Cell cell(int index) const noexcept { return {index % width_, index / width_}; }

  /// Free 4-neighbours in the fixed order +x, -x, +y, -y.
  [[nodiscard]] std::vector<Cell> neighbors(Cell c) const;

  /// Breadth-first distances (in moves) from `source` to every cell, indexed
  /// by index(); -1 for blocked or unreachable cells. Because the graph is
  /// undirected, this is also the exact distance-to-go towards `source` and
  /// serves as a perfect single-agent heuristic.
  [[nodiscard]] std::vector<int> distances_from(Cell source) const;

  /// Text form, top row first ('.' free, '@' blocked).
  [[nodiscard]] std::vector<std::string> to_ascii() const;

  friend bool operator==(const GridMap&, const GridMap&) = default;

 private:
  int width_{0};
  int height_{0};
  std::vector<char> blocked_;
};

/// Placement of a grid in the world: cell (x, y) has its centre at
/// origin + cell_size * (x + 0.5, y + 0.5) [m].
struct WorldFrame {
  Vec2 origin{Vec2::Zero()};  ///< world position of the grid's lower-left corner [m]
  double cell_size{1.0};      ///< [m]

  [[nodiscard]] Vec2 to_world(Cell c) const noexcept;
  /// Cell containing the world point p (may be out of the map's bounds).
  [[nodiscard]] Cell to_cell(const Vec2& p) const noexcept;
};

}  // namespace multirobotcore::mapf
