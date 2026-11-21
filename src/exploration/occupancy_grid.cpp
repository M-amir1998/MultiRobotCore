#include "multirobotcore/exploration/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace multirobotcore::exploration {

OccupancyGrid::OccupancyGrid(int width, int height, double resolution, Vec2 origin, Occupancy fill)
    : width_(width), height_(height), resolution_(resolution), origin_(std::move(origin)) {
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("OccupancyGrid: dimensions must be positive");
  }
  if (!(resolution > 0.0) || !std::isfinite(resolution)) {
    throw std::invalid_argument("OccupancyGrid: resolution must be finite and positive");
  }
  cells_.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), fill);
}

OccupancyGrid OccupancyGrid::from_ascii(const std::vector<std::string>& rows, double resolution,
                                        Vec2 origin) {
  if (rows.empty() || rows.front().empty()) {
    throw std::invalid_argument("OccupancyGrid::from_ascii: empty map");
  }
  const auto width = static_cast<int>(rows.front().size());
  const auto height = static_cast<int>(rows.size());
  OccupancyGrid grid(width, height, resolution, std::move(origin), Occupancy::kFree);
  for (int r = 0; r < height; ++r) {
    const std::string& row = rows[static_cast<std::size_t>(r)];
    if (static_cast<int>(row.size()) != width) {
      throw std::invalid_argument("OccupancyGrid::from_ascii: ragged rows");
    }
    for (int x = 0; x < width; ++x) {
      const char ch = row[static_cast<std::size_t>(x)];
      const GridCell c{x, height - 1 - r};
      if (ch == '#') {
        grid.set(c, Occupancy::kOccupied);
      } else if (ch != '.') {
        throw std::invalid_argument(
            std::string("OccupancyGrid::from_ascii: unexpected character '") + ch + "'");
      }
    }
  }
  return grid;
}

OccupancyGrid OccupancyGrid::load_ascii(const std::filesystem::path& path, double resolution,
                                        Vec2 origin) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("OccupancyGrid::load_ascii: cannot open " + path.string());
  }
  std::vector<std::string> rows;
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty() || line.front() == ';') {
      continue;
    }
    rows.push_back(line);
  }
  return from_ascii(rows, resolution, std::move(origin));
}

void OccupancyGrid::set(const GridCell& c, Occupancy value) {
  if (!in_bounds(c)) {
    throw std::out_of_range("OccupancyGrid::set: cell out of bounds");
  }
  cells_[index(c)] = value;
}

GridCell OccupancyGrid::world_to_cell(const Vec2& p) const noexcept {
  const Vec2 local = (p - origin_) / resolution_;
  return GridCell{static_cast<int>(std::floor(local.x())), static_cast<int>(std::floor(local.y()))};
}

Vec2 OccupancyGrid::cell_center(const GridCell& c) const noexcept {
  return origin_ + resolution_ * Vec2(c.x + 0.5, c.y + 0.5);
}

std::size_t OccupancyGrid::count(Occupancy value) const noexcept {
  return static_cast<std::size_t>(std::count(cells_.begin(), cells_.end(), value));
}

}  // namespace multirobotcore::exploration
