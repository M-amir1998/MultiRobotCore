#include "multirobotcore/mapf/grid_map.hpp"

#include <cmath>
#include <deque>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace multirobotcore::mapf {
namespace {

bool is_blocked_char(char c) {
  return c == '@' || c == 'O' || c == 'T' || c == 'W' || c == '#';
}

bool is_free_char(char c) {
  return c == '.' || c == 'G' || c == 'S' || c == 'E' || c == ' ';
}

std::string strip_cr(std::string line) {
  while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
    line.pop_back();
  }
  return line;
}

}  // namespace

GridMap::GridMap(int width, int height) : width_(width), height_(height) {
  if (width < 0 || height < 0) {
    throw std::invalid_argument("GridMap: dimensions must be non-negative");
  }
  blocked_.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);
}

GridMap GridMap::from_ascii(const std::vector<std::string>& rows) {
  if (rows.empty()) {
    return GridMap(0, 0);
  }
  const auto height = static_cast<int>(rows.size());
  const auto width = static_cast<int>(rows.front().size());
  GridMap map(width, height);
  for (int r = 0; r < height; ++r) {
    const std::string& row = rows[static_cast<std::size_t>(r)];
    if (static_cast<int>(row.size()) != width) {
      throw std::invalid_argument("GridMap::from_ascii: rows must have equal length");
    }
    const int y = height - 1 - r;  // first text row is the top of the map
    for (int x = 0; x < width; ++x) {
      const char c = row[static_cast<std::size_t>(x)];
      if (is_blocked_char(c)) {
        map.set_blocked({x, y}, true);
      } else if (!is_free_char(c)) {
        throw std::invalid_argument(std::string("GridMap::from_ascii: unknown map character '") +
                                    c + "'");
      }
    }
  }
  return map;
}

GridMap GridMap::parse_movingai(std::istream& in) {
  std::string token;
  int height = -1;
  int width = -1;
  while (in >> token) {
    if (token == "type") {
      in >> token;  // octile; movement model is ours (4-connected)
    } else if (token == "height") {
      in >> height;
    } else if (token == "width") {
      in >> width;
    } else if (token == "map") {
      break;
    } else {
      throw std::invalid_argument("GridMap::parse_movingai: unexpected header token '" + token +
                                  "'");
    }
  }
  if (height <= 0 || width <= 0) {
    throw std::invalid_argument("GridMap::parse_movingai: missing or invalid width/height");
  }
  std::string line;
  std::getline(in, line);  // rest of the "map" line
  std::vector<std::string> rows;
  while (static_cast<int>(rows.size()) < height && std::getline(in, line)) {
    line = strip_cr(line);
    if (line.empty()) {
      continue;
    }
    rows.push_back(line);
  }
  if (static_cast<int>(rows.size()) != height) {
    throw std::invalid_argument("GridMap::parse_movingai: expected " + std::to_string(height) +
                                " map rows");
  }
  for (const auto& row : rows) {
    if (static_cast<int>(row.size()) != width) {
      throw std::invalid_argument("GridMap::parse_movingai: row width does not match header");
    }
  }
  return from_ascii(rows);
}

GridMap GridMap::load_movingai(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("GridMap::load_movingai: cannot open " + path.string());
  }
  return parse_movingai(in);
}

std::size_t GridMap::num_free_cells() const noexcept {
  std::size_t count = 0;
  for (const char b : blocked_) {
    count += b == 0 ? 1U : 0U;
  }
  return count;
}

void GridMap::set_blocked(Cell c, bool blocked) {
  if (!in_bounds(c)) {
    throw std::out_of_range("GridMap::set_blocked: cell out of bounds");
  }
  blocked_[static_cast<std::size_t>(index(c))] = blocked ? 1 : 0;
}

std::vector<Cell> GridMap::neighbors(Cell c) const {
  std::vector<Cell> out;
  out.reserve(4);
  for (const Cell n :
       {Cell{c.x + 1, c.y}, Cell{c.x - 1, c.y}, Cell{c.x, c.y + 1}, Cell{c.x, c.y - 1}}) {
    if (is_free(n)) {
      out.push_back(n);
    }
  }
  return out;
}

std::vector<int> GridMap::distances_from(Cell source) const {
  std::vector<int> dist(num_cells(), -1);
  if (!is_free(source)) {
    return dist;
  }
  std::deque<Cell> queue{source};
  dist[static_cast<std::size_t>(index(source))] = 0;
  while (!queue.empty()) {
    const Cell c = queue.front();
    queue.pop_front();
    const int d = dist[static_cast<std::size_t>(index(c))];
    for (const Cell n : neighbors(c)) {
      int& dn = dist[static_cast<std::size_t>(index(n))];
      if (dn < 0) {
        dn = d + 1;
        queue.push_back(n);
      }
    }
  }
  return dist;
}

std::vector<std::string> GridMap::to_ascii() const {
  std::vector<std::string> rows;
  rows.reserve(static_cast<std::size_t>(height_));
  for (int y = height_ - 1; y >= 0; --y) {
    std::string row;
    row.reserve(static_cast<std::size_t>(width_));
    for (int x = 0; x < width_; ++x) {
      row.push_back(is_free({x, y}) ? '.' : '@');
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

Vec2 WorldFrame::to_world(Cell c) const noexcept {
  return origin + cell_size * Vec2(static_cast<double>(c.x) + 0.5, static_cast<double>(c.y) + 0.5);
}

Cell WorldFrame::to_cell(const Vec2& p) const noexcept {
  const Vec2 q = (p - origin) / cell_size;
  return {static_cast<int>(std::floor(q.x())), static_cast<int>(std::floor(q.y()))};
}

}  // namespace multirobotcore::mapf
