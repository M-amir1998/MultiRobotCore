#include "multirobotcore/exploration/sensor.hpp"

#include "multirobotcore/math/numerics.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace multirobotcore::exploration {

std::vector<GridCell> simulate_scan(const OccupancyGrid& truth, const Vec2& origin,
                                    const RangeSensorConfig& config) {
  if (!(config.range > 0.0) || config.num_beams < 1) {
    throw std::invalid_argument("simulate_scan: range and beam count must be positive");
  }
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const Vec2 start = (origin - truth.origin()) / truth.resolution();  // grid units
  const double range_cells = config.range / truth.resolution();
  std::vector<char> seen(truth.size(), 0);

  for (int b = 0; b < config.num_beams; ++b) {
    // Beams are offset by half an angular step so that no beam runs exactly
    // along a grid diagonal, where the traversal tie-break would depend on
    // the last bits of cos/sin and differ between math libraries.
    const double angle = math::kTwoPi * (static_cast<double>(b) + 0.5) / config.num_beams;
    const double dx = std::cos(angle);
    const double dy = std::sin(angle);
    int x = static_cast<int>(std::floor(start.x()));
    int y = static_cast<int>(std::floor(start.y()));
    const int step_x = dx > 0.0 ? 1 : -1;
    const int step_y = dy > 0.0 ? 1 : -1;
    const double delta_x = std::abs(dx) > 1e-12 ? 1.0 / std::abs(dx) : kInf;
    const double delta_y = std::abs(dy) > 1e-12 ? 1.0 / std::abs(dy) : kInf;
    double t_max_x = kInf;
    double t_max_y = kInf;
    if (std::abs(dx) > 1e-12) {
      t_max_x = dx > 0.0 ? (std::floor(start.x()) + 1.0 - start.x()) / dx
                         : (start.x() - std::floor(start.x())) / -dx;
    }
    if (std::abs(dy) > 1e-12) {
      t_max_y = dy > 0.0 ? (std::floor(start.y()) + 1.0 - start.y()) / dy
                         : (start.y() - std::floor(start.y())) / -dy;
    }
    double t = 0.0;  // parameter (distance in cells) at which the current cell is entered
    while (t <= range_cells) {
      const GridCell c{x, y};
      if (!truth.in_bounds(c)) {
        break;
      }
      seen[truth.index(c)] = 1;
      if (truth.at(c) == Occupancy::kOccupied) {
        break;
      }
      if (t_max_x < t_max_y) {
        t = t_max_x;
        t_max_x += delta_x;
        x += step_x;
      } else {
        t = t_max_y;
        t_max_y += delta_y;
        y += step_y;
      }
    }
  }
  std::vector<GridCell> observed;
  for (std::size_t i = 0; i < seen.size(); ++i) {
    if (seen[i] != 0) {
      observed.push_back(truth.cell(i));
    }
  }
  return observed;
}

std::size_t integrate_scan(OccupancyGrid& belief, const OccupancyGrid& truth,
                           std::span<const GridCell> observed) {
  if (belief.width() != truth.width() || belief.height() != truth.height()) {
    throw std::invalid_argument("integrate_scan: belief and truth sizes differ");
  }
  std::size_t newly_known = 0;
  for (const GridCell& c : observed) {
    if (belief.at(c) == Occupancy::kUnknown) {
      ++newly_known;
    }
    belief.set(c, truth.at(c));
  }
  return newly_known;
}

}  // namespace multirobotcore::exploration
