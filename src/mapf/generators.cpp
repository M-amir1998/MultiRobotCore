#include "multirobotcore/mapf/generators.hpp"

#include "deterministic_random.hpp"

#include <algorithm>
#include <deque>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace multirobotcore::mapf {

GridMap make_open_map(int width, int height) {
  return {width, height};
}

GridMap make_random_map(int width, int height, double density, std::mt19937_64& rng) {
  if (density < 0.0 || density > 1.0) {
    throw std::invalid_argument("make_random_map: density must be in [0, 1]");
  }
  GridMap map(width, height);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (detail::uniform_unit(rng) < density) {
        map.set_blocked({x, y}, true);
      }
    }
  }
  return map;
}

GridMap make_maze(int width, int height, double loop_fraction, std::mt19937_64& rng) {
  const int w = std::max(width, 3) | 1;
  const int h = std::max(height, 3) | 1;
  GridMap map(w, h);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      map.set_blocked({x, y}, true);
    }
  }
  // Iterative depth-first carving over the odd-coordinate "rooms".
  std::vector<Cell> stack{{1, 1}};
  map.set_blocked({1, 1}, false);
  while (!stack.empty()) {
    const Cell c = stack.back();
    std::vector<Cell> options;
    for (const Cell d : {Cell{2, 0}, Cell{-2, 0}, Cell{0, 2}, Cell{0, -2}}) {
      const Cell n{c.x + d.x, c.y + d.y};
      if (n.x > 0 && n.y > 0 && n.x < w - 1 && n.y < h - 1 && !map.is_free(n)) {
        options.push_back(n);
      }
    }
    if (options.empty()) {
      stack.pop_back();
      continue;
    }
    const Cell next = options[detail::uniform_index(rng, options.size())];
    map.set_blocked({(c.x + next.x) / 2, (c.y + next.y) / 2}, false);
    map.set_blocked(next, false);
    stack.push_back(next);
  }
  // Open additional walls between rooms to create cycles.
  for (int y = 1; y < h - 1; ++y) {
    for (int x = 1; x < w - 1; ++x) {
      const bool horizontal_wall = (x % 2 == 0) && (y % 2 == 1);
      const bool vertical_wall = (x % 2 == 1) && (y % 2 == 0);
      if ((horizontal_wall || vertical_wall) && !map.is_free({x, y}) &&
          detail::uniform_unit(rng) < loop_fraction) {
        map.set_blocked({x, y}, false);
      }
    }
  }
  return map;
}

GridMap make_corridor_map(int room_width, int room_height, int corridor_length) {
  if (room_width < 1 || room_height < 1 || corridor_length < 1) {
    throw std::invalid_argument("make_corridor_map: dimensions must be positive");
  }
  const int w = 2 * room_width + corridor_length;
  GridMap map(w, room_height);
  const int corridor_y = room_height / 2;
  for (int x = room_width; x < room_width + corridor_length; ++x) {
    for (int y = 0; y < room_height; ++y) {
      if (y != corridor_y) {
        map.set_blocked({x, y}, true);
      }
    }
  }
  return map;
}

Warehouse make_warehouse(const WarehouseLayout& layout) {
  if (layout.shelf_block_columns < 1 || layout.shelf_block_rows < 1 || layout.shelf_length < 1 ||
      layout.shelf_depth < 1 || layout.aisle_width < 1 || layout.station_margin < 1) {
    throw std::invalid_argument("make_warehouse: layout parameters must be positive");
  }
  const int cols = layout.shelf_block_columns;
  const int rows = layout.shelf_block_rows;
  const int width =
      2 * layout.station_margin + cols * layout.shelf_length + (cols + 1) * layout.aisle_width;
  const int height = rows * layout.shelf_depth + (rows + 1) * layout.aisle_width;
  Warehouse wh{GridMap(width, height), {}, {}};
  std::set<Cell> pickups;
  for (int r = 0; r < rows; ++r) {
    const int y0 = layout.aisle_width + r * (layout.shelf_depth + layout.aisle_width);
    for (int c = 0; c < cols; ++c) {
      const int x0 = layout.station_margin + layout.aisle_width +
                     c * (layout.shelf_length + layout.aisle_width);
      for (int dy = 0; dy < layout.shelf_depth; ++dy) {
        for (int dx = 0; dx < layout.shelf_length; ++dx) {
          wh.map.set_blocked({x0 + dx, y0 + dy}, true);
        }
      }
      for (int dx = 0; dx < layout.shelf_length; ++dx) {
        pickups.insert({x0 + dx, y0 - 1});
        pickups.insert({x0 + dx, y0 + layout.shelf_depth});
      }
    }
  }
  for (const Cell p : pickups) {
    if (wh.map.is_free(p)) {
      wh.pickups.push_back(p);
    }
  }
  for (int y = 0; y < height; ++y) {
    wh.stations.push_back({0, y});
  }
  for (int y = 0; y < height; ++y) {
    wh.stations.push_back({width - 1, y});
  }
  return wh;
}

std::vector<Cell> largest_component(const GridMap& map) {
  std::vector<int> label(map.num_cells(), -1);
  std::vector<Cell> best;
  int next_label = 0;
  for (int y = 0; y < map.height(); ++y) {
    for (int x = 0; x < map.width(); ++x) {
      const Cell seed{x, y};
      if (!map.is_free(seed) || label[static_cast<std::size_t>(map.index(seed))] >= 0) {
        continue;
      }
      std::vector<Cell> component;
      std::deque<Cell> queue{seed};
      label[static_cast<std::size_t>(map.index(seed))] = next_label;
      while (!queue.empty()) {
        const Cell c = queue.front();
        queue.pop_front();
        component.push_back(c);
        for (const Cell n : map.neighbors(c)) {
          int& l = label[static_cast<std::size_t>(map.index(n))];
          if (l < 0) {
            l = next_label;
            queue.push_back(n);
          }
        }
      }
      ++next_label;
      if (component.size() > best.size()) {
        best = std::move(component);
      }
    }
  }
  std::sort(best.begin(), best.end());
  return best;
}

Instance random_instance(const GridMap& map, std::size_t num_agents, std::mt19937_64& rng) {
  const std::vector<Cell> component = largest_component(map);
  return random_instance(map, num_agents, component, component, rng);
}

Instance random_instance(const GridMap& map, std::size_t num_agents,
                         const std::vector<Cell>& start_candidates,
                         const std::vector<Cell>& goal_candidates, std::mt19937_64& rng) {
  if (start_candidates.size() < num_agents || goal_candidates.size() < num_agents) {
    throw std::invalid_argument("random_instance: not enough candidate cells for " +
                                std::to_string(num_agents) + " agents");
  }
  std::vector<Cell> starts = start_candidates;
  std::vector<Cell> goals = goal_candidates;
  detail::shuffle(starts, rng);
  detail::shuffle(goals, rng);
  Instance instance{map, {}};
  for (std::size_t a = 0; a < num_agents; ++a) {
    instance.agents.push_back({starts[a], goals[a]});
  }
  validate_instance(instance);
  return instance;
}

}  // namespace multirobotcore::mapf
