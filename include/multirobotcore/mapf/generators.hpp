#pragma once

/// @file generators.hpp
/// Deterministic map and instance generators for tests, demos and benchmarks.

#include "multirobotcore/mapf/grid_map.hpp"
#include "multirobotcore/mapf/instance.hpp"

#include <cstddef>
#include <random>
#include <vector>

namespace multirobotcore::mapf {

/// Obstacle-free width x height grid.
[[nodiscard]] GridMap make_open_map(int width, int height);

/// Grid with each cell blocked independently with probability `density`.
[[nodiscard]] GridMap make_random_map(int width, int height, double density, std::mt19937_64& rng);

/// Perfect maze (iterative depth-first carving on odd cells) with a fraction
/// of additional walls removed to create cycles, which MAPF agents need in
/// order to pass each other. Width and height are rounded up to odd values.
[[nodiscard]] GridMap make_maze(int width, int height, double loop_fraction, std::mt19937_64& rng);

/// Two open rooms connected by a single-cell-wide corridor.
[[nodiscard]] GridMap make_corridor_map(int room_width, int room_height, int corridor_length);

struct WarehouseLayout {
  int shelf_block_columns{4};  ///< shelf blocks per row of blocks
  int shelf_block_rows{3};     ///< rows of shelf blocks
  int shelf_length{5};         ///< cells per shelf block (horizontal)
  int shelf_depth{2};          ///< cells per shelf block (vertical)
  int aisle_width{1};          ///< free cells between shelf blocks
  int station_margin{3};       ///< free columns on each side for stations
};

struct Warehouse {
  GridMap map;
  std::vector<Cell> stations;  ///< free cells on the left/right edges (pick/drop stations)
  std::vector<Cell> pickups;   ///< free cells adjacent to shelves (shelf access points)
};

/// Warehouse with rows of shelf blocks separated by aisles and station
/// columns on both sides.
[[nodiscard]] Warehouse make_warehouse(const WarehouseLayout& layout);

/// Cells of the largest 4-connected free component.
[[nodiscard]] std::vector<Cell> largest_component(const GridMap& map);

/// Random instance with distinct starts and distinct goals drawn from the
/// largest connected component. Throws std::invalid_argument if the
/// component has fewer than `num_agents` cells.
[[nodiscard]] Instance random_instance(const GridMap& map, std::size_t num_agents,
                                       std::mt19937_64& rng);

/// Random instance whose starts and goals are drawn from the given candidate
/// cell sets (e.g. stations -> shelf pickups).
[[nodiscard]] Instance random_instance(const GridMap& map, std::size_t num_agents,
                                       const std::vector<Cell>& start_candidates,
                                       const std::vector<Cell>& goal_candidates,
                                       std::mt19937_64& rng);

}  // namespace multirobotcore::mapf
