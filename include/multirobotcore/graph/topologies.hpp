#pragma once

/// @file topologies.hpp
/// Standard topologies used in tests, benchmarks and fixed-topology scenarios.

#include "multirobotcore/graph/graph.hpp"

#include <cstddef>
#include <random>

namespace multirobotcore::graph {

[[nodiscard]] Graph complete_graph(std::size_t n);
[[nodiscard]] Graph path_graph(std::size_t n);
[[nodiscard]] Graph ring_graph(std::size_t n);
/// Undirected star centred at `center`.
[[nodiscard]] Graph star_graph(std::size_t n, NodeId center = 0);
/// Directed cycle 0 -> 1 -> ... -> n-1 -> 0.
[[nodiscard]] Graph directed_ring(std::size_t n);
/// Directed chain 0 -> 1 -> ... -> n-1 (node 0 is the only spanning-tree root).
[[nodiscard]] Graph directed_chain(std::size_t n);
/// Undirected 4-connected rows x cols lattice; node id = r * cols + c.
[[nodiscard]] Graph grid_graph(std::size_t rows, std::size_t cols);
/// Erdos-Renyi G(n, p) with an explicit generator.
[[nodiscard]] Graph random_graph(std::size_t n, double probability, std::mt19937_64& rng);

}  // namespace multirobotcore::graph
