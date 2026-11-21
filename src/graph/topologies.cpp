#include "multirobotcore/graph/topologies.hpp"

#include <stdexcept>

namespace multirobotcore::graph {

Graph complete_graph(std::size_t n) {
  Graph g = Graph::undirected(n);
  for (NodeId i = 0; i < n; ++i) {
    for (NodeId j = i + 1; j < n; ++j) {
      g.add_edge(i, j);
    }
  }
  return g;
}

Graph path_graph(std::size_t n) {
  Graph g = Graph::undirected(n);
  for (NodeId i = 0; i + 1 < n; ++i) {
    g.add_edge(i, i + 1);
  }
  return g;
}

Graph ring_graph(std::size_t n) {
  Graph g = path_graph(n);
  if (n > 2) {
    g.add_edge(n - 1, 0);
  }
  return g;
}

Graph star_graph(std::size_t n, NodeId center) {
  if (n > 0 && center >= n) {
    throw std::out_of_range("star_graph: center out of range");
  }
  Graph g = Graph::undirected(n);
  for (NodeId i = 0; i < n; ++i) {
    if (i != center) {
      g.add_edge(center, i);
    }
  }
  return g;
}

Graph directed_ring(std::size_t n) {
  Graph g = Graph::directed(n);
  if (n < 2) {
    return g;
  }
  for (NodeId i = 0; i < n; ++i) {
    g.add_edge(i, (i + 1) % n);
  }
  return g;
}

Graph directed_chain(std::size_t n) {
  Graph g = Graph::directed(n);
  for (NodeId i = 0; i + 1 < n; ++i) {
    g.add_edge(i, i + 1);
  }
  return g;
}

Graph grid_graph(std::size_t rows, std::size_t cols) {
  Graph g = Graph::undirected(rows * cols);
  for (std::size_t r = 0; r < rows; ++r) {
    for (std::size_t c = 0; c < cols; ++c) {
      const NodeId v = r * cols + c;
      if (c + 1 < cols) {
        g.add_edge(v, v + 1);
      }
      if (r + 1 < rows) {
        g.add_edge(v, v + cols);
      }
    }
  }
  return g;
}

Graph random_graph(std::size_t n, double probability, std::mt19937_64& rng) {
  if (probability < 0.0 || probability > 1.0) {
    throw std::invalid_argument("random_graph: probability must be in [0, 1]");
  }
  Graph g = Graph::undirected(n);
  std::bernoulli_distribution edge(probability);
  for (NodeId i = 0; i < n; ++i) {
    for (NodeId j = i + 1; j < n; ++j) {
      if (edge(rng)) {
        g.add_edge(i, j);
      }
    }
  }
  return g;
}

}  // namespace multirobotcore::graph
