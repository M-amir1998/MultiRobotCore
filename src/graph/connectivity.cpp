#include "multirobotcore/graph/connectivity.hpp"

#include "multirobotcore/graph/matrices.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <deque>
#include <stdexcept>
#include <string>

namespace multirobotcore::graph {
namespace {

void sort_components(std::vector<std::vector<NodeId>>& components) {
  for (auto& c : components) {
    std::sort(c.begin(), c.end());
  }
  std::sort(components.begin(), components.end(),
            [](const auto& a, const auto& b) { return a.front() < b.front(); });
}

void require_undirected(const Graph& graph, const char* what) {
  if (graph.is_directed()) {
    throw std::invalid_argument(std::string(what) + ": graph must be undirected");
  }
}

Eigen::MatrixXd symmetric_laplacian(const Graph& graph) {
  return laplacian_matrix(graph);
}

}  // namespace

std::vector<std::vector<NodeId>> connected_components(const Graph& graph) {
  const std::size_t n = graph.num_nodes();
  std::vector<bool> visited(n, false);
  std::vector<std::vector<NodeId>> components;
  for (NodeId root = 0; root < n; ++root) {
    if (visited[root]) {
      continue;
    }
    std::vector<NodeId> component;
    std::deque<NodeId> queue{root};
    visited[root] = true;
    while (!queue.empty()) {
      const NodeId v = queue.front();
      queue.pop_front();
      component.push_back(v);
      for (const auto* list : {&graph.out_neighbors(v), &graph.in_neighbors(v)}) {
        for (const Neighbor& w : *list) {
          if (!visited[w.id]) {
            visited[w.id] = true;
            queue.push_back(w.id);
          }
        }
      }
    }
    components.push_back(std::move(component));
  }
  sort_components(components);
  return components;
}

std::vector<std::vector<NodeId>> strongly_connected_components(const Graph& graph) {
  // Iterative Tarjan: an explicit call stack avoids recursion depth limits.
  const std::size_t n = graph.num_nodes();
  constexpr std::size_t kUnvisited = kUnreachable;
  std::vector<std::size_t> index(n, kUnvisited);
  std::vector<std::size_t> lowlink(n, 0);
  std::vector<bool> on_stack(n, false);
  std::vector<NodeId> stack;
  std::vector<std::vector<NodeId>> components;
  std::size_t next_index = 0;

  struct Frame {
    NodeId node;
    std::size_t next_child;
  };

  for (NodeId root = 0; root < n; ++root) {
    if (index[root] != kUnvisited) {
      continue;
    }
    std::vector<Frame> calls{{root, 0}};
    index[root] = lowlink[root] = next_index++;
    stack.push_back(root);
    on_stack[root] = true;

    while (!calls.empty()) {
      const NodeId v = calls.back().node;
      const auto& successors = graph.out_neighbors(v);
      if (calls.back().next_child < successors.size()) {
        const NodeId w = successors[calls.back().next_child++].id;
        if (index[w] == kUnvisited) {
          index[w] = lowlink[w] = next_index++;
          stack.push_back(w);
          on_stack[w] = true;
          calls.push_back({w, 0});
        } else if (on_stack[w]) {
          lowlink[v] = std::min(lowlink[v], index[w]);
        }
        continue;
      }
      calls.pop_back();
      if (!calls.empty()) {
        const NodeId parent = calls.back().node;
        lowlink[parent] = std::min(lowlink[parent], lowlink[v]);
      }
      if (lowlink[v] == index[v]) {
        std::vector<NodeId> component;
        NodeId w = 0;
        do {
          w = stack.back();
          stack.pop_back();
          on_stack[w] = false;
          component.push_back(w);
        } while (w != v);
        components.push_back(std::move(component));
      }
    }
  }
  sort_components(components);
  return components;
}

bool is_connected(const Graph& graph) {
  if (graph.num_nodes() <= 1) {
    return true;
  }
  return graph.is_directed() ? strongly_connected_components(graph).size() == 1
                             : connected_components(graph).size() == 1;
}

std::vector<std::size_t> hop_distances(const Graph& graph, NodeId source) {
  const std::size_t n = graph.num_nodes();
  if (source >= n) {
    throw std::out_of_range("hop_distances: source out of range");
  }
  std::vector<std::size_t> dist(n, kUnreachable);
  std::deque<NodeId> queue{source};
  dist[source] = 0;
  while (!queue.empty()) {
    const NodeId v = queue.front();
    queue.pop_front();
    for (const Neighbor& w : graph.out_neighbors(v)) {
      if (dist[w.id] == kUnreachable) {
        dist[w.id] = dist[v] + 1;
        queue.push_back(w.id);
      }
    }
  }
  return dist;
}

std::vector<NodeId> spanning_tree_roots(const Graph& graph) {
  std::vector<NodeId> roots;
  for (NodeId r = 0; r < graph.num_nodes(); ++r) {
    const auto dist = hop_distances(graph, r);
    if (std::none_of(dist.begin(), dist.end(), [](std::size_t d) { return d == kUnreachable; })) {
      roots.push_back(r);
    }
  }
  return roots;
}

bool has_spanning_tree(const Graph& graph) {
  return graph.num_nodes() == 0 || !spanning_tree_roots(graph).empty();
}

std::optional<std::size_t> diameter(const Graph& graph) {
  std::size_t result = 0;
  for (NodeId s = 0; s < graph.num_nodes(); ++s) {
    for (const std::size_t d : hop_distances(graph, s)) {
      if (d == kUnreachable) {
        return std::nullopt;
      }
      result = std::max(result, d);
    }
  }
  return result;
}

Eigen::VectorXd laplacian_spectrum(const Graph& graph) {
  require_undirected(graph, "laplacian_spectrum");
  if (graph.num_nodes() == 0) {
    return {};
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric_laplacian(graph),
                                                              Eigen::EigenvaluesOnly);
  return solver.eigenvalues();
}

double algebraic_connectivity(const Graph& graph) {
  require_undirected(graph, "algebraic_connectivity");
  if (graph.num_nodes() < 2) {
    return 0.0;
  }
  // L is positive semidefinite; clamp round-off below zero.
  return std::max(0.0, laplacian_spectrum(graph)(1));
}

Eigen::VectorXd fiedler_vector(const Graph& graph) {
  require_undirected(graph, "fiedler_vector");
  if (graph.num_nodes() < 2) {
    throw std::invalid_argument("fiedler_vector: graph needs at least two nodes");
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric_laplacian(graph));
  Eigen::VectorXd v = solver.eigenvectors().col(1);
  // Deterministic sign: first non-negligible component positive.
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    if (std::abs(v(i)) > 1e-12) {
      if (v(i) < 0.0) {
        v = -v;
      }
      break;
    }
  }
  return v;
}

}  // namespace multirobotcore::graph
