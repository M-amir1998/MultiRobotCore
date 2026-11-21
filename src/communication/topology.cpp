#include "multirobotcore/communication/topology.hpp"

#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace multirobotcore::communication {

graph::Graph disk_graph(std::span<const Vec2> positions, double radius) {
  if (!(radius >= 0.0)) {
    throw std::invalid_argument("disk_graph: radius must be non-negative");
  }
  const std::size_t n = positions.size();
  graph::Graph g = graph::Graph::undirected(n);
  const double r2 = radius * radius;
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      if ((positions[i] - positions[j]).squaredNorm() <= r2) {
        g.add_edge(i, j);
      }
    }
  }
  return g;
}

graph::Graph knn_graph(std::span<const Vec2> positions, std::size_t k, bool symmetric) {
  const std::size_t n = positions.size();
  graph::Graph g = symmetric ? graph::Graph::undirected(n) : graph::Graph::directed(n);
  std::vector<std::size_t> order(n);
  for (std::size_t i = 0; i < n; ++i) {
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
      return (positions[a] - positions[i]).squaredNorm() <
             (positions[b] - positions[i]).squaredNorm();
    });
    std::size_t added = 0;
    for (const std::size_t j : order) {
      if (added == k) {
        break;
      }
      if (j == i) {
        continue;
      }
      g.add_edge(j, i);  // i listens to j
      ++added;
    }
  }
  return g;
}

graph::Graph apply_dropout(const graph::Graph& g, double probability, std::mt19937_64& rng) {
  if (probability < 0.0 || probability > 1.0) {
    throw std::invalid_argument("apply_dropout: probability must be in [0, 1]");
  }
  graph::Graph out(g.num_nodes(), g.directedness());
  std::bernoulli_distribution drop(probability);
  for (const graph::Edge& e : g.edges()) {
    if (!drop(rng)) {
      out.add_edge(e.source, e.target, e.weight);
    }
  }
  return out;
}

graph::Graph filter_line_of_sight(const graph::Graph& g, std::span<const Vec2> positions,
                                  const math::ObstacleSet& obstacles) {
  if (positions.size() != g.num_nodes()) {
    throw std::invalid_argument("filter_line_of_sight: positions/graph size mismatch");
  }
  graph::Graph out(g.num_nodes(), g.directedness());
  for (const graph::Edge& e : g.edges()) {
    if (!obstacles.blocks(math::Segment{positions[e.source], positions[e.target]})) {
      out.add_edge(e.source, e.target, e.weight);
    }
  }
  return out;
}

CommunicationModel::CommunicationModel(TopologyConfig config,
                                       std::optional<graph::Graph> fixed_graph)
    : config_(config), fixed_(std::move(fixed_graph)) {
  if (config_.model == TopologyModel::kFixed && !fixed_) {
    throw std::invalid_argument("CommunicationModel: fixed topology requires a graph");
  }
}

graph::Graph CommunicationModel::build(std::span<const Vec2> positions, std::mt19937_64& rng,
                                       std::span<const bool> active,
                                       const math::ObstacleSet* obstacles) const {
  graph::Graph g;
  switch (config_.model) {
    case TopologyModel::kFixed:
      if (fixed_->num_nodes() != positions.size()) {
        throw std::invalid_argument("CommunicationModel: fixed graph size mismatch");
      }
      g = *fixed_;
      break;
    case TopologyModel::kDisk:
      g = disk_graph(positions, config_.radius);
      break;
    case TopologyModel::kKNearest:
      g = knn_graph(positions, config_.k, config_.symmetric);
      break;
  }
  if (config_.line_of_sight && obstacles != nullptr && !obstacles->empty()) {
    g = filter_line_of_sight(g, positions, *obstacles);
  }
  if (config_.dropout_probability > 0.0) {
    g = apply_dropout(g, config_.dropout_probability, rng);
  }
  if (!active.empty()) {
    if (active.size() != g.num_nodes()) {
      throw std::invalid_argument("CommunicationModel: active mask size mismatch");
    }
    for (std::size_t i = 0; i < active.size(); ++i) {
      if (!active[i]) {
        g.isolate_node(i);
      }
    }
  }
  return g;
}

}  // namespace multirobotcore::communication
