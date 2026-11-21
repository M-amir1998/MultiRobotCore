#include "multirobotcore/graph/graph.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>

namespace multirobotcore::graph {
namespace {

auto find_neighbor(const std::vector<Neighbor>& list, NodeId id) {
  return std::lower_bound(list.begin(), list.end(), id,
                          [](const Neighbor& n, NodeId value) { return n.id < value; });
}

}  // namespace

Graph::Graph(std::size_t num_nodes, Directedness directedness)
    : directedness_(directedness), in_(num_nodes), out_(num_nodes) {}

std::size_t Graph::num_edges() const noexcept {
  std::size_t total = 0;
  for (const auto& list : out_) {
    total += list.size();
  }
  return is_directed() ? total : total / 2;
}

NodeId Graph::add_node() {
  in_.emplace_back();
  out_.emplace_back();
  return in_.size() - 1;
}

void Graph::check_node(NodeId node) const {
  if (node >= num_nodes()) {
    throw std::out_of_range("Graph: node " + std::to_string(node) + " out of range (" +
                            std::to_string(num_nodes()) + " nodes)");
  }
}

bool Graph::upsert(std::vector<Neighbor>& list, NodeId id, double weight) {
  auto it = std::lower_bound(list.begin(), list.end(), id,
                             [](const Neighbor& n, NodeId value) { return n.id < value; });
  if (it != list.end() && it->id == id) {
    it->weight = weight;
    return false;
  }
  list.insert(it, Neighbor{id, weight});
  return true;
}

bool Graph::erase(std::vector<Neighbor>& list, NodeId id) {
  auto it = std::lower_bound(list.begin(), list.end(), id,
                             [](const Neighbor& n, NodeId value) { return n.id < value; });
  if (it != list.end() && it->id == id) {
    list.erase(it);
    return true;
  }
  return false;
}

void Graph::add_directed(NodeId source, NodeId target, double weight) {
  upsert(out_[source], target, weight);
  upsert(in_[target], source, weight);
}

bool Graph::remove_directed(NodeId source, NodeId target) {
  const bool removed = erase(out_[source], target);
  erase(in_[target], source);
  return removed;
}

void Graph::add_edge(NodeId source, NodeId target, double weight) {
  check_node(source);
  check_node(target);
  if (source == target) {
    throw std::invalid_argument("Graph::add_edge: self-loops are not allowed");
  }
  if (!std::isfinite(weight) || weight <= 0.0) {
    throw std::invalid_argument("Graph::add_edge: weight must be finite and positive");
  }
  add_directed(source, target, weight);
  if (!is_directed()) {
    add_directed(target, source, weight);
  }
}

bool Graph::remove_edge(NodeId source, NodeId target) {
  check_node(source);
  check_node(target);
  bool removed = remove_directed(source, target);
  if (!is_directed()) {
    removed = remove_directed(target, source) || removed;
  }
  return removed;
}

void Graph::isolate_node(NodeId node) {
  check_node(node);
  for (const Neighbor& n : out_[node]) {
    erase(in_[n.id], node);
  }
  for (const Neighbor& n : in_[node]) {
    erase(out_[n.id], node);
  }
  out_[node].clear();
  in_[node].clear();
}

void Graph::clear_edges() noexcept {
  for (auto& list : in_) {
    list.clear();
  }
  for (auto& list : out_) {
    list.clear();
  }
}

bool Graph::has_edge(NodeId source, NodeId target) const {
  return weight(source, target).has_value();
}

std::optional<double> Graph::weight(NodeId source, NodeId target) const {
  check_node(source);
  check_node(target);
  const auto& list = out_[source];
  const auto it = find_neighbor(list, target);
  if (it != list.end() && it->id == target) {
    return it->weight;
  }
  return std::nullopt;
}

const std::vector<Neighbor>& Graph::in_neighbors(NodeId node) const {
  check_node(node);
  return in_[node];
}

const std::vector<Neighbor>& Graph::out_neighbors(NodeId node) const {
  check_node(node);
  return out_[node];
}

std::vector<NodeId> Graph::neighbor_ids(NodeId node) const {
  const auto& list = in_neighbors(node);
  std::vector<NodeId> ids;
  ids.reserve(list.size());
  for (const Neighbor& n : list) {
    ids.push_back(n.id);
  }
  return ids;
}

double Graph::in_degree(NodeId node) const {
  const auto& list = in_neighbors(node);
  return std::accumulate(list.begin(), list.end(), 0.0,
                         [](double acc, const Neighbor& n) { return acc + n.weight; });
}

double Graph::out_degree(NodeId node) const {
  const auto& list = out_neighbors(node);
  return std::accumulate(list.begin(), list.end(), 0.0,
                         [](double acc, const Neighbor& n) { return acc + n.weight; });
}

std::vector<Edge> Graph::edges() const {
  std::vector<Edge> result;
  for (NodeId source = 0; source < out_.size(); ++source) {
    for (const Neighbor& n : out_[source]) {
      if (!is_directed() && n.id < source) {
        continue;
      }
      result.push_back(Edge{source, n.id, n.weight});
    }
  }
  return result;
}

bool Graph::is_symmetric() const {
  for (NodeId source = 0; source < out_.size(); ++source) {
    for (const Neighbor& n : out_[source]) {
      const auto reverse = weight(n.id, source);
      if (!reverse || *reverse != n.weight) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace multirobotcore::graph
