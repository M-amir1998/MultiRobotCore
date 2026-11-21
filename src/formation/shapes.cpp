#include "multirobotcore/formation/shapes.hpp"

#include "multirobotcore/math/numerics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace multirobotcore::formation {
namespace {

/// Places `n` slots in rows of `row_length(r)` lattice positions; row r sits
/// at x = -r * row_spacing and full rows are centred laterally. A partial last
/// row occupies the central lattice positions of its row, so the lattice
/// spacing (and hence the minimum neighbour distance) is preserved.
template <typename RowLength>
std::vector<Vec2> rows_layout(std::size_t n, double spacing, double row_spacing,
                              RowLength row_length) {
  std::vector<Vec2> out;
  out.reserve(n);
  std::size_t placed = 0;
  for (std::size_t r = 0; placed < n; ++r) {
    const std::size_t full = row_length(r);
    const std::size_t count = std::min(full, n - placed);
    const std::size_t first = (full - count) / 2;
    for (std::size_t j = 0; j < count; ++j) {
      const double lattice = static_cast<double>(first + j) - 0.5 * static_cast<double>(full - 1);
      out.emplace_back(-static_cast<double>(r) * row_spacing, lattice * spacing);
    }
    placed += count;
  }
  return out;
}

}  // namespace

std::string_view to_string(Shape shape) noexcept {
  switch (shape) {
    case Shape::kLine:
      return "line";
    case Shape::kColumn:
      return "column";
    case Shape::kTriangle:
      return "triangle";
    case Shape::kSquare:
      return "square";
    case Shape::kCircle:
      return "circle";
    case Shape::kV:
      return "v";
  }
  return "unknown";
}

std::optional<Shape> shape_from_string(std::string_view name) noexcept {
  for (const Shape s : {Shape::kLine, Shape::kColumn, Shape::kTriangle, Shape::kSquare,
                        Shape::kCircle, Shape::kV}) {
    if (to_string(s) == name) {
      return s;
    }
  }
  return std::nullopt;
}

std::vector<Vec2> centered(std::vector<Vec2> offsets) {
  if (offsets.empty()) {
    return offsets;
  }
  Vec2 mean = Vec2::Zero();
  for (const Vec2& d : offsets) {
    mean += d;
  }
  mean /= static_cast<double>(offsets.size());
  for (Vec2& d : offsets) {
    d -= mean;
  }
  return offsets;
}

std::vector<Vec2> make_v_shape(std::size_t n, double spacing, double half_angle) {
  if (!(spacing > 0.0)) {
    throw std::invalid_argument("make_v_shape: spacing must be positive");
  }
  std::vector<Vec2> out;
  out.reserve(n);
  for (std::size_t k = 0; k < n; ++k) {
    if (k == 0) {
      out.emplace_back(0.0, 0.0);
      continue;
    }
    const auto arm = static_cast<double>((k + 1) / 2);
    const double side = (k % 2 == 1) ? 1.0 : -1.0;
    out.emplace_back(-arm * spacing * std::cos(half_angle),
                     side * arm * spacing * std::sin(half_angle));
  }
  return centered(std::move(out));
}

std::vector<Vec2> make_shape(Shape shape, std::size_t n, double spacing) {
  if (!(spacing > 0.0)) {
    throw std::invalid_argument("make_shape: spacing must be positive");
  }
  if (n == 0) {
    return {};
  }
  std::vector<Vec2> out;
  switch (shape) {
    case Shape::kLine:
      out = rows_layout(n, spacing, spacing, [n](std::size_t) { return n; });
      break;
    case Shape::kColumn:
      out = rows_layout(n, spacing, spacing, [](std::size_t) { return std::size_t{1}; });
      break;
    case Shape::kTriangle:
      out = rows_layout(n, spacing, spacing * std::sqrt(3.0) / 2.0,
                        [](std::size_t r) { return r + 1; });
      break;
    case Shape::kSquare: {
      const auto side = static_cast<std::size_t>(std::ceil(std::sqrt(static_cast<double>(n))));
      out = rows_layout(n, spacing, spacing, [side](std::size_t) { return side; });
      break;
    }
    case Shape::kCircle: {
      if (n == 1) {
        out.emplace_back(0.0, 0.0);
        break;
      }
      const double radius = spacing / (2.0 * std::sin(math::kPi / static_cast<double>(n)));
      for (std::size_t i = 0; i < n; ++i) {
        const double a = math::kTwoPi * static_cast<double>(i) / static_cast<double>(n);
        out.emplace_back(radius * std::cos(a), radius * std::sin(a));
      }
      break;
    }
    case Shape::kV:
      return make_v_shape(n, spacing, 35.0 * math::kPi / 180.0);
  }
  return centered(std::move(out));
}

std::vector<Vec2> relative_to_slot(std::span<const Vec2> offsets, std::size_t slot) {
  if (slot >= offsets.size()) {
    throw std::out_of_range("relative_to_slot: slot out of range");
  }
  std::vector<Vec2> out(offsets.begin(), offsets.end());
  const Vec2 origin = offsets[slot];
  for (Vec2& d : out) {
    d -= origin;
  }
  return out;
}

Vec2 to_world(const Pose2& reference, const Vec2& offset) noexcept {
  return reference.position + math::rotate(offset, reference.heading);
}

std::vector<Vec2> to_world(const Pose2& reference, std::span<const Vec2> offsets) {
  std::vector<Vec2> out;
  out.reserve(offsets.size());
  for (const Vec2& d : offsets) {
    out.push_back(to_world(reference, d));
  }
  return out;
}

graph::Graph henneberg_graph(std::span<const Vec2> offsets) {
  const std::size_t n = offsets.size();
  graph::Graph g = graph::Graph::undirected(n);
  if (n < 2) {
    return g;
  }
  auto non_collinear = [&](std::size_t k, std::size_t a, std::size_t b) {
    const Vec2 da = offsets[a] - offsets[k];
    const Vec2 db = offsets[b] - offsets[k];
    return std::abs(math::cross(da, db)) > 1e-6 * da.norm() * db.norm();
  };
  // Anchors for vertex k among the placed set: the closest placed robot and
  // the closest placed robot not collinear with it and k (if any).
  auto anchors = [&](std::size_t k, const std::vector<std::size_t>& placed) {
    std::vector<std::size_t> order = placed;
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
      return (offsets[a] - offsets[k]).squaredNorm() < (offsets[b] - offsets[k]).squaredNorm();
    });
    for (std::size_t idx = 1; idx < order.size(); ++idx) {
      if (non_collinear(k, order[0], order[idx])) {
        return std::pair{std::pair{order[0], order[idx]}, true};
      }
    }
    return std::pair{std::pair{order[0], order[1]}, false};
  };

  std::vector<std::size_t> placed{0};
  std::vector<bool> used(n, false);
  used[0] = true;
  // Second vertex: closest to robot 0.
  std::size_t second = 1;
  for (std::size_t k = 1; k < n; ++k) {
    if ((offsets[k] - offsets[0]).squaredNorm() < (offsets[second] - offsets[0]).squaredNorm()) {
      second = k;
    }
  }
  g.add_edge(0, second);
  placed.push_back(second);
  used[second] = true;

  // Greedy Henneberg type-I additions: prefer the unplaced robot closest to
  // the placed set that admits a non-collinear anchor pair.
  while (placed.size() < n) {
    std::size_t best = n;
    double best_dist = std::numeric_limits<double>::infinity();
    bool best_generic = false;
    for (std::size_t k = 0; k < n; ++k) {
      if (used[k]) {
        continue;
      }
      double dist = std::numeric_limits<double>::infinity();
      for (const std::size_t q : placed) {
        dist = std::min(dist, (offsets[k] - offsets[q]).squaredNorm());
      }
      const bool generic = anchors(k, placed).second;
      if ((generic && !best_generic) || (generic == best_generic && dist < best_dist)) {
        best = k;
        best_dist = dist;
        best_generic = generic;
      }
    }
    const auto chosen = anchors(best, placed).first;
    g.add_edge(chosen.first, best);
    g.add_edge(chosen.second, best);
    placed.push_back(best);
    used[best] = true;
  }
  return g;
}

std::vector<double> desired_distances(const graph::Graph& graph, std::span<const Vec2> offsets) {
  if (offsets.size() != graph.num_nodes()) {
    throw std::invalid_argument("desired_distances: one offset per node required");
  }
  std::vector<double> out;
  for (const graph::Edge& e : graph.edges()) {
    out.push_back((offsets[e.source] - offsets[e.target]).norm());
  }
  return out;
}

}  // namespace multirobotcore::formation
