#pragma once

// Independent reference solvers for tiny MAPF instances, used to validate the
// optimality of Conflict-Based Search. They search the joint state space of
// all agents directly and share no code with the planners under test.

#include "multirobotcore/mapf/instance.hpp"

#include <cstddef>
#include <deque>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace mrc_test {

namespace mapf = multirobotcore::mapf;

/// All joint successors of `positions` (cell indices): every agent moves to a
/// free 4-neighbour or waits; vertex and swap conflicts are excluded.
inline std::vector<std::vector<int>> joint_successors(const mapf::GridMap& map,
                                                      const std::vector<int>& positions) {
  const std::size_t n = positions.size();
  std::vector<std::vector<int>> options(n);
  for (std::size_t a = 0; a < n; ++a) {
    const mapf::Cell c = map.cell(positions[a]);
    options[a].push_back(positions[a]);
    for (const mapf::Cell nb : {mapf::Cell{c.x + 1, c.y}, mapf::Cell{c.x - 1, c.y},
                                mapf::Cell{c.x, c.y + 1}, mapf::Cell{c.x, c.y - 1}}) {
      if (map.is_free(nb)) {
        options[a].push_back(map.index(nb));
      }
    }
  }
  std::vector<std::vector<int>> result;
  std::vector<int> next(n);
  // Odometer enumeration of the Cartesian product.
  std::vector<std::size_t> choice(n, 0);
  while (true) {
    for (std::size_t a = 0; a < n; ++a) {
      next[a] = options[a][choice[a]];
    }
    bool valid = true;
    for (std::size_t a = 0; a < n && valid; ++a) {
      for (std::size_t b = a + 1; b < n && valid; ++b) {
        if (next[a] == next[b]) {
          valid = false;  // vertex conflict
        } else if (next[a] == positions[b] && next[b] == positions[a]) {
          valid = false;  // swap conflict
        }
      }
    }
    if (valid) {
      result.push_back(next);
    }
    std::size_t k = 0;
    while (k < n && ++choice[k] == options[k].size()) {
      choice[k] = 0;
      ++k;
    }
    if (k == n) {
      break;
    }
  }
  return result;
}

/// Optimal sum of costs by Dijkstra over (positions, pending goal waits).
/// An agent waiting at its goal accrues "pending" waits at zero cost; they
/// are paid only if it later leaves, which reproduces the standard SOC
/// definition (cost = time of final arrival at the goal).
inline std::optional<std::size_t> optimal_sum_of_costs(const mapf::Instance& instance,
                                                       std::size_t max_cost = 40) {
  const auto& map = instance.map;
  const std::size_t n = instance.agents.size();
  std::vector<int> goals(n);
  std::vector<int> start(2 * n, 0);
  for (std::size_t a = 0; a < n; ++a) {
    goals[a] = map.index(instance.agents[a].goal);
    start[a] = map.index(instance.agents[a].start);
  }
  using State = std::vector<int>;  // positions followed by pending waits
  std::map<State, std::size_t> best;
  std::priority_queue<std::pair<std::size_t, State>, std::vector<std::pair<std::size_t, State>>,
                      std::greater<>>
      open;
  best[start] = 0;
  open.push({0, start});
  while (!open.empty()) {
    auto [cost, state] = open.top();
    open.pop();
    if (best[state] < cost) {
      continue;
    }
    bool done = true;
    for (std::size_t a = 0; a < n; ++a) {
      done = done && state[a] == goals[a];
    }
    if (done) {
      return cost;
    }
    const std::vector<int> positions(state.begin(), state.begin() + static_cast<long>(n));
    for (const auto& next : joint_successors(map, positions)) {
      State s(2 * n);
      std::size_t step = 0;
      for (std::size_t a = 0; a < n; ++a) {
        s[a] = next[a];
        const int pending = state[n + a];
        if (positions[a] == goals[a] && next[a] == goals[a]) {
          s[n + a] = pending + 1;  // free while it never leaves again
        } else {
          s[n + a] = 0;
          step += 1 + static_cast<std::size_t>(pending);
        }
      }
      const std::size_t new_cost = cost + step;
      if (new_cost > max_cost) {
        continue;
      }
      auto it = best.find(s);
      if (it == best.end() || new_cost < it->second) {
        best[s] = new_cost;
        open.push({new_cost, s});
      }
    }
  }
  return std::nullopt;
}

/// Optimal makespan by breadth-first search over joint positions.
inline std::optional<std::size_t> optimal_makespan(const mapf::Instance& instance,
                                                   std::size_t max_depth = 40) {
  const auto& map = instance.map;
  const std::size_t n = instance.agents.size();
  std::vector<int> start(n);
  std::vector<int> goal(n);
  for (std::size_t a = 0; a < n; ++a) {
    start[a] = map.index(instance.agents[a].start);
    goal[a] = map.index(instance.agents[a].goal);
  }
  std::set<std::vector<int>> seen{start};
  std::deque<std::pair<std::vector<int>, std::size_t>> queue{{start, 0}};
  while (!queue.empty()) {
    auto [state, depth] = queue.front();
    queue.pop_front();
    if (state == goal) {
      return depth;
    }
    if (depth >= max_depth) {
      continue;
    }
    for (auto& next : joint_successors(map, state)) {
      if (seen.insert(next).second) {
        queue.emplace_back(std::move(next), depth + 1);
      }
    }
  }
  return std::nullopt;
}

}  // namespace mrc_test
