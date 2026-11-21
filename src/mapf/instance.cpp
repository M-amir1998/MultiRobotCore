#include "multirobotcore/mapf/instance.hpp"

#include <algorithm>
#include <cstdlib>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace multirobotcore::mapf {
namespace {

std::string cell_text(Cell c) {
  return "(" + std::to_string(c.x) + ", " + std::to_string(c.y) + ")";
}

}  // namespace

void validate_instance(const Instance& instance) {
  std::set<Cell> starts;
  std::set<Cell> goals;
  for (std::size_t a = 0; a < instance.agents.size(); ++a) {
    const Agent& agent = instance.agents[a];
    if (!instance.map.is_free(agent.start) || !instance.map.is_free(agent.goal)) {
      throw std::invalid_argument("MAPF instance: agent " + std::to_string(a) +
                                  " has a blocked or out-of-bounds start/goal");
    }
    if (!starts.insert(agent.start).second) {
      throw std::invalid_argument("MAPF instance: duplicate start " + cell_text(agent.start));
    }
    if (!goals.insert(agent.goal).second) {
      throw std::invalid_argument("MAPF instance: duplicate goal " + cell_text(agent.goal));
    }
  }
}

Cell position_at(const Path& path, std::size_t t) {
  if (path.empty()) {
    throw std::invalid_argument("position_at: empty path");
  }
  return path[std::min(t, path.size() - 1)];
}

std::size_t path_cost(const Path& path) {
  if (path.empty()) {
    return 0;
  }
  std::size_t i = path.size() - 1;
  while (i > 0 && path[i - 1] == path.back()) {
    --i;
  }
  return i;
}

std::size_t sum_of_costs(const std::vector<Path>& paths) {
  std::size_t total = 0;
  for (const Path& p : paths) {
    total += path_cost(p);
  }
  return total;
}

std::size_t makespan(const std::vector<Path>& paths) {
  std::size_t worst = 0;
  for (const Path& p : paths) {
    worst = std::max(worst, path_cost(p));
  }
  return worst;
}

Path trim_trailing_waits(Path path) {
  if (!path.empty()) {
    path.resize(path_cost(path) + 1);
  }
  return path;
}

PathCheck check_paths(const Instance& instance, const std::vector<Path>& paths) {
  if (paths.size() != instance.agents.size()) {
    return {false, "expected one path per agent"};
  }
  for (std::size_t a = 0; a < paths.size(); ++a) {
    const Path& p = paths[a];
    const std::string who = "agent " + std::to_string(a) + ": ";
    if (p.empty()) {
      return {false, who + "empty path"};
    }
    if (p.front() != instance.agents[a].start) {
      return {false, who + "path does not begin at the start"};
    }
    if (p.back() != instance.agents[a].goal) {
      return {false, who + "path does not end at the goal"};
    }
    for (std::size_t t = 0; t < p.size(); ++t) {
      if (!instance.map.is_free(p[t])) {
        return {false, who + "blocked cell " + cell_text(p[t]) + " at t = " + std::to_string(t)};
      }
      if (t > 0) {
        const int step = std::abs(p[t].x - p[t - 1].x) + std::abs(p[t].y - p[t - 1].y);
        if (step > 1) {
          return {false, who + "illegal move at t = " + std::to_string(t - 1)};
        }
      }
    }
  }
  return {};
}

std::vector<TimedWaypoint> to_waypoints(const Path& path, const WorldFrame& frame,
                                        double step_duration) {
  if (!(step_duration > 0.0)) {
    throw std::invalid_argument("to_waypoints: step duration must be positive");
  }
  std::vector<TimedWaypoint> out;
  out.reserve(path.size());
  for (std::size_t t = 0; t < path.size(); ++t) {
    out.push_back({static_cast<double>(t) * step_duration, frame.to_world(path[t])});
  }
  return out;
}

}  // namespace multirobotcore::mapf
