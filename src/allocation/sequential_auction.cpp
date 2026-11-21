#include "multirobotcore/allocation/sequential_auction.hpp"

#include <algorithm>
#include <limits>

namespace multirobotcore::allocation {

double route_length(const Vec2& start, std::span<const Task> tasks,
                    std::span<const std::size_t> order) {
  double length = 0.0;
  Vec2 at = start;
  for (const std::size_t k : order) {
    const Task& t = tasks[k];
    length += (t.position - at).norm();
    at = t.position;
    if (t.dropoff) {
      length += (*t.dropoff - at).norm();
      at = *t.dropoff;
    }
  }
  return length;
}

RoutePlan sequential_single_item_auction(std::span<const Robot> robots, std::span<const Task> tasks,
                                         const SequentialAuctionConfig& config) {
  RoutePlan plan;
  plan.sequences.assign(robots.size(), {});
  plan.route_length.assign(robots.size(), 0.0);
  std::vector<bool> allocated(tasks.size(), false);
  std::size_t remaining = tasks.size();

  while (remaining > 0) {
    // Best (robot, task, insertion position) by bid value.
    double best_bid = std::numeric_limits<double>::infinity();
    std::size_t best_robot = 0;
    std::size_t best_task = 0;
    std::size_t best_pos = 0;
    double best_length = 0.0;
    bool found = false;
    for (std::size_t r = 0; r < robots.size(); ++r) {
      if (plan.sequences[r].size() >= config.max_tasks_per_robot) {
        continue;
      }
      for (std::size_t k = 0; k < tasks.size(); ++k) {
        if (allocated[k] || tasks[k].demand > robots[r].capacity) {
          continue;
        }
        ++plan.bids;
        // Cheapest insertion of task k into robot r's route.
        for (std::size_t pos = 0; pos <= plan.sequences[r].size(); ++pos) {
          std::vector<std::size_t> candidate = plan.sequences[r];
          candidate.insert(candidate.begin() + static_cast<std::ptrdiff_t>(pos), k);
          const double length = route_length(robots[r].position, tasks, candidate);
          const double bid =
              config.objective == TeamObjective::kMiniSum ? length - plan.route_length[r] : length;
          if (bid < best_bid - 1e-12) {
            best_bid = bid;
            best_robot = r;
            best_task = k;
            best_pos = pos;
            best_length = length;
            found = true;
          }
        }
      }
    }
    if (!found) {
      break;  // capacity or demand constraints prevent further allocation
    }
    auto& seq = plan.sequences[best_robot];
    seq.insert(seq.begin() + static_cast<std::ptrdiff_t>(best_pos), best_task);
    plan.route_length[best_robot] = best_length;
    allocated[best_task] = true;
    --remaining;
    ++plan.rounds;
  }
  for (std::size_t k = 0; k < tasks.size(); ++k) {
    if (!allocated[k]) {
      plan.unallocated.push_back(k);
    }
  }
  for (const double l : plan.route_length) {
    plan.total_length += l;
    plan.max_length = std::max(plan.max_length, l);
  }
  return plan;
}

}  // namespace multirobotcore::allocation
