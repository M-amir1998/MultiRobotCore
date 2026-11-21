#include "multirobotcore/exploration/exploration.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <deque>
#include <stdexcept>
#include <utility>

namespace multirobotcore::exploration {
namespace {

constexpr std::array<GridCell, 4> kFour{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};

}  // namespace

ExplorationSimulation::ExplorationSimulation(OccupancyGrid truth, std::vector<GridCell> starts,
                                             ExplorationConfig config)
    : truth_(std::move(truth)), config_(config), robots_(std::move(starts)) {
  const std::size_t n = robots_.size();
  if (n == 0 || n > 64) {
    throw std::invalid_argument("ExplorationSimulation: between 1 and 64 robots required");
  }
  if (config_.replan_period == 0 || !(config_.completion_fraction > 0.0) ||
      config_.completion_fraction > 1.0) {
    throw std::invalid_argument("ExplorationSimulation: invalid configuration");
  }
  for (const GridCell& s : robots_) {
    if (!truth_.in_bounds(s) || truth_.at(s) != Occupancy::kFree) {
      throw std::invalid_argument("ExplorationSimulation: robots must start on free cells");
    }
  }
  belief_ = OccupancyGrid(truth_.width(), truth_.height(), truth_.resolution(), truth_.origin(),
                          Occupancy::kUnknown);
  observers_.assign(truth_.size(), 0);

  // Ground-truth free space reachable from the starts defines 100% coverage.
  reachable_mask_.assign(truth_.size(), 0);
  std::deque<GridCell> queue(robots_.begin(), robots_.end());
  for (const GridCell& s : robots_) {
    reachable_mask_[truth_.index(s)] = 1;
  }
  while (!queue.empty()) {
    const GridCell c = queue.front();
    queue.pop_front();
    for (const GridCell& d : kFour) {
      const GridCell nb{c.x + d.x, c.y + d.y};
      if (truth_.in_bounds(nb) && truth_.at(nb) == Occupancy::kFree &&
          reachable_mask_[truth_.index(nb)] == 0) {
        reachable_mask_[truth_.index(nb)] = 1;
        queue.push_back(nb);
      }
    }
  }
  reachable_free_ =
      static_cast<std::size_t>(std::count(reachable_mask_.begin(), reachable_mask_.end(), 1));

  targets_.assign(n, std::nullopt);
  plans_.assign(n, {});
  result_.paths.reserve(n);
  for (const GridCell& s : robots_) {
    result_.paths.push_back({s});
  }
  result_.distance.assign(n, 0.0);

  std::vector<GridCell> newly_known;
  scan_all(newly_known);
  result_.coverage.push_back(coverage());
  record(newly_known);
  if (coverage() >= config_.completion_fraction - 1e-12 || !frontier_reachable()) {
    finished_ = true;
    result_.complete = true;
  }
}

double ExplorationSimulation::coverage() const noexcept {
  return reachable_free_ == 0
             ? 1.0
             : static_cast<double>(known_reachable_) / static_cast<double>(reachable_free_);
}

void ExplorationSimulation::scan_all(std::vector<GridCell>& newly_known) {
  for (std::size_t i = 0; i < robots_.size(); ++i) {
    const auto observed = simulate_scan(truth_, truth_.cell_center(robots_[i]), config_.sensor);
    std::size_t uploaded = 0;
    for (const GridCell& c : observed) {
      const std::size_t idx = truth_.index(c);
      observers_[idx] |= std::uint64_t{1} << i;
      if (belief_.at(c) == Occupancy::kUnknown) {
        belief_.set(c, truth_.at(c));
        newly_known.push_back(c);
        ++uploaded;
        if (reachable_mask_[idx] != 0) {
          ++known_reachable_;
        }
      }
    }
    if (uploaded > 0) {
      ++result_.messages;
      result_.bytes += uploaded * config_.message_bytes_per_cell;
    }
  }
}

bool ExplorationSimulation::frontier_reachable() const {
  // Multi-source BFS over the belief from all robots; any frontier reached?
  std::vector<char> visited(belief_.size(), 0);
  std::deque<GridCell> queue;
  for (const GridCell& r : robots_) {
    if (visited[belief_.index(r)] == 0) {
      visited[belief_.index(r)] = 1;
      queue.push_back(r);
    }
  }
  while (!queue.empty()) {
    const GridCell c = queue.front();
    queue.pop_front();
    if (is_frontier(belief_, c)) {
      return true;
    }
    for (const GridCell& d : kFour) {
      const GridCell nb{c.x + d.x, c.y + d.y};
      if (belief_.in_bounds(nb) && belief_.at(nb) == Occupancy::kFree &&
          visited[belief_.index(nb)] == 0) {
        visited[belief_.index(nb)] = 1;
        queue.push_back(nb);
      }
    }
  }
  return false;
}

void ExplorationSimulation::replan() {
  const auto frontiers = detect_frontiers(belief_);
  auto clusters = cluster_frontiers(belief_, frontiers, config_.planner.min_cluster_size);
  if (clusters.empty() && !frontiers.empty()) {
    clusters = cluster_frontiers(belief_, frontiers, 1);
  }
  const TargetAssignment assignment = assign_targets(belief_, robots_, clusters, config_.planner,
                                                     config_.sensor.range / belief_.resolution());
  for (std::size_t i = 0; i < robots_.size(); ++i) {
    targets_[i] = assignment.targets[i];
    plans_[i].clear();
    if (targets_[i]) {
      auto path = shortest_path(belief_, robots_[i], *targets_[i]);
      if (!path.empty()) {
        plans_[i].assign(path.begin() + 1, path.end());
      }
    }
  }
  ++result_.replans;
  result_.messages += robots_.size();
  result_.bytes += robots_.size() * 8;  // one target cell per robot
}

void ExplorationSimulation::record(const std::vector<GridCell>& newly_known) {
  if (!config_.record_frames) {
    return;
  }
  ExplorationFrame f;
  f.step = step_;
  f.robots = robots_;
  f.targets = targets_;
  f.newly_known = newly_known;
  f.frontier = detect_frontiers(belief_);
  f.coverage = coverage();
  result_.frames.push_back(std::move(f));
}

bool ExplorationSimulation::step() {
  if (finished_) {
    return false;
  }
  bool need_replan = since_replan_ >= config_.replan_period || step_ == 0;
  for (std::size_t i = 0; i < robots_.size() && !need_replan; ++i) {
    const bool idle = plans_[i].empty();
    const bool stale = targets_[i] && !is_frontier(belief_, *targets_[i]);
    need_replan = idle || stale;
  }
  if (need_replan) {
    replan();
    since_replan_ = 0;
  }

  for (std::size_t i = 0; i < robots_.size(); ++i) {
    if (plans_[i].empty()) {
      continue;
    }
    const GridCell next = plans_[i].front();
    if (belief_.at(next) != Occupancy::kFree) {
      plans_[i].clear();
      continue;
    }
    robots_[i] = next;
    plans_[i].erase(plans_[i].begin());
    result_.paths[i].push_back(next);
    result_.distance[i] += belief_.resolution();
  }
  ++step_;
  ++since_replan_;

  std::vector<GridCell> newly_known;
  scan_all(newly_known);
  const double cov = coverage();
  result_.coverage.push_back(cov);
  if (!result_.steps_to_90 && cov >= 0.90) {
    result_.steps_to_90 = step_;
  }
  if (!result_.steps_to_95 && cov >= 0.95) {
    result_.steps_to_95 = step_;
  }
  record(newly_known);

  const bool done = cov >= config_.completion_fraction - 1e-12 || !frontier_reachable();
  if (done || step_ >= config_.max_steps) {
    finished_ = true;
    result_.complete = done;
  }
  return !finished_;
}

ExplorationResult ExplorationSimulation::run() {
  while (step()) {
  }
  result_.steps = step_;
  result_.observed_cells = 0;
  result_.redundant_cells = 0;
  for (const std::uint64_t mask : observers_) {
    const int observers = std::popcount(mask);
    if (observers >= 1) {
      ++result_.observed_cells;
    }
    if (observers >= 2) {
      ++result_.redundant_cells;
    }
  }
  return result_;
}

}  // namespace multirobotcore::exploration
