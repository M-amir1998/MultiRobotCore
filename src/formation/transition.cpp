#include "multirobotcore/formation/transition.hpp"

#include "multirobotcore/allocation/hungarian.hpp"
#include "multirobotcore/math/numerics.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace multirobotcore::formation {

std::vector<std::size_t> assign_slots(std::span<const Vec2> positions, std::span<const Vec2> slots,
                                      std::optional<std::pair<std::size_t, std::size_t>> pinned) {
  if (positions.size() > slots.size()) {
    throw std::invalid_argument("assign_slots: more robots than slots");
  }
  const auto n = static_cast<Eigen::Index>(positions.size());
  const auto m = static_cast<Eigen::Index>(slots.size());
  allocation::CostMatrix cost(n, m);
  for (Eigen::Index i = 0; i < n; ++i) {
    for (Eigen::Index j = 0; j < m; ++j) {
      cost(i, j) = (positions[static_cast<std::size_t>(i)] - slots[static_cast<std::size_t>(j)])
                       .squaredNorm();
    }
  }
  if (pinned) {
    if (pinned->first >= positions.size() || pinned->second >= slots.size()) {
      throw std::out_of_range("assign_slots: pinned pair out of range");
    }
    const auto r = static_cast<Eigen::Index>(pinned->first);
    const auto c = static_cast<Eigen::Index>(pinned->second);
    for (Eigen::Index j = 0; j < m; ++j) {
      if (j != c) {
        cost(r, j) = allocation::kInfeasible;
      }
    }
    for (Eigen::Index i = 0; i < n; ++i) {
      if (i != r) {
        cost(i, c) = allocation::kInfeasible;
      }
    }
  }
  const allocation::Assignment a = allocation::solve_hungarian(cost);
  std::vector<std::size_t> out(positions.size());
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (!a.robot_to_task[i]) {
      throw std::logic_error("assign_slots: no feasible slot assignment");
    }
    out[i] = *a.robot_to_task[i];
  }
  return out;
}

FormationTransition::FormationTransition(std::vector<Vec2> from, std::vector<Vec2> to,
                                         double start_time, double duration, bool reassign,
                                         std::optional<std::pair<std::size_t, std::size_t>> pinned)
    : from_(std::move(from)), start_(start_time), duration_(std::max(duration, 0.0)) {
  if (from_.size() != to.size()) {
    throw std::invalid_argument("FormationTransition: shapes must have the same size");
  }
  if (reassign) {
    assignment_ = assign_slots(from_, to, pinned);
  } else {
    assignment_.resize(from_.size());
    for (std::size_t i = 0; i < assignment_.size(); ++i) {
      assignment_[i] = i;
    }
  }
  target_.reserve(from_.size());
  for (const std::size_t slot : assignment_) {
    target_.push_back(to[slot]);
  }
}

std::vector<Vec2> FormationTransition::offsets_at(double t) const {
  const double tau = duration_ > 0.0 ? (t - start_) / duration_ : (t >= start_ ? 1.0 : 0.0);
  const double s = math::smoothstep5(tau);
  std::vector<Vec2> out(from_.size());
  for (std::size_t i = 0; i < from_.size(); ++i) {
    out[i] = (1.0 - s) * from_[i] + s * target_[i];
  }
  return out;
}

std::vector<Vec2> FormationTransition::offset_rates_at(double t) const {
  std::vector<Vec2> out(from_.size(), Vec2::Zero());
  if (duration_ <= 0.0) {
    return out;
  }
  const double tau = (t - start_) / duration_;
  if (tau <= 0.0 || tau >= 1.0) {
    return out;
  }
  // d/dt smoothstep5 = 30 tau^2 (1 - tau)^2 / duration
  const double rate = 30.0 * tau * tau * (1.0 - tau) * (1.0 - tau) / duration_;
  for (std::size_t i = 0; i < from_.size(); ++i) {
    out[i] = rate * (target_[i] - from_[i]);
  }
  return out;
}

}  // namespace multirobotcore::formation
