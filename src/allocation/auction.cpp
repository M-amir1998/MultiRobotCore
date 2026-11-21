#include "multirobotcore/allocation/auction.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>

namespace multirobotcore::allocation {
namespace {

constexpr double kNegInf = -std::numeric_limits<double>::infinity();

/// Benefit problem with n robots, m real tasks and n private dummy objects
/// (object m + i is "robot i stays unassigned", benefit 0).
struct Problem {
  std::size_t n{0};
  std::size_t m{0};
  Eigen::MatrixXd benefit;  // n x m, -inf for infeasible pairs
  double cost_scale{1.0};   // max |finite cost|
  double benefit_scale{1.0};
};

Problem make_problem(const CostMatrix& costs) {
  validate_costs(costs);
  Problem p;
  p.n = static_cast<std::size_t>(costs.rows());
  p.m = static_cast<std::size_t>(costs.cols());
  double largest = 0.0;
  for (Eigen::Index i = 0; i < costs.rows(); ++i) {
    for (Eigen::Index j = 0; j < costs.cols(); ++j) {
      if (std::isfinite(costs(i, j))) {
        largest = std::max(largest, std::abs(costs(i, j)));
      }
    }
  }
  p.cost_scale = std::max(largest, 1.0);
  // Same lexicographic penalty as the Hungarian solver: one more feasible
  // pair always outweighs any re-arrangement of the others.
  const double k = static_cast<double>(std::max<std::size_t>(std::min(p.n, p.m), 1));
  const double big_m = 2.0 * k * p.cost_scale + 1.0;
  p.benefit = costs.unaryExpr([big_m](double c) { return std::isfinite(c) ? big_m - c : kNegInf; });
  p.benefit_scale = big_m + p.cost_scale;
  return p;
}

struct Bid {
  std::size_t object;
  double amount;
};

/// Best bid of robot i given prices (objects: real tasks then dummies).
Bid best_bid(const Problem& p, const std::vector<double>& prices, std::size_t i, double eps) {
  double v1 = kNegInf;
  double v2 = kNegInf;
  std::size_t best = p.m + i;  // private dummy
  auto consider = [&](std::size_t object, double benefit) {
    const double v = benefit - prices[object];
    if (v > v1) {
      v2 = v1;
      v1 = v;
      best = object;
    } else if (v > v2) {
      v2 = v;
    }
  };
  for (std::size_t j = 0; j < p.m; ++j) {
    const double b = p.benefit(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
    if (b != kNegInf) {
      consider(j, b);
    }
  }
  consider(p.m + i, 0.0);
  const double increment = (v2 == kNegInf ? 0.0 : v1 - v2) + eps;
  return Bid{best, prices[best] + increment};
}

struct PhaseState {
  std::vector<std::optional<std::size_t>> owner;     // object -> robot
  std::vector<std::optional<std::size_t>> assigned;  // robot -> object
};

/// Runs the forward auction with fixed epsilon from the given prices.
bool run_phase(const Problem& p, double eps, BiddingMode mode, std::vector<double>& prices,
               PhaseState& state, AuctionResult& stats, std::size_t max_rounds,
               std::vector<AuctionRound>* trace) {
  const std::size_t objects = p.m + p.n;
  state.owner.assign(objects, std::nullopt);
  state.assigned.assign(p.n, std::nullopt);
  std::deque<std::size_t> unassigned;
  for (std::size_t i = 0; i < p.n; ++i) {
    unassigned.push_back(i);
  }

  while (!unassigned.empty()) {
    if (stats.rounds >= max_rounds) {
      return false;
    }
    ++stats.rounds;
    if (mode == BiddingMode::kGaussSeidel) {
      const std::size_t i = unassigned.front();
      unassigned.pop_front();
      const Bid bid = best_bid(p, prices, i, eps);
      if (const auto previous = state.owner[bid.object]) {
        state.assigned[*previous] = std::nullopt;
        unassigned.push_back(*previous);
      }
      state.owner[bid.object] = i;
      state.assigned[i] = bid.object;
      prices[bid.object] = bid.amount;
      ++stats.bids;
      stats.messages += 1 + p.n;
      continue;
    }

    // Jacobi: every unassigned robot bids against the same prices.
    std::vector<std::optional<std::pair<std::size_t, double>>> winning(objects);
    AuctionRound round;
    for (const std::size_t i : unassigned) {
      const Bid bid = best_bid(p, prices, i, eps);
      auto& w = winning[bid.object];
      if (!w || bid.amount > w->second) {
        w = std::pair{i, bid.amount};
      }
      ++stats.bids;
      ++stats.messages;
      if (trace != nullptr) {
        round.bids.push_back(BidRecord{i, bid.object, bid.amount});
      }
    }
    unassigned.clear();
    for (std::size_t j = 0; j < objects; ++j) {
      if (!winning[j]) {
        continue;
      }
      if (const auto previous = state.owner[j]) {
        state.assigned[*previous] = std::nullopt;
      }
      state.owner[j] = winning[j]->first;
      state.assigned[winning[j]->first] = j;
      prices[j] = winning[j]->second;
      stats.messages += p.n;  // price broadcast
    }
    for (std::size_t i = 0; i < p.n; ++i) {
      if (!state.assigned[i]) {
        unassigned.push_back(i);
      }
    }
    if (trace != nullptr) {
      round.owner.assign(state.owner.begin(),
                         state.owner.begin() + static_cast<std::ptrdiff_t>(p.m));
      round.prices.assign(prices.begin(), prices.begin() + static_cast<std::ptrdiff_t>(p.m));
      trace->push_back(std::move(round));
    }
  }
  return true;
}

AuctionResult finish(const CostMatrix& costs, const Problem& p, const PhaseState& state,
                     std::vector<double> prices, AuctionResult stats) {
  std::vector<std::optional<std::size_t>> robot_to_task(p.n, std::nullopt);
  for (std::size_t i = 0; i < p.n; ++i) {
    if (state.assigned[i] && *state.assigned[i] < p.m) {
      robot_to_task[i] = *state.assigned[i];
    }
  }
  stats.assignment = make_assignment(costs, robot_to_task);
  prices.resize(p.m);
  stats.prices = std::move(prices);
  return stats;
}

}  // namespace

AuctionResult solve_auction(const CostMatrix& costs, const AuctionConfig& config) {
  const Problem p = make_problem(costs);
  AuctionResult stats;
  if (p.n == 0) {
    stats.assignment = make_assignment(costs, {});
    stats.converged = true;
    return stats;
  }
  const double eps =
      config.epsilon > 0.0 ? config.epsilon : 1e-3 * p.cost_scale / static_cast<double>(p.n);
  std::vector<double> prices(p.m + p.n, 0.0);
  PhaseState state;
  stats.converged =
      run_phase(p, eps, config.mode, prices, state, stats, config.max_rounds, nullptr);
  stats.epsilon = eps;
  return finish(costs, p, state, std::move(prices), std::move(stats));
}

AuctionResult solve_auction_traced(const CostMatrix& costs, double epsilon,
                                   std::vector<AuctionRound>& trace) {
  if (!(epsilon > 0.0)) {
    throw std::invalid_argument("solve_auction_traced: epsilon must be positive");
  }
  const Problem p = make_problem(costs);
  AuctionResult stats;
  std::vector<double> prices(p.m + p.n, 0.0);
  PhaseState state;
  trace.clear();
  stats.converged =
      run_phase(p, epsilon, BiddingMode::kJacobi, prices, state, stats, 10'000'000, &trace);
  stats.epsilon = epsilon;
  return finish(costs, p, state, std::move(prices), std::move(stats));
}

}  // namespace multirobotcore::allocation
