#pragma once

/// @file auction.hpp
/// Auction algorithm for the assignment problem (Bertsekas 1988, 1992).
///
/// Robots (persons) bid for tasks (objects) whose prices rise with every bid.
/// With benefits b_ij and prices p_j, an unassigned robot i finds its best
/// task j* = argmax_j (b_ij - p_j) with value v1 and second-best value v2, and
/// bids p_j* + (v1 - v2) + eps. The assignment that results satisfies
/// eps-complementary slackness, so its total benefit is within n * eps of the
/// optimum (n = number of robots); for integer benefits eps < 1/n gives an
/// optimal assignment.
///
/// Costs are converted to benefits b_ij = M - c_ij with M chosen like the
/// Hungarian penalty, and every robot has a private "stay unassigned" object
/// of benefit 0. The auction therefore solves the same lexicographic problem
/// as solve_hungarian(): maximise the number of feasible pairs, then minimise
/// cost, for rectangular problems and infeasible (+infinity) pairs alike.
///
/// Epsilon scaling is deliberately not used: in this asymmetric formulation
/// (more objects than bidders) the n * eps bound also requires every object
/// that ends unassigned to have zero price, which a single forward-auction
/// phase guarantees (an object that received a bid stays assigned) but
/// scaling phases that restart from non-zero prices do not. Forward/reverse
/// auctions (Bertsekas & Castanon 1992) remove that restriction.
///
/// Bidding modes:
///  - Gauss-Seidel: one unassigned robot bids at a time (sequential);
///  - Jacobi: all unassigned robots bid simultaneously in synchronous rounds,
///    the natural distributed implementation (one round = one bid/award
///    exchange).
/// Message accounting (shared price board model): each bid is one message and
/// each price change is broadcast to all robots.

#include "multirobotcore/allocation/types.hpp"

#include <cstddef>
#include <vector>

namespace multirobotcore::allocation {

enum class BiddingMode { kGaussSeidel, kJacobi };

struct AuctionConfig {
  BiddingMode mode{BiddingMode::kJacobi};
  /// Minimum bid increment in benefit (= cost) units. The assignment cost is
  /// within num_robots * epsilon of optimal. 0 selects
  /// 1e-3 * max|cost| / num_robots (total error at most 0.1% of max|cost|).
  double epsilon{0.0};
  std::size_t max_rounds{10'000'000};
};

struct AuctionResult {
  Assignment assignment;
  std::size_t rounds{0};  ///< bidding rounds (Jacobi) or single bids (Gauss-Seidel)
  std::size_t bids{0};
  std::size_t messages{0};     ///< bids + price broadcasts
  double epsilon{0.0};         ///< epsilon used
  std::vector<double> prices;  ///< final task prices (benefit units)
  bool converged{false};
};

/// One bid placed during a round, recorded for visualisation.
struct BidRecord {
  std::size_t robot{0};
  std::size_t task{0};
  double price{0.0};  ///< bid amount (new price if it wins)
};

struct AuctionRound {
  std::vector<BidRecord> bids;
  std::vector<std::optional<std::size_t>> owner;  ///< task -> robot after the round
  std::vector<double> prices;                     ///< task prices after the round
};

/// Runs the auction. Throws std::invalid_argument for NaN/-infinity costs.
[[nodiscard]] AuctionResult solve_auction(const CostMatrix& costs,
                                          const AuctionConfig& config = {});

/// Jacobi auction that also returns every round, for animation and teaching.
[[nodiscard]] AuctionResult solve_auction_traced(const CostMatrix& costs, double epsilon,
                                                 std::vector<AuctionRound>& trace);

}  // namespace multirobotcore::allocation
