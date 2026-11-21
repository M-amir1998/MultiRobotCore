#pragma once

// Exhaustive reference solver for tiny assignment problems, used to validate
// the Hungarian and auction implementations.

#include "multirobotcore/allocation/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace mrc_test {

struct ReferenceAssignment {
  std::size_t feasible_pairs{0};
  double cost{std::numeric_limits<double>::infinity()};
};

/// Enumerates every injective map from the smaller side into the larger side
/// and returns the lexicographic optimum (max feasible pairs, then min cost).
inline ReferenceAssignment brute_force_assignment(const multirobotcore::allocation::CostMatrix& c) {
  const auto rows = static_cast<std::size_t>(c.rows());
  const auto cols = static_cast<std::size_t>(c.cols());
  const bool transpose = rows > cols;
  const std::size_t small = transpose ? cols : rows;
  const std::size_t large = transpose ? rows : cols;
  auto cost = [&](std::size_t s, std::size_t l) {
    return transpose ? c(static_cast<Eigen::Index>(l), static_cast<Eigen::Index>(s))
                     : c(static_cast<Eigen::Index>(s), static_cast<Eigen::Index>(l));
  };
  ReferenceAssignment best{0, 0.0};
  bool first = true;
  std::vector<std::size_t> perm(large);
  std::iota(perm.begin(), perm.end(), std::size_t{0});
  do {
    std::size_t feasible = 0;
    double total = 0.0;
    for (std::size_t s = 0; s < small; ++s) {
      const double v = cost(s, perm[s]);
      if (std::isfinite(v)) {
        ++feasible;
        total += v;
      }
    }
    if (first || feasible > best.feasible_pairs ||
        (feasible == best.feasible_pairs && total < best.cost)) {
      best = {feasible, total};
      first = false;
    }
  } while (std::next_permutation(perm.begin(), perm.end()));
  return best;
}

}  // namespace mrc_test
