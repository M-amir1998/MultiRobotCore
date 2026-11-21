#include "multirobotcore/allocation/hungarian.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace multirobotcore::allocation {
namespace {

/// Shortest augmenting path Hungarian method for an n x m matrix with
/// n <= m and finite entries. Returns, for each row, its assigned column.
std::vector<std::size_t> hungarian_rows_le_cols(const Eigen::MatrixXd& a) {
  const auto n = static_cast<std::size_t>(a.rows());
  const auto m = static_cast<std::size_t>(a.cols());
  constexpr double kInf = std::numeric_limits<double>::infinity();
  // 1-based potentials: u for rows, v for columns; p[j] = row matched to
  // column j (0 = none); way[j] = previous column on the alternating path.
  std::vector<double> u(n + 1, 0.0);
  std::vector<double> v(m + 1, 0.0);
  std::vector<std::size_t> p(m + 1, 0);
  std::vector<std::size_t> way(m + 1, 0);
  auto cost = [&a](std::size_t i, std::size_t j) {
    return a(static_cast<Eigen::Index>(i - 1), static_cast<Eigen::Index>(j - 1));
  };
  for (std::size_t i = 1; i <= n; ++i) {
    p[0] = i;
    std::size_t j0 = 0;
    std::vector<double> minv(m + 1, kInf);
    std::vector<char> used(m + 1, 0);
    do {
      used[j0] = 1;
      const std::size_t i0 = p[j0];
      double delta = kInf;
      std::size_t j1 = 0;
      for (std::size_t j = 1; j <= m; ++j) {
        if (used[j] != 0) {
          continue;
        }
        const double reduced = cost(i0, j) - u[i0] - v[j];
        if (reduced < minv[j]) {
          minv[j] = reduced;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (std::size_t j = 0; j <= m; ++j) {
        if (used[j] != 0) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    // Augment along the alternating path.
    do {
      const std::size_t j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }
  std::vector<std::size_t> row_to_col(n, 0);
  for (std::size_t j = 1; j <= m; ++j) {
    if (p[j] != 0) {
      row_to_col[p[j] - 1] = j - 1;
    }
  }
  return row_to_col;
}

}  // namespace

Assignment solve_hungarian(const CostMatrix& costs) {
  validate_costs(costs);
  const auto n = static_cast<std::size_t>(costs.rows());
  const auto m = static_cast<std::size_t>(costs.cols());
  std::vector<std::optional<std::size_t>> robot_to_task(n, std::nullopt);
  if (n == 0 || m == 0) {
    return make_assignment(costs, robot_to_task);
  }

  // Replace infeasible entries by a penalty larger than any difference between
  // the feasible parts of two assignments of cardinality k = min(n, m), so the
  // optimum first minimises the number of infeasible pairs.
  double largest = 0.0;
  bool any_infeasible = false;
  for (Eigen::Index i = 0; i < costs.rows(); ++i) {
    for (Eigen::Index j = 0; j < costs.cols(); ++j) {
      if (std::isinf(costs(i, j))) {
        any_infeasible = true;
      } else {
        largest = std::max(largest, std::abs(costs(i, j)));
      }
    }
  }
  const double k = static_cast<double>(std::min(n, m));
  const double penalty = 2.0 * k * std::max(largest, 1.0) + 1.0;
  Eigen::MatrixXd work = costs;
  if (any_infeasible) {
    work = costs.unaryExpr([penalty](double c) { return std::isinf(c) ? penalty : c; });
  }

  if (n <= m) {
    const auto row_to_col = hungarian_rows_le_cols(work);
    for (std::size_t r = 0; r < n; ++r) {
      robot_to_task[r] = row_to_col[r];
    }
  } else {
    const Eigen::MatrixXd transposed = work.transpose();
    const auto task_to_row = hungarian_rows_le_cols(transposed);
    for (std::size_t t = 0; t < m; ++t) {
      robot_to_task[task_to_row[t]] = t;
    }
  }
  // Infeasible pairs are reported as unassigned.
  for (std::size_t r = 0; r < n; ++r) {
    if (robot_to_task[r] && std::isinf(costs(static_cast<Eigen::Index>(r),
                                             static_cast<Eigen::Index>(*robot_to_task[r])))) {
      robot_to_task[r] = std::nullopt;
    }
  }
  return make_assignment(costs, robot_to_task);
}

Assignment solve_hungarian_max(const Eigen::MatrixXd& benefits) {
  for (Eigen::Index i = 0; i < benefits.rows(); ++i) {
    for (Eigen::Index j = 0; j < benefits.cols(); ++j) {
      if (std::isnan(benefits(i, j)) || benefits(i, j) == std::numeric_limits<double>::infinity()) {
        throw std::invalid_argument("solve_hungarian_max: benefits contain NaN or +infinity");
      }
    }
  }
  const CostMatrix costs = -benefits;  // -(-inf) = +inf marks infeasible pairs
  Assignment a = solve_hungarian(costs);
  a.total_cost = -a.total_cost;  // report total benefit
  return a;
}

}  // namespace multirobotcore::allocation
