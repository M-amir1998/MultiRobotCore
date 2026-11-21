#pragma once

/// @file hungarian.hpp
/// Optimal linear assignment: Hungarian / Kuhn-Munkres method (Kuhn 1955;
/// Munkres 1957) in the O(n^2 m) shortest-augmenting-path form with dual
/// potentials (cf. Jonker & Volgenant 1987).
///
/// Rectangular problems (n robots, m tasks) are supported directly: when
/// n <= m every robot receives a task, otherwise every task receives a robot,
/// and the total cost of the assigned pairs is minimised.
///
/// Infeasible pairs (+infinity) are handled lexicographically: the solver
/// first maximises the number of feasible pairs that can be assigned within the
/// cardinality min(n, m), then minimises cost. Infeasible pairs are never
/// reported as assigned.

#include "multirobotcore/allocation/types.hpp"

namespace multirobotcore::allocation {

/// Solves min sum c(i, sigma(i)). Throws std::invalid_argument for NaN or
/// -infinity entries.
[[nodiscard]] Assignment solve_hungarian(const CostMatrix& costs);

/// Maximisation variant: maximises total benefit (infeasible = -infinity).
[[nodiscard]] Assignment solve_hungarian_max(const Eigen::MatrixXd& benefits);

}  // namespace multirobotcore::allocation
