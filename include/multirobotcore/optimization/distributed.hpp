#pragma once

/// @file distributed.hpp
/// Distributed optimisation of a sum of local least-squares costs
///
///     minimise F(x) = sum_i f_i(x),   f_i(x) = 1/2 ||A_i x - b_i||^2
///
/// where agent i knows only (A_i, b_i) and communicates with its neighbours
/// in an undirected graph. Two classical methods are provided:
///
///  - Distributed (sub)gradient descent (Nedic & Ozdaglar 2009):
///        x_i <- sum_j W_ij x_j - alpha_k grad f_i(x_i)
///    with Metropolis-Hastings weights W. With a constant step it converges
///    only to a neighbourhood of the optimum; a diminishing step converges
///    slowly but exactly.
///  - Decentralised consensus ADMM (Shi, Ling, Yuan, Wu & Yin 2014):
///        x_i^{k+1} = argmin f_i(x) + x^T a_i^k
///                    + rho sum_{j in N_i} ||x - (x_i^k + x_j^k) / 2||^2
///        a_i^{k+1} = a_i^k + rho sum_{j in N_i} (x_i^{k+1} - x_j^{k+1})
///    Each primal update is a small linear solve. For this problem class the
///    iteration converges to the exact optimum, linearly when F is strongly
///    convex.
///
/// Every iteration of either method is one communication round in which each
/// agent sends its estimate to each neighbour (2 |E| messages).

#include "multirobotcore/graph/graph.hpp"
#include "multirobotcore/math/types.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <random>
#include <span>
#include <vector>

namespace multirobotcore::optimization {

struct LocalLeastSquares {
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
};

class DistributedLeastSquares {
 public:
  explicit DistributedLeastSquares(std::vector<LocalLeastSquares> locals);

  [[nodiscard]] std::size_t num_agents() const noexcept { return locals_.size(); }
  [[nodiscard]] Eigen::Index dimension() const noexcept { return dimension_; }
  [[nodiscard]] const LocalLeastSquares& local(std::size_t i) const { return locals_.at(i); }

  [[nodiscard]] double local_objective(std::size_t i, const Eigen::VectorXd& x) const;
  [[nodiscard]] Eigen::VectorXd local_gradient(std::size_t i, const Eigen::VectorXd& x) const;
  [[nodiscard]] double objective(const Eigen::VectorXd& x) const;

  /// Minimiser of F by a Cholesky (LDLT) solve of the normal equations.
  /// Throws std::runtime_error when sum_i A_i^T A_i is singular.
  [[nodiscard]] Eigen::VectorXd centralized_solution() const;

 private:
  std::vector<LocalLeastSquares> locals_;
  Eigen::Index dimension_{0};
};

struct IterationRecord {
  std::size_t iteration{0};
  double objective_gap{0.0};    ///< F(mean_i x_i) - F(x*)
  double consensus_error{0.0};  ///< max_i ||x_i - mean_j x_j||
  double solution_error{0.0};   ///< max_i ||x_i - x*||
};

struct DistributedRun {
  std::vector<Eigen::VectorXd> estimates;  ///< final estimate of every agent
  std::vector<IterationRecord> history;    ///< one record per iteration (incl. k = 0)
  /// trajectory[k][i]: estimate of agent i after k iterations (if recorded).
  std::vector<std::vector<Eigen::VectorXd>> trajectory;
  std::size_t iterations{0};
  std::size_t communication_rounds{0};
  std::size_t messages{0};
  bool converged{false};
};

enum class StepRule {
  kConstant,     ///< alpha_k = step
  kDiminishing,  ///< alpha_k = step / sqrt(k + 1)
};

struct GradientDescentConfig {
  double step{0.1};
  StepRule rule{StepRule::kDiminishing};
  std::size_t max_iterations{5000};
  double tolerance{1e-6};  ///< stop when max_i ||x_i - x*|| <= tolerance (1 + ||x*||)
  bool record_trajectory{false};
};

struct AdmmConfig {
  double rho{1.0};
  std::size_t max_iterations{2000};
  double tolerance{1e-6};
  bool record_trajectory{false};
};

[[nodiscard]] DistributedRun distributed_gradient_descent(const DistributedLeastSquares& problem,
                                                          const graph::Graph& graph,
                                                          const std::vector<Eigen::VectorXd>& x0,
                                                          const GradientDescentConfig& config);

[[nodiscard]] DistributedRun decentralized_admm(const DistributedLeastSquares& problem,
                                                const graph::Graph& graph,
                                                const std::vector<Eigen::VectorXd>& x0,
                                                const AdmmConfig& config);

// --- Cooperative bearing-only localisation -------------------------------------

/// Robot i at p_i measures the bearing beta_i [rad] to a target at theta. The
/// line-of-bearing constraint n_i^T (theta - p_i) = 0 with
/// n_i = (-sin beta_i, cos beta_i) gives f_i(theta) = 1/2 (n_i^T theta - n_i^T p_i)^2:
/// rank one, so no robot can localise the target alone.
[[nodiscard]] DistributedLeastSquares bearing_localization_problem(
    std::span<const Vec2> robot_positions, std::span<const double> bearings);

/// Bearings from robots to `target` with additive Gaussian noise [rad].
[[nodiscard]] std::vector<double> measure_bearings(std::span<const Vec2> robot_positions,
                                                   const Vec2& target, double noise_std,
                                                   std::mt19937_64& rng);

}  // namespace multirobotcore::optimization
