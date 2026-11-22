#include "multirobotcore/optimization/distributed.hpp"

#include "multirobotcore/graph/connectivity.hpp"
#include "multirobotcore/graph/matrices.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace multirobotcore::optimization {
namespace {

Eigen::Index idx(std::size_t i) {
  return static_cast<Eigen::Index>(i);
}

void check_inputs(const DistributedLeastSquares& problem, const graph::Graph& graph,
                  const std::vector<Eigen::VectorXd>& x0) {
  if (graph.num_nodes() != problem.num_agents() || x0.size() != problem.num_agents()) {
    throw std::invalid_argument("distributed optimisation: graph/initial size mismatch");
  }
  if (graph.is_directed()) {
    throw std::invalid_argument("distributed optimisation: graph must be undirected");
  }
  if (!graph::is_connected(graph)) {
    throw std::invalid_argument("distributed optimisation: communication graph must be connected");
  }
  for (const auto& x : x0) {
    if (x.size() != problem.dimension()) {
      throw std::invalid_argument("distributed optimisation: initial estimate dimension mismatch");
    }
  }
}

/// Fills the iteration record; returns the solution error.
IterationRecord evaluate(const DistributedLeastSquares& problem,
                         const std::vector<Eigen::VectorXd>& x, const Eigen::VectorXd& optimum,
                         double optimal_value, std::size_t k) {
  Eigen::VectorXd mean = Eigen::VectorXd::Zero(problem.dimension());
  for (const auto& xi : x) {
    mean += xi;
  }
  mean /= static_cast<double>(x.size());
  IterationRecord r;
  r.iteration = k;
  r.objective_gap = std::max(0.0, problem.objective(mean) - optimal_value);
  for (const auto& xi : x) {
    r.consensus_error = std::max(r.consensus_error, (xi - mean).norm());
    r.solution_error = std::max(r.solution_error, (xi - optimum).norm());
  }
  return r;
}

}  // namespace

DistributedLeastSquares::DistributedLeastSquares(std::vector<LocalLeastSquares> locals)
    : locals_(std::move(locals)) {
  if (locals_.empty()) {
    throw std::invalid_argument("DistributedLeastSquares: at least one agent required");
  }
  dimension_ = locals_.front().A.cols();
  for (const auto& l : locals_) {
    if (l.A.cols() != dimension_ || l.A.rows() != l.b.size()) {
      throw std::invalid_argument("DistributedLeastSquares: inconsistent local problem sizes");
    }
  }
}

double DistributedLeastSquares::local_objective(std::size_t i, const Eigen::VectorXd& x) const {
  const auto& l = locals_.at(i);
  return 0.5 * (l.A * x - l.b).squaredNorm();
}

Eigen::VectorXd DistributedLeastSquares::local_gradient(std::size_t i,
                                                        const Eigen::VectorXd& x) const {
  const auto& l = locals_.at(i);
  return l.A.transpose() * (l.A * x - l.b);
}

double DistributedLeastSquares::objective(const Eigen::VectorXd& x) const {
  double total = 0.0;
  for (std::size_t i = 0; i < locals_.size(); ++i) {
    total += local_objective(i, x);
  }
  return total;
}

Eigen::VectorXd DistributedLeastSquares::centralized_solution() const {
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(dimension_, dimension_);
  Eigen::VectorXd g = Eigen::VectorXd::Zero(dimension_);
  for (const auto& l : locals_) {
    h += l.A.transpose() * l.A;
    g += l.A.transpose() * l.b;
  }
  const Eigen::LDLT<Eigen::MatrixXd> ldlt(h);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive() ||
      ldlt.vectorD().cwiseAbs().minCoeff() <=
          1e-12 * std::max(1.0, ldlt.vectorD().cwiseAbs().maxCoeff())) {
    throw std::runtime_error("DistributedLeastSquares: normal equations are singular");
  }
  return ldlt.solve(g);
}

DistributedRun distributed_gradient_descent(const DistributedLeastSquares& problem,
                                            const graph::Graph& graph,
                                            const std::vector<Eigen::VectorXd>& x0,
                                            const GradientDescentConfig& config) {
  check_inputs(problem, graph, x0);
  const Eigen::VectorXd optimum = problem.centralized_solution();
  const double f_star = problem.objective(optimum);
  const double tol = config.tolerance * (1.0 + optimum.norm());
  const Eigen::MatrixXd w = graph::metropolis_weights(graph);
  const std::size_t n = problem.num_agents();

  DistributedRun run;
  std::vector<Eigen::VectorXd> x = x0;
  run.history.push_back(evaluate(problem, x, optimum, f_star, 0));
  if (config.record_trajectory) {
    run.trajectory.push_back(x);
  }
  for (std::size_t k = 0; k < config.max_iterations && run.history.back().solution_error > tol;
       ++k) {
    const double alpha = config.rule == StepRule::kConstant
                             ? config.step
                             : config.step / std::sqrt(static_cast<double>(k) + 1.0);
    std::vector<Eigen::VectorXd> next(n);
    for (std::size_t i = 0; i < n; ++i) {
      Eigen::VectorXd mix = w(idx(i), idx(i)) * x[i];
      for (const graph::Neighbor& j : graph.in_neighbors(i)) {
        mix += w(idx(i), idx(j.id)) * x[j.id];
      }
      next[i] = mix - alpha * problem.local_gradient(i, x[i]);
    }
    x = std::move(next);
    ++run.iterations;
    ++run.communication_rounds;
    run.messages += 2 * graph.num_edges();
    run.history.push_back(evaluate(problem, x, optimum, f_star, run.iterations));
    if (config.record_trajectory) {
      run.trajectory.push_back(x);
    }
  }
  run.converged = run.history.back().solution_error <= tol;
  run.estimates = std::move(x);
  return run;
}

DistributedRun decentralized_admm(const DistributedLeastSquares& problem, const graph::Graph& graph,
                                  const std::vector<Eigen::VectorXd>& x0,
                                  const AdmmConfig& config) {
  check_inputs(problem, graph, x0);
  if (!(config.rho > 0.0)) {
    throw std::invalid_argument("decentralized_admm: rho must be positive");
  }
  const Eigen::VectorXd optimum = problem.centralized_solution();
  const double f_star = problem.objective(optimum);
  const double tol = config.tolerance * (1.0 + optimum.norm());
  const std::size_t n = problem.num_agents();
  const Eigen::Index d = problem.dimension();

  // Each agent factorises its (constant) primal system once.
  std::vector<Eigen::LDLT<Eigen::MatrixXd>> systems;
  systems.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const auto& l = problem.local(i);
    const auto degree = static_cast<double>(graph.in_neighbors(i).size());
    systems.emplace_back(l.A.transpose() * l.A +
                         2.0 * config.rho * degree * Eigen::MatrixXd::Identity(d, d));
  }

  DistributedRun run;
  std::vector<Eigen::VectorXd> x = x0;
  std::vector<Eigen::VectorXd> dual(n, Eigen::VectorXd::Zero(d));
  run.history.push_back(evaluate(problem, x, optimum, f_star, 0));
  if (config.record_trajectory) {
    run.trajectory.push_back(x);
  }
  for (std::size_t k = 0; k < config.max_iterations && run.history.back().solution_error > tol;
       ++k) {
    std::vector<Eigen::VectorXd> next(n);
    for (std::size_t i = 0; i < n; ++i) {
      const auto& l = problem.local(i);
      Eigen::VectorXd rhs = l.A.transpose() * l.b - dual[i];
      for (const graph::Neighbor& j : graph.in_neighbors(i)) {
        rhs += config.rho * (x[i] + x[j.id]);
      }
      next[i] = systems[i].solve(rhs);
    }
    x = std::move(next);
    for (std::size_t i = 0; i < n; ++i) {
      for (const graph::Neighbor& j : graph.in_neighbors(i)) {
        dual[i] += config.rho * (x[i] - x[j.id]);
      }
    }
    ++run.iterations;
    ++run.communication_rounds;
    run.messages += 2 * graph.num_edges();
    run.history.push_back(evaluate(problem, x, optimum, f_star, run.iterations));
    if (config.record_trajectory) {
      run.trajectory.push_back(x);
    }
  }
  run.converged = run.history.back().solution_error <= tol;
  run.estimates = std::move(x);
  return run;
}

DistributedLeastSquares bearing_localization_problem(std::span<const Vec2> robot_positions,
                                                     std::span<const double> bearings) {
  if (robot_positions.size() != bearings.size()) {
    throw std::invalid_argument("bearing_localization_problem: one bearing per robot required");
  }
  std::vector<LocalLeastSquares> locals;
  locals.reserve(bearings.size());
  for (std::size_t i = 0; i < bearings.size(); ++i) {
    const Vec2 normal(-std::sin(bearings[i]), std::cos(bearings[i]));
    LocalLeastSquares l;
    l.A = normal.transpose();
    l.b = Eigen::VectorXd::Constant(1, normal.dot(robot_positions[i]));
    locals.push_back(std::move(l));
  }
  return DistributedLeastSquares(std::move(locals));
}

std::vector<double> measure_bearings(std::span<const Vec2> robot_positions, const Vec2& target,
                                     double noise_std, std::mt19937_64& rng) {
  // sigma must be positive for std::normal_distribution; unused when noise_std <= 0.
  std::normal_distribution<double> noise(0.0, noise_std > 0.0 ? noise_std : 1.0);
  std::vector<double> out;
  out.reserve(robot_positions.size());
  for (const Vec2& p : robot_positions) {
    const Vec2 d = target - p;
    out.push_back(std::atan2(d.y(), d.x()) + (noise_std > 0.0 ? noise(rng) : 0.0));
  }
  return out;
}

}  // namespace multirobotcore::optimization
