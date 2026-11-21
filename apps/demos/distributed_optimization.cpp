// Distributed optimisation demonstration: cooperative bearing-only target
// localisation solved by distributed gradient descent (constant and
// diminishing steps) and decentralised consensus ADMM over the communication
// graph, compared with the centralised least-squares solution.
//
// Output: JSON consumed by scripts/render/render_distributed_optimization.py.
//
// mrc-modules: optimization communication io

#include "demo_common.hpp"

#include "multirobotcore/communication/topology.hpp"
#include "multirobotcore/graph/connectivity.hpp"
#include "multirobotcore/io/json.hpp"
#include "multirobotcore/optimization/distributed.hpp"

#include <iostream>
#include <random>
#include <vector>

namespace mrc = multirobotcore;
namespace io = multirobotcore::io;
namespace opt = multirobotcore::optimization;
using mrc_apps::Json;

namespace {

Json run_json(const opt::DistributedRun& run, std::size_t record) {
  Json err = Json::array();
  Json cons = Json::array();
  Json gap = Json::array();
  for (const auto& h : run.history) {
    err.push_back(io::round_to(h.solution_error, 9));
    cons.push_back(io::round_to(h.consensus_error, 9));
    gap.push_back(io::round_to(h.objective_gap, 12));
  }
  Json traj = Json::array();
  for (std::size_t k = 0; k < run.trajectory.size() && k <= record; ++k) {
    Json step = Json::array();
    for (const auto& x : run.trajectory[k]) {
      step.push_back(Json::array({io::round_to(x(0), 4), io::round_to(x(1), 4)}));
    }
    traj.push_back(std::move(step));
  }
  return Json{{"iterations", run.iterations},
              {"converged", run.converged},
              {"messages", run.messages},
              {"solution_error", err},
              {"consensus_error", cons},
              {"objective_gap", gap},
              {"trajectory", traj}};
}

}  // namespace

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args = mrc_apps::parse_args(argc, argv, "distributed_optimization",
                                           "configs/optimization/distributed_optimization.json");
    const Json cfg = mrc_apps::load_config(args);
    std::mt19937_64 rng(cfg.value("seed", std::uint64_t{5}));
    const auto field = io::aabb_from_json(cfg.at("field"));
    const auto n = cfg.value("num_robots", std::size_t{10});
    const mrc::Vec2 target = io::vec2_from_json(cfg.at("target"));
    const double radius = cfg.value("comm_radius", 7.0);

    std::uniform_real_distribution<double> ux(field.min.x(), field.max.x());
    std::uniform_real_distribution<double> uy(field.min.y(), field.max.y());
    std::vector<mrc::Vec2> robots;
    mrc::graph::Graph graph;
    for (int attempt = 0; attempt < 10000; ++attempt) {
      robots.clear();
      while (robots.size() < n) {
        const mrc::Vec2 p(ux(rng), uy(rng));
        if ((p - target).norm() > 3.0) {
          robots.push_back(p);
        }
      }
      graph = mrc::communication::disk_graph(robots, radius);
      if (mrc::graph::is_connected(graph)) {
        break;
      }
    }
    const auto bearings =
        opt::measure_bearings(robots, target, cfg.value("bearing_noise", 0.03), rng);
    const auto problem = opt::bearing_localization_problem(robots, bearings);
    const auto optimum = problem.centralized_solution();
    const auto record = cfg.value("record_iterations", std::size_t{120});

    // Every robot starts from its own position as its initial guess.
    std::vector<Eigen::VectorXd> x0;
    for (const auto& p : robots) {
      x0.emplace_back(p);
    }
    const Json& dgd_cfg = cfg.at("dgd");
    opt::GradientDescentConfig dgd;
    dgd.step = dgd_cfg.value("step", 0.6);
    dgd.max_iterations = dgd_cfg.value("max_iterations", std::size_t{4000});
    dgd.tolerance = dgd_cfg.value("tolerance", 1e-4);
    dgd.record_trajectory = true;
    dgd.rule = opt::StepRule::kDiminishing;
    const auto dgd_dim = opt::distributed_gradient_descent(problem, graph, x0, dgd);
    dgd.rule = opt::StepRule::kConstant;
    const auto dgd_const = opt::distributed_gradient_descent(problem, graph, x0, dgd);
    const Json& admm_cfg = cfg.at("admm");
    opt::AdmmConfig admm;
    admm.rho = admm_cfg.value("rho", 0.5);
    admm.max_iterations = admm_cfg.value("max_iterations", std::size_t{4000});
    admm.tolerance = admm_cfg.value("tolerance", 1e-4);
    admm.record_trajectory = true;
    const auto admm_run = opt::decentralized_admm(problem, graph, x0, admm);
    std::cout << "ADMM " << admm_run.iterations << " iterations (converged " << admm_run.converged
              << "), DGD diminishing " << dgd_dim.iterations << " (" << dgd_dim.converged
              << "), DGD constant " << dgd_const.iterations << " (" << dgd_const.converged << ")\n";

    Json bearings_json = Json::array();
    for (const double b : bearings) {
      bearings_json.push_back(io::round_to(b, 6));
    }
    Json result = mrc_apps::metadata("mrc_demo_distributed_optimization", cfg);
    result["field"] = io::to_json(field);
    result["robots"] = io::to_json(std::span<const mrc::Vec2>(robots));
    result["target"] = io::to_json(target);
    result["bearings"] = bearings_json;
    result["graph"] = io::to_json(graph);
    result["optimum"] = io::to_json(mrc::Vec2(optimum));
    result["admm"] = run_json(admm_run, record);
    result["dgd_diminishing"] = run_json(dgd_dim, record);
    result["dgd_constant"] = run_json(dgd_const, record);
    mrc_apps::write_result(args, result);
  });
}
