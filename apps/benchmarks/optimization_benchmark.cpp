// Distributed optimisation experiments: DGD (diminishing step) vs.
// decentralised ADMM on cooperative bearing localisation, versus team size and
// topology. Metrics: iterations (= communication rounds) to a relative
// solution error, messages, runtime, final consensus error.
//
// mrc-modules: optimization communication io

#include "demo_common.hpp"
#include "experiment_common.hpp"

#include "multirobotcore/communication/topology.hpp"
#include "multirobotcore/graph/connectivity.hpp"
#include "multirobotcore/graph/topologies.hpp"
#include "multirobotcore/math/random.hpp"
#include "multirobotcore/optimization/distributed.hpp"

#include <random>
#include <string>
#include <vector>

namespace mrc = multirobotcore;
namespace opt = multirobotcore::optimization;
using mrc_apps::Json;
using mrc_apps::Sample;

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args = mrc_apps::parse_args(argc, argv, "optimization_benchmark",
                                           "configs/optimization/distributed_optimization.json");
    const Json root = mrc_apps::load_config(args);
    const Json& cfg = root.at("benchmark");
    const auto seeds = cfg.value("seeds", std::size_t{5});
    const double tol = cfg.value("tolerance", 1e-4);
    Json rows = Json::array();
    for (const auto& topo_json : cfg.at("topologies")) {
      const std::string topo = topo_json.get<std::string>();
      for (const auto& n_json : cfg.at("team_sizes")) {
        const auto n = n_json.get<std::size_t>();
        for (const std::string method : {"dgd", "admm"}) {
          Sample iters;
          Sample msgs;
          Sample runtime;
          Sample consensus;
          Sample final_error;
          std::size_t converged = 0;
          for (std::size_t s = 0; s < seeds; ++s) {
            std::mt19937_64 rng(100 + s);
            const double side = 4.0 * std::sqrt(static_cast<double>(n));
            std::uniform_real_distribution<double> u(0.0, side);
            const mrc::Vec2 target(0.5 * side, 0.5 * side);
            std::vector<mrc::Vec2> robots;
            mrc::graph::Graph g;
            for (int attempt = 0; attempt < 10000; ++attempt) {
              robots.clear();
              for (std::size_t i = 0; i < n; ++i) {
                robots.push_back(mrc::math::sample_vec2(rng, u));
              }
              g = topo == "ring" ? mrc::graph::ring_graph(n)
                                 : mrc::communication::disk_graph(robots, 7.0);
              if (mrc::graph::is_connected(g)) {
                break;
              }
            }
            const auto problem = opt::bearing_localization_problem(
                robots, opt::measure_bearings(robots, target, 0.03, rng));
            std::vector<Eigen::VectorXd> x0;
            for (const auto& p : robots) {
              x0.emplace_back(p);
            }
            mrc_apps::Stopwatch clock;
            opt::DistributedRun run;
            if (method == "dgd") {
              opt::GradientDescentConfig c;
              c.step = 0.6;
              c.rule = opt::StepRule::kDiminishing;
              c.max_iterations = 20000;
              c.tolerance = tol;
              run = opt::distributed_gradient_descent(problem, g, x0, c);
            } else {
              opt::AdmmConfig c;
              c.rho = 0.5;
              c.max_iterations = 20000;
              c.tolerance = tol;
              run = opt::decentralized_admm(problem, g, x0, c);
            }
            runtime.add(clock.elapsed_ms());
            iters.add(static_cast<double>(run.iterations));
            msgs.add(static_cast<double>(run.messages));
            consensus.add(run.history.back().consensus_error);
            final_error.add(run.history.back().solution_error);
            converged += run.converged ? 1 : 0;
          }
          rows.push_back(Json{{"method", method},
                              {"topology", topo},
                              {"robots", n},
                              {"iterations", iters.json(1)},
                              {"messages", msgs.json(0)},
                              {"runtime_ms", runtime.json(4)},
                              {"final_consensus_error", consensus.json(8)},
                              {"final_solution_error", final_error.json(8)},
                              {"converged_runs", converged},
                              {"runs", seeds}});
        }
      }
    }
    Json result = mrc_apps::metadata("mrc_experiment_optimization_benchmark", root);
    result["rows"] = rows;
    mrc_apps::write_result(args, result);
  });
}
