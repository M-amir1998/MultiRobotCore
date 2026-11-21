// Consensus experiments:
//   scaling     iterations / communication rounds / messages / runtime versus
//               team size for several topologies (synchronous, lossless)
//   robustness  message-passing consensus under 0/5/10/20% random loss with
//               skip and stale-value policies (rounds to agreement, bias)
//
// mrc-modules: consensus communication graph io

#include "demo_common.hpp"
#include "experiment_common.hpp"

#include "multirobotcore/communication/network.hpp"
#include "multirobotcore/communication/topology.hpp"
#include "multirobotcore/consensus/consensus.hpp"
#include "multirobotcore/consensus/distributed.hpp"
#include "multirobotcore/graph/connectivity.hpp"
#include "multirobotcore/graph/topologies.hpp"

#include <random>
#include <string>
#include <vector>

namespace mrc = multirobotcore;
namespace io = multirobotcore::io;
using mrc_apps::Json;
using mrc_apps::Sample;

namespace {

mrc::consensus::StateMatrix random_values(std::size_t n, std::mt19937_64& rng) {
  std::uniform_real_distribution<double> u(0.0, 10.0);
  mrc::consensus::StateMatrix x(static_cast<Eigen::Index>(n), 1);
  for (Eigen::Index i = 0; i < x.rows(); ++i) {
    x(i, 0) = u(rng);
  }
  return x;
}

/// Connected random geometric graph: n robots uniformly in a square whose
/// side keeps the expected degree roughly constant.
mrc::graph::Graph random_geometric(std::size_t n, std::mt19937_64& rng) {
  const double side = std::sqrt(static_cast<double>(n)) * 2.0;
  std::uniform_real_distribution<double> u(0.0, side);
  for (int attempt = 0; attempt < 1000; ++attempt) {
    std::vector<mrc::Vec2> p;
    for (std::size_t i = 0; i < n; ++i) {
      p.emplace_back(u(rng), u(rng));
    }
    auto g = mrc::communication::disk_graph(p, 3.2);
    if (mrc::graph::is_connected(g)) {
      return g;
    }
  }
  throw std::runtime_error("could not sample a connected geometric graph");
}

mrc::graph::Graph make_topology(const std::string& name, std::size_t n, std::mt19937_64& rng) {
  if (name == "ring") {
    return mrc::graph::ring_graph(n);
  }
  if (name == "path") {
    return mrc::graph::path_graph(n);
  }
  if (name == "star") {
    return mrc::graph::star_graph(n);
  }
  if (name == "complete") {
    return mrc::graph::complete_graph(n);
  }
  return random_geometric(n, rng);
}

}  // namespace

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args =
        mrc_apps::parse_args(argc, argv, "consensus_benchmark", "configs/consensus/benchmark.json");
    const Json cfg = mrc_apps::load_config(args);
    const auto seeds = cfg.value("seeds", std::size_t{5});
    const double tol = cfg.value("relative_tolerance", 1e-4);

    Json scaling = Json::array();
    for (const auto& topo : cfg.at("topologies")) {
      for (const auto& n_json : cfg.at("team_sizes")) {
        const auto n = n_json.get<std::size_t>();
        const std::string name = topo.get<std::string>();
        Sample iterations;
        Sample runtime;
        Sample lambda2;
        Sample messages;
        bool converged_all = true;
        for (std::size_t s = 0; s < seeds; ++s) {
          std::mt19937_64 rng(1000 + s);
          const auto g = make_topology(name, n, rng);
          const auto x0 = random_values(n, rng);
          const double eps = 0.9 * mrc::consensus::max_convex_step(g);
          const double range = x0.maxCoeff() - x0.minCoeff();
          mrc_apps::Stopwatch clock;
          const auto run = mrc::consensus::run_consensus(g, x0, {eps, 200000, tol * range, false});
          runtime.add(clock.elapsed_ms());
          converged_all = converged_all && run.converged;
          iterations.add(static_cast<double>(run.iterations));
          lambda2.add(mrc::graph::algebraic_connectivity(g));
          messages.add(static_cast<double>(run.iterations) * 2.0 *
                       static_cast<double>(g.num_edges()));
        }
        scaling.push_back(Json{{"topology", name},
                               {"robots", n},
                               {"converged", converged_all},
                               {"iterations", iterations.json(1)},
                               {"lambda2", lambda2.json(5)},
                               {"messages", messages.json(0)},
                               {"runtime_ms", runtime.json(4)}});
      }
    }

    Json robustness = Json::array();
    const auto robust = cfg.at("robustness");
    for (const auto& topo : robust.at("topologies")) {
      for (const auto& policy_name : {"skip", "last_received"}) {
        for (const auto& p_json : robust.at("drop_rates")) {
          const double p = p_json.get<double>();
          const auto n = robust.value("robots", std::size_t{20});
          Sample rounds;
          Sample bias;
          Sample delivered;
          std::size_t failures = 0;
          for (std::size_t s = 0; s < seeds; ++s) {
            std::mt19937_64 rng(2000 + s);
            const auto g = make_topology(topo.get<std::string>(), n, rng);
            const auto x0 = random_values(n, rng);
            const double range = x0.maxCoeff() - x0.minCoeff();
            mrc::communication::ChannelConfig channel;
            channel.drop_probability = p;
            mrc::communication::NetworkSimulator net(n, channel, 3000 + s);
            const auto policy = std::string(policy_name) == "skip"
                                    ? mrc::consensus::MissingPolicy::kSkip
                                    : mrc::consensus::MissingPolicy::kUseLastReceived;
            mrc::consensus::DistributedConsensus dc(
                g, x0, {0.9 * mrc::consensus::max_convex_step(g), 0.1, policy, 1});
            std::size_t k = 0;
            const std::size_t max_rounds = 20000;
            while (k < max_rounds && mrc::consensus::disagreement(dc.state()) > 1e-3 * range) {
              dc.round(net);
              ++k;
            }
            if (k >= max_rounds) {
              ++failures;
            }
            rounds.add(static_cast<double>(k));
            bias.add(
                std::abs(mrc::consensus::average(dc.state())(0) - mrc::consensus::average(x0)(0)) /
                range);
            delivered.add(net.stats().delivery_ratio());
          }
          robustness.push_back(Json{{"topology", topo.get<std::string>()},
                                    {"policy", policy_name},
                                    {"drop_rate", p},
                                    {"robots", n},
                                    {"rounds_to_agreement", rounds.json(1)},
                                    {"relative_bias", bias.json(5)},
                                    {"delivery_ratio", delivered.json(4)},
                                    {"failures", failures}});
        }
      }
    }
    Json result = mrc_apps::metadata("mrc_experiment_consensus_benchmark", cfg);
    result["scaling"] = scaling;
    result["robustness"] = robustness;
    mrc_apps::write_result(args, result);
  });
}
