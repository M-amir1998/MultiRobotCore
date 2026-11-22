// Consensus demonstrations: scalar average consensus, communication failure
// and recovery through the network simulator, and planar rendezvous.
//
// Output: JSON consumed by scripts/render/render_consensus.py.
//
// mrc-modules: consensus simulation io

#include "demo_common.hpp"

#include "multirobotcore/communication/network.hpp"
#include "multirobotcore/communication/topology.hpp"
#include "multirobotcore/consensus/consensus.hpp"
#include "multirobotcore/consensus/distributed.hpp"
#include "multirobotcore/graph/connectivity.hpp"
#include "multirobotcore/graph/topologies.hpp"
#include "multirobotcore/io/json.hpp"
#include "multirobotcore/math/numerics.hpp"
#include "multirobotcore/math/random.hpp"
#include "multirobotcore/simulation/simulator.hpp"

#include <cmath>
#include <random>
#include <vector>

namespace mrc = multirobotcore;
namespace io = multirobotcore::io;
using mrc_apps::Json;

namespace {

Json values_json(const mrc::consensus::StateMatrix& x) {
  Json row = Json::array();
  for (Eigen::Index i = 0; i < x.rows(); ++i) {
    row.push_back(io::round_to(x(i, 0), 5));
  }
  return row;
}

Json circle_layout(std::size_t n) {
  std::vector<mrc::Vec2> p;
  for (std::size_t i = 0; i < n; ++i) {
    const double a = mrc::math::kPi / 2.0 -
                     2.0 * mrc::math::kPi * static_cast<double>(i) / static_cast<double>(n);
    p.emplace_back(std::cos(a), std::sin(a));
  }
  return io::to_json(p);
}

mrc::graph::Graph ring_with_chords(std::size_t n, std::size_t extra, std::mt19937_64& rng) {
  mrc::graph::Graph g = mrc::graph::ring_graph(n);
  std::uniform_int_distribution<std::size_t> pick(0, n - 1);
  std::size_t added = 0;
  while (added < extra) {
    const std::size_t a = pick(rng);
    const std::size_t b = pick(rng);
    if (a != b && !g.has_edge(a, b)) {
      g.add_edge(a, b);
      ++added;
    }
  }
  return g;
}

mrc::consensus::StateMatrix random_values(std::size_t n, double lo, double hi,
                                          std::mt19937_64& rng) {
  std::uniform_real_distribution<double> u(lo, hi);
  mrc::consensus::StateMatrix x(static_cast<Eigen::Index>(n), 1);
  for (Eigen::Index i = 0; i < x.rows(); ++i) {
    x(i, 0) = u(rng);
  }
  return x;
}

Json scalar_scenario(const Json& cfg, std::mt19937_64& rng) {
  const auto n = cfg.value("num_agents", std::size_t{12});
  const auto graph = ring_with_chords(n, cfg.value("extra_edges", std::size_t{4}), rng);
  const auto range = cfg.value("value_range", std::vector<double>{0.0, 10.0});
  const auto x0 = random_values(n, range.at(0), range.at(1), rng);
  const double eps = cfg.value("step_fraction", 0.8) * mrc::consensus::max_convex_step(graph);
  const auto iterations = cfg.value("iterations", std::size_t{60});
  const auto run = mrc::consensus::run_consensus(graph, x0, {eps, iterations, 0.0, true});

  Json history = Json::array();
  for (const auto& x : run.history) {
    history.push_back(values_json(x));
  }
  Json disagreement = Json::array();
  for (const double d : run.disagreement) {
    disagreement.push_back(io::round_to(d, 8));
  }
  return Json{
      {"graph", io::to_json(graph)},
      {"layout", circle_layout(n)},
      {"epsilon", io::round_to(eps, 6)},
      {"algebraic_connectivity", io::round_to(mrc::graph::algebraic_connectivity(graph), 6)},
      {"contraction_factor",
       io::round_to(mrc::consensus::discrete_contraction_factor(graph, eps), 6)},
      {"average", io::round_to(mrc::consensus::average(x0)(0), 6)},
      {"values", history},
      {"disagreement", disagreement}};
}

Json partition_scenario(const Json& cfg, std::uint64_t seed, std::mt19937_64& rng) {
  const auto n = cfg.value("num_agents", std::size_t{12});
  const mrc::graph::Graph graph = mrc::graph::ring_graph(n);
  // The two halves that the outages separate start in disjoint value ranges,
  // so separate agreement during the partition is visible.
  const auto low = cfg.value("low_range", std::vector<double>{0.0, 4.0});
  const auto high = cfg.value("high_range", std::vector<double>{6.0, 10.0});
  mrc::consensus::StateMatrix x0(static_cast<Eigen::Index>(n), 1);
  const auto lo_vals = random_values(n / 2, low.at(0), low.at(1), rng);
  const auto hi_vals = random_values(n - n / 2, high.at(0), high.at(1), rng);
  x0 << lo_vals, hi_vals;
  const double period = cfg.value("round_period", 0.1);
  const auto rounds = cfg.value("rounds", std::size_t{110});
  const auto cut = cfg.value("cut_round", std::size_t{12});
  const auto restore = cfg.value("restore_round", std::size_t{60});
  const double eps = cfg.value("step_fraction", 0.8) * mrc::consensus::max_convex_step(graph);

  mrc::communication::ChannelConfig channel;
  channel.drop_probability = cfg.value("drop_probability", 0.05);
  mrc::communication::NetworkSimulator network(n, channel, seed);
  Json cut_links = Json::array();
  for (const auto& link : cfg.at("cut_links")) {
    const auto a = link.at(0).get<std::size_t>();
    const auto b = link.at(1).get<std::size_t>();
    network.add_link_outage(
        {a, b, static_cast<double>(cut) * period, static_cast<double>(restore) * period});
    cut_links.push_back(Json::array({a, b}));
  }
  mrc::consensus::DistributedConsensus protocol(
      graph, x0, {eps, period, mrc::consensus::MissingPolicy::kSkip, 1});
  Json history = Json::array({values_json(x0)});
  for (std::size_t k = 0; k < rounds; ++k) {
    history.push_back(values_json(protocol.round(network)));
  }
  return Json{{"graph", io::to_json(graph)},
              {"layout", circle_layout(n)},
              {"epsilon", io::round_to(eps, 6)},
              {"cut_round", cut},
              {"restore_round", restore},
              {"cut_links", cut_links},
              {"average", io::round_to(mrc::consensus::average(x0)(0), 6)},
              {"values", history},
              {"network", io::to_json(network.stats())}};
}

std::vector<mrc::Pose2> connected_start(std::size_t n, double arena, double radius,
                                        double min_spacing, std::mt19937_64& rng) {
  // Draw initial positions until the disk graph is connected (deterministic
  // given the seed): linear rendezvous only gathers a connected group.
  // Robots must also start without overlapping.
  std::uniform_real_distribution<double> u(0.0, arena);
  std::vector<mrc::Pose2> poses;
  for (std::size_t attempt = 0; attempt < 1000; ++attempt) {
    poses.clear();
    std::vector<mrc::Vec2> p;
    for (std::size_t i = 0; i < n; ++i) {
      poses.push_back({mrc::math::sample_vec2(rng, u), 0.0});
      p.push_back(poses.back().position);
    }
    bool separated = true;
    for (std::size_t i = 0; i < n && separated; ++i) {
      for (std::size_t j = i + 1; j < n; ++j) {
        separated = separated && (p[i] - p[j]).norm() >= min_spacing;
      }
    }
    if (separated && mrc::graph::is_connected(mrc::communication::disk_graph(p, radius))) {
      break;
    }
  }
  return poses;
}

double spread_of(const std::vector<mrc::Vec2>& p) {
  mrc::consensus::StateMatrix x(static_cast<Eigen::Index>(p.size()), 2);
  for (std::size_t i = 0; i < p.size(); ++i) {
    x.row(static_cast<Eigen::Index>(i)) = p[i].transpose();
  }
  return mrc::consensus::disagreement(x);
}

Json run_rendezvous(const Json& cfg, const std::vector<mrc::Pose2>& poses, bool separation) {
  const std::size_t n = poses.size();
  const double arena = cfg.value("arena", 12.0);
  const double radius = cfg.value("comm_radius", 4.6);
  const double gain = cfg.value("gain", 0.5);
  const double vmax = cfg.value("max_speed", 0.8);

  mrc::simulation::WorldConfig world;
  world.dt = cfg.value("dt", 0.05);
  world.record_every = cfg.value("record_every", std::size_t{2});
  world.bounds = mrc::math::AABB{{0.0, 0.0}, {arena, arena}};
  mrc::simulation::RobotSpec spec;
  spec.radius = cfg.value("robot_radius", 0.18);
  spec.max_speed = vmax;
  mrc::simulation::Simulator sim(world, std::vector<mrc::simulation::RobotSpec>(n, spec), poses);
  const double safety = 2.0 * spec.radius + cfg.value("safety_margin", 0.12);
  const double repulsion_gain = cfg.value("repulsion_gain", 2.0);

  const auto steps = static_cast<std::size_t>(std::round(cfg.value("duration", 16.0) / world.dt));
  Json edges_per_frame = Json::array();
  Json spread = Json::array();
  auto record = [&] {
    Json e = Json::array();
    for (const auto& edge : mrc::communication::disk_graph(sim.positions(), radius).edges()) {
      e.push_back(Json::array({edge.source, edge.target}));
    }
    edges_per_frame.push_back(std::move(e));
    spread.push_back(io::round_to(spread_of(sim.positions()), 5));
  };
  record();
  bool stayed_connected = true;
  double convergence_time = -1.0;
  for (std::size_t k = 0; k < steps; ++k) {
    const auto positions = sim.positions();
    const auto g = mrc::communication::disk_graph(positions, radius);
    stayed_connected = stayed_connected && mrc::graph::is_connected(g);
    const auto u_cmd = separation ? mrc::consensus::rendezvous_with_separation_velocities(
                                        g, positions, gain, safety, repulsion_gain, vmax)
                                  : mrc::consensus::rendezvous_velocities(g, positions, gain, vmax);
    sim.step(u_cmd);
    if (convergence_time < 0.0 && spread_of(sim.positions()) < cfg.value("converged_spread", 1.0)) {
      convergence_time = sim.time();
    }
    if (sim.step_count() % world.record_every == 0) {
      record();
    }
  }
  Json out{{"variant", separation ? "consensus_with_separation" : "pure_consensus"},
           {"stayed_connected", stayed_connected},
           {"convergence_time", io::round_to(convergence_time, 3)},
           {"final_spread", io::round_to(spread_of(sim.positions()), 5)},
           {"metrics", io::to_json(sim.metrics())}};
  if (separation) {
    out["trajectory"] = io::frames_to_json(sim.frames());
    out["edges"] = edges_per_frame;
    out["spread"] = spread;
  }
  return out;
}

Json rendezvous_scenario(const Json& cfg, std::mt19937_64& rng) {
  const auto n = cfg.value("num_robots", std::size_t{12});
  const double arena = cfg.value("arena", 12.0);
  const double radius = cfg.value("comm_radius", 4.6);
  const auto poses = connected_start(n, arena, radius, cfg.value("min_initial_spacing", 1.0), rng);
  return Json{{"arena", arena},
              {"comm_radius", radius},
              {"robot_radius", cfg.value("robot_radius", 0.18)},
              {"pure", run_rendezvous(cfg, poses, false)},
              {"separation", run_rendezvous(cfg, poses, true)}};
}

}  // namespace

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args =
        mrc_apps::parse_args(argc, argv, "consensus", "configs/consensus/consensus.json");
    const Json cfg = mrc_apps::load_config(args);
    const auto seed = cfg.value("seed", std::uint64_t{7});
    std::mt19937_64 rng(seed);
    Json result = mrc_apps::metadata("mrc_demo_consensus", cfg);
    result["scalar"] = scalar_scenario(cfg.at("scalar"), rng);
    result["partition"] = partition_scenario(cfg.at("partition"), seed, rng);
    result["rendezvous"] = rendezvous_scenario(cfg.at("rendezvous"), rng);
    mrc_apps::write_result(args, result);
  });
}
