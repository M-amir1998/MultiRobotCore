// Voronoi coverage demonstration: robots deployed in a corner spread over a
// non-uniform density with the gradient controller p_i' = -k (p_i - C_i),
// executed in the simulator. A limited-range (distributed) variant runs from
// the same initial positions for comparison.
//
// Output: JSON consumed by scripts/render/render_voronoi_coverage.py.
//
// mrc-modules: coverage simulation io

#include "demo_common.hpp"

#include "multirobotcore/coverage/coverage_control.hpp"
#include "multirobotcore/io/json.hpp"
#include "multirobotcore/math/random.hpp"
#include "multirobotcore/simulation/simulator.hpp"

#include <cmath>
#include <random>
#include <vector>

namespace mrc = multirobotcore;
namespace cov = multirobotcore::coverage;
namespace io = multirobotcore::io;
using mrc_apps::Json;

namespace {

cov::DensityFunction density_from_json(const Json& cfg) {
  std::vector<cov::GaussianBump> bumps;
  for (const auto& b : cfg.at("bumps")) {
    bumps.push_back({io::vec2_from_json(b.at("mean")), b.at("sigma").get<double>(),
                     b.at("weight").get<double>()});
  }
  return cov::DensityFunction::gaussian_mixture(std::move(bumps), cfg.value("floor", 0.1));
}

std::vector<mrc::Vec2> deploy(const Json& cfg, std::mt19937_64& rng) {
  const auto box = cfg.at("deploy_box");
  const mrc::Vec2 lo = io::vec2_from_json(box.at(0));
  const mrc::Vec2 hi = io::vec2_from_json(box.at(1));
  const auto n = cfg.value("num_robots", std::size_t{16});
  const double spacing = cfg.value("min_initial_spacing", 0.45);
  std::uniform_real_distribution<double> ux(lo.x(), hi.x());
  std::uniform_real_distribution<double> uy(lo.y(), hi.y());
  std::vector<mrc::Vec2> p;
  std::size_t attempts = 0;
  while (p.size() < n && attempts < 100000) {
    ++attempts;
    const mrc::Vec2 q = mrc::math::sample_vec2(rng, ux, uy);
    bool ok = true;
    for (const auto& other : p) {
      ok = ok && (q - other).norm() >= spacing;
    }
    if (ok) {
      p.push_back(q);
    }
  }
  if (p.size() < n) {
    throw std::runtime_error("cannot place robots with the requested spacing");
  }
  return p;
}

Json polygon_json(const mrc::math::ConvexPolygon& poly) {
  return io::to_json(std::span<const mrc::Vec2>(poly.vertices()), 3);
}

Json metrics_json(const cov::CoverageMetrics& m) {
  return Json{{"cost", io::round_to(m.cost, 5)},
              {"mean_distance", io::round_to(m.mean_distance, 5)},
              {"max_uncovered_distance", io::round_to(m.max_uncovered_distance, 5)},
              {"max_centroid_error", io::round_to(m.max_centroid_error, 5)},
              {"min_separation", io::round_to(m.min_separation, 5)}};
}

struct RunOutput {
  Json json;
  double settling_time{-1.0};
};

RunOutput run(const Json& cfg, const mrc::math::ConvexPolygon& domain,
              const cov::DensityFunction& density, const std::vector<mrc::Vec2>& start,
              std::optional<double> radius, bool record_cells) {
  const auto& ctrl = cfg.at("controller");
  const cov::CoverageController controller(domain, density,
                                           {ctrl.value("gain", 1.0), ctrl.value("max_speed", 0.8),
                                            ctrl.value("subdivisions", 4), radius});
  const auto& sim_cfg = cfg.at("simulation");
  mrc::simulation::WorldConfig world;
  world.dt = sim_cfg.value("dt", 0.05);
  world.record_every = sim_cfg.value("record_every", std::size_t{4});
  world.bounds = domain.bounding_box();
  mrc::simulation::RobotSpec spec;
  spec.radius = sim_cfg.value("robot_radius", 0.15);
  spec.max_speed = ctrl.value("max_speed", 0.8);
  std::vector<mrc::Pose2> poses;
  for (const auto& p : start) {
    poses.push_back({p, 0.0});
  }
  mrc::simulation::Simulator sim(world, std::vector<mrc::simulation::RobotSpec>(start.size(), spec),
                                 poses);
  const auto steps =
      static_cast<std::size_t>(std::round(sim_cfg.value("duration", 30.0) / world.dt));
  const double tolerance = cfg.value("convergence_tolerance", 0.05);

  Json cells = Json::array();
  Json centroids = Json::array();
  Json cost = Json::array();
  Json centroid_error = Json::array();
  Json certified = Json::array();
  RunOutput out;
  std::size_t total_pairs = 0;
  auto record = [&](const cov::CoverageState& state) {
    cost.push_back(io::round_to(state.cost, 5));
    centroid_error.push_back(io::round_to(state.max_centroid_error, 5));
    if (radius) {
      std::size_t ok = 0;
      for (const bool b : state.certified_exact) {
        ok += b ? 1U : 0U;
      }
      certified.push_back(ok);
    }
    if (record_cells) {
      Json frame_cells = Json::array();
      Json frame_centroids = Json::array();
      for (std::size_t i = 0; i < state.cells.size(); ++i) {
        frame_cells.push_back(polygon_json(state.cells[i].polygon));
        frame_centroids.push_back(io::to_json(state.integrals[i].centroid, 3));
      }
      cells.push_back(std::move(frame_cells));
      centroids.push_back(std::move(frame_centroids));
    }
  };
  for (std::size_t k = 0; k <= steps; ++k) {
    cov::CoverageState state;
    const auto positions = sim.positions();
    const auto u = controller.velocities(positions, &state);
    total_pairs += state.communicating_pairs;
    // Settling time: the error stays below the tolerance from here to the end.
    if (state.max_centroid_error >= tolerance) {
      out.settling_time = -1.0;
    } else if (out.settling_time < 0.0) {
      out.settling_time = sim.time();
    }
    if (sim.step_count() % world.record_every == 0) {
      record(state);
    }
    if (k < steps) {
      sim.step(u);
    }
  }
  // A settling time must leave a confirmation window before the horizon ends.
  if (out.settling_time >= 0.0 &&
      sim.time() - out.settling_time < cfg.value("settle_window", 2.0)) {
    out.settling_time = -1.0;
  }
  const auto final_positions = sim.positions();
  out.json =
      Json{{"communication_radius", radius ? Json(*radius) : Json(nullptr)},
           {"settling_time", io::round_to(out.settling_time, 3)},
           {"cost", cost},
           {"max_centroid_error", centroid_error},
           {"initial_metrics", metrics_json(cov::evaluate_metrics(start, domain, density))},
           {"final_metrics", metrics_json(cov::evaluate_metrics(final_positions, domain, density))},
           {"simulation", io::to_json(sim.metrics())}};
  if (radius) {
    out.json["certified_exact_cells"] = certified;
    out.json["position_messages"] = 2 * total_pairs;  // each in-range pair exchanges both ways
  }
  if (record_cells) {
    out.json["trajectory"] = io::frames_to_json(sim.frames(), 3);
    out.json["cells"] = cells;
    out.json["centroids"] = centroids;
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args =
        mrc_apps::parse_args(argc, argv, "coverage", "configs/coverage/coverage.json");
    const Json cfg = mrc_apps::load_config(args);
    std::mt19937_64 rng(cfg.value("seed", std::uint64_t{3}));

    const mrc::math::ConvexPolygon domain(io::vec2_list_from_json(cfg.at("domain")));
    const cov::DensityFunction density = density_from_json(cfg.at("density"));
    const auto start = deploy(cfg, rng);

    // Density samples on a regular grid for the heatmap.
    const auto grid = cfg.value("density_grid", std::vector<int>{120, 90});
    const auto box = domain.bounding_box();
    Json density_values = Json::array();
    for (int r = 0; r < grid.at(1); ++r) {
      Json row = Json::array();
      for (int c = 0; c < grid.at(0); ++c) {
        const mrc::Vec2 q(box.min.x() + (c + 0.5) * box.width() / grid.at(0),
                          box.min.y() + (r + 0.5) * box.height() / grid.at(1));
        row.push_back(io::round_to(density(q), 3));
      }
      density_values.push_back(std::move(row));
    }

    Json result = mrc_apps::metadata("mrc_demo_coverage", cfg);
    result["domain"] = polygon_json(domain);
    result["density_grid"] =
        Json{{"extent", Json::array({box.min.x(), box.max.x(), box.min.y(), box.max.y()})},
             {"values", density_values}};
    result["robot_radius"] = cfg.at("simulation").value("robot_radius", 0.15);
    result["centralized"] = run(cfg, domain, density, start, std::nullopt, true).json;
    result["distributed"] = run(cfg, domain, density, start,
                                cfg.at("distributed").value("communication_radius", 3.0), true)
                                .json;
    mrc_apps::write_result(args, result);
  });
}
