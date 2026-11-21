// Coverage scalability and limited-range study.
//
//  1. Scalability: Lloyd iterations on a 20 m x 20 m domain with a two-bump
//     density for N in {5, 10, 20, 50, 100} robots and several seeds. The
//     tolerance is 1 % of the nominal cell size 20 / sqrt(N).
//  2. Limited range: N = 50 robots run the limited-range controller for
//     several communication radii (multiples of the nominal spacing) and are
//     compared with the centralised partition.
//
// mrc-modules: coverage io

#include "demo_common.hpp"

#include "multirobotcore/coverage/coverage_control.hpp"
#include "multirobotcore/io/json.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace mrc = multirobotcore;
namespace cov = multirobotcore::coverage;
namespace io = multirobotcore::io;
using mrc_apps::Json;

namespace {

constexpr double kSide = 20.0;

cov::DensityFunction benchmark_density() {
  return cov::DensityFunction::gaussian_mixture({{{14.0, 13.0}, 2.5, 5.0}, {{5.0, 6.0}, 1.8, 3.0}},
                                                0.2);
}

std::vector<mrc::Vec2> uniform_positions(std::size_t n, std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> u(0.0, kSide);
  std::vector<mrc::Vec2> p;
  for (std::size_t i = 0; i < n; ++i) {
    p.emplace_back(u(rng), u(rng));
  }
  return p;
}

}  // namespace

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args =
        mrc_apps::parse_args(argc, argv, "coverage_benchmark", "configs/coverage/benchmark.json");
    const Json cfg = mrc_apps::load_config(args);
    const auto domain = mrc::math::ConvexPolygon::from_box({{0.0, 0.0}, {kSide, kSide}});
    const auto density = benchmark_density();
    const auto max_iterations = cfg.value("max_iterations", 300);
    const auto seeds = cfg.value("seeds", std::vector<std::uint64_t>{1, 2, 3});

    Json scalability = Json::array();
    for (const auto n : cfg.value("robot_counts", std::vector<std::size_t>{5, 10, 20, 50, 100})) {
      const double tolerance = 0.01 * kSide / std::sqrt(static_cast<double>(n));
      for (const auto seed : seeds) {
        const cov::CoverageController controller(domain, density, {1.0, 1.0, 4, {}});
        auto p = uniform_positions(n, seed);
        const double initial_cost = controller.evaluate(p).cost;
        int iterations = 0;
        bool converged = false;
        mrc_apps::Stopwatch watch;
        cov::CoverageState state;
        while (iterations < max_iterations) {
          p = controller.lloyd_step(p, &state);
          ++iterations;
          if (state.max_centroid_error < tolerance) {
            converged = true;
            break;
          }
        }
        const double elapsed = watch.elapsed_ms();
        const auto metrics = cov::evaluate_metrics(p, domain, density, 150);
        scalability.push_back(
            Json{{"robots", n},
                 {"seed", seed},
                 {"iterations", iterations},
                 {"converged", converged},
                 {"tolerance_m", io::round_to(tolerance, 4)},
                 {"ms_per_iteration", io::round_to(elapsed / iterations, 4)},
                 {"initial_cost", io::round_to(initial_cost, 3)},
                 {"final_cost", io::round_to(metrics.cost, 3)},
                 {"mean_distance_m", io::round_to(metrics.mean_distance, 4)},
                 {"max_uncovered_m", io::round_to(metrics.max_uncovered_distance, 4)},
                 {"min_separation_m", io::round_to(metrics.min_separation, 4)}});
      }
    }

    // Limited-range study.
    Json limited = Json::array();
    const auto n_limited = cfg.value("limited_range_robots", std::size_t{50});
    const double spacing = kSide / std::sqrt(static_cast<double>(n_limited));
    const auto iterations_limited = cfg.value("limited_range_iterations", 200);
    for (const auto seed : seeds) {
      // Centralised reference from the same start.
      const cov::CoverageController central(domain, density, {1.0, 1.0, 4, {}});
      auto ref = uniform_positions(n_limited, seed);
      for (int k = 0; k < iterations_limited; ++k) {
        ref = central.lloyd_step(ref);
      }
      const double ref_cost = central.evaluate(ref).cost;
      for (const double factor :
           cfg.value("radius_factors", std::vector<double>{1.0, 1.5, 2.0, 3.0})) {
        const double radius = factor * spacing;
        const cov::CoverageController controller(domain, density, {1.0, 1.0, 4, radius});
        auto p = uniform_positions(n_limited, seed);
        std::size_t messages = 0;
        cov::CoverageState state;
        for (int k = 0; k < iterations_limited; ++k) {
          p = controller.lloyd_step(p, &state);
          messages += 2 * state.communicating_pairs;
        }
        const auto final_state = controller.controller_state(p);
        const auto exact = central.evaluate(p);
        const auto certified = static_cast<std::size_t>(std::count(
            final_state.certified_exact.begin(), final_state.certified_exact.end(), true));
        limited.push_back(Json{
            {"seed", seed},
            {"radius_factor", factor},
            {"radius_m", io::round_to(radius, 3)},
            {"certified_fraction", io::round_to(static_cast<double>(certified) / n_limited, 4)},
            {"final_cost", io::round_to(exact.cost, 3)},
            {"centralized_cost", io::round_to(ref_cost, 3)},
            {"cost_ratio", io::round_to(exact.cost / ref_cost, 5)},
            {"exact_centroid_error_m", io::round_to(exact.max_centroid_error, 4)},
            {"messages_per_iteration",
             io::round_to(static_cast<double>(messages) / iterations_limited, 2)}});
      }
    }

    Json result = mrc_apps::metadata("mrc_experiment_coverage_benchmark", cfg);
    result["domain_side_m"] = kSide;
    result["scalability"] = scalability;
    result["limited_range"] = Json{{"robots", n_limited},
                                   {"nominal_spacing_m", io::round_to(spacing, 4)},
                                   {"iterations", iterations_limited},
                                   {"runs", limited}};
    mrc_apps::write_result(args, result);
  });
}
