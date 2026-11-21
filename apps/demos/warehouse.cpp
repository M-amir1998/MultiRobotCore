// Flagship integrated scenario: a multi-robot warehouse mission.
//
//   task requests -> Hungarian allocation on grid travel costs -> prioritized
//   planning / CBS -> timed waypoints -> tracking -> ORCA -> simulator
//
// Output: JSON consumed by scripts/render/render_warehouse.py.
//
// mrc-modules: scenarios io

#include "demo_common.hpp"

#include "multirobotcore/io/json.hpp"
#include "multirobotcore/scenarios/warehouse_mission.hpp"

#include <iostream>

namespace sc = multirobotcore::scenarios;
using mrc_apps::Json;

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args =
        mrc_apps::parse_args(argc, argv, "warehouse", "configs/warehouse/warehouse_mission.json");
    const Json cfg = mrc_apps::load_config(args);
    const auto config = sc::warehouse_mission_config_from_json(cfg);
    const auto result = sc::run_warehouse_mission(config);
    std::cout << "tasks " << result.tasks_completed << "/" << result.tasks.size()
              << " delivered, mean completion " << result.mean_completion_time << " s, "
              << result.replans << " replans (mean " << result.mean_planning_ms << " ms, max "
              << result.max_planning_ms << " ms), " << result.planning_failures
              << " planning failures, robot contacts " << result.metrics.robot_collisions
              << ", obstacle contacts " << result.metrics.obstacle_collisions
              << ", max tracking error " << result.max_tracking_error << " m\n";
    Json out = mrc_apps::metadata("mrc_demo_warehouse", cfg);
    out["mission"] = sc::to_json(result);
    mrc_apps::write_result(args, out);
  });
}
