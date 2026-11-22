#pragma once

/// @file formation_mission.hpp
/// Integrated formation scenario: dispersed robots gather into a formation
/// around a leader, travel along a path, contract into a narrow shape to pass
/// a passage between obstacles and restore the cruise shape afterwards.
///
/// Information flow: the leader broadcasts its pose and velocity through the
/// network simulator (range-limited, optionally lossy and delayed) and every
/// follower relays the freshest leader state it holds to its neighbours
/// (multi-hop flooding, one hop per control step). Followers extrapolate the
/// freshest state for at most `extrapolation_horizon` seconds. Relative positions of neighbours are
/// sensed locally (optionally with Gaussian noise). Followers run the
/// displacement-consensus formation law pinned to their leader estimate, plus
/// short-range separation and obstacle repulsion terms.
///
/// Failure injection: the leader can fail at a given time. Without
/// re-election the followers lose their reference and the mission stalls;
/// with re-election they run min-identifier flooding over the network and the
/// new leader continues along the path.

#include "multirobotcore/communication/network.hpp"
#include "multirobotcore/formation/shapes.hpp"
#include "multirobotcore/io/json.hpp"
#include "multirobotcore/math/geometry.hpp"
#include "multirobotcore/math/obstacles.hpp"
#include "multirobotcore/simulation/simulator.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace multirobotcore::scenarios {

struct FormationMissionConfig {
  std::size_t num_robots{7};
  double spacing{1.4};           ///< formation neighbour spacing [m]
  double robot_radius{0.25};     ///< [m]
  double max_speed{1.0};         ///< [m/s]
  double max_acceleration{2.0};  ///< [m/s^2]
  double leader_speed{0.6};      ///< [m/s]
  double dt{0.05};               ///< control step [s]
  double duration{70.0};         ///< [s]
  double gather_time{10.0};      ///< leader waits while the team gathers [s]
  std::uint64_t seed{11};
  math::AABB arena{{0.0, 0.0}, {40.0, 16.0}};
  math::AABB spawn{{1.0, 2.0}, {9.0, 14.0}};
  std::vector<Vec2> leader_path{{6.0, 8.0}, {36.0, 8.0}};
  formation::Shape cruise_shape{formation::Shape::kV};
  formation::Shape narrow_shape{formation::Shape::kColumn};
  math::ObstacleSet obstacles;
  double passage_x_min{20.0};     ///< passage entrance along x [m]
  double passage_x_max{24.0};     ///< passage exit along x [m]
  double contract_distance{6.0};  ///< contract when the leader is this far before the entrance [m]
  double restore_clearance{1.5};  ///< restore when every robot is this far past the exit [m]
  double transition_duration{5.0};      ///< [s]
  double transition_speed_factor{0.5};  ///< leader speed scale while reconfiguring
  double comm_radius{6.0};              ///< [m]
  double formation_gain{0.6};           ///< consensus coupling gain [1/s]
  double pinning_gain{1.0};             ///< leader tracking gain [1/s]
  double separation_gain{2.0};          ///< barrier gain [m^2/s]
  double safety_margin{0.3};            ///< separation acts below 2 r + margin [m]
  double obstacle_gain{0.3};
  double obstacle_influence{0.6};             ///< [m]
  double extrapolation_horizon{1.0};          ///< [s]
  std::optional<double> leader_failure_time;  ///< [s]
  bool reelect_leader{true};
  double leader_timeout{0.5};  ///< silence that triggers an election [s]
  double drop_probability{0.0};
  double latency{0.0};            ///< [s]
  double measurement_noise{0.0};  ///< std. dev. of position estimates [m]
  std::size_t record_every{4};
};

struct MissionEvent {
  double time{0.0};
  std::string label;
};

struct FormationMissionResult {
  std::vector<simulation::Frame> frames;
  std::vector<std::vector<std::array<std::size_t, 2>>> edges;  ///< per recorded frame
  std::vector<std::vector<Vec2>> slots;  ///< desired world slot per robot, per recorded frame
  std::vector<std::size_t> leader;       ///< leader index per recorded frame
  std::vector<double> rmse;              ///< slot RMSE of operational robots per recorded frame
  std::vector<MissionEvent> events;
  simulation::SimulationMetrics metrics;
  communication::NetworkStats network;
  double mean_rmse{0.0};          ///< after the gathering phase [m]
  double max_rmse{0.0};           ///< after the gathering phase [m]
  double final_rmse{0.0};         ///< [m]
  double convergence_time{-1.0};  ///< first time the RMSE drops below 0.15 m [s]
  double path_progress{0.0};      ///< fraction of the leader path completed
  bool stalled{false};
  std::size_t final_leader{0};
};

[[nodiscard]] FormationMissionResult run_formation_mission(const FormationMissionConfig& cfg);

/// Reads a configuration; absent keys keep their defaults.
[[nodiscard]] FormationMissionConfig formation_mission_config_from_json(const io::Json& json);

[[nodiscard]] io::Json to_json(const FormationMissionResult& result);

/// Fraction of a polyline's length covered by the projection of `point` onto it.
[[nodiscard]] double polyline_progress(const std::vector<Vec2>& path, const Vec2& point);

}  // namespace multirobotcore::scenarios
