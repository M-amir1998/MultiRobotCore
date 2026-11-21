#pragma once

/// @file robot_state.hpp
/// Typed multi-robot state. Robots are identified by dense indices 0..N-1;
/// every per-robot container in the library uses the same indexing.

#include "multirobotcore/math/types.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

namespace multirobotcore::models {

using RobotId = std::size_t;

enum class RobotStatus : std::uint8_t {
  kIdle,      ///< available, no goal
  kActive,    ///< executing a goal or task
  kFinished,  ///< reached its final goal
  kStopped,   ///< halted (e.g. injected failure); still occupies space
  kFailed,    ///< removed from the team (communication and motion)
};

[[nodiscard]] std::string_view to_string(RobotStatus status) noexcept;

/// True when the robot can move and communicate.
[[nodiscard]] constexpr bool is_operational(RobotStatus status) noexcept {
  return status == RobotStatus::kIdle || status == RobotStatus::kActive ||
         status == RobotStatus::kFinished;
}

struct RobotState {
  RobotId id{0};
  Vec2 position{Vec2::Zero()};      ///< [m]
  double heading{0.0};              ///< [rad]
  Vec2 velocity{Vec2::Zero()};      ///< [m/s]
  std::optional<Vec2> goal;         ///< current navigation goal [m]
  std::optional<std::size_t> task;  ///< index of the assigned task
  std::vector<RobotId> neighbors;   ///< current communication neighbours
  RobotStatus status{RobotStatus::kIdle};
};

}  // namespace multirobotcore::models
