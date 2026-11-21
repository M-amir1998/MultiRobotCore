#include "multirobotcore/models/robot_state.hpp"

namespace multirobotcore::models {

std::string_view to_string(RobotStatus status) noexcept {
  switch (status) {
    case RobotStatus::kIdle:
      return "idle";
    case RobotStatus::kActive:
      return "active";
    case RobotStatus::kFinished:
      return "finished";
    case RobotStatus::kStopped:
      return "stopped";
    case RobotStatus::kFailed:
      return "failed";
  }
  return "unknown";
}

}  // namespace multirobotcore::models
