#pragma once

/// @file events.hpp
/// Deterministic failure injection and scenario events.
///
/// Events are scheduled at absolute simulation times and released by
/// EventSchedule::pop_due() in (time, insertion) order. apply_event() applies
/// the physical and communication effects of an event; task events are
/// returned to the scenario, which owns the task model.

#include "multirobotcore/math/types.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>
#include <vector>

namespace multirobotcore::communication {
class NetworkSimulator;
}

namespace multirobotcore::simulation {

class Simulator;

enum class EventType : std::uint8_t {
  kRobotStop,        ///< robot halts but keeps communicating
  kRobotFailure,     ///< robot halts and stops communicating
  kRobotRecover,     ///< robot resumes (status active)
  kLeaderLoss,       ///< the designated leader fails (same effect as kRobotFailure)
  kLinkFailure,      ///< link {robot, other} down for `duration` seconds
  kObstacleAppears,  ///< disc obstacle at `position` with `radius`
  kTaskArrival,      ///< new task `task` at `position` (scenario-handled)
  kTaskCancelled,    ///< task `task` cancelled (scenario-handled)
};

[[nodiscard]] std::string_view to_string(EventType type) noexcept;

struct ScheduledEvent {
  double time{0.0};  ///< [s]
  EventType type{EventType::kRobotStop};
  std::size_t robot{0};
  std::size_t other{0};
  double duration{std::numeric_limits<double>::infinity()};  ///< [s]
  std::size_t task{0};
  Vec2 position{Vec2::Zero()};
  double radius{0.0};  ///< [m]
};

class EventSchedule {
 public:
  void add(const ScheduledEvent& event);

  /// Removes and returns every event with time <= now, ordered by time and
  /// then insertion order.
  [[nodiscard]] std::vector<ScheduledEvent> pop_due(double now);

  [[nodiscard]] std::size_t pending() const noexcept { return events_.size(); }
  [[nodiscard]] bool empty() const noexcept { return events_.empty(); }

 private:
  struct Entry {
    ScheduledEvent event;
    std::uint64_t order;
  };
  std::vector<Entry> events_;
  std::uint64_t next_order_{0};
};

/// Applies robot, link and obstacle events to the simulator and (optionally)
/// the network. Returns false for task events, which the scenario handles.
bool apply_event(const ScheduledEvent& event, Simulator& simulator,
                 communication::NetworkSimulator* network);

}  // namespace multirobotcore::simulation
