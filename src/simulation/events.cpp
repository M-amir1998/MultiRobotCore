#include "multirobotcore/simulation/events.hpp"

#include "multirobotcore/communication/network.hpp"
#include "multirobotcore/simulation/simulator.hpp"

#include <algorithm>

namespace multirobotcore::simulation {

std::string_view to_string(EventType type) noexcept {
  switch (type) {
    case EventType::kRobotStop:
      return "robot_stop";
    case EventType::kRobotFailure:
      return "robot_failure";
    case EventType::kRobotRecover:
      return "robot_recover";
    case EventType::kLeaderLoss:
      return "leader_loss";
    case EventType::kLinkFailure:
      return "link_failure";
    case EventType::kObstacleAppears:
      return "obstacle_appears";
    case EventType::kTaskArrival:
      return "task_arrival";
    case EventType::kTaskCancelled:
      return "task_cancelled";
  }
  return "unknown";
}

void EventSchedule::add(const ScheduledEvent& event) {
  events_.push_back(Entry{event, next_order_++});
}

std::vector<ScheduledEvent> EventSchedule::pop_due(double now) {
  std::vector<Entry> due;
  std::vector<Entry> remaining;
  for (const Entry& e : events_) {
    (e.event.time <= now ? due : remaining).push_back(e);
  }
  events_ = std::move(remaining);
  std::sort(due.begin(), due.end(), [](const Entry& a, const Entry& b) {
    return a.event.time != b.event.time ? a.event.time < b.event.time : a.order < b.order;
  });
  std::vector<ScheduledEvent> out;
  out.reserve(due.size());
  for (const Entry& e : due) {
    out.push_back(e.event);
  }
  return out;
}

bool apply_event(const ScheduledEvent& event, Simulator& simulator,
                 communication::NetworkSimulator* network) {
  switch (event.type) {
    case EventType::kRobotStop:
      simulator.set_status(event.robot, models::RobotStatus::kStopped);
      return true;
    case EventType::kRobotFailure:
    case EventType::kLeaderLoss:
      simulator.set_status(event.robot, models::RobotStatus::kFailed);
      if (network != nullptr) {
        network->add_node_outage({event.robot, event.time, event.time + event.duration});
      }
      return true;
    case EventType::kRobotRecover:
      simulator.set_status(event.robot, models::RobotStatus::kActive);
      return true;
    case EventType::kLinkFailure:
      if (network != nullptr) {
        network->add_link_outage(
            {event.robot, event.other, event.time, event.time + event.duration});
      }
      return true;
    case EventType::kObstacleAppears:
      simulator.add_obstacle(math::Circle{event.position, event.radius});
      return true;
    case EventType::kTaskArrival:
    case EventType::kTaskCancelled:
      return false;
  }
  return false;
}

}  // namespace multirobotcore::simulation
