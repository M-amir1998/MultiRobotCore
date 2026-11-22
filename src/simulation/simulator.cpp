#include "multirobotcore/simulation/simulator.hpp"

#include "multirobotcore/math/numerics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace multirobotcore::simulation {
namespace {

std::size_t pair_index(std::size_t i, std::size_t j, std::size_t n) noexcept {
  return i * n + j;  // only i < j entries are used
}

}  // namespace

Simulator::Simulator(WorldConfig config, std::vector<RobotSpec> specs,
                     std::vector<Pose2> initial_poses)
    : config_(std::move(config)), specs_(std::move(specs)) {
  if (!(config_.dt > 0.0)) {
    throw std::invalid_argument("Simulator: dt must be positive");
  }
  if (config_.record_every == 0) {
    throw std::invalid_argument("Simulator: record_every must be at least 1");
  }
  if (specs_.size() != initial_poses.size()) {
    throw std::invalid_argument("Simulator: specs/initial poses size mismatch");
  }
  const std::size_t n = specs_.size();
  for (const RobotSpec& s : specs_) {
    if (!(s.radius >= 0.0) || !(s.max_speed >= 0.0) || !(s.max_acceleration > 0.0)) {
      throw std::invalid_argument("Simulator: invalid robot specification");
    }
  }
  robots_.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    robots_[i].id = i;
    robots_[i].position = initial_poses[i].position;
    robots_[i].heading = initial_poses[i].heading;
    robots_[i].status = models::RobotStatus::kActive;
  }
  metrics_.distance_travelled.assign(n, 0.0);
  metrics_.control_effort.assign(n, 0.0);
  pair_contact_.assign(n * n, 0);
  obstacle_contact_.assign(n, 0);
  boundary_hit_.assign(n, 0);
  detect_collisions();
  history_.push_back(positions());
  if (config_.record) {
    record_frame();
  }
}

std::vector<Vec2> Simulator::positions() const {
  std::vector<Vec2> p;
  p.reserve(robots_.size());
  for (const auto& r : robots_) {
    p.push_back(r.position);
  }
  return p;
}

std::vector<Vec2> Simulator::velocities() const {
  std::vector<Vec2> v;
  v.reserve(robots_.size());
  for (const auto& r : robots_) {
    v.push_back(r.velocity);
  }
  return v;
}

std::vector<bool> Simulator::operational_mask() const {
  std::vector<bool> mask;
  mask.reserve(robots_.size());
  for (const auto& r : robots_) {
    mask.push_back(models::is_operational(r.status));
  }
  return mask;
}

void Simulator::set_status(std::size_t i, models::RobotStatus status) {
  robots_.at(i).status = status;
  if (!models::is_operational(status)) {
    robots_[i].velocity.setZero();
  }
}

void Simulator::set_goal(std::size_t i, std::optional<Vec2> goal) {
  robots_.at(i).goal = std::move(goal);
}

void Simulator::set_task(std::size_t i, std::optional<std::size_t> task) {
  robots_.at(i).task = task;
}

void Simulator::set_neighbors(std::size_t i, std::vector<models::RobotId> neighbors) {
  robots_.at(i).neighbors = std::move(neighbors);
}

void Simulator::add_obstacle(const math::Circle& circle) {
  config_.obstacles.circles.push_back(circle);
}

void Simulator::add_obstacle(const math::AABB& box) {
  config_.obstacles.boxes.push_back(box);
}

void Simulator::integrate(std::size_t i, const Vec2& command) {
  models::RobotState& r = robots_[i];
  const RobotSpec& s = specs_[i];
  const double dt = config_.dt;
  const Vec2 v_des = math::clamp_norm(command, s.max_speed);
  const Vec2 start = r.position;

  switch (s.dynamics) {
    case Dynamics::kSingleIntegrator: {
      Vec2 v = v_des;
      if (std::isfinite(s.max_acceleration)) {
        v = r.velocity + math::clamp_norm(v_des - r.velocity, s.max_acceleration * dt);
      }
      r.position = models::step_single_integrator(r.position, v, dt);
      r.velocity = v;
      if (v.norm() > 1e-9) {
        r.heading = std::atan2(v.y(), v.x());
      }
      break;
    }
    case Dynamics::kDoubleIntegrator: {
      const Vec2 a = math::clamp_norm(s.velocity_gain * (v_des - r.velocity), s.max_acceleration);
      const auto next = models::step_double_integrator({r.position, r.velocity}, a, dt);
      r.position = next.position;
      r.velocity = math::clamp_norm(next.velocity, s.max_speed);
      if (r.velocity.norm() > 1e-9) {
        r.heading = std::atan2(r.velocity.y(), r.velocity.x());
      }
      break;
    }
    case Dynamics::kUnicycle:
    case Dynamics::kDifferentialDrive: {
      const models::UnicycleLimits limits{s.max_speed, s.max_yaw_rate, false};
      const models::UnicycleState state{r.position, r.heading};
      models::UnicycleInput u = models::track_velocity_lookahead(state, v_des, s.lookahead, limits);
      models::UnicycleState next;
      if (s.dynamics == Dynamics::kUnicycle) {
        next = models::step_unicycle(state, u, dt);
      } else {
        next =
            models::step_differential_drive(state, models::body_to_wheels(u, s.drive), s.drive, dt);
      }
      r.position = next.position;
      r.heading = next.heading;
      r.velocity = (next.position - start) / dt;
      break;
    }
  }
  metrics_.distance_travelled[i] += (r.position - start).norm();
  metrics_.control_effort[i] += v_des.squaredNorm() * dt;
}

void Simulator::enforce_bounds(std::size_t i) {
  boundary_hit_[i] = 0;
  if (!config_.bounds) {
    return;
  }
  models::RobotState& r = robots_[i];
  const double rad = specs_[i].radius;
  const Vec2 lo = config_.bounds->min + Vec2(rad, rad);
  const Vec2 hi = config_.bounds->max - Vec2(rad, rad);
  const Vec2 clamped = r.position.cwiseMax(lo).cwiseMin(hi);
  if ((clamped - r.position).squaredNorm() > 0.0) {
    r.position = clamped;
    boundary_hit_[i] = 1;
  }
}

void Simulator::detect_collisions() {
  const std::size_t n = robots_.size();
  const double now = time();
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      const double clearance =
          (robots_[i].position - robots_[j].position).norm() - specs_[i].radius - specs_[j].radius;
      metrics_.min_pair_clearance = std::min(metrics_.min_pair_clearance, clearance);
      const bool contact = clearance < 0.0;
      char& flag = pair_contact_[pair_index(i, j, n)];
      if (contact && flag == 0) {
        ++metrics_.robot_collisions;
        collision_events_.push_back({now, i, j});
      }
      flag = contact ? 1 : 0;
    }
  }
  for (std::size_t i = 0; i < n; ++i) {
    bool contact = boundary_hit_[i] != 0;
    if (!config_.obstacles.empty()) {
      const double clearance = config_.obstacles.distance(robots_[i].position) - specs_[i].radius;
      metrics_.min_obstacle_clearance = std::min(metrics_.min_obstacle_clearance, clearance);
      contact = contact || clearance < 0.0;
    }
    if (contact && obstacle_contact_[i] == 0) {
      ++metrics_.obstacle_collisions;
      collision_events_.push_back({now, i, std::nullopt});
    }
    obstacle_contact_[i] = contact ? 1 : 0;
  }
}

void Simulator::record_frame() {
  Frame f;
  f.time = time();
  f.poses.reserve(robots_.size());
  for (const auto& r : robots_) {
    f.poses.push_back(Pose2{r.position, r.heading});
    f.velocities.push_back(r.velocity);
    f.status.push_back(r.status);
  }
  frames_.push_back(std::move(f));
}

void Simulator::step(std::span<const Vec2> desired_velocities) {
  if (desired_velocities.size() != robots_.size()) {
    throw std::invalid_argument("Simulator::step: one velocity command per robot required");
  }
  for (std::size_t i = 0; i < robots_.size(); ++i) {
    if (!models::is_operational(robots_[i].status)) {
      robots_[i].velocity.setZero();
      boundary_hit_[i] = 0;
      continue;
    }
    if (!math::is_finite(desired_velocities[i])) {
      throw std::invalid_argument("Simulator::step: non-finite velocity command");
    }
    integrate(i, desired_velocities[i]);
    enforce_bounds(i);
  }
  ++step_count_;
  detect_collisions();

  history_.push_back(positions());
  const auto window_steps =
      static_cast<std::size_t>(std::ceil(config_.stall.window / config_.dt - 1e-9));
  while (history_.size() > window_steps + 1) {
    history_.pop_front();
  }
  if (config_.record && step_count_ % config_.record_every == 0) {
    record_frame();
  }
}

std::vector<std::size_t> Simulator::stalled_robots() const {
  std::vector<std::size_t> stalled;
  if (time() + 1e-9 < config_.stall.window || history_.empty()) {
    return stalled;
  }
  const std::vector<Vec2>& past = history_.front();
  for (std::size_t i = 0; i < robots_.size(); ++i) {
    if (robots_[i].status != models::RobotStatus::kActive) {
      continue;
    }
    if ((robots_[i].position - past[i]).norm() < config_.stall.min_progress) {
      stalled.push_back(i);
    }
  }
  return stalled;
}

bool Simulator::stalled() const {
  const auto active = static_cast<std::size_t>(std::count_if(
      robots_.begin(), robots_.end(),
      [](const models::RobotState& r) { return r.status == models::RobotStatus::kActive; }));
  return active > 0 && stalled_robots().size() == active;
}

}  // namespace multirobotcore::simulation
