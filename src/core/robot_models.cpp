#include "multirobotcore/models/robot_models.hpp"

#include "multirobotcore/math/numerics.hpp"

#include <algorithm>
#include <cmath>

namespace multirobotcore::models {

Vec2 step_single_integrator(const Vec2& position, const Vec2& velocity, double dt) noexcept {
  return position + dt * velocity;
}

DoubleIntegratorState step_double_integrator(const DoubleIntegratorState& state,
                                             const Vec2& acceleration, double dt) noexcept {
  return DoubleIntegratorState{state.position + dt * state.velocity + 0.5 * dt * dt * acceleration,
                               state.velocity + dt * acceleration};
}

UnicycleState step_unicycle(const UnicycleState& state, const UnicycleInput& input,
                            double dt) noexcept {
  const double theta = state.heading;
  const double v = input.linear;
  const double w = input.angular;
  UnicycleState next;
  if (std::abs(w) < 1e-9) {
    next.position = state.position + v * dt * Vec2(std::cos(theta), std::sin(theta));
    next.heading = math::wrap_angle(theta + w * dt);
    return next;
  }
  const double theta_next = theta + w * dt;
  const double r = v / w;
  next.position = state.position + r * Vec2(std::sin(theta_next) - std::sin(theta),
                                            -std::cos(theta_next) + std::cos(theta));
  next.heading = math::wrap_angle(theta_next);
  return next;
}

UnicycleInput saturate(const UnicycleInput& input, const UnicycleLimits& limits) noexcept {
  const double v_min = limits.allow_reverse ? -limits.max_linear : 0.0;
  return UnicycleInput{std::clamp(input.linear, v_min, limits.max_linear),
                       std::clamp(input.angular, -limits.max_angular, limits.max_angular)};
}

UnicycleInput track_velocity_lookahead(const UnicycleState& state, const Vec2& desired_velocity,
                                       double lookahead_distance,
                                       const UnicycleLimits& limits) noexcept {
  const double c = std::cos(state.heading);
  const double s = std::sin(state.heading);
  const double l = std::max(lookahead_distance, 1e-6);
  // Inverse of q' = [[c, -l s], [s, l c]] [v, w]^T (determinant l > 0).
  const UnicycleInput raw{c * desired_velocity.x() + s * desired_velocity.y(),
                          (-s * desired_velocity.x() + c * desired_velocity.y()) / l};
  return saturate(raw, limits);
}

UnicycleInput wheels_to_body(const WheelSpeeds& wheels,
                             const DifferentialDriveParams& params) noexcept {
  return UnicycleInput{0.5 * params.wheel_radius * (wheels.right + wheels.left),
                       params.wheel_radius * (wheels.right - wheels.left) / params.track_width};
}

WheelSpeeds body_to_wheels(const UnicycleInput& body,
                           const DifferentialDriveParams& params) noexcept {
  const double half_track = 0.5 * params.track_width;
  return WheelSpeeds{(body.linear - body.angular * half_track) / params.wheel_radius,
                     (body.linear + body.angular * half_track) / params.wheel_radius};
}

WheelSpeeds saturate(const WheelSpeeds& wheels, const DifferentialDriveParams& params) noexcept {
  const double peak = std::max(std::abs(wheels.left), std::abs(wheels.right));
  if (peak <= params.max_wheel_speed || peak <= 0.0) {
    return wheels;
  }
  const double scale = params.max_wheel_speed / peak;
  return WheelSpeeds{wheels.left * scale, wheels.right * scale};
}

UnicycleState step_differential_drive(const UnicycleState& state, const WheelSpeeds& wheels,
                                      const DifferentialDriveParams& params, double dt) noexcept {
  return step_unicycle(state, wheels_to_body(saturate(wheels, params), params), dt);
}

UnicycleState step_bicycle(const UnicycleState& state, const BicycleInput& input,
                           const BicycleParams& params, double dt) noexcept {
  const double steering = std::clamp(input.steering, -params.max_steering, params.max_steering);
  const double yaw_rate = input.speed * std::tan(steering) / params.wheelbase;
  return step_unicycle(state, UnicycleInput{input.speed, yaw_rate}, dt);
}

}  // namespace multirobotcore::models
