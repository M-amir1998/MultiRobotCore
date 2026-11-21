#pragma once

/// @file robot_models.hpp
/// Classical planar mobile-robot models with exact zero-order-hold
/// discretisation.
///
///  - Single integrator:   p' = u
///  - Double integrator:   p'' = a
///  - Unicycle:            x' = v cos(theta), y' = v sin(theta), theta' = omega
///  - Differential drive:  unicycle driven by left/right wheel speeds
///  - Kinematic bicycle:   unicycle with omega = v tan(delta) / L
///
/// All step functions integrate exactly for inputs held constant over dt.

#include "multirobotcore/math/types.hpp"

#include <limits>

namespace multirobotcore::models {

// --- Single integrator --------------------------------------------------------

/// p(t + dt) = p(t) + dt * velocity.
[[nodiscard]] Vec2 step_single_integrator(const Vec2& position, const Vec2& velocity,
                                          double dt) noexcept;

// --- Double integrator --------------------------------------------------------

struct DoubleIntegratorState {
  Vec2 position{Vec2::Zero()};  ///< [m]
  Vec2 velocity{Vec2::Zero()};  ///< [m/s]
};

/// Exact ZOH discretisation: p+ = p + dt v + dt^2/2 a, v+ = v + dt a.
[[nodiscard]] DoubleIntegratorState step_double_integrator(const DoubleIntegratorState& state,
                                                           const Vec2& acceleration,
                                                           double dt) noexcept;

// --- Unicycle -------------------------------------------------------------------

struct UnicycleState {
  Vec2 position{Vec2::Zero()};  ///< [m]
  double heading{0.0};          ///< [rad]
};

struct UnicycleInput {
  double linear{0.0};   ///< forward speed v [m/s]
  double angular{0.0};  ///< yaw rate omega [rad/s]
};

struct UnicycleLimits {
  double max_linear{std::numeric_limits<double>::infinity()};
  double max_angular{std::numeric_limits<double>::infinity()};
  bool allow_reverse{false};
};

/// Exact arc integration (straight line when |omega| is negligible).
[[nodiscard]] UnicycleState step_unicycle(const UnicycleState& state, const UnicycleInput& input,
                                          double dt) noexcept;

/// Clamps an input to the given limits.
[[nodiscard]] UnicycleInput saturate(const UnicycleInput& input,
                                     const UnicycleLimits& limits) noexcept;

/// Velocity tracking by feedback linearisation of the off-axis point
/// q = p + l [cos(theta), sin(theta)]: returns the (v, omega) that make q move
/// with `desired_velocity`. A standard way to drive nonholonomic robots with
/// planners and controllers formulated for holonomic agents.
[[nodiscard]] UnicycleInput track_velocity_lookahead(const UnicycleState& state,
                                                     const Vec2& desired_velocity,
                                                     double lookahead_distance,
                                                     const UnicycleLimits& limits) noexcept;

// --- Differential drive -------------------------------------------------------

struct DifferentialDriveParams {
  double wheel_radius{0.05};  ///< [m]
  double track_width{0.30};   ///< distance between wheel contact points [m]
  double max_wheel_speed{std::numeric_limits<double>::infinity()};  ///< [rad/s]
};

struct WheelSpeeds {
  double left{0.0};   ///< [rad/s]
  double right{0.0};  ///< [rad/s]
};

/// Forward kinematics: wheel angular speeds -> body (v, omega).
[[nodiscard]] UnicycleInput wheels_to_body(const WheelSpeeds& wheels,
                                           const DifferentialDriveParams& params) noexcept;

/// Inverse kinematics: body (v, omega) -> wheel angular speeds.
[[nodiscard]] WheelSpeeds body_to_wheels(const UnicycleInput& body,
                                         const DifferentialDriveParams& params) noexcept;

/// Scales both wheel speeds by the same factor so that neither exceeds the
/// limit, which preserves the commanded path curvature.
[[nodiscard]] WheelSpeeds saturate(const WheelSpeeds& wheels,
                                   const DifferentialDriveParams& params) noexcept;

/// One differential-drive step: saturates the wheels, then integrates exactly.
[[nodiscard]] UnicycleState step_differential_drive(const UnicycleState& state,
                                                    const WheelSpeeds& wheels,
                                                    const DifferentialDriveParams& params,
                                                    double dt) noexcept;

// --- Kinematic bicycle ----------------------------------------------------------

struct BicycleParams {
  double wheelbase{0.5};     ///< [m]
  double max_steering{0.6};  ///< [rad]
};

struct BicycleInput {
  double speed{0.0};     ///< rear-axle speed [m/s]
  double steering{0.0};  ///< front-wheel angle [rad]
};

/// Rear-axle-referenced kinematic bicycle with saturated steering.
[[nodiscard]] UnicycleState step_bicycle(const UnicycleState& state, const BicycleInput& input,
                                         const BicycleParams& params, double dt) noexcept;

}  // namespace multirobotcore::models
