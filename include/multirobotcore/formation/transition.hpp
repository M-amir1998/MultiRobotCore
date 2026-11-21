#pragma once

/// @file transition.hpp
/// Smooth, assignment-aware transitions between formation shapes.
///
/// Robots are assigned to target slots by minimising the sum of squared
/// distances (Hungarian method). For synchronised straight-line motion this
/// is the assignment used by CAPT (Turpin, Michael & Kumar 2014), which yields
/// collision-free trajectories when start and goal locations are separated by
/// more than 2 sqrt(2) robot radii. Offsets are then blended with a quintic
/// smoothstep, so slot velocities and accelerations are continuous: no
/// discontinuous teleportation of the desired formation.

#include "multirobotcore/math/types.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace multirobotcore::formation {

/// Minimum sum-of-squared-distance assignment of robots to slots. Returns
/// slot index per robot. Requires positions.size() <= slots.size().
/// `pinned` optionally forces robot pinned->first onto slot pinned->second
/// (e.g. the leader keeps the leader slot).
[[nodiscard]] std::vector<std::size_t> assign_slots(
    std::span<const Vec2> positions, std::span<const Vec2> slots,
    std::optional<std::pair<std::size_t, std::size_t>> pinned = std::nullopt);

class FormationTransition {
 public:
  /// Transition of offsets (formation frame) from `from` to `to` starting at
  /// `start_time` and lasting `duration` seconds. With `reassign`, robot i
  /// moves to slot assign_slots(from, to)[i]; otherwise to slot i.
  FormationTransition(std::vector<Vec2> from, std::vector<Vec2> to, double start_time,
                      double duration, bool reassign = true,
                      std::optional<std::pair<std::size_t, std::size_t>> pinned = std::nullopt);

  /// Blended offsets at time t (clamped to the endpoints outside the window).
  [[nodiscard]] std::vector<Vec2> offsets_at(double t) const;

  /// Time derivative of the blended offsets (feedforward for tracking).
  [[nodiscard]] std::vector<Vec2> offset_rates_at(double t) const;

  [[nodiscard]] bool finished(double t) const noexcept { return t >= start_ + duration_; }
  [[nodiscard]] const std::vector<std::size_t>& assignment() const noexcept { return assignment_; }
  /// Final offsets indexed by robot (target slots after assignment).
  [[nodiscard]] const std::vector<Vec2>& target() const noexcept { return target_; }

 private:
  std::vector<Vec2> from_;
  std::vector<Vec2> target_;
  std::vector<std::size_t> assignment_;
  double start_;
  double duration_;
};

}  // namespace multirobotcore::formation
