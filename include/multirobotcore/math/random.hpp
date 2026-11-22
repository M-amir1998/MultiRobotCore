#pragma once

/// @file random.hpp
/// Sampling helpers whose results do not depend on the compiler.
///
/// Two things make seeded results compiler-dependent:
///
/// 1. The standard distributions (std::uniform_real_distribution, ...) are
///    implementation-defined; only the engines are fully specified. The
///    `uniform_*` helpers below transform raw std::mt19937_64 output
///    explicitly (see docs/adr/0007-portable-sampling.md).
/// 2. The evaluation order of function arguments is unspecified, so
///    `Vec2(dist(rng), dist(rng))` draws the two coordinates in a
///    compiler-dependent order: GCC and MSVC evaluate the arguments right to
///    left on x86-64, Clang left to right, and the same seed yields mirrored
///    points. `sample_vec2` sequences the draws explicitly. It draws the
///    y-coordinate first, the order GCC and MSVC already used, so reference
///    outputs produced with those compilers are unchanged.

#include "multirobotcore/math/types.hpp"

#include <random>

namespace multirobotcore::math {

/// Uniform double in [0, 1) from the 53 high bits of one 64-bit draw.
[[nodiscard]] inline double uniform_unit(std::mt19937_64& rng) {
  return static_cast<double>(rng() >> 11U) * 0x1.0p-53;
}

/// Uniform double in [lo, hi) built on `uniform_unit`.
[[nodiscard]] inline double uniform_real(std::mt19937_64& rng, double lo, double hi) {
  return lo + (hi - lo) * uniform_unit(rng);
}

/// Planar sample with x drawn from `dist_x` and y from `dist_y`; the draw
/// order is fixed (y first, then x).
template <typename Rng, typename DistX, typename DistY>
[[nodiscard]] Vec2 sample_vec2(Rng& rng, DistX& dist_x, DistY& dist_y) {
  const double y = dist_y(rng);
  const double x = dist_x(rng);
  return {x, y};
}

/// Planar sample with both coordinates drawn from `dist` (y first, then x).
template <typename Rng, typename Dist>
[[nodiscard]] Vec2 sample_vec2(Rng& rng, Dist& dist) {
  return sample_vec2(rng, dist, dist);
}

}  // namespace multirobotcore::math
