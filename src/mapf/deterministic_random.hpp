#pragma once

// Internal: platform-independent random helpers.
//
// The standard distributions (std::uniform_int_distribution, std::shuffle, ...)
// are implementation-defined, so the same seed yields different sequences on
// MSVC and libstdc++. MAPF instances are generated with these helpers instead,
// which depend only on the fully specified std::mt19937_64 engine, so that
// generated maps and instances are identical on every platform.

#include <cstddef>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

namespace multirobotcore::mapf::detail {

/// Uniform integer in [0, n) (n > 0). Modulo bias is below 2^-40 for n < 2^24.
[[nodiscard]] inline std::size_t uniform_index(std::mt19937_64& rng, std::size_t n) {
  return static_cast<std::size_t>(rng() % static_cast<std::uint64_t>(n));
}

/// Uniform double in [0, 1) with 53 random bits.
[[nodiscard]] inline double uniform_unit(std::mt19937_64& rng) {
  return static_cast<double>(rng() >> 11U) * 0x1.0p-53;
}

/// Fisher-Yates shuffle.
template <typename T>
void shuffle(std::vector<T>& values, std::mt19937_64& rng) {
  for (std::size_t i = values.size(); i > 1; --i) {
    std::swap(values[i - 1], values[uniform_index(rng, i)]);
  }
}

}  // namespace multirobotcore::mapf::detail
