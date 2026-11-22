# ADR 0007 — Portable sampling where results must match across compilers

**Status:** accepted

**Context.** `std::mt19937_64` produces the same sequence on every platform,
but `std::uniform_*_distribution` and `std::normal_distribution` are
implementation-defined, so seeded results differ between MSVC and libstdc++.

A second, independent source of divergence is the evaluation order of
function arguments, which C++ leaves unspecified: `Vec2(d(rng), d(rng))` draws
the two coordinates right to left with GCC and MSVC on x86-64 and left to
right with Clang, so the same seed yields mirrored points. A seeded Lloyd
regression test failed under Clang for exactly this reason.

**Decision.** Components whose outputs serve as cross-platform references
(MAPF map and instance generators, warehouse task generation) derive samples
from the raw generator output with explicit transforms (53-bit uniform
doubles, inverse-CDF exponentials); the shared transforms live in
`include/multirobotcore/math/random.hpp`. Elsewhere the standard distributions
are used and tests assert properties rather than exact seeded values; tests
that pin exact seeded values use the portable transforms.

Every planar sample is drawn with `math::sample_vec2`, which sequences the two
draws explicitly. It draws y first, the order GCC and MSVC already used, so the
committed assets and benchmark pages, rendered with MSVC, remain valid; this
was checked by comparing demo and experiment outputs before and after the
change (only wall-clock timing fields differ).

**Consequences.** The Linux (GCC 13 and Clang 18) and Windows (MSVC) test
suites pass with the same expectations. Results that still rely on the
standard distributions can differ between standard libraries (libstdc++,
MSVC STL, libc++).
