# ADR 0007 — Portable sampling where results must match across compilers

**Status:** accepted

**Context.** `std::mt19937_64` produces the same sequence on every platform,
but `std::uniform_*_distribution` and `std::normal_distribution` are
implementation-defined, so seeded results differ between MSVC and libstdc++.

**Decision.** Components whose outputs serve as cross-platform references
(MAPF map and instance generators, warehouse task generation) derive samples
from the raw generator output with explicit transforms (53-bit uniform
doubles, inverse-CDF exponentials). Elsewhere the standard distributions are
used and tests assert properties rather than exact seeded values.

**Consequences.** The Linux (GCC 13) and Windows (MSVC) test suites pass with
the same expectations.
