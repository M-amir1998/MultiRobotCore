# ADR 0004 — Bounded Voronoi cells by half-plane clipping

**Status:** accepted

**Context.** Coverage control needs the Voronoi cell of every robot inside a
bounded convex domain, robust to near-degenerate configurations.

**Decision.** The cell of robot `i` is the domain clipped by the bisector
half-planes `{q : (p_j - p_i) . q <= (|p_j|^2 - |p_i|^2) / 2}` of the other
robots (Sutherland–Hodgman against one half-plane at a time: `O(N)` per cell,
`O(N^2)` per partition). This uses only the convex-polygon kernel of `core`
and is exact up to floating-point rounding; no general Voronoi or Delaunay
library is added.

**Consequences.** Quadratic cost per partition is adequate for teams of up to
a few hundred robots; a Delaunay-based construction would be needed beyond
that. The limited-range variant follows directly: a robot clips only against
the robots it can hear.
