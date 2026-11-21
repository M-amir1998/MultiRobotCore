# Space-Time A*

Header: `include/multirobotcore/mapf/space_time_astar.hpp` · Module: `MultiRobotCore::mapf`

## Purpose

Single-agent shortest path on a grid **in space and time**: the path must avoid
cells and moves that are forbidden at specific timesteps. It is the low-level
solver shared by [Prioritized Planning](prioritized_planning.md) (forbidden =
reserved by higher-priority agents) and [Conflict-Based Search](conflict_based_search.md)
(forbidden = constraints of a constraint-tree node).

## Problem formulation

- Graph: a 4-connected grid `G = (V, E)` of free cells (`GridMap`).
- State: `(v, t)` with `v ∈ V` and integer timestep `t ≥ 0`.
- Actions: move to a 4-neighbour or **wait** in place. Every action costs 1.
- Constraints (`ConstraintTable`):
  - vertex `⟨v, t⟩`: the agent may not occupy `v` at time `t`;
  - edge `⟨u → v, t⟩`: the agent may not move from `u` to `v` between `t` and `t + 1`;
  - permanent `⟨v, ≥ t⟩`: `v` is blocked from `t` on (another agent parked there).
- Output: path `π = (π_0 = s, …, π_T = g)` minimising the **arrival time** `T`,
  such that the agent can afterwards stay at `g` forever.

Discrete units: cells and timesteps. Conversion to metres and seconds happens
only through `WorldFrame` (cell size, origin) and `to_waypoints(path, frame,
step_duration)`.

## Mathematics

A* orders states by `f(v, t) = g(v, t) + h(v)` with `g = t` and
`h(v) = d_G(v, g)`, the exact single-agent distance computed once by
breadth-first search from the goal. Because every action (including waiting)
costs 1 and `d_G` is a shortest-path metric, `h` is admissible and consistent:
`h(v) ≤ 1 + h(v')` for every successor `v'`, including the wait `v' = v`.

**Goal condition.** Let `τ_g` be the latest vertex constraint on the goal.
A state `(g, t)` is accepted only if `t > τ_g` and the goal is not permanently
blocked; otherwise the agent would collide after "finishing".

**Time invariance.** Let `T_c` be the largest time referenced by any
constraint. For `t > T_c` no constraint applies to any transition, so all
states `(v, t)` with `t > T_c` are equivalent up to their arrival time.
Duplicate detection therefore uses the key `(v, min(t, T_c + 1))`; the first
expansion of a merged key has the smallest arrival time, so this is exact.

**Termination and completeness.** After `T_c` the problem is a static
shortest-path problem whose solution has at most `|V_free|` moves. The default
horizon `T_c + |V_free| + 1` therefore never cuts off a solution that exists,
and guarantees termination when none does.

## Assumptions

- Agents occupy exactly one cell per timestep; moves take one timestep.
- Collisions are modelled by vertex and swap (edge) conflicts only (the
  standard MAPF model of Stern et al. 2019); "following" an agent into the
  cell it is vacating is allowed.
- Agents park at their goal after arrival.

## Algorithm

```
open ← {(s, 0)}; closed ← ∅
while open ≠ ∅:
    (v, t) ← pop min by (f, conflicts, −g, insertion order)
    if key(v, t) ∈ closed: continue
    closed ← closed ∪ {key(v, t)}
    if v = goal and t > τ_goal: return reconstructed path
    if t ≥ horizon: continue
    for v' in neighbours(v) ∪ {v}:                 # moves and wait
        if ⟨v', t+1⟩ forbidden or (v' ≠ v and ⟨v → v', t⟩ forbidden): continue
        push (v', t + 1) unless dominated
return failure
```

## Implementation notes

- `ConstraintTable` stores vertex constraints in a hash set keyed by
  `(cell, t)`, edge constraints by `(from, to, t)`, permanent blocks by cell,
  and the latest vertex constraint per cell for the goal test.
- `reserve_path()` turns a higher-priority path into vertex reservations, the
  reverse of every move (no swaps) and a permanent block at its goal.
- Optional `ConflictAvoidanceTable`: among states with equal `f`, prefer those
  that conflict with fewer other agents' current paths. This is only a
  tie-breaker, so the returned cost is unchanged; it reduces the work CBS has
  to do afterwards.
- Dominance pruning keeps, per state key, the best `(arrival time, conflicts)`;
  an earlier arrival always dominates.
- All tie-breaking is deterministic (insertion order), so results are
  reproducible across runs and platforms.

## Complexity

With constraints up to `T_c`, at most `|V| (T_c + 2)` distinct states exist, each
with at most five successors: `O(|V| T_c log(|V| T_c))` time with a binary
heap. With a perfect heuristic and no constraints the search expands only the
states of one optimal path (verified in the unit tests).

## Configuration

`LowLevelOptions`:

| Field | Meaning | Default |
|---|---|---|
| `max_time` | horizon; negative selects `T_c + \|V_free\| + 1` | `-1` |
| `conflict_avoidance` | optional tie-breaking table | `nullptr` |

## Example

```cpp
#include <multirobotcore/mapf/space_time_astar.hpp>
namespace mapf = multirobotcore::mapf;

const auto map = mapf::GridMap::from_ascii({"....", ".@@.", "...."});
mapf::ConstraintTable constraints(map);
constraints.add_vertex({1, 0}, 1);            // (1,0) is occupied at t = 1
mapf::LowLevelStats stats;
const auto path = mapf::space_time_astar(map, {0, 0}, {3, 0}, constraints,
                                         map.distances_from({3, 0}), stats);
// path waits once at (0,0): cost 4 instead of 3
```

## Benchmark

Google Benchmark (`benchmarks/bench_mapf.cpp`, `mrc_bench_mapf`): corner-to-corner
search on open grids and planning against 10/40 reserved paths. See also the
planner comparison in [docs/benchmarks/mapf.md](../benchmarks/mapf.md).

## Known limitations

- 4-connected grids with unit-cost moves only (no diagonal moves, no
  heterogeneous speeds or footprints larger than one cell).
- Continuous robots following the resulting timed paths must stay
  synchronised with the discrete timesteps; see the integration notes in the
  warehouse demo for what happens when local avoidance delays a robot.

## References

- P. E. Hart, N. J. Nilsson, B. Raphael, "A Formal Basis for the Heuristic
  Determination of Minimum Cost Paths," *IEEE Transactions on Systems Science
  and Cybernetics*, 4(2):100–107, 1968.
- D. Silver, "Cooperative Pathfinding," *Proc. AAAI Conference on Artificial
  Intelligence and Interactive Digital Entertainment (AIIDE)*, 2005.
- R. Stern et al., "Multi-Agent Pathfinding: Definitions, Variants, and
  Benchmarks," *Proc. Symposium on Combinatorial Search (SoCS)*, 2019.
