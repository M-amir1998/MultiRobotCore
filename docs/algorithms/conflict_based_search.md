# Conflict-Based Search (CBS)

Header: `include/multirobotcore/mapf/conflict_based_search.hpp` · Module: `MultiRobotCore::mapf`

## Purpose

Optimal multi-agent path finding on grids: collision-free paths that minimise
the sum of costs (or the makespan). CBS is the coordinated planner of the
library; [Prioritized Planning](prioritized_planning.md) is the fast,
incomplete alternative, and independent A* the unsafe baseline.

## Problem formulation

Classical MAPF (Stern et al. 2019) on a 4-connected grid with move and wait
actions of unit cost. Agents `a_i` go from distinct starts `s_i` to distinct goals
`g_i` and park there. Conflicts:

- vertex conflict `⟨a_i, a_j, v, t⟩`: both occupy `v` at time `t`;
- edge (swap) conflict `⟨a_i, a_j, u, v, t⟩`: `a_i` moves `u → v` while `a_j` moves
  `v → u` between `t` and `t + 1`.

Objectives: `SOC = Σ_i T_i` or makespan `max_i T_i`, where `T_i` is the time of the
agent's final arrival at its goal (trailing waits are free).

## Mathematics

CBS (Sharon et al. 2015) searches a **constraint tree** (CT). A node `N` holds a
set of constraints `C_N`, one path per agent that is individually shortest
subject to that agent's constraints, and a cost:

```
cost(N) = Σ_i T(π_i^N)      (sum of costs)      or      max_i T(π_i^N)      (makespan)
```

**Lower bound.** Any valid solution consistent with `C_N` gives each agent a path
that satisfies its constraints and is therefore no shorter than `π_i^N`. Hence
`cost(N)` is a lower bound on every solution in the subtree of `N`, for both
objectives.

**Completeness of the split.** A conflict `⟨a_i, a_j, v, t⟩` creates two children
with `⟨a_i, v, t⟩` and `⟨a_j, v, t⟩` (edge conflicts: `⟨a_i, u→v, t⟩` and
`⟨a_j, v→u, t⟩`). Every valid solution violates at most one of them, so it
remains consistent with at least one child.

Best-first expansion by cost therefore returns an optimal solution: the first
conflict-free node expanded has cost ≤ the cost of every unexplored subtree.

**Conflict prioritisation (Boyarski et al. 2015).** A conflict is *cardinal* if
both children necessarily have a higher cost, *semi-cardinal* if one does. This
is decided with the agent's multi-valued decision diagram (MDD): the union of
all its shortest constrained paths. The conflict is forced for the agent if
the MDD has a single node at the conflicting vertex (or a single edge) at that
time, or if the agent is already parked there. CBS splits cardinal conflicts
first, then semi-cardinal, then the earliest conflict. This affects only the
choice of conflict, so optimality is preserved, while the tree usually gets
smaller (the intersection demo: 1955 → 1082 expanded nodes).

## Assumptions

- Same MAPF model as [Space-Time A*](space_time_astar.md).
- Optimality holds for the configured objective among solutions of this model;
  it says nothing about continuous-time execution.

## Algorithm

```
root.paths ← individually shortest paths;  OPEN ← {root}
while OPEN ≠ ∅ and limits not reached:
    N ← pop min by (cost, #conflicts, generation order)
    c ← chosen conflict of N (cardinal > semi-cardinal > earliest)
    if c = none: return N.paths                                  # optimal
    for each agent a in c:
        child ← N + constraint(a, c)
        child.paths[a] ← SpaceTimeA*(a, constraints of a on the branch)
        if a path exists: push child                             # else prune
return timeout / node limit / no solution
```

## Implementation notes

- Nodes store only the constraint they add plus a parent index; the constraints
  of an agent are collected along the branch. Paths of unchanged agents are
  shared between nodes (`std::shared_ptr<const Path>`).
- Low-level ties are broken towards fewer conflicts with the other agents'
  current paths (conflict-avoidance table), which never changes path cost.
- MDD singleton layers are computed lazily per (node, agent) and shared with
  children whose agent was not replanned.
- Wall-clock and node limits return `kTimeout` / `kNodeLimit`. CBS cannot prove
  that an instance is unsolvable in general (the constraint tree is unbounded),
  so unsolvable instances end in one of these limits.

## Complexity

Exponential in the number of conflicts that must be resolved in the worst case;
each node costs one low-level search plus conflict detection
`O(k · T)` for `k` agents and horizon `T`. Practical performance depends far more
on map structure than on team size (see the benchmark).

## Configuration

`CbsOptions`:

| Field | Meaning | Default |
|---|---|---|
| `objective` | `kSumOfCosts` or `kMakespan` | `kSumOfCosts` |
| `max_high_level_nodes` | expansion limit | `200000` |
| `time_limit_s` | wall-clock limit | `30.0` |
| `use_conflict_avoidance` | low-level tie-breaking | `true` |
| `prioritize_cardinal_conflicts` | MDD-based conflict choice | `true` |

## Example

```cpp
#include <multirobotcore/mapf/conflict_based_search.hpp>
namespace mapf = multirobotcore::mapf;

mapf::Instance instance{mapf::GridMap::from_ascii({"##.##", "....."}),
                        {{{0, 0}, {4, 0}}, {{4, 0}, {0, 0}}}};
const auto result = mapf::ConflictBasedSearch().solve(instance);
// result.status == kSolved, result.sum_of_costs == 11 (one wait + pocket detour)

// Hand the plan to continuous controllers: one SI waypoint per timestep.
const mapf::WorldFrame frame{{0.0, 0.0}, 0.5};           // 0.5 m cells
const auto waypoints = mapf::to_waypoints(result.paths[0], frame, 0.5);  // 0.5 s per step
```

## Validation

- `tests/unit/mapf/test_planners.cpp` compares CBS with an independent
  joint-state reference solver (Dijkstra over joint positions with exact
  pending-wait accounting for the sum of costs, BFS for the makespan) on random
  tiny instances with 2–3 agents, and checks crafted cases: two-agent swap with
  a passing pocket, intersection, corridor with a side room, node limit,
  unreachable goals.
- Property tests: every solution is collision-free (`find_conflicts` is empty),
  starts and ends correctly, and moves only between neighbouring cells; the
  independent cost is a lower bound and prioritized planning an upper bound.
- `tests/regression/mapf/` records outputs of the demo instances.

## Benchmark

[docs/benchmarks/mapf.md](../benchmarks/mapf.md), 3 s limit per instance:

- Open 32×32: solved 11 of 12 instances up to 40 robots (one 40-robot timeout).
- Warehouse with one-lane aisles: all instances up to 10 robots, 2 of 3 with 15,
  1 of 3 with 20. Mean sum of costs over solved instances: 1.03–1.05 × the
  lower bound, against 1.07–1.14 × for prioritized planning.
- 8×8 congestion: all instances up to 16 robots (30 ms median at 16).
- Maze and narrow corridor: timeouts already with 5–6 robots on some seeds.

Micro-benchmarks (`mrc_bench_mapf`, warehouse instance, release build):
4 robots 0.55 ms, 8 robots 4.4 ms, 12 robots 33 ms per solve.

## Known limitations

- **Corridor symmetry.** When two agents must pass each other in a long
  one-lane corridor, each split only delays one agent by one timestep, and
  the number of CT nodes grows rapidly with the corridor length. This explains
  the maze/corridor timeouts. Symmetry-breaking constraints (Li et al. 2019)
  address it and are not implemented here.
- No bypassing, disjoint splitting or bounded-suboptimal variants (e.g. ECBS)
  yet; for large dense teams use prioritized planning.
- Optimal in the discrete model only: continuous robots that are delayed (for
  example by local collision avoidance) no longer follow the timed plan.

## References

- G. Sharon, R. Stern, A. Felner, N. R. Sturtevant, "Conflict-Based Search for
  Optimal Multi-Agent Pathfinding," *Artificial Intelligence*, 219:40–66, 2015.
- E. Boyarski, A. Felner, R. Stern, G. Sharon, D. Tolpin, O. Betzalel,
  E. Shimony, "ICBS: Improved Conflict-Based Search Algorithm for Multi-Agent
  Pathfinding," *Proc. IJCAI*, 2015.
- J. Li, D. Harabor, P. J. Stuckey, H. Ma, S. Koenig, "Symmetry-Breaking
  Constraints for Grid-Based Multi-Agent Path Finding," *Proc. AAAI*, 2019.
- T. Standley, "Finding Optimal Solutions to Cooperative Pathfinding Problems,"
  *Proc. AAAI*, 2010 (joint-state search, used for the reference solver idea).
- R. Stern et al., "Multi-Agent Pathfinding: Definitions, Variants, and
  Benchmarks," *Proc. SoCS*, 2019.
- N. R. Sturtevant, "Benchmarks for Grid-Based Pathfinding," *IEEE Transactions
  on Computational Intelligence and AI in Games*, 4(2):144–148, 2012 (map format).
