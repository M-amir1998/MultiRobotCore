# Prioritized Planning

Header: `include/multirobotcore/mapf/planners.hpp` · Module: `MultiRobotCore::mapf`

## Purpose

Fast, decoupled multi-agent path finding: agents are planned one at a time in a
priority order, each treating the paths of all higher-priority agents as moving
obstacles. It scales to large teams but is **incomplete** and **suboptimal**,
and its result depends on the ordering. The library also provides the
uncoordinated baseline `plan_independent()`, which shows why some coordination
is required at all.

## Problem formulation

Classical MAPF (Stern et al. 2019): a grid `G`, agents `a_1 … a_k` with distinct
starts `s_i` and distinct goals `g_i`. Find paths `π_i` without vertex or swap
conflicts, where agents park at their goals after arrival. Objectives:

- sum of costs `SOC = Σ_i T_i` (arrival time `T_i` of agent `i`);
- makespan `max_i T_i`.

## Mathematics

For an ordering `σ`, agent `σ(j)` solves

```
π_σ(j) = argmin_π  T(π)   subject to   no conflict with π_σ(1), …, π_σ(j−1)
```

with [Space-Time A*](space_time_astar.md) against a reservation table. Each
sub-problem is solved optimally, but the composition is not: `SOC(PP) ≥ SOC*`,
and a sub-problem may be infeasible although the joint problem is solvable.

## Assumptions

- Same MAPF model as Space-Time A*; higher-priority agents ignore lower ones.
- A reserved goal blocks the cell forever after arrival.

## Algorithm

```
for attempt in 1 … max_attempts:
    order ← initial order (attempt 1) or seeded random shuffle
    R ← ∅                                   # reservation table
    for a in order:
        π_a ← SpaceTimeA*(s_a, g_a, R)
        if π_a = ∅: failure of this ordering → next attempt
        R ← R ∪ reserve(π_a)                # cells, reverse moves, parked goal
    return paths
return FAILED
```

Orderings (`PriorityOrder`): agent index, shortest start-goal distance first,
longest first, seeded random, or a user-supplied permutation. Restarting with
random orders is a common, simple remedy for ordering failures (cf. the
randomised search over priority schemes of Bennewitz, Burgard and Thrun 2002).

## Implementation notes

- Reservations include the reverse of each higher-priority move, which forbids
  swaps, and a permanent block of the goal from arrival on.
- The goal test of Space-Time A* forbids finishing at a goal that a
  higher-priority agent passes later.
- All random orders come from a seeded `std::mt19937_64` through
  platform-independent helpers, so runs are reproducible across compilers.
- `PlannerResult::order` and `attempts` report the ordering that succeeded.

## Complexity

`k` low-level searches per attempt; each grows with the number of reservations.
Runtime is typically a few milliseconds for tens of agents (see the benchmark).

## Configuration

`PrioritizedOptions`: `order`, `custom_order`, `seed`, `max_attempts`.

## Example

```cpp
mapf::PrioritizedOptions options;
options.order = mapf::PriorityOrder::kShortestFirst;
options.max_attempts = 10;  // then seeded random restarts
const auto result = mapf::plan_prioritized(instance, options);
if (result.solved()) { /* result.paths, result.sum_of_costs */ }
```

## Benchmark

From [docs/benchmarks/mapf.md](../benchmarks/mapf.md) (3 seeds per cell):

- Open 32×32: every instance solved up to 40 robots in about 5 ms, with a sum
  of costs within 0.1 % of the uncoordinated lower bound.
- Warehouse (one-lane aisles): every instance solved up to 20 robots, sum of
  costs 0.6–14 % above the lower bound, while CBS timed out on 3 of the 6
  instances with 15–20 robots.
- Narrow corridor and 8×8 congestion: a single priority order failed on 7 of
  24 instances; with up to 10 random restarts only 1 failure remained (8 robots
  in the corridor), including instances where CBS timed out.

## Known limitations (demonstrated)

- **Incompleteness.** On the 1×5 corridor with one passing pocket
  (`tests/unit/mapf/test_planners.cpp`, `FailsOnPocketSwapForEveryOrder`) both
  orders fail although CBS finds the optimum (SOC 11). The same happens on the
  four-robot intersection of the CBS animation.
- **Ordering-dependent suboptimality.** `OrderingChangesTheResult` shows
  different costs for different random orders on the same instance.
- Lower-priority agents may be forced into long detours or waits; there is no
  fairness guarantee.

## References

- M. Erdmann, T. Lozano-Pérez, "On Multiple Moving Objects," *Algorithmica*,
  2:477–521, 1987.
- D. Silver, "Cooperative Pathfinding," *Proc. AIIDE*, 2005.
- M. Bennewitz, W. Burgard, S. Thrun, "Finding and Optimizing Solvable
  Priority Schemes for Decoupled Path Planning Techniques for Teams of Mobile
  Robots," *Robotics and Autonomous Systems*, 41(2–3):89–99, 2002.
- R. Stern et al., "Multi-Agent Pathfinding: Definitions, Variants, and
  Benchmarks," *Proc. SoCS*, 2019.
