// Minimal downstream program: links individual exported module targets.

#include <multirobotcore/allocation/hungarian.hpp>
#include <multirobotcore/consensus/consensus.hpp>
#include <multirobotcore/graph/topologies.hpp>
#include <multirobotcore/mapf/conflict_based_search.hpp>

#include <iostream>

int main() {
  namespace mrc = multirobotcore;

  // Two robots swap places in a corridor; one yields into the passing bay.
  const auto map = mrc::mapf::GridMap::from_ascii({"....", "@.@@"});
  mrc::mapf::Instance instance{map, {{{0, 1}, {3, 1}}, {{3, 1}, {0, 1}}}};
  const auto plan = mrc::mapf::ConflictBasedSearch().solve(instance);

  mrc::allocation::CostMatrix costs(2, 2);
  costs << 4.0, 1.0, 2.0, 5.0;
  const auto assignment = mrc::allocation::solve_hungarian(costs);

  const auto graph = mrc::graph::ring_graph(4);
  mrc::consensus::StateMatrix x(4, 1);
  x << 0.0, 1.0, 2.0, 3.0;
  const auto run = mrc::consensus::run_consensus(graph, x, {0.3, 1000, 1e-9, false});

  std::cout << "CBS solved: " << plan.solved() << ", sum of costs " << plan.sum_of_costs
            << "; assignment cost " << assignment.total_cost << "; consensus value "
            << run.final_state(0, 0) << "\n";
  const bool ok = plan.solved() && assignment.total_cost == 3.0 && run.converged;
  return ok ? 0 : 1;
}
