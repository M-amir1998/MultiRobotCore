# Graph foundations

Module: `MultiRobotCore::graph` — headers `graph/graph.hpp`, `graph/matrices.hpp`,
`graph/connectivity.hpp`, `graph/topologies.hpp`.

## Purpose

Communication, sensing and formation relationships between robots are graphs.
Most guarantees of distributed coordination (consensus, formation control,
distributed optimisation) are statements about the algebraic and topological
properties of these graphs, so the library represents them explicitly and
exposes the quantities that the theory refers to.

## Conventions

A graph has `N` nodes indexed `0 .. N-1` (one per robot). A **directed edge
`j -> i` means that information flows from `j` to `i`**: robot `i` receives or
measures the state of robot `j`. This is the convention of Ren & Beard (2005)
and Olfati-Saber, Fax & Murray (2007). An undirected edge `{i, j}` is stored as
the two directed edges. Weights are finite and strictly positive; self-loops
are rejected.

| Quantity | Definition |
|---|---|
| Adjacency | `A(i, j) = w(j -> i)` |
| In-neighbours | `N_i = { j : j -> i }` — the robots `i` listens to |
| In-degree | `d_i = sum_j A(i, j)` |
| Laplacian | `L = D - A`, `D = diag(d_i)` |
| Normalised Laplacian (undirected) | `L_sym = I - D^{-1/2} A D^{-1/2}` |
| Incidence (oriented) | `B(target, e) = +1`, `B(source, e) = -1` |
| Perron matrix | `P = I - eps L` |
| Metropolis weights | `W(i, j) = 1 / (1 + max(deg_i, deg_j))`, `W(i, i) = 1 - sum_j W(i, j)` |

## Mathematics

Properties verified by the test suite (`tests/unit/graph`):

- `L 1 = 0` for every graph (zero row sums).
- For undirected graphs `L` is symmetric positive semidefinite and
  `L = B W B^T` for any edge orientation, where `W` is the diagonal matrix of
  edge weights.
- The multiplicity of the eigenvalue 0 of an undirected Laplacian equals the
  number of connected components; the second-smallest eigenvalue `lambda_2`
  (algebraic connectivity, Fiedler 1973) is positive if and only if the graph
  is connected. Closed forms used as test oracles:
  - complete graph `K_n`: `lambda_2 = n`;
  - path `P_n`: `lambda_2 = 2 (1 - cos(pi / n))`;
  - cycle `C_n`: `lambda_2 = 2 (1 - cos(2 pi / n))`;
  - star: `lambda_2 = 1`.
- The spectrum of `L_sym` lies in `[0, 2]`.
- `P = I - eps L` is non-negative and row stochastic when `0 < eps <= 1 / max_i d_i`;
  Metropolis weights are symmetric and doubly stochastic.
- A digraph supports first-order consensus if and only if it contains a
  **directed spanning tree** (Ren & Beard 2005); `spanning_tree_roots()`
  returns every node from which all other nodes are reachable along the
  direction of information flow.

## Connectivity analyses

| Function | Result |
|---|---|
| `connected_components` | weakly connected components, sorted |
| `strongly_connected_components` | iterative Tarjan (1972) |
| `is_connected` | connected (undirected) / strongly connected (directed) |
| `has_spanning_tree`, `spanning_tree_roots` | consensus reachability |
| `hop_distances`, `diameter` | breadth-first search; diameter bounds flooding rounds |
| `algebraic_connectivity`, `fiedler_vector`, `laplacian_spectrum` | self-adjoint eigensolver (Eigen) |

## Implementation notes

- Adjacency lists are kept sorted by neighbour id, so iteration order (and
  therefore every result) is deterministic.
- Dynamic updates (`add_edge` updates the weight of an existing edge,
  `remove_edge`, `isolate_node` for failed robots, `add_node`) keep the in- and
  out-lists consistent.
- Dense matrices are produced on demand; algorithms that only need `-L x` use
  the neighbour lists directly (`O(|E|)` per product).

## Complexity

Components, diameter and spanning-tree roots are BFS-based (`O(N (N + |E|))`
for the all-pairs quantities). Spectral quantities use a dense symmetric
eigensolver, `O(N^3)`, adequate for teams of up to a few hundred robots.

## References

- M. Mesbahi, M. Egerstedt, *Graph Theoretic Methods in Multiagent Networks*,
  Princeton University Press, 2010.
- F. Bullo, J. Cortés, S. Martínez, *Distributed Control of Robotic Networks*,
  Princeton University Press, 2009.
- M. Fiedler, "Algebraic connectivity of graphs," *Czechoslovak Mathematical
  Journal*, 23(2):298–305, 1973.
- R. Tarjan, "Depth-first search and linear graph algorithms," *SIAM Journal on
  Computing*, 1(2):146–160, 1972.
- W. Ren, R. W. Beard, "Consensus seeking in multiagent systems under
  dynamically changing interaction topologies," *IEEE Transactions on Automatic
  Control*, 50(5):655–661, 2005.
- L. Xiao, S. Boyd, S. Lall, "A scheme for robust distributed sensor fusion
  based on average consensus," *Proc. IPSN*, 2005 (Metropolis weights).
