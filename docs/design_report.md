# Design Report: Safe Semantic Planner

## 1. Problem statement

Given a finite set of states represented by Cartesian vectors, a directed set of transitions, an initial state, a goal state, and a set of bad states, find a path that reaches the goal without visiting any bad state. Each transition has a cost, safety score, reliability score, and availability flag. The environment can change, requiring a new plan.

## 2. State representation

Each `State` contains a unique unsigned integer ID and a vector of `double` values. The example uses two-dimensional embeddings `(x,y)`, but the Euclidean-distance helper supports any consistent vector dimension. Each `Transition` has a unique ID, source and destination IDs, nonnegative cost, safety in `[0,1]`, reliability in `(0,1]`, and an availability flag.

## 3. Data structures

- `unordered_map<StateId, const State*>` provides average O(1) state lookup.
- `unordered_set<StateId>` stores bad-state membership.
- An adjacency map stores available valid outgoing transitions.
- A min-priority queue selects the unsettled state with the lowest accumulated objective score.
- Distance, predecessor-state, and predecessor-transition maps support shortest-path search and reconstruction.

## 4. Algorithm

The implementation uses Dijkstra's algorithm. It excludes unavailable transitions and any transition whose source or destination is a bad state. It also rejects malformed transitions. The start state is assigned score zero and placed in a min-priority queue. The algorithm repeatedly removes the lowest-score state, relaxes outgoing transitions, and records predecessors when a better score is found. Search ends when the goal is removed from the queue or the frontier is empty. If the goal is reached, predecessor maps reconstruct the state and transition paths.

The assignment suggests LPA* or D* Lite. Dijkstra was selected here for a compact, understandable baseline with nonnegative weighted objectives. Dynamic changes are supported by rerunning the planner on the updated problem. This demonstrates replanning behavior, but does not preserve incremental search state as LPA*/D* Lite would.

## 5. Objective and heuristic

No heuristic is used; Dijkstra is equivalent to uniform-cost search with heuristic `h(n)=0`. The objective edge weight is:

`edgeScore = cost + wD/(distanceToBad + epsilon) + wR*(-ln(reliability)) + wS*(1 - transitionSafety)`

Default values are `wD=1.0`, `wR=1.0`, `wS=0.25`, and `epsilon=0.1`. Distance to bad states is the minimum Euclidean distance between a candidate state's embedding and all defined bad-state embeddings. When no bad states are defined, the distance penalty is zero. Reliability is represented as a value in `(0,1]`; its negative logarithm is nonnegative and grows as reliability decreases.

This scalar objective balances route cost, proximity to bad states, reliability, and transition safety. Weight tuning affects the trade-off. It does not guarantee that the globally maximum-clearance path is selected for every graph; the reported minimum path distance lets users inspect the resulting safety margin. Bad states are never allowed, regardless of weights.

## 6. Safety computation

After a path is reconstructed, the planner calculates the Euclidean distance from each path state to its nearest bad state and reports the minimum of those distances. If there are no bad states, the result is reported as N/A in the console and represented internally as infinity. The planner also reports the product of transition reliabilities along the path as a simple cumulative reliability estimate, assuming transition reliabilities can be multiplied.

## 7. Dynamic environment and replanning

Updates may change the goal, bad-state set, transition availability, or transition list. The caller updates the `PlanningProblem` and calls `plan()` again. The test program demonstrates an unavailable transition, a changed goal, and a newly added shortcut. This is simple and correct for small graphs but repeats work. An incremental LPA*/D* Lite implementation could retain `g`/`rhs` values (LPA*) or `g` and `rhs` with a keyed priority queue (D* Lite), update affected vertices after graph changes, and reuse previous search information.

## 8. Complexity

Let `V` be the number of states, `E` the number of transitions, `d` the embedding dimension, and `B` the number of bad states.

- Building maps and adjacency lists: expected `O(V + E)`.
- Each distance-to-bad calculation: `O(Bd)`.
- Edge relaxation can call distance-to-bad for each examined transition, giving a straightforward worst-case bound of `O((V + E) log V + E B d)` for this implementation (plus path/report calculations).
- Space complexity: `O(V + E + Bd)` for graph/search structures and embeddings; the distance calculation itself uses `O(d)` extra space.

For a fixed low dimension and modest bad-state count, Dijkstra's principal graph-search cost is `O((V+E) log V)`. A spatial index or cached distances could reduce repeated safety-distance calculations.

## 9. Experimental evaluation

The executable runs six scenarios aligned with the assignment. It reports success, path, transition IDs, raw path cost, minimum bad-state distance, cumulative reliability, explored states, and planning time. Timing and explored-state counts should be copied from an actual local run into `experimental_results.md`; timing varies by computer and compiler. Additional memory profiling can be performed with platform tools if required by the instructor.

## 10. Limitations and future work

- Implement true LPA* or D* Lite incremental replanning if specifically required.
- Add automated assertions for expected paths and no bad-state visits.
- Add file-based input (CSV/JSON) and a visualization of the graph.
- Tune objective weights using a documented parameter study.
- If the project must include machine learning, learn a heuristic or edge-risk estimate from a labeled dataset and compare it against the current hand-designed score; this implementation itself does not train an ML model.
