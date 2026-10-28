# Safe Semantic Planner in a Finite Cartesian State Space

**Course:** PCCST503 – Machine Learning, Assignment 1  
**Language:** C++17  
**Approach:** Dijkstra graph search with a safety/reliability-aware edge objective and replanning after updates.

## Overview

This project plans a route from an initial state to a goal state in a directed graph. Each state has a Cartesian embedding. Each transition stores cost, safety, reliability and availability. Bad states are never entered. The demo runs six assignment-aligned scenarios, including reachability, bad-state avoidance, safety/cost trade-off, unavailable transitions, goal updates, and added shortcuts.

> **Algorithm note:** The assignment suggests LPA* or D* Lite. This implementation uses Dijkstra's algorithm and recomputes a plan after each change. It is a clear baseline that satisfies the planning behavior and demonstrates dynamic replanning, but it does **not** implement incremental LPA*/D* Lite data reuse. If your instructor requires one of those algorithms specifically, this should be upgraded before submission.

## Build and run

### Option A: g++ (Linux, macOS, or MinGW on Windows)

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic src/main.cpp -o safe_planner
./safe_planner
```

On Windows PowerShell, run `./safe_planner.exe` after compiling to that name.

### Option B: CMake

```bash
cmake -S . -B build
cmake --build build
./build/safe_planner
```

Windows executable may be at `build/Debug/safe_planner.exe` or `build/Release/safe_planner.exe`, depending on the generator.

## Data model

- `State`: ID and vector embedding, e.g. `(x, y)`.
- `Transition`: ID, source, destination, nonnegative cost, safety in `[0,1]`, reliability in `(0,1]`, and availability flag.
- `PlanningProblem`: initial and goal IDs, bad-state IDs, states, and directed transitions.
- `PlanningResult`: success flag, state and transition paths, total transition cost, minimum distance to a bad state, cumulative reliability, explored-state count, and elapsed planning time.

## Objective used

The shortest-path search minimizes the sum of these edge weights:

`edgeScore = cost + wD/(distanceToBad + epsilon) + wR*(-ln(reliability)) + wS*(1 - transitionSafety)`

The default weights are `wD=1.0`, `wR=1.0`, `wS=0.25`, and `epsilon=0.1`. The distance penalty favors states farther from bad states; the reliability penalty discourages unreliable transitions; and the transition-safety penalty discourages low-safety transitions. Bad states are excluded entirely. The result separately reports raw transition cost and the minimum Euclidean distance to any bad state on the chosen path.

## Test scenarios

The program prints results for all six scenarios. Timing is machine-dependent; do not copy example timing values into a report. Run the program on your machine and record its actual output in `docs/experimental_results.md`.

## Repository layout

```text
safe_semantic_planner/
├── src/main.cpp
├── docs/design_report.md
├── docs/experimental_results.md
├── docs/user_manual.md
├── tests/test_plan.md
├── CMakeLists.txt
├── .gitignore
└── README.md
```

## Limitations

1. Replanning reruns Dijkstra from scratch after each update.
2. Safety is a weighted preference, not a hard minimum-distance constraint. Bad states themselves are strictly forbidden.
3. The implementation skips malformed transitions (negative cost, out-of-range safety, or invalid reliability) rather than failing the entire problem.
4. The sample is a finite graph; it does not learn a heuristic from data, so this is a graph-planning assignment implementation rather than a trained ML model.

## License

For coursework and learning. Add your institution's required license or attribution if applicable.
