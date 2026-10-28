# User Manual

## Requirements

- A C++17 compiler such as GCC, Clang, or MSVC.
- Optional: CMake 3.16 or later.

## Run the project

1. Extract the project folder.
2. Open a terminal in the project root.
3. Compile:

   ```bash
   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic src/main.cpp -o safe_planner
   ```

4. Run `./safe_planner` on Linux/macOS, or `./safe_planner.exe` in Windows PowerShell if that is the output filename.
5. Read the six test sections printed in the terminal.

## Change the problem

The demonstration problems are defined near the bottom of `src/main.cpp` in `main()`. Add or edit `State` objects, transitions, bad-state IDs, the initial state, and the goal state. State IDs must be unique. Embedding dimensions should be consistent. Transition cost must be nonnegative, safety must be between 0 and 1, and reliability must be greater than 0 and at most 1.

To simulate a dynamic update, modify a transition's `available` flag, update `goalState`, add a bad state, or insert a new transition, then call `planner.plan(problem)` again.

## Interpret the output

- `Status`: whether a safe path was found.
- `State path`: state IDs in visit order.
- `Transition IDs`: transition IDs used by the path.
- `Total transition cost`: sum of original transition costs, excluding penalties.
- `Minimum distance to bad state`: minimum Euclidean clearance over all path states.
- `Cumulative reliability`: product of transition reliability values.
- `Explored states`: states popped from the priority queue during the search.
- `Planning time (ms)`: measured time for the planning call; it depends on hardware and system load.

## Troubleshooting

- `No safe path found`: check that a directed path exists, transitions are available, and the goal is not isolated.
- `initial state or goal is marked as bad`: remove that state from the bad-state list or choose a different start/goal.
- Compilation errors: verify that the compiler supports C++17 and that the command is run from the project root.
