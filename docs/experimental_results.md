# Experimental Results

Run the program on your own machine and record the actual output below. Do not invent or reuse timing values from another computer.

| Test | Scenario | Expected behavior | Actual result from local run |
|---|---|---|---|
| 1 | Basic reachability | Returns `0 -> 1 -> 2 -> 3` | Fill after running |
| 2 | Bad-state avoidance | Avoids bad state `2`; uses `0 -> 3 -> 4 -> 5` | Fill after running |
| 3 | Safety/cost trade-off | Chooses the path with the lowest combined weighted objective | Record actual path and explain trade-off |
| 4 | Transition unavailable | Avoids unavailable transition `41` and uses alternate route | Fill after running |
| 5a/5b | Goal update | Replans for the changed goal | Fill after running |
| 6a/6b | Shortcut added | Replanning discovers the lower-cost shortcut | Fill after running |

## Metrics to record

For each run, record goal success, number of bad states visited (must be zero), total path cost, minimum distance to bad states, explored states, planning time, and cumulative reliability. If memory usage is required, measure it with a platform-specific profiler and record the tool used.

## Analysis prompts

1. Did each reachable test reach its goal?
2. Was any bad state included in a returned path? It should never be.
3. How did the safety weight affect the chosen route in Test 3?
4. What changed when a transition became unavailable?
5. Did the new shortcut reduce raw transition cost?
6. Compare planning time across the small test cases, while noting that such tiny graphs are not a meaningful large-scale benchmark.
