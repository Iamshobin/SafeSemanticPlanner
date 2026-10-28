# Test Plan

The `main()` function contains six manual demonstration tests corresponding to the assignment's illustrative cases.

| Test | Input change | Check |
|---|---|---|
| 1 | Simple directed chain | Success and state path `0,1,2,3` |
| 2 | State 2 is forbidden | Returned path must not contain 2 |
| 3 | Two routes with different clearance and cost | Inspect chosen route and reported clearance |
| 4 | Disable transition 41 | Returned path must use the alternate route |
| 5 | Change goal from 2 to 3 | Second plan ends at 3 |
| 6 | Add transition 0 to 3 | Second plan should use the shortcut |

Recommended additional assertions for a formal test suite: every adjacent pair in `statePath` must match a selected available transition; no state in `statePath` may be bad; the first and last states must equal initial and goal; and `transitionPath.size() + 1 == statePath.size()` for nontrivial successful paths.
