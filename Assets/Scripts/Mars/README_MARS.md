# MARS: what was missing and what was added

Paper: C. Tonola, M. Faroni, M. Beschi, N. Pedrocchi, "Anytime informed multi-path replanning strategy for complex environments," *IEEE Access* 11, 4105–4116, 2023. doi:10.1109/ACCESS.2023.3235652

MARS is a **replanning** method for **one** robot. "Multi-path" means it keeps several alternative paths to the same goal. When an obstacle blocks the current path, MARS quickly connects the robot to one of those stored paths, then uses any time left in its budget to improve the result. Before these changes, the project only had the preparation step: RRT-Connect plus shortcutting.

## How the grid works (what MARS is built on)
- `MazeGridGenerator`: a 25 × 15 maze (`0` = free, `1` = wall). Cell `(x, y)` sits at world position `origin + (x·cellSize, 0, y·cellSize)`, with cellSize = 1. The exit is cell (21, 0).
- `MazeNavigationGrid`: wraps the maze and provides:
  - `IsWalkable`, `CellToWorld` and `WorldToCell` (rounded to the nearest cell);
  - 8-way neighbours that don't cut corners;
  - `SetWalkable`, which fires `CellChanged` (D* Lite and DTA listen to it);
  - `GetRandomWorldPosition` for sampling.
- MARS works in **continuous space on the XZ plane**. Each segment is checked with `MarsCollisionChecker`: points every 0.2 cells must all be walkable, and diagonal corner cuts are rejected. Path cost is Euclidean length, as in the paper.
- **Added** `MazeNavigationGrid.Revision`, a counter that goes up every time a cell changes. MARS caches each edge's collision check and re-checks it only when the map has changed since. This is the paper's "connections already checked are not rechecked."

## Paper → code
| Paper | Code |
|---|---|
| Initial path: RRT-Connect | `MarsPlanner.PlanFirstPath` → `RRTConnectPlanner` |
| Initial path optimized with Informed RRT* + *wrapping* (Kim & Song) | `MarsInformedPlanner` (bounded by the path's cost) + `MarsPathOptimizer.Wrap` (**new**) |
| Set of precomputed paths, same start/goal (3 in the experiments) | `Settings.pathSetSize = 3`, `MarsPlanner.PlanAlternative` |
| Every path merged into one tree/graph G; first- and second-order connections | `MarsPlanner.Node / Edge / Path`. Connection edges are marked `IsConnection`. |
| Robot position projected onto the path; Q1 = nodes between the robot and the obstacle | `Replan` → `Project`, `q1` |
| P = paths with their cost-to-go; Q2 sorted by distance to x_n | `CollectTargets`, `q2` |
| Pruning: `g(x_n) + ‖x_n − x_j‖ + c2go(x_j) ≥ best` → skip | first check inside the `foreach` loop |
| informedPlan: reuse a connection in G, else grow a subtree from x_n inside the informed ellipse | `InformedPlan`: cache → direct segment → `MarsInformedPlanner.TryConnect` (ellipse sampling; uniform while the cost is infinite) |
| Subtree reuse | One tree per x_n, reused for every x_j during the same replan |
| searchBetterPath: DFS over G with lazy collision checks, anytime | `SearchBetterPath` / `Dfs` (branch and bound with the heuristic ‖x − goal‖) |
| Time budget (200 ms) | `Settings.replanBudgetMs = 200` |
| New paths are added to P and reused later | `AddReplannedPath` (keeps up to `maxStoredPaths`) |
| Three threads: execution (500 Hz), collision checking (30 Hz), replanning | `MarsAgent`: `GridAgentMovement` moves the agent, the path is checked at `checkRate = 30` Hz, and the agent replans when blocked |
| Robot stops if replanning fails | Safety stop: the agent stays stopped and retries at the next check |
| Metrics: success rate, collisions (safety stops), normalized length | `ReplanningBenchmarkManager` |

## Where this differs from the paper (say this in your report)
1. **2D point robot on a grid maze** instead of 3D/6–18-DoF arms, so cost is length on the XZ plane.
2. **How the alternative paths are made:** the paper doesn't specify this. Here, each new path is planned with the middle ±20 % (`diversityBand`) of every previous path temporarily forbidden, so it has to take another corridor. If no alternative exists, the band is halved; if there still isn't one, MARS keeps fewer paths. Near-duplicates (≥ 90 % of cells shared) are discarded.
3. **Single thread:** Unity calls are synchronous, so the replan runs inside the collision check instead of on its own thread. The 200 ms budget is still enforced with a stopwatch.
4. **Subtrees are reused within one replan** (across all its x_j). They're dropped between replans, because the map changed in between. Connections that were found *are* kept in G and reused in later replans.
5. **Obstacles appear and stay** (blocked cells). The paper uses moving obstacles.

## Files
| File | Status |
|---|---|
| `MarsPlanner.cs` | **new**: MARS itself (path set, graph, replanning, anytime search) |
| `MarsInformedPlanner.cs` | **new**: Informed RRT* with ellipse sampling and a reusable tree |
| `MarsPathfinder.cs` | **new**: component with the same API as D* Lite, RRT and DTA (`Initialize`, `TryGetPath`) |
| `MarsAgent.cs` | **new**: live demo; the agent follows the path and replans when blocked |
| `ReplanObstacleUtility.cs` | **new**: places obstacles ahead on the path without sealing the goal |
| `MarsPathOptimizer.cs` | added `Wrap()`; `Optimize()` unchanged |
| `BenchmarkObstacleManager.cs` | **renamed** from `BenchMarkObstacle.cs` (Unity needs file name = class name); works without a prefab |
| `RRTConnectTest.cs` | **renamed** from `RRTConnectTest1.cs` (same reason) |
| `MarsSamplingTest.cs` | now seeded (it used `UnityEngine.Random` without a seed) |
| `../MazeNavigationGrid.cs` | **renamed** from `MazaNavigationGrid.cs`; added `Revision` |
| `MarsBenchmarkManager.cs`, `RRTConnectPlanner.cs`, `MarsSampler.cs`, `MarsCollisionChecker.cs` | unchanged |

Each renamed file keeps its `.meta` file, so its GUID and all scene references survive. Copy both files together, and delete the old-named files from your project.

## Try it live
1. Open the scene with the maze (`MazeGridGenerator` + `MazeNavigationGrid`).
2. Add **MarsPathfinder** to an empty GameObject and drag the grid into it.
3. Make an agent GameObject with **GridAgentMovement** and **MarsAgent**. Assign the Pathfinder, a Goal transform at cell (21, 0) and the Movement.
4. Add **BenchmarkObstacleManager** (grid + an optional cube prefab) and assign it to the MarsAgent's *Obstacles* field.
5. Press Play. Then either turn on *Auto Drop Obstacles*, or use the MarsAgent's ⋮ menu → **Drop Obstacle Ahead**.

In the Scene view: gray = alternative paths, green = current path, yellow = paths created by replanning. The Console prints each replan's time, cost, connection attempts and whether the graph search improved it.

## Measure it like the paper
Add **ReplanningBenchmarkManager** to the benchmark scene, and leave MasterBenchmarkManager disabled while it runs. It runs MARS, RRT-Connect from scratch, D* Lite, TA-RRT* and DTA one after another. In each run, obstacles appear on the agent's path while it moves; obstacle placement uses the same seed for every algorithm. It writes:
- `Replanificacion_Resultados.csv`: one row per run.
- `Replanificacion_Resumen.csv`: per algorithm, the success rate, normalized length (traversed ÷ initial), safety stops, and mean, median and maximum replan time.

"RRT-Connect from scratch" is the obvious baseline: it's what your MARS did before. If MARS beats it on replan time while keeping the success rate, that is the paper's main claim, reproduced on your maze.
