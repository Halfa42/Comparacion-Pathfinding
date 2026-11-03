# Master Benchmark (MasterBenchmarkManager)

`MasterBenchmarkManager.cs` runs every algorithm on the same scenario and writes all the results to one place.

**How it runs:** it goes through the algorithms one at a time, in the order set in **Execution Order**. Each algorithm finishes all of its runs before the next one starts:

```
CBS        warm-up -> run 1 ... run 10
FlowField  warm-up -> run 1 ... run 10
DStarLite  warm-up -> run 1 ... run 10
MARS       warm-up -> run 1 ... run 10
TARRTStar  warm-up -> run 1 ... run 10
DTA        warm-up -> run 1 ... run 10
-> write the summary
```

If **Agent Counts** has more than one value (for example `{2, 4, 6, 8, 10}`), each algorithm runs all its runs at 2 agents, then all at 4 agents, and so on, before the next algorithm starts.

## Scenario
This is the same scenario the existing CBS, FlowField, D* Lite and MARS benchmarks use:
- Maze: the one `MazeGridGenerator` builds, read through `MazeNavigationGrid`. CBS and FlowField get the same maze turned into the `int[,]` format they expect.
- Starts: `(13 + i, 13)` for each agent i. Goal: the exit at `(21, 0)`.
- 10 runs, 10 agents and 1 unrecorded warm-up run per algorithm by default.

## Setup (about 2 minutes)
1. Copy the `Benchmark` folder into `Assets/Scripts/`.
2. Make a new scene, or duplicate one of your algorithm scenes. It needs:
   - a GameObject with **MazeGridGenerator**, and
   - **MazeNavigationGrid**, with its `Maze` field assigned.
3. Add **MasterBenchmarkManager** to any GameObject.
4. Press Play.

You don't have to add the algorithm components yourself. If CBSManager, FlowFieldManager, DStarLitePathfinder, RRTPathfinder, DTAPathfinder or DTATerrainMap is missing, the benchmark finds it in the scene. If it isn't in the scene either, the benchmark adds it to its own GameObject with default settings and connects it to the grid. If you want the exact settings your teammates tuned (iterations, radii and so on), put their configured components in the scene or drag them into the reference slots.

The old per-algorithm benchmark and test scripts all start in `Start()`. The benchmark disables them so they don't run at the same time; you can turn this off with *Disable Other Benchmark Scripts*.

A progress label appears in the top-left of the Game view. You can also re-run the benchmark from the component's ⋮ menu → **Run Master Benchmark**.

## Output (in `Assets/` by default)
**`Benchmark_Todos_Resultados.csv`** has one row per algorithm, agent count and run. Each row is written as soon as its run finishes, so stopping Play mode partway still keeps the runs that are done.

| Column | Meaning |
|---|---|
| Algoritmo, Agentes, Ejecucion | which algorithm, how many agents, which run |
| Exito | whether every agent got a path |
| Nodos Expandidos | the algorithm's own search-effort counter (see below) |
| Tiempo (ms) | planning time for all agents. Frame waits between agents are not counted. |
| Suma de Costos / Makespan | the sum and the maximum of time steps per agent (1 cell per step) |
| Longitud Total (unidades) | the total geometric length of all paths, in world units |
| RAM Pico (MB) | the highest managed memory reading during the run, above the starting level (approximate) |
| Conflictos | vertex and swap conflicts, using the same model as CBS (an agent disappears when it reaches the goal) |
| Movimientos Invalidos | moves into walls, illegal jumps or corner cuts. For continuous paths, the segments that fail `MarsCollisionChecker`. |
| Solucion Valida | Exito, with 0 conflicts and 0 invalid moves |
| Semilla | the seed used (only for MARS and TA-RRT*) |

**`Benchmark_Todos_Resumen.csv`** has one row per algorithm and agent count: success rate, the mean, standard deviation, median, minimum and maximum time, and the means of nodes, SOC, makespan, length, RAM, conflicts and invalid moves. Time and RAM are averaged over all runs. Path metrics are averaged over successful runs only.

The same summary is also printed as a table in the Console when the benchmark finishes.

## Things to say when you present the comparison
- **The algorithms solve different problems.** CBS and FlowField are *multi-agent* and move in 4 directions, so they avoid collisions between agents. D* Lite, DTA, MARS and TA-RRT* plan *each agent separately*, so conflicts are expected. That is a real finding, not a bug.
- **Movement differs too.** D* Lite and DTA move in 8 directions, where a diagonal counts as 1 step in SOC and makespan but √2 in length. MARS and TA-RRT* move in continuous space, and their paths are turned into cells at 1 cell per step (the same method `MarsBenchmarkManager` uses). For path quality, compare **Longitud Total**: it is the fairest geometric measure across all six.
- **"Nodos Expandidos" counts different things:** CBS counts high-level constraint-tree nodes; FlowField counts BFS cells; D* Lite counts queue pops; MARS and TA-RRT* count tree nodes; DTA counts abstract plus refinement A* expansions. Compare it within one algorithm (for example, how it grows with the number of agents), not across algorithms.
- **Randomness:** MARS and TA-RRT* use a different seed for each run (`seed + run*1000 + agent`), so you get a reproducible spread of results. TA-RRT*'s seed is set through reflection on `RRTPathfinder.seed`. Turn *Ta Rrt Vary Seed Per Run* off to use the component's own seed for every run.
- **DTA:** the abstraction is rebuilt at the start of every run, and that cost is included in the time, because every other algorithm starts from scratch. Turn *Dta Rebuild Abstraction Each Run* off to measure only the cost of queries on a map that is already built.
- **TA-RRT\* is the slow one:** it always uses its whole budget of 12,000 iterations. Expect it to take a few minutes in total. The Editor stays responsive between agents.
