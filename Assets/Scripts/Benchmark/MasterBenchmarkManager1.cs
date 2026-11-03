using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using UnityEngine;
using Debug = UnityEngine.Debug;

/// <summary>
/// Master benchmark: runs every pathfinding algorithm on the SAME maze, starts and goal,
/// one algorithm at a time. Algorithm 1 completes all of its runs, then algorithm 2
/// starts, and so on, in the order given by "Execution Order".
///
/// Scenario (identical to the existing per-algorithm benchmarks):
///   - Maze: the one built by MazeGridGenerator (read through MazeNavigationGrid).
///   - Starts: (13 + i, 13) for i = 0..agents-1.   Goal: exit at (21, 0).
///
/// Output (in Application.dataPath unless "Output Folder" is set):
///   - Benchmark_Todos_Resultados.csv : one row per (algorithm, agents, run).
///   - Benchmark_Todos_Resumen.csv    : one row per (algorithm, agents) with mean / std / median...
///
/// Setup: put this on any GameObject in a scene that has MazeGridGenerator + MazeNavigationGrid.
/// Missing algorithm components are found in the scene or, if "Auto Create Missing" is on,
/// added to this GameObject and wired to the grid automatically.
/// </summary>
public class MasterBenchmarkManager : MonoBehaviour
{
    public enum Algorithm
    {
        CBS,
        FlowField,
        DStarLite,
        RRTConnect, // RRT-Connect + shortcut optimizer (what the old MarsBenchmarkManager measured)
        MARS,       // MarsPlanner offline phase: path set (RRT-Connect + Informed RRT* + wrapping, alternatives)
        TARRTStar,  // RRTPathfinder (terrain-aware RRT*)
        DTA         // Dynamic Terrain Abstraction
    }

    // ------------------------------------------------------------------ settings

    [Header("Execution order (each algorithm finishes all its runs before the next starts)")]
    [SerializeField] private List<Algorithm> executionOrder = new List<Algorithm>
    {
        Algorithm.CBS, Algorithm.FlowField, Algorithm.DStarLite,
        Algorithm.RRTConnect, Algorithm.MARS, Algorithm.TARRTStar, Algorithm.DTA
    };

    [Header("Scenario (same as the existing benchmarks)")]
    [SerializeField, Min(1)] private int numberOfRuns = 10;
    [Tooltip("One full block of runs per value. {10} = same as the existing benchmarks. " +
             "Use e.g. {2,4,6,8,10} to measure how each algorithm scales.")]
    [SerializeField] private int[] agentCounts = { 10 };
    [SerializeField] private Vector2Int firstStartCell = new Vector2Int(13, 13);
    [SerializeField] private Vector2Int goalCell = new Vector2Int(21, 0);
    [Tooltip("Unrecorded runs per algorithm before measuring (JIT / cache warm-up).")]
    [SerializeField, Min(0)] private int warmupRuns = 1;

    [Header("Run control")]
    [SerializeField] private bool runOnStart = true;
    [Tooltip("Wait one frame between agents of single-agent planners so the Editor stays responsive. " +
             "The wait is NOT included in the measured time.")]
    [SerializeField] private bool yieldBetweenAgents = true;
    [Tooltip("Disables the old per-algorithm benchmark/test scripts in the scene so they don't run at the same time.")]
    [SerializeField] private bool disableOtherBenchmarkScripts = true;
    [Tooltip("Empty = Application.dataPath (the Assets folder).")]
    [SerializeField] private string outputFolder = "";

    [Header("References (searched in the scene if empty)")]
    [SerializeField] private MazeNavigationGrid grid;
    [SerializeField] private CBSManager cbs;
    [SerializeField] private FlowFieldManager flowField;
    [SerializeField] private DStarLitePathfinder dStarLite;
    [SerializeField] private RRTPathfinder taRrtStar;
    [SerializeField] private DTAPathfinder dta;
    [Tooltip("Adds missing algorithm components to this GameObject and connects them to the grid.")]
    [SerializeField] private bool autoCreateMissing = true;

    [Header("MARS (offline phase; the replanning itself is measured by ReplanningBenchmarkManager)")]
    [SerializeField] private MarsPlanner.Settings marsSettings = new MarsPlanner.Settings();

    [Header("RRT-Connect (same defaults as MarsBenchmarkManager)")]
    [SerializeField] private int marsSeed = 12345;
    [SerializeField] private bool marsVarySeedPerRun = true;
    [SerializeField] private float marsStepSize = 0.75f;
    [SerializeField] private int marsMaxIterations = 5000;
    [SerializeField] private bool marsOptimizePath = true;
    [SerializeField] private int marsOptimizationIterations = 200;

    [Header("TA-RRT*")]
    [Tooltip("Like MARS: seed + run*1000 + agent. Off = the RRTPathfinder's own seed for every run " +
             "(then every run returns the same path and only the time varies).")]
    [SerializeField] private bool taRrtVarySeedPerRun = true;
    [SerializeField] private int taRrtSeed = 12345;

    [Header("DTA")]
    [Tooltip("Rebuild the terrain abstraction at the start of every run so its construction cost is " +
             "measured (fair, since every other algorithm starts from scratch). Off = only the first build is paid.")]
    [SerializeField] private bool dtaRebuildAbstractionEachRun = true;

    // ------------------------------------------------------------------ state

    private const string RunsFile = "Benchmark_Todos_Resultados.csv";
    private const string SummaryFile = "Benchmark_Todos_Resumen.csv";
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private int[,] mazeMatrix;      // [y, x], 0 = free, 1 = wall (format used by CBS / FlowField)
    private bool running;
    private string status = "";

    private class RunResult
    {
        public bool Ok = true;
        public string Reason = "";
        public double Ms;
        public long Nodes;
        public long PeakBytes;
        public int Seed = -1;
        public List<List<Vector2Int>> CellPaths = new List<List<Vector2Int>>();   // one cell per time step
        public List<List<Vector3>> ContinuousPaths;                               // MARS / TA-RRT* only

        public void SampleMemory() => PeakBytes = Math.Max(PeakBytes, GC.GetTotalMemory(false));

        public void Fail(string reason)
        {
            Ok = false;
            Reason = reason;
        }
    }

    private class Row
    {
        public Algorithm Alg;
        public int Agents, Run, Seed;
        public bool Ok, Valid;
        public long Nodes;
        public double Ms, RamMB, Length;
        public int Soc, Makespan, Conflicts, InvalidMoves;
        public string Reason;
    }

    // ------------------------------------------------------------------ Unity entry points

    private void Awake()
    {
        if (disableOtherBenchmarkScripts) DisableOtherScripts();
    }

    private IEnumerator Start()
    {
        if (!runOnStart) yield break;
        yield return null; // let every Awake/Start (maze building) finish first
        yield return StartCoroutine(RunAll());
    }

    [ContextMenu("Run Master Benchmark")]
    public void RunFromMenu()
    {
        if (!Application.isPlaying) { Debug.LogWarning("MasterBenchmark: enter Play mode first."); return; }
        if (running) { Debug.LogWarning("MasterBenchmark: already running."); return; }
        StartCoroutine(RunAll());
    }

    private void OnGUI()
    {
        if (string.IsNullOrEmpty(status)) return;
        GUI.Label(new Rect(10, 10, 900, 25), status);
    }

    // ------------------------------------------------------------------ main loop

    private IEnumerator RunAll()
    {
        running = true;
        if (!ResolveReferences()) { running = false; yield break; }

        mazeMatrix = BuildMazeMatrix();

        string folder = string.IsNullOrEmpty(outputFolder) ? Application.dataPath : outputFolder;
        Directory.CreateDirectory(folder);
        string runsPath = Path.Combine(folder, RunsFile);
        string summaryPath = Path.Combine(folder, SummaryFile);

        var allRows = new List<Row>();
        var totalClock = Stopwatch.StartNew();

        using (var writer = new StreamWriter(runsPath, false))
        {
            writer.WriteLine("Algoritmo,Agentes,Ejecucion,Exito,Nodos Expandidos,Tiempo (ms),Suma de Costos,Makespan," +
                             "Longitud Total (unidades),RAM Pico (MB),Conflictos,Movimientos Invalidos,Solucion Valida,Semilla,Motivo Fallo");
            writer.Flush();

            for (int a = 0; a < executionOrder.Count; a++)
            {
                Algorithm alg = executionOrder[a];
                if (!IsAvailable(alg, out string missing))
                {
                    Debug.LogWarning($"MasterBenchmark: skipping {alg} ({missing}).");
                    continue;
                }

                Debug.Log($"===== MasterBenchmark: [{a + 1}/{executionOrder.Count}] {alg} =====");

                foreach (int agents in agentCounts)
                {
                    if (!BuildScenario(agents, out var starts, out string scenarioError))
                    {
                        Debug.LogError($"MasterBenchmark: {agents} agents is not valid: {scenarioError}");
                        continue;
                    }

                    // Warm-up (not recorded)
                    for (int w = 0; w < warmupRuns; w++)
                    {
                        status = $"{alg} | {agents} agents | warm-up {w + 1}/{warmupRuns}";
                        var discard = new RunResult();
                        yield return StartCoroutine(RunOnce(alg, starts, -(w + 1), discard));
                    }

                    // Measured runs
                    for (int run = 1; run <= numberOfRuns; run++)
                    {
                        status = $"{alg} | {agents} agents | run {run}/{numberOfRuns}   " +
                                 $"(algorithm {a + 1}/{executionOrder.Count})";

                        GC.Collect();
                        GC.WaitForPendingFinalizers();
                        GC.Collect();
                        long startMemory = GC.GetTotalMemory(true);

                        var result = new RunResult { PeakBytes = startMemory };
                        yield return StartCoroutine(RunOnce(alg, starts, run, result));
                        result.SampleMemory();

                        Row row = Evaluate(alg, agents, run, result, startMemory);
                        allRows.Add(row);
                        writer.WriteLine(FormatRow(row));
                        writer.Flush(); // partial results survive if Play mode is stopped

                        Debug.Log(string.Format(Inv,
                            "{0} | {1} agentes | ejecución {2}: {3} | nodos={4} | {5:F2} ms | SOC={6} | makespan={7} | conflictos={8}",
                            alg, agents, run, row.Ok ? "OK" : "FALLÓ (" + row.Reason + ")",
                            row.Nodes, row.Ms, row.Soc, row.Makespan, row.Conflicts));

                        yield return null;
                    }
                }
            }
        }

        WriteSummary(summaryPath, allRows);
        totalClock.Stop();

        status = $"Benchmark finished in {totalClock.Elapsed.TotalSeconds:F1} s. Results: {runsPath}";
        Debug.Log("¡Benchmark maestro terminado!\n  Ejecuciones: " + runsPath + "\n  Resumen:     " + summaryPath +
                  "\n" + SummaryTable(allRows));
        running = false;
    }

    private IEnumerator RunOnce(Algorithm alg, List<Vector2Int> starts, int run, RunResult r)
    {
        switch (alg)
        {
            case Algorithm.CBS:       RunCBS(starts, r); break;
            case Algorithm.FlowField: RunFlowField(starts, r); break;
            case Algorithm.DStarLite: yield return StartCoroutine(RunDStarLite(starts, r)); break;
            case Algorithm.RRTConnect: yield return StartCoroutine(RunRrtConnect(starts, run, r)); break;
            case Algorithm.MARS:      yield return StartCoroutine(RunMars(starts, run, r)); break;
            case Algorithm.TARRTStar: yield return StartCoroutine(RunTaRrtStar(starts, run, r)); break;
            case Algorithm.DTA:       yield return StartCoroutine(RunDta(starts, r)); break;
        }
    }

    // ------------------------------------------------------------------ algorithm adapters

    // CBS: multi-agent, coordinated. One call plans every agent.
    private void RunCBS(List<Vector2Int> starts, RunResult r)
    {
        var goals = Goals(starts.Count);
        var sw = Stopwatch.StartNew();
        var paths = cbs.FindPaths(mazeMatrix, starts, goals);
        sw.Stop();

        r.Ms = sw.Elapsed.TotalMilliseconds;
        r.Nodes = cbs.TotalNodesExpanded;
        r.PeakBytes = Math.Max(r.PeakBytes, cbs.PeakMemoryBytes);
        if (paths == null || !cbs.Success) { r.Fail(cbs.FailureReason); return; }
        r.CellPaths = paths;
    }

    // Flow field: multi-agent, one shared field + step simulation with priority rules.
    private void RunFlowField(List<Vector2Int> starts, RunResult r)
    {
        var goals = Goals(starts.Count);
        var sw = Stopwatch.StartNew();
        var paths = flowField.FindPaths(mazeMatrix, starts, goals);
        sw.Stop();

        r.Ms = sw.Elapsed.TotalMilliseconds;
        r.Nodes = flowField.TotalNodesExpanded;
        r.PeakBytes = Math.Max(r.PeakBytes, flowField.PeakMemoryBytes);
        if (paths == null || !flowField.Success) { r.Fail(flowField.FailureReason); return; }
        r.CellPaths = paths;
    }

    // D* Lite: single-agent, each agent planned independently (no coordination).
    private IEnumerator RunDStarLite(List<Vector2Int> starts, RunResult r)
    {
        Vector3 goalWorld = grid.CellToWorld(goalCell);
        var sw = new Stopwatch();

        for (int i = 0; i < starts.Count; i++)
        {
            Vector3 startWorld = grid.CellToWorld(starts[i]);

            sw.Restart();
            bool initOk = dStarLite.Initialize(startWorld, goalWorld);
            bool found = initOk && dStarLite.TryGetPath(startWorld, out List<Vector3> worldPath) &&
                         AddWorldPathAsCells(worldPath, r);
            sw.Stop();

            r.Ms += sw.Elapsed.TotalMilliseconds;
            r.Nodes += dStarLite.NodesExpanded;
            r.SampleMemory();

            if (!found) { r.Fail(initOk ? $"Agente {i} no tiene ruta" : $"Agente {i}: inicio o meta bloqueados"); yield break; }
            if (yieldBetweenAgents) yield return null;
        }
    }

    // MARS offline phase, single-agent, continuous space: builds the path set; the agent's path is
    // the best (current) one. Time includes the alternatives, because MARS needs them to replan.
    private IEnumerator RunMars(List<Vector2Int> starts, int run, RunResult r)
    {
        int runSeed = marsVarySeedPerRun ? marsSettings.seed + run * 1000 : marsSettings.seed;
        r.Seed = runSeed;
        r.ContinuousPaths = new List<List<Vector3>>();
        Vector3 goalWorld = grid.CellToWorld(goalCell);
        var sw = new Stopwatch();

        for (int i = 0; i < starts.Count; i++)
        {
            MarsPlanner.Settings settings = marsSettings.Clone();
            settings.seed = runSeed + i;
            var planner = new MarsPlanner(grid, settings);

            sw.Restart();
            bool found = planner.Initialize(grid.CellToWorld(starts[i]), goalWorld);
            sw.Stop();

            r.Ms += sw.Elapsed.TotalMilliseconds;
            r.Nodes += planner.OfflineTreeNodes;
            r.SampleMemory();

            if (!found) { r.Fail($"Agente {i}: MARS {planner.FailureReason}"); yield break; }
            List<Vector3> path = planner.CurrentPath.Waypoints();
            r.ContinuousPaths.Add(path);
            r.CellPaths.Add(Discretize(path));
            if (yieldBetweenAgents) yield return null;
        }
    }

    // RRT-Connect + random shortcut optimizer, single-agent, continuous space.
    private IEnumerator RunRrtConnect(List<Vector2Int> starts, int run, RunResult r)
    {
        int runSeed = marsVarySeedPerRun ? marsSeed + run * 1000 : marsSeed;
        r.Seed = runSeed;
        r.ContinuousPaths = new List<List<Vector3>>();
        Vector3 goalWorld = grid.CellToWorld(goalCell);
        var sw = new Stopwatch();

        for (int i = 0; i < starts.Count; i++)
        {
            int agentSeed = runSeed + i;

            sw.Restart();
            var planner = new RRTConnectPlanner(grid, agentSeed, marsStepSize);
            bool found = planner.TryPlan(grid.CellToWorld(starts[i]), goalWorld, out List<Vector3> path, marsMaxIterations);
            if (found && marsOptimizePath)
                path = new MarsPathOptimizer(grid, agentSeed + 1).Optimize(path, marsOptimizationIterations);
            sw.Stop();

            r.Ms += sw.Elapsed.TotalMilliseconds;
            r.Nodes += planner.LastTreeNodes;
            r.SampleMemory();

            if (!found) { r.Fail($"Agente {i}: RRT-Connect sin ruta en {marsMaxIterations} iteraciones"); yield break; }
            r.ContinuousPaths.Add(path);
            r.CellPaths.Add(Discretize(path));
            if (yieldBetweenAgents) yield return null;
        }
    }

    // TA-RRT*: RRTPathfinder, single-agent, continuous space, anytime (uses its whole iteration budget).
    private IEnumerator RunTaRrtStar(List<Vector2Int> starts, int run, RunResult r)
    {
        FieldInfo seedField = typeof(RRTPathfinder).GetField("seed", BindingFlags.Instance | BindingFlags.NonPublic);
        int originalSeed = seedField != null ? (int)seedField.GetValue(taRrtStar) : 0;
        int runSeed = taRrtVarySeedPerRun ? taRrtSeed + run * 1000 : originalSeed;
        r.Seed = runSeed;
        r.ContinuousPaths = new List<List<Vector3>>();
        Vector3 goalWorld = grid.CellToWorld(goalCell);
        var sw = new Stopwatch();

        for (int i = 0; i < starts.Count; i++)
        {
            if (taRrtVarySeedPerRun && seedField != null) seedField.SetValue(taRrtStar, runSeed + i);
            Vector3 startWorld = grid.CellToWorld(starts[i]);

            sw.Restart();
            bool initOk = taRrtStar.Initialize(startWorld, goalWorld);
            bool found = initOk && taRrtStar.TryGetPath(startWorld, out List<Vector3> path) && Keep(path, r);
            sw.Stop();

            r.Ms += sw.Elapsed.TotalMilliseconds;
            r.Nodes += taRrtStar.NodeCount;
            r.SampleMemory();

            if (!found)
            {
                r.Fail(initOk ? $"Agente {i}: TA-RRT* sin ruta en el presupuesto de iteraciones"
                              : $"Agente {i}: inicio o meta bloqueados / sin holgura");
                break;
            }
            if (yieldBetweenAgents) yield return null;
        }

        if (seedField != null) seedField.SetValue(taRrtStar, originalSeed);
    }

    private bool Keep(List<Vector3> path, RunResult r)
    {
        if (path == null || path.Count == 0) return false;
        r.ContinuousPaths.Add(path);
        r.CellPaths.Add(Discretize(path));
        return true;
    }

    // DTA: abstract A* over terrain regions + refinement A*, single-agent.
    private IEnumerator RunDta(List<Vector2Int> starts, RunResult r)
    {
        // Mark the abstraction for rebuilding; the rebuild itself happens inside the first
        // TryGetPath (EnsureBuilt), so it IS included in the measured time.
        if (dtaRebuildAbstractionEachRun) dta.TerrainMap.ReloadTerrain();

        Vector3 goalWorld = grid.CellToWorld(goalCell);
        var sw = new Stopwatch();

        for (int i = 0; i < starts.Count; i++)
        {
            Vector3 startWorld = grid.CellToWorld(starts[i]);

            sw.Restart();
            bool initOk = dta.Initialize(startWorld, goalWorld);
            bool found = initOk && dta.TryGetPath(startWorld, out List<Vector3> worldPath) &&
                         AddWorldPathAsCells(worldPath, r);
            sw.Stop();

            r.Ms += sw.Elapsed.TotalMilliseconds;
            r.Nodes += dta.ExpandedAbstractNodes + dta.ExpandedRefinementNodes;
            r.SampleMemory();

            if (!found) { r.Fail(initOk ? $"Agente {i} no tiene ruta" : $"Agente {i}: inicio o meta bloqueados"); yield break; }
            if (yieldBetweenAgents) yield return null;
        }
    }

    private bool AddWorldPathAsCells(List<Vector3> worldPath, RunResult r)
    {
        if (worldPath == null || worldPath.Count == 0) return false;
        r.CellPaths.Add(worldPath.Select(p => grid.WorldToCell(p)).ToList());
        return true;
    }

    // ------------------------------------------------------------------ metrics (identical for every algorithm)

    private Row Evaluate(Algorithm alg, int agents, int run, RunResult r, long startMemory)
    {
        var row = new Row
        {
            Alg = alg, Agents = agents, Run = run, Seed = r.Seed,
            Ok = r.Ok, Reason = r.Ok ? "" : r.Reason,
            Nodes = r.Nodes, Ms = r.Ms,
            RamMB = Math.Max(0, r.PeakBytes - startMemory) / (1024.0 * 1024.0),
            Soc = -1, Makespan = -1, Conflicts = -1, InvalidMoves = -1, Length = -1
        };
        if (!r.Ok || r.CellPaths.Count == 0) return row;

        row.Soc = r.CellPaths.Sum(p => p.Count - 1);
        row.Makespan = r.CellPaths.Max(p => p.Count - 1);
        row.Conflicts = CountConflicts(r.CellPaths);

        if (r.ContinuousPaths != null)
        {
            var checker = new MarsCollisionChecker(grid);
            row.Length = r.ContinuousPaths.Sum(p => ContinuousLength(p));
            row.InvalidMoves = r.ContinuousPaths.Sum(p => CountInvalidSegments(p, checker));
        }
        else
        {
            row.Length = r.CellPaths.Sum(p => CellPathLength(p));
            row.InvalidMoves = r.CellPaths.Sum(p => CountInvalidMoves(p));
        }

        row.Valid = row.Conflicts == 0 && row.InvalidMoves == 0;
        return row;
    }

    // Same model as CBSManager.FindFirstConflict: vertex + edge (swap) conflicts,
    // an agent DISAPPEARS when it reaches the goal.
    private static int CountConflicts(List<List<Vector2Int>> paths)
    {
        int total = 0;
        int maxTime = paths.Max(p => p.Count);
        for (int t = 0; t < maxTime; t++)
            for (int i = 0; i < paths.Count; i++)
            {
                if (t >= paths[i].Count) continue;
                for (int j = i + 1; j < paths.Count; j++)
                {
                    if (t >= paths[j].Count) continue;
                    if (paths[i][t] == paths[j][t]) total++;
                    if (t + 1 < paths[i].Count && t + 1 < paths[j].Count &&
                        paths[i][t] == paths[j][t + 1] && paths[i][t + 1] == paths[j][t] &&
                        paths[i][t] != paths[i][t + 1])
                        total++;
                }
            }
        return total;
    }

    // A move is valid if the cell is walkable and it is a wait or a legal grid neighbour
    // (8-connected, no corner cutting — 4-connected moves are a subset).
    private int CountInvalidMoves(List<Vector2Int> path)
    {
        int invalid = 0;
        for (int k = 0; k < path.Count; k++)
        {
            if (!grid.IsWalkable(path[k])) { invalid++; continue; }
            if (k > 0 && path[k] != path[k - 1] && !grid.GetNeighbours(path[k - 1]).Contains(path[k])) invalid++;
        }
        return invalid;
    }

    private static int CountInvalidSegments(List<Vector3> path, MarsCollisionChecker checker)
    {
        int invalid = 0;
        for (int k = 1; k < path.Count; k++)
            if (!checker.IsConnectionValid(path[k - 1], path[k])) invalid++;
        return invalid;
    }

    private double CellPathLength(List<Vector2Int> path)
    {
        double total = 0;
        for (int k = 1; k < path.Count; k++)
            total += Vector2Int.Distance(path[k - 1], path[k]) * grid.CellSize; // waits add 0
        return total;
    }

    private static double ContinuousLength(List<Vector3> path)
    {
        double total = 0;
        for (int k = 1; k < path.Count; k++)
        {
            Vector3 a = path[k - 1], b = path[k];
            a.y = b.y = 0f;
            total += Vector3.Distance(a, b);
        }
        return total;
    }

    /// <summary>
    /// Continuous path -> one cell per time step, advancing 1 cell (CellSize) per step,
    /// so SOC / makespan / conflicts use the same units as CBS (same method as MarsBenchmarkManager).
    /// </summary>
    private List<Vector2Int> Discretize(List<Vector3> path)
    {
        float step = grid.CellSize;
        var cells = new List<Vector2Int> { grid.WorldToCell(path[0]) };
        float carry = 0f;

        for (int k = 1; k < path.Count; k++)
        {
            Vector3 a = path[k - 1], b = path[k];
            a.y = b.y = 0f;
            float len = Vector3.Distance(a, b);
            if (len < 1e-6f) continue;

            float d = step - carry;
            while (d <= len)
            {
                cells.Add(grid.WorldToCell(Vector3.Lerp(path[k - 1], path[k], d / len)));
                d += step;
            }
            carry = len - (d - step);
        }

        Vector2Int last = grid.WorldToCell(path[path.Count - 1]);
        if (cells[cells.Count - 1] != last) cells.Add(last);
        return cells;
    }

    // ------------------------------------------------------------------ output

    private static string FormatRow(Row r)
    {
        return string.Format(Inv, "{0},{1},{2},{3},{4},{5:F3},{6},{7},{8:F2},{9:F3},{10},{11},{12},{13},{14}",
            r.Alg, r.Agents, r.Run, r.Ok, r.Nodes, r.Ms, r.Soc, r.Makespan, r.Length, r.RamMB,
            r.Conflicts, r.InvalidMoves, r.Ok && r.Valid, r.Seed >= 0 ? r.Seed.ToString(Inv) : "", Csv(r.Reason));
    }

    private void WriteSummary(string path, List<Row> rows)
    {
        using (var w = new StreamWriter(path, false))
        {
            w.WriteLine("Algoritmo,Agentes,Ejecuciones,Exitos,Tasa Exito (%),Soluciones Validas," +
                        "Tiempo Medio (ms),Tiempo Desv (ms),Tiempo Mediana (ms),Tiempo Min (ms),Tiempo Max (ms)," +
                        "Nodos Medio,Suma de Costos Media,Makespan Medio,Longitud Media (unidades)," +
                        "RAM Pico Media (MB),Conflictos Medio,Movimientos Invalidos Medio");

            foreach (var g in rows.GroupBy(x => new { x.Alg, x.Agents }))
            {
                var all = g.ToList();
                var ok = all.Where(x => x.Ok).ToList();
                var times = all.Select(x => x.Ms).ToList(); // failed runs also cost time

                w.WriteLine(string.Format(Inv,
                    "{0},{1},{2},{3},{4:F1},{5},{6:F3},{7:F3},{8:F3},{9:F3},{10:F3},{11:F1},{12},{13},{14},{15:F3},{16},{17}",
                    g.Key.Alg, g.Key.Agents, all.Count, ok.Count, 100.0 * ok.Count / all.Count,
                    all.Count(x => x.Ok && x.Valid),
                    times.Average(), StdDev(times), Median(times), times.Min(), times.Max(),
                    all.Average(x => (double)x.Nodes),
                    MeanOrBlank(ok, x => x.Soc, "F1"), MeanOrBlank(ok, x => x.Makespan, "F1"),
                    MeanOrBlank(ok, x => x.Length, "F2"),
                    all.Average(x => x.RamMB),
                    MeanOrBlank(ok, x => x.Conflicts, "F1"), MeanOrBlank(ok, x => x.InvalidMoves, "F1")));
            }
        }
    }

    private static string SummaryTable(List<Row> rows)
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine(string.Format("{0,-10} {1,7} {2,8} {3,12} {4,12} {5,10} {6,9} {7,10}",
            "Algoritmo", "Agentes", "Exito%", "Tiempo(ms)", "Nodos", "SOC", "Makespan", "Conflictos"));
        foreach (var g in rows.GroupBy(x => new { x.Alg, x.Agents }))
        {
            var ok = g.Where(x => x.Ok).ToList();
            sb.AppendLine(string.Format(Inv, "{0,-10} {1,7} {2,8:F0} {3,12:F2} {4,12:F0} {5,10} {6,9} {7,10}",
                g.Key.Alg, g.Key.Agents, 100.0 * ok.Count / g.Count(), g.Average(x => x.Ms), g.Average(x => (double)x.Nodes),
                MeanOrBlank(ok, x => x.Soc, "F1"), MeanOrBlank(ok, x => x.Makespan, "F1"), MeanOrBlank(ok, x => x.Conflicts, "F1")));
        }
        return sb.ToString();
    }

    private static string MeanOrBlank(List<Row> rows, Func<Row, double> f, string fmt)
        => rows.Count == 0 ? "" : rows.Average(f).ToString(fmt, Inv);

    private static double StdDev(List<double> v)
    {
        if (v.Count < 2) return 0;
        double mean = v.Average();
        return Math.Sqrt(v.Sum(x => (x - mean) * (x - mean)) / (v.Count - 1)); // sample std
    }

    private static double Median(List<double> v)
    {
        var s = v.OrderBy(x => x).ToList();
        int n = s.Count;
        return n % 2 == 1 ? s[n / 2] : (s[n / 2 - 1] + s[n / 2]) / 2.0;
    }

    private static string Csv(string s)
    {
        if (string.IsNullOrEmpty(s)) return "";
        return s.IndexOfAny(new[] { ',', '"', '\n' }) >= 0 ? "\"" + s.Replace("\"", "\"\"") + "\"" : s;
    }

    // ------------------------------------------------------------------ scenario

    private bool BuildScenario(int agents, out List<Vector2Int> starts, out string error)
    {
        starts = new List<Vector2Int>();
        error = "";
        if (!grid.IsWalkable(goalCell)) { error = $"goal {goalCell} is not walkable"; return false; }

        for (int i = 0; i < agents; i++)
        {
            var s = new Vector2Int(firstStartCell.x + i, firstStartCell.y);
            if (!grid.IsWalkable(s)) { error = $"start {s} of agent {i} is a wall / outside the maze"; return false; }
            if (s == goalCell) { error = $"agent {i} starts on the goal"; return false; }
            starts.Add(s);
        }
        return agents > 0;
    }

    private List<Vector2Int> Goals(int n) => Enumerable.Repeat(goalCell, n).ToList();

    private int[,] BuildMazeMatrix()
    {
        int w = grid.Width, h = grid.Depth;
        var m = new int[h, w];
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++)
                m[y, x] = grid.IsWalkable(new Vector2Int(x, y)) ? 0 : 1;
        return m;
    }

    // ------------------------------------------------------------------ references / setup

    private bool ResolveReferences()
    {
        if (grid == null) grid = FindObjectOfType<MazeNavigationGrid>();
        if (grid == null || !grid.IsInside(Vector2Int.zero))
        {
            Debug.LogError("MasterBenchmark: needs a MazeNavigationGrid with its MazeGridGenerator assigned.");
            return false;
        }

        if (cbs == null) cbs = FindOrCreate<CBSManager>(null);
        if (flowField == null) flowField = FindOrCreate<FlowFieldManager>(null);
        if (dStarLite == null) dStarLite = FindOrCreate<DStarLitePathfinder>(c => SetPrivate(c, "grid", grid));
        if (taRrtStar == null) taRrtStar = FindOrCreate<RRTPathfinder>(c => SetPrivate(c, "grid", grid));
        if (dta == null)
        {
            dta = FindOrCreate<DTAPathfinder>(c =>
            {
                DTATerrainMap map = FindObjectOfType<DTATerrainMap>();
                if (map == null)
                {
                    map = gameObject.AddComponent<DTATerrainMap>();
                    SetPrivate(map, "grid", grid);
                    Reenable(map);
                }
                SetPrivate(c, "terrainMap", map);
            });
        }

        // The single-agent planners must use the same grid instance as everything else.
        CheckGrid("DStarLitePathfinder", dStarLite != null ? dStarLite.Grid : null);
        CheckGrid("RRTPathfinder", taRrtStar != null ? taRrtStar.Grid : null);
        CheckGrid("DTAPathfinder", dta != null ? dta.Grid : null);
        return true;
    }

    private void CheckGrid(string who, MazeNavigationGrid other)
    {
        if (other != null && other != grid)
            Debug.LogWarning($"MasterBenchmark: {who} uses a different MazeNavigationGrid than the benchmark.");
    }

    private T FindOrCreate<T>(Action<T> configure) where T : MonoBehaviour
    {
        T found = FindObjectOfType<T>();
        if (found != null || !autoCreateMissing) return found;

        T created = gameObject.AddComponent<T>();
        configure?.Invoke(created);
        Reenable(created); // OnEnable already ran before the fields were set; run it again to subscribe events
        Debug.Log($"MasterBenchmark: created {typeof(T).Name} with default settings.");
        return created;
    }

    private static void Reenable(MonoBehaviour c)
    {
        c.enabled = false;
        c.enabled = true;
    }

    private static void SetPrivate(object target, string field, object value)
    {
        FieldInfo f = target.GetType().GetField(field, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public);
        if (f != null) f.SetValue(target, value);
        else Debug.LogError($"MasterBenchmark: field '{field}' not found on {target.GetType().Name}.");
    }

    private bool IsAvailable(Algorithm alg, out string missing)
    {
        missing = "";
        switch (alg)
        {
            case Algorithm.CBS:       if (cbs == null) missing = "no CBSManager"; break;
            case Algorithm.FlowField: if (flowField == null) missing = "no FlowFieldManager"; break;
            case Algorithm.DStarLite: if (dStarLite == null || dStarLite.Grid == null) missing = "no DStarLitePathfinder with a grid"; break;
            case Algorithm.RRTConnect:
            case Algorithm.MARS:      break; // plain C# classes, only need the grid
            case Algorithm.TARRTStar: if (taRrtStar == null || taRrtStar.Grid == null) missing = "no RRTPathfinder with a grid"; break;
            case Algorithm.DTA:
                if (dta == null || dta.TerrainMap == null || dta.Grid == null) missing = "no DTAPathfinder with a DTATerrainMap and grid";
                break;
        }
        return missing.Length == 0;
    }

    private static void DisableOtherScripts()
    {
        Disable<BenchmarkManager>();
        Disable<FlowFieldBenchmarkManager>();
        Disable<DStarLiteBenchmarkManager>();
        Disable<MarsBenchmarkManager>();
        Disable<DStarLiteTest>();
        Disable<RRTTest>();
        Disable<DTATest>();
        Disable<RRTConnectTest>();
        Disable<MarsSamplingTest>();
        Disable<AgentPathFollower>();
        Disable<MarsAgent>();
    }

    private static void Disable<T>() where T : MonoBehaviour
    {
        foreach (T c in FindObjectsOfType<T>())
        {
            if (!c.enabled) continue;
            c.enabled = false;
            Debug.Log($"MasterBenchmark: disabled {typeof(T).Name} on '{c.gameObject.name}' so it doesn't run at the same time.");
        }
    }
}
