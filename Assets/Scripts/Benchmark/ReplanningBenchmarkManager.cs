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
/// Replanning benchmark in the style of the MARS paper (Tonola et al., IEEE Access 2023, Sec. V):
/// the agent moves along its path and obstacles APPEAR on the path ahead of it; the algorithm must
/// replan from the agent's current position. Algorithms run one after another (all runs of the
/// first, then all runs of the second, ...).
///
/// Simulation (fast-forward, not real time, so results do not depend on the frame rate):
///   - the agent moves at Robot Speed; the remaining path is collision-checked Collision Check Rate
///     times per simulated second (30 Hz in the paper);
///   - Obstacles Per Run obstacles appear at evenly spaced fractions of the initial path length, in a
///     cell of the remaining path between Min/Max Ahead Distance that does NOT seal the goal;
///   - when the path is blocked the algorithm replans (TryGetPath from the agent's position). If it fails
///     the agent waits (safety stop) and retries at the next check; too many in a row = failed run.
///
/// Metrics (as in the paper): success rate, safety stops (the paper's proxy for collisions),
/// normalized path length = travelled length / initial path length, plus replanning times.
///
/// Output: Replanificacion_Resultados.csv (one row per run) and Replanificacion_Resumen.csv.
/// Keep only ONE benchmark component (this or MasterBenchmarkManager) enabled at a time.
/// </summary>
public class ReplanningBenchmarkManager : MonoBehaviour
{
    public enum Replanner
    {
        MARS,                   // MarsPlanner: path set + multi-path replanning
        RRTConnectFromScratch,  // baseline: RRT-Connect + shortcut optimizer from the current position
        DStarLite,              // incremental replanning
        TARRTStar,              // RRTPathfinder, rebuilds from the current position
        DTA                     // abstraction repaired locally, then A*
    }

    [Header("Execution order")]
    [SerializeField] private List<Replanner> executionOrder = new List<Replanner>
    {
        Replanner.MARS, Replanner.RRTConnectFromScratch, Replanner.DStarLite, Replanner.TARRTStar, Replanner.DTA
    };

    [Header("Scenario")]
    [SerializeField, Min(1)] private int numberOfRuns = 20;
    [SerializeField] private Vector2Int startCell = new Vector2Int(13, 13);
    [SerializeField] private Vector2Int goalCell = new Vector2Int(21, 0);
    [Tooltip("Run r uses seed + r for obstacle placement, identical for every algorithm.")]
    [SerializeField] private int seed = 2024;

    [Header("Obstacles appearing during execution")]
    [SerializeField, Min(0)] private int obstaclesPerRun = 3;
    [SerializeField] private float minAheadDistance = 1.5f;
    [SerializeField] private float maxAheadDistance = 6f;
    [Tooltip("Optional: shows the obstacles with its prefab. Without it the cells are blocked directly.")]
    [SerializeField] private BenchmarkObstacleManager obstacleManager;

    [Header("Execution model")]
    [SerializeField] private float robotSpeed = 1.5f;
    [SerializeField] private float collisionCheckRate = 30f;
    [SerializeField] private int maxConsecutiveFailedReplans = 30;
    [SerializeField] private float maxSimulatedSeconds = 120f;

    [Header("MARS")]
    [SerializeField] private MarsPlanner.Settings marsSettings = new MarsPlanner.Settings();
    [SerializeField] private bool marsVarySeedPerRun = true;

    [Header("RRT-Connect baseline (same defaults as MarsBenchmarkManager)")]
    [SerializeField] private float rrtStepSize = 0.75f;
    [SerializeField] private int rrtMaxIterations = 5000;
    [SerializeField] private int rrtOptimizationIterations = 200;

    [Header("References (searched in the scene if empty)")]
    [SerializeField] private MazeNavigationGrid grid;
    [SerializeField] private DStarLitePathfinder dStarLite;
    [SerializeField] private RRTPathfinder taRrtStar;
    [SerializeField] private DTAPathfinder dta;
    [SerializeField] private bool autoCreateMissing = true;

    [Header("Run control")]
    [SerializeField] private bool runOnStart = true;
    [SerializeField] private bool disableOtherBenchmarkScripts = true;
    [Tooltip("Empty = Application.dataPath (the Assets folder).")]
    [SerializeField] private string outputFolder = "";

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    private readonly List<Vector2Int> blocked = new List<Vector2Int>();
    private MarsCollisionChecker checker;
    private string status = "";

    // ------------------------------------------------------------------ adapters

    private interface IReplanner
    {
        bool Initialize(Vector3 start, Vector3 goal);
        bool TryGetPath(Vector3 position, out List<Vector3> path);
        string Extra { get; }
    }

    private delegate bool TryPathFn(Vector3 position, out List<Vector3> path);

    private sealed class ComponentReplanner : IReplanner
    {
        private readonly Func<Vector3, Vector3, bool> init;
        private readonly TryPathFn tryPath;
        public ComponentReplanner(Func<Vector3, Vector3, bool> init, TryPathFn tryPath) { this.init = init; this.tryPath = tryPath; }
        public bool Initialize(Vector3 start, Vector3 goal) => init(start, goal);
        public bool TryGetPath(Vector3 position, out List<Vector3> path) => tryPath(position, out path);
        public string Extra => "";
    }

    private sealed class MarsReplanner : IReplanner
    {
        private readonly MarsPlanner planner;
        public MarsReplanner(MazeNavigationGrid grid, MarsPlanner.Settings settings) { planner = new MarsPlanner(grid, settings); }
        public bool Initialize(Vector3 start, Vector3 goal) => planner.Initialize(start, goal);

        public bool TryGetPath(Vector3 position, out List<Vector3> path)
        {
            path = null;
            if (planner.IsValidFrom(position)) { path = planner.RemainingWaypoints(position); return true; }
            MarsPlanner.ReplanResult r = planner.Replan(position);
            if (r.Success) path = r.Waypoints;
            return r.Success;
        }

        public string Extra => string.Format(Inv, "caminos iniciales={0}; nodos grafo={1}; offline={2:F1} ms",
                                             planner.InitialPathCount, planner.GraphNodeCount, planner.OfflineMilliseconds);
    }

    private sealed class RrtConnectReplanner : IReplanner
    {
        private readonly MazeNavigationGrid grid;
        private readonly float step;
        private readonly int iterations, optimization;
        private int seed;
        private Vector3 goal;

        public RrtConnectReplanner(MazeNavigationGrid grid, int seed, float step, int iterations, int optimization)
        {
            this.grid = grid; this.seed = seed; this.step = step; this.iterations = iterations; this.optimization = optimization;
        }

        public bool Initialize(Vector3 start, Vector3 goal) { this.goal = goal; return true; }

        public bool TryGetPath(Vector3 position, out List<Vector3> path)
        {
            seed++;
            var planner = new RRTConnectPlanner(grid, seed, step);
            if (!planner.TryPlan(position, goal, out path, iterations)) return false;
            if (optimization > 0) path = new MarsPathOptimizer(grid, seed + 1).Optimize(path, optimization);
            return true;
        }

        public string Extra => "";
    }

    private class Row
    {
        public string Alg;
        public int Run, Obstacles, Replans, ReplansOk, SafetyStops;
        public bool Success;
        public string Reason = "";
        public double InitialMs, SimSeconds;
        public float InitialLength = -1, Travelled;
        public List<double> ReplanMs = new List<double>();
        public string Extra = "";
        public float Normalized => Success && InitialLength > 0 ? Travelled / InitialLength : -1f;
    }

    // ------------------------------------------------------------------ Unity

    private void Awake()
    {
        if (!disableOtherBenchmarkScripts) return;
        Disable<BenchmarkManager>(); Disable<FlowFieldBenchmarkManager>(); Disable<DStarLiteBenchmarkManager>();
        Disable<MarsBenchmarkManager>(); Disable<DStarLiteTest>(); Disable<RRTTest>(); Disable<DTATest>();
        Disable<RRTConnectTest>(); Disable<MarsSamplingTest>(); Disable<AgentPathFollower>(); Disable<MarsAgent>();
    }

    private IEnumerator Start()
    {
        if (!runOnStart) yield break;
        yield return null;
        yield return StartCoroutine(RunAll());
    }

    [ContextMenu("Run Replanning Benchmark")]
    public void RunFromMenu()
    {
        if (Application.isPlaying) StartCoroutine(RunAll());
    }

    private void OnGUI()
    {
        if (!string.IsNullOrEmpty(status)) GUI.Label(new Rect(10, 35, 900, 25), status);
    }

    // ------------------------------------------------------------------ main loop

    private IEnumerator RunAll()
    {
        if (!ResolveReferences()) yield break;
        checker = new MarsCollisionChecker(grid);

        if (!grid.IsWalkable(startCell) || !grid.IsWalkable(goalCell))
        {
            Debug.LogError("ReplanningBenchmark: start or goal cell is a wall.");
            yield break;
        }

        string folder = string.IsNullOrEmpty(outputFolder) ? Application.dataPath : outputFolder;
        Directory.CreateDirectory(folder);
        string runsPath = Path.Combine(folder, "Replanificacion_Resultados.csv");
        string summaryPath = Path.Combine(folder, "Replanificacion_Resumen.csv");
        var rows = new List<Row>();

        using (var writer = new StreamWriter(runsPath, false))
        {
            writer.WriteLine("Algoritmo,Ejecucion,Exito,Tiempo Plan Inicial (ms),Longitud Inicial,Longitud Recorrida," +
                             "Longitud Normalizada,Obstaculos,Replanificaciones,Replan Exitosas,Paradas de Seguridad," +
                             "Replan Medio (ms),Replan Mediana (ms),Replan Max (ms),Tiempo Simulado (s),Motivo,Extra");
            writer.Flush();

            foreach (Replanner alg in executionOrder)
            {
                if (!IsAvailable(alg)) { Debug.LogWarning($"ReplanningBenchmark: skipping {alg} (component missing)."); continue; }
                Debug.Log($"===== ReplanningBenchmark: {alg} =====");

                for (int run = 1; run <= numberOfRuns; run++)
                {
                    status = $"Replanning benchmark | {alg} | run {run}/{numberOfRuns}";
                    var row = new Row { Alg = alg.ToString(), Run = run };
                    IReplanner replanner = Create(alg, run);

                    yield return StartCoroutine(Simulate(replanner, run, row));
                    row.Extra = replanner.Extra;
                    rows.Add(row);

                    writer.WriteLine(FormatRow(row));
                    writer.Flush();
                    Debug.Log(string.Format(Inv, "{0} ejecución {1}: {2} | replans={3} | paradas={4} | long. norm.={5:F3} | replan medio={6:F2} ms",
                        row.Alg, run, row.Success ? "OK" : "FALLÓ (" + row.Reason + ")", row.Replans, row.SafetyStops,
                        row.Normalized, row.ReplanMs.Count > 0 ? row.ReplanMs.Average() : 0));
                    yield return null;
                }
            }
        }

        WriteSummary(summaryPath, rows);
        status = "Replanning benchmark finished: " + runsPath;
        Debug.Log("¡Benchmark de replanificación terminado!\n  " + runsPath + "\n  " + summaryPath);
    }

    private IReplanner Create(Replanner alg, int run)
    {
        switch (alg)
        {
            case Replanner.MARS:
                MarsPlanner.Settings settings = marsSettings.Clone();
                if (marsVarySeedPerRun) settings.seed = marsSettings.seed + run * 1000;
                return new MarsReplanner(grid, settings);
            case Replanner.RRTConnectFromScratch:
                return new RrtConnectReplanner(grid, marsSettings.seed + run * 1000, rrtStepSize, rrtMaxIterations, rrtOptimizationIterations);
            case Replanner.DStarLite:
                return new ComponentReplanner(dStarLite.Initialize, dStarLite.TryGetPath);
            case Replanner.TARRTStar:
                return new ComponentReplanner(taRrtStar.Initialize, taRrtStar.TryGetPath);
            default:
                return new ComponentReplanner(dta.Initialize, dta.TryGetPath);
        }
    }

    // ------------------------------------------------------------------ simulation

    private IEnumerator Simulate(IReplanner alg, int run, Row row)
    {
        ClearObstacles();
        var random = new System.Random(seed + run);
        Vector3 start = grid.CellToWorld(startCell);
        Vector3 goal = grid.CellToWorld(goalCell);

        List<Vector3> path = null;
        var sw = Stopwatch.StartNew();
        bool ok = alg.Initialize(start, goal) && alg.TryGetPath(start, out path);
        sw.Stop();
        row.InitialMs = sw.Elapsed.TotalMilliseconds;

        if (!ok || path == null || path.Count == 0)
        {
            row.Reason = "sin ruta inicial";
            yield break;
        }

        row.InitialLength = LengthFrom(start, path, 0);
        var thresholds = new float[obstaclesPerRun];
        for (int i = 0; i < obstaclesPerRun; i++) thresholds[i] = row.InitialLength * (i + 1) / (obstaclesPerRun + 1);

        Vector3 robot = start;
        int index = 0, nextObstacle = 0, consecutiveFails = 0;
        float dt = 1f / collisionCheckRate;
        int maxTicks = Mathf.CeilToInt(maxSimulatedSeconds * collisionCheckRate);

        for (int tick = 0; tick < maxTicks; tick++)
        {
            row.SimSeconds = tick * dt;

            // An obstacle appears on the path ahead.
            if (nextObstacle < obstaclesPerRun && row.Travelled >= thresholds[nextObstacle])
            {
                if (ReplanObstacleUtility.TryPickCellAhead(grid, robot, path, index, minAheadDistance, maxAheadDistance,
                                                           goalCell, random, out Vector2Int cell))
                {
                    Block(cell);
                    row.Obstacles++;
                }
                nextObstacle++;
            }

            // Collision check of the remaining path -> replan if blocked.
            if (!PathValid(robot, path, index))
            {
                row.Replans++;
                List<Vector3> newPath = null;
                var replanClock = Stopwatch.StartNew();
                bool got = alg.TryGetPath(robot, out newPath);
                replanClock.Stop();
                row.ReplanMs.Add(replanClock.Elapsed.TotalMilliseconds);

                if (got && newPath != null && newPath.Count > 0 && PathValid(robot, newPath, 0))
                {
                    path = newPath;
                    index = 0;
                    row.ReplansOk++;
                    consecutiveFails = 0;
                }
                else
                {
                    row.SafetyStops++;
                    consecutiveFails++;
                    if (consecutiveFails >= maxConsecutiveFailedReplans)
                    {
                        row.Reason = $"parada de seguridad: {consecutiveFails} replanificaciones fallidas seguidas";
                        break;
                    }
                    yield return null;
                    continue; // the agent waits this tick
                }
                yield return null; // keep the Editor responsive (simulated time is not affected)
            }

            // Move along the path.
            float step = robotSpeed * dt;
            while (step > 0f && index < path.Count)
            {
                float d = Flat(robot, path[index]);
                if (d <= step)
                {
                    robot = path[index];
                    index++;
                    step -= d;
                    row.Travelled += d;
                }
                else
                {
                    robot += (path[index] - robot) * (step / d);
                    row.Travelled += step;
                    step = 0f;
                }
            }

            if (index >= path.Count)
            {
                if (Flat(robot, goal) < 0.05f) row.Success = true;
                else row.Reason = "la ruta no termina en la meta";
                break;
            }
        }

        if (!row.Success && row.Reason.Length == 0) row.Reason = "tiempo de simulación agotado";
        ClearObstacles();
    }

    private bool PathValid(Vector3 robot, List<Vector3> path, int index)
    {
        if (index >= path.Count) return true;
        if (!checker.IsConnectionValid(robot, path[index])) return false;
        for (int k = index + 1; k < path.Count; k++)
            if (!checker.IsConnectionValid(path[k - 1], path[k])) return false;
        return true;
    }

    private static float LengthFrom(Vector3 robot, List<Vector3> path, int index)
    {
        float total = 0f;
        Vector3 previous = robot;
        for (int k = index; k < path.Count; k++)
        {
            total += Flat(previous, path[k]);
            previous = path[k];
        }
        return total;
    }

    private void Block(Vector2Int cell)
    {
        if (obstacleManager != null) obstacleManager.AddObstacle(cell);
        else if (grid.SetWalkable(cell, false)) blocked.Add(cell);
    }

    private void ClearObstacles()
    {
        if (obstacleManager != null) obstacleManager.ClearObstacles();
        foreach (Vector2Int c in blocked) grid.SetWalkable(c, true);
        blocked.Clear();
    }

    private static float Flat(Vector3 a, Vector3 b)
    {
        float dx = a.x - b.x, dz = a.z - b.z;
        return Mathf.Sqrt(dx * dx + dz * dz);
    }

    // ------------------------------------------------------------------ output

    private static string FormatRow(Row r)
    {
        double mean = r.ReplanMs.Count > 0 ? r.ReplanMs.Average() : 0;
        double max = r.ReplanMs.Count > 0 ? r.ReplanMs.Max() : 0;
        return string.Format(Inv, "{0},{1},{2},{3:F3},{4:F3},{5:F3},{6:F4},{7},{8},{9},{10},{11:F3},{12:F3},{13:F3},{14:F2},{15},{16}",
            r.Alg, r.Run, r.Success, r.InitialMs, r.InitialLength, r.Travelled, r.Normalized, r.Obstacles, r.Replans,
            r.ReplansOk, r.SafetyStops, mean, Median(r.ReplanMs), max, r.SimSeconds, Csv(r.Reason), Csv(r.Extra));
    }

    private static void WriteSummary(string path, List<Row> rows)
    {
        using (var w = new StreamWriter(path, false))
        {
            w.WriteLine("Algoritmo,Ejecuciones,Exitos,Tasa Exito (%),Longitud Normalizada Media,Longitud Normalizada Desv," +
                        "Paradas de Seguridad Media,Replanificaciones Totales,Replan Exitosas (%),Replan Medio (ms),Replan Mediana (ms)," +
                        "Replan Max (ms),Tiempo Plan Inicial Medio (ms),Obstaculos Medio");

            foreach (var g in rows.GroupBy(r => r.Alg))
            {
                var all = g.ToList();
                var ok = all.Where(r => r.Success).ToList();
                var normalized = ok.Select(r => (double)r.Normalized).ToList();
                var replanTimes = all.SelectMany(r => r.ReplanMs).ToList();
                int replans = all.Sum(r => r.Replans);

                w.WriteLine(string.Format(Inv, "{0},{1},{2},{3:F1},{4},{5},{6:F2},{7},{8},{9},{10},{11},{12:F3},{13:F2}",
                    g.Key, all.Count, ok.Count, 100.0 * ok.Count / all.Count,
                    normalized.Count > 0 ? normalized.Average().ToString("F4", Inv) : "",
                    normalized.Count > 1 ? StdDev(normalized).ToString("F4", Inv) : "",
                    all.Average(r => (double)r.SafetyStops), replans,
                    replans > 0 ? (100.0 * all.Sum(r => r.ReplansOk) / replans).ToString("F1", Inv) : "",
                    replanTimes.Count > 0 ? replanTimes.Average().ToString("F3", Inv) : "",
                    replanTimes.Count > 0 ? Median(replanTimes).ToString("F3", Inv) : "",
                    replanTimes.Count > 0 ? replanTimes.Max().ToString("F3", Inv) : "",
                    all.Average(r => r.InitialMs), all.Average(r => (double)r.Obstacles)));
            }
        }
    }

    private static double Median(List<double> v)
    {
        if (v.Count == 0) return 0;
        var s = v.OrderBy(x => x).ToList();
        return s.Count % 2 == 1 ? s[s.Count / 2] : (s[s.Count / 2 - 1] + s[s.Count / 2]) / 2.0;
    }

    private static double StdDev(List<double> v)
    {
        double mean = v.Average();
        return Math.Sqrt(v.Sum(x => (x - mean) * (x - mean)) / (v.Count - 1));
    }

    private static string Csv(string s)
    {
        if (string.IsNullOrEmpty(s)) return "";
        return s.IndexOfAny(new[] { ',', '"', '\n', ';' }) >= 0 ? "\"" + s.Replace("\"", "\"\"") + "\"" : s;
    }

    // ------------------------------------------------------------------ setup

    private bool ResolveReferences()
    {
        if (grid == null) grid = FindObjectOfType<MazeNavigationGrid>();
        if (grid == null || !grid.IsInside(Vector2Int.zero))
        {
            Debug.LogError("ReplanningBenchmark: needs a MazeNavigationGrid with its MazeGridGenerator assigned.");
            return false;
        }

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
                    map.enabled = false;
                    map.enabled = true;
                }
                SetPrivate(c, "terrainMap", map);
            });
        }

        if (obstacleManager != null && obstacleManager.Grid != grid)
            Debug.LogWarning("ReplanningBenchmark: the BenchmarkObstacleManager uses a different grid.");
        return true;
    }

    private bool IsAvailable(Replanner alg)
    {
        switch (alg)
        {
            case Replanner.DStarLite: return dStarLite != null && dStarLite.Grid == grid;
            case Replanner.TARRTStar: return taRrtStar != null && taRrtStar.Grid == grid;
            case Replanner.DTA: return dta != null && dta.Grid == grid;
            default: return true;
        }
    }

    private T FindOrCreate<T>(Action<T> configure) where T : MonoBehaviour
    {
        T found = FindObjectOfType<T>();
        if (found != null || !autoCreateMissing) return found;
        T created = gameObject.AddComponent<T>();
        configure(created);
        created.enabled = false; // re-run OnEnable so the component subscribes to grid events
        created.enabled = true;
        return created;
    }

    private static void SetPrivate(object target, string field, object value)
    {
        FieldInfo f = target.GetType().GetField(field, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public);
        if (f != null) f.SetValue(target, value);
        else Debug.LogError($"ReplanningBenchmark: field '{field}' not found on {target.GetType().Name}.");
    }

    private static void Disable<T>() where T : MonoBehaviour
    {
        foreach (T c in FindObjectsOfType<T>())
            if (c.enabled) c.enabled = false;
    }
}
