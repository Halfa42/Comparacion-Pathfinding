using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

// Each completed query reports its nodes, calculation milliseconds and managed-memory peak.
// Planners expose data; the benchmark owns accumulation, movement sampling and CSV output.
public interface IPathfindingMetrics
{
    event Action<long, double, long> SearchCompleted;
    bool CollectBenchmarkMetrics { get; set; }
}

[DisallowMultipleComponent, DefaultExecutionOrder(-50)]
public sealed class AlgorithmRunBenchmark : MonoBehaviour
{
    [SerializeField] private GameObject agent;
    [SerializeField] private MazeNavigationGrid grid;
    [Tooltip("For standalone use. Leave off when a sequence controller starts measurements.")]
    [SerializeField] private bool runOnStart;

    public string CsvPath { get; private set; }
    public static string WorkbookPath => Path.Combine(Application.dataPath, "AlgorithmSequence_Benchmark_Resultados.xlsx");
    public bool IsRecording { get; private set; }
    private readonly Stopwatch clock = new Stopwatch();
    private readonly Vector3[] segment = new Vector3[2];
    private readonly Dictionary<string, List<string[]>> results = new Dictionary<string, List<string[]>>();
    private Transform trackedAgent;
    private IPathfindingMetrics source;
    private MonoBehaviour sourceComponent;
    private AlgorithmSequenceController.IAlgorithmStrategy strategy;
    private bool originalCollectMetrics;
    private bool sessionStarted;
    private StreamWriter writer;
    private string algorithm, validationError;
    private int run;
    private Vector3 previousPosition;
    private long nodes, steps, baseMemory, peakMemory;
    private double calculationMs;

    private void Start()
    {
        if (runOnStart) BeginMeasurement();
    }

    public void BeginSession()
    {
        EndSession();
        results.Clear();
        sessionStarted = true;
    }

    [ContextMenu("Begin Measurement")]
    public void BeginMeasurement()
    {
        if (!Application.isPlaying || !isActiveAndEnabled || agent == null || grid == null) return;
        BeginSession();
        BeginRun(agent, grid, agent.name);
    }

    public void BeginRun(GameObject actor, MazeNavigationGrid navigationGrid, string label, int execution = 0)
    {
        EndRun(false);
        if (!Application.isPlaying || !isActiveAndEnabled || actor == null || navigationGrid == null) return;
        if (!sessionStarted) BeginSession();
        agent = actor;
        grid = navigationGrid;
        algorithm = label;
        run = execution > 0 ? execution : results.TryGetValue(label, out var previousRuns) ? previousRuns.Count + 1 : 1;
        string fileName = label;
        foreach (char invalid in Path.GetInvalidFileNameChars()) fileName = fileName.Replace(invalid, '_');
        string filePath = Path.Combine(Application.dataPath, fileName + "_Sequence_Benchmark_Resultados.csv");
        if (writer == null || CsvPath != filePath)
        {
            writer?.Dispose();
            CsvPath = filePath;
            bool append = results.ContainsKey(label);
            writer = new StreamWriter(CsvPath, append, new UTF8Encoding(true));
            if (!append) writer.WriteLine(BenchmarkWorkbook.Header);
            writer.Flush();
            UnityEngine.Debug.Log("Benchmark CSV: " + CsvPath, this);
        }
        // A shared follower measures the whole query for every planner. Do not
        // subscribe to its native planner too, which would count the search twice.
        UniversalPathfindingAgent follower = actor.GetComponent<UniversalPathfindingAgent>();
        source = follower;
        sourceComponent = follower;
        strategy = follower;
        if (follower == null)
        foreach (MonoBehaviour component in actor.GetComponentsInChildren<MonoBehaviour>())
        {
            if (component is IPathfindingMetrics metrics)
            {
                if (source != null)
                {
                    UnityEngine.Debug.LogError("Universal benchmark: use one planner per measured agent.", this);
                    source = null;
                    return;
                }
                source = metrics;
                sourceComponent = component;
            }
            if (component is AlgorithmSequenceController.IAlgorithmStrategy lifecycle) strategy = lifecycle;
        }
        GridAgentMovement movement = actor.GetComponentInChildren<GridAgentMovement>();
        trackedAgent = movement != null ? movement.transform : actor.transform;
        previousPosition = trackedAgent.position;
        nodes = steps = 0;
        calculationMs = 0;
        validationError = grid.IsWalkable(grid.WorldToCell(previousPosition)) ? null : "Inicio bloqueado";
        baseMemory = peakMemory = GC.GetTotalMemory(true);
        IsRecording = true;
        if (source != null)
        {
            originalCollectMetrics = source.CollectBenchmarkMetrics;
            source.CollectBenchmarkMetrics = true;
            source.SearchCompleted += OnSearchCompleted;
        }
        else
            UnityEngine.Debug.LogWarning(label + ": no IPathfindingMetrics; nodes and calculation time will be -1.", this);
    }

    private void OnSearchCompleted(long expandedNodes, double planningMs, long plannerPeakMemory)
    {
        nodes = nodes < 0 || expandedNodes < 0 ? -1 : nodes + expandedNodes;
        calculationMs += planningMs;
        peakMemory = Math.Max(peakMemory, plannerPeakMemory);
    }

    private void LateUpdate()
    {
        if (!IsRecording) return;
        if (agent == null || trackedAgent == null || grid == null) { EndRun(false); return; }
        SamplePosition();
        if (strategy != null && strategy.IsSolved) EndRun(true);
    }

    private void SamplePosition()
    {
        if (trackedAgent == null || grid == null) return;
        Vector3 position = trackedAgent.position;
        Vector2Int start = grid.WorldToCell(previousPosition), end = grid.WorldToCell(position);
        if (validationError == null && start != end)
        {
            segment[0] = previousPosition;
            segment[1] = position;
            clock.Restart();
            bool valid = GridBenchmarkPaths.TryProject(grid, segment, start, end, out var cells, out var reason);
            clock.Stop();
            calculationMs += clock.Elapsed.TotalMilliseconds;
            if (valid) steps += cells.Count - 1;
            else validationError = reason;
        }
        previousPosition = position;
        peakMemory = Math.Max(peakMemory, GC.GetTotalMemory(false));
    }

    [ContextMenu("Finish Measurement")]
    public void FinishMeasurement() => EndRun(true);

    public void EndRun(bool caught)
    {
        if (!IsRecording) return;
        SamplePosition();
        IsRecording = false;
        bool hasMetrics = source != null;
        if (source != null)
        {
            source.SearchCompleted -= OnSearchCompleted;
            if (sourceComponent != null) source.CollectBenchmarkMetrics = originalCollectMetrics;
            source = null;
        }
        bool success = caught && agent != null && trackedAgent != null && grid != null && validationError == null;
        var culture = CultureInfo.InvariantCulture;
        double ramMb = Math.Max(0L, peakMemory - baseMemory) / (1024d * 1024d);
        // One pursuing agent: SOC and makespan both equal its travelled grid steps.
        string cost = (success ? steps : -1).ToString(culture);
        string[] row =
        {
            run.ToString(culture), success.ToString(),
            (hasMetrics ? nodes : -1).ToString(culture), (hasMetrics ? calculationMs : -1).ToString("F2", culture),
            cost, cost, ramMb.ToString("F2", culture)
        };
        writer.WriteLine(string.Join(",", row));
        writer.Flush();
        if (!results.TryGetValue(algorithm, out var rows)) results.Add(algorithm, rows = new List<string[]>());
        rows.Add(row);
        SaveWorkbook();
        if (validationError != null)
            UnityEngine.Debug.LogWarning(algorithm + ": benchmark inválido: " + validationError, this);
    }

    public void EndSession()
    {
        try { EndRun(agent != null && strategy != null && strategy.IsSolved); }
        finally
        {
            sessionStarted = false;
            if (writer != null)
            {
                writer.Dispose();
                writer = null;
                UnityEngine.Debug.Log("Benchmark: " + (SaveWorkbook() ? WorkbookPath : CsvPath), this);
            }
        }
    }

    private bool SaveWorkbook()
    {
        if (results.Count == 0) return false;
        try { BenchmarkWorkbook.Save(WorkbookPath, results); return true; }
        catch (Exception error) when (error is IOException || error is UnauthorizedAccessException)
        {
            UnityEngine.Debug.LogError("No se pudo guardar el Excel. Comprueba que no esté abierto. " + error.Message, this);
            return false;
        }
    }

    private void OnDisable() => EndSession();
}
