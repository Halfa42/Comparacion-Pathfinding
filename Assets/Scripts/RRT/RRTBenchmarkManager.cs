using System.Collections.Generic;
using UnityEngine;

[DefaultExecutionOrder(-100)]
public class RRTBenchmarkManager : GridPathfindingBenchmark
{
    public RRTPathfinder rrtPathfinder;
    [SerializeField] private int baseSeed = 12345;
    [SerializeField] private bool varySeedsBetweenRuns = true;

    protected override MazeNavigationGrid Grid => rrtPathfinder != null ? rrtPathfinder.Grid : null;
    protected override string AlgorithmName => "TA-RRT*";
    protected override string CsvFileName => "RRT_Benchmark_Resultados.csv";
    protected override string NodeCounting => "Intentos de extension del arbol (no nodos CBS)";
    private int originalSeed;
    private bool originalCollectMetrics;
    private bool originalBenchmarkRunning;

    protected override void BeginBenchmark()
    {
        originalSeed = rrtPathfinder.Seed;
        originalCollectMetrics = rrtPathfinder.CollectBenchmarkMetrics;
        originalBenchmarkRunning = rrtPathfinder.IsBenchmarkRunning;
        rrtPathfinder.IsBenchmarkRunning = true;
        rrtPathfinder.CollectBenchmarkMetrics = true;
    }

    protected override void EndBenchmark()
    {
        if (rrtPathfinder == null) return;
        rrtPathfinder.Seed = originalSeed;
        rrtPathfinder.CollectBenchmarkMetrics = originalCollectMetrics;
        rrtPathfinder.IsBenchmarkRunning = originalBenchmarkRunning;
    }

    protected override void PrepareRun() => rrtPathfinder.ClearSearch();

    private int SeedFor(int run, int agent)
    {
        int offset = varySeedsBetweenRuns ? Mathf.Max(0, run - 1) * NumberOfAgents : 0;
        return unchecked(baseSeed + offset + agent);
    }

    protected override bool Plan(Vector3 start, Vector3 goal, int run, int agent,
        out List<Vector3> path, out PlannerMetrics metrics)
    {
        path = null;
        rrtPathfinder.Seed = SeedFor(run, agent);
        bool success = rrtPathfinder.Initialize(start, goal) && rrtPathfinder.TryGetPath(start, out path);
        metrics = new PlannerMetrics
        {
            nodes = rrtPathfinder.ExpansionAttempts,
            peakMemory = rrtPathfinder.PeakMemoryBytes
        };
        return success;
    }

    protected override bool NativePathValid(IReadOnlyList<Vector3> path) => rrtPathfinder.IsPathCollisionFree(path);
}
