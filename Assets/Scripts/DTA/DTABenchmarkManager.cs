using System.Collections.Generic;
using UnityEngine;

public class DTABenchmarkManager : GridPathfindingBenchmark
{
    public DTAPathfinder dtaPathfinder;
    [Tooltip("Rebuild once per run, preserving terrain patches and runtime assignments.")]
    [SerializeField] private bool rebuildAbstractionEachRun = true;

    protected override MazeNavigationGrid Grid => dtaPathfinder != null ? dtaPathfinder.Grid : null;
    protected override string AlgorithmName => "DTA";
    protected override string CsvFileName => "DTA_Benchmark_Resultados.csv";
    protected override string NodeCounting => "Expansiones A* abstracto (excluye el refinamiento)";
    protected override string AbstractionMode => rebuildAbstractionEachRun ? "Reconstruida por ejecucion" : "Reutilizada";
    private bool originalCollectMetrics;
    private DTATerrainMap observedMap;

    protected override void BeginBenchmark()
    {
        originalCollectMetrics = dtaPathfinder.CollectBenchmarkMetrics;
        dtaPathfinder.CollectBenchmarkMetrics = true;
        observedMap = dtaPathfinder.TerrainMap;
        observedMap.Changed += MarkScenarioChanged;
        dtaPathfinder.CostsChanged += MarkScenarioChanged;
    }

    protected override void EndBenchmark()
    {
        if (observedMap != null) observedMap.Changed -= MarkScenarioChanged;
        observedMap = null;
        if (dtaPathfinder == null) return;
        dtaPathfinder.CostsChanged -= MarkScenarioChanged;
        dtaPathfinder.CollectBenchmarkMetrics = originalCollectMetrics;
    }

    protected override void PrepareRun()
    {
        dtaPathfinder.ClearSearch();
        if (rebuildAbstractionEachRun) dtaPathfinder.TerrainMap.ClearAbstraction();
    }

    protected override bool Plan(Vector3 start, Vector3 goal, int run, int agent,
        out List<Vector3> path, out PlannerMetrics metrics)
    {
        path = null;
        bool success = dtaPathfinder.Initialize(start, goal) && dtaPathfinder.TryGetPath(start, out path);
        metrics = new PlannerMetrics
        {
            nodes = dtaPathfinder.ExpandedAbstractNodes,
            peakMemory = dtaPathfinder.PeakMemoryBytes
        };
        return success;
    }

    protected override bool NativePathValid(IReadOnlyList<Vector3> path)
    {
        if (path == null || path.Count == 0) return false;
        for (int i = 0; i < path.Count; i++)
        {
            Vector2Int cell = Grid.WorldToCell(path[i]);
            if (!Grid.IsWalkable(cell)) return false;
            if (i == 0) continue;
            Vector2Int previous = Grid.WorldToCell(path[i - 1]);
            if (previous == cell) continue;
            bool adjacent = false;
            foreach (Vector2Int neighbour in Grid.GetNeighbours(previous))
                if (neighbour == cell) { adjacent = true; break; }
            if (!adjacent) return false;
        }
        return true;
    }
}
