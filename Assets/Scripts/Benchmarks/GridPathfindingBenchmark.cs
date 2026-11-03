using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Shared CSV runner for individual planners. It does not add MAPF coordination.
/// Each agent is planned synchronously; frames between agents are not timed.
/// </summary>
public abstract class GridPathfindingBenchmark : MonoBehaviour
{
    [SerializeField, Min(1)] private int numberOfRuns = 10;
    [SerializeField, Range(2, 11)] private int numberOfAgents = 10;
    [SerializeField, Min(1)] private int maxPathSteps = 200;
    [SerializeField] private bool runOnStart = true;

    protected int NumberOfAgents => numberOfAgents;
    protected abstract MazeNavigationGrid Grid { get; }
    protected abstract string AlgorithmName { get; }
    protected abstract string CsvFileName { get; }
    protected abstract string NodeCounting { get; }
    protected virtual string AbstractionMode => "";

    protected struct PlannerMetrics
    {
        public long nodes, peakMemory;
    }

    private sealed class RunData
    {
        public bool success = true, valid;
        public string reason = "";
        public long nodes, baseMemory, peakMemory, sumOfCosts;
        public int makespan;
        public double planningMs, projectionMs;
        public readonly List<List<Vector2Int>> paths = new List<List<Vector2Int>>();
    }

    private static readonly CultureInfo Invariant = CultureInfo.InvariantCulture;
    private StreamWriter writer;
    private MazeNavigationGrid observedGrid;
    private bool running, setup, scenarioChanged;

    protected abstract void BeginBenchmark();
    protected abstract void EndBenchmark();
    protected abstract void PrepareRun();
    protected abstract bool Plan(Vector3 start, Vector3 goal, int run, int agent,
        out List<Vector3> path, out PlannerMetrics metrics);
    protected abstract bool NativePathValid(IReadOnlyList<Vector3> path);

    protected virtual void Start()
    {
        if (runOnStart) RunBenchmark();
    }

    [ContextMenu("Run Benchmark")]
    public void RunBenchmark()
    {
        if (!Application.isPlaying || running) return;
        if (numberOfRuns < 1 || numberOfAgents < 2 || numberOfAgents > 11 || maxPathSteps < 1 ||
            !GridBenchmarkPaths.MatchesReferenceMaze(Grid))
        {
            Debug.LogError(AlgorithmName + ": asigna el planificador y la grilla; se requiere el mismo " +
                "laberinto 25x15 de CBS/Flow Fields, 2..11 agentes y limites positivos.", this);
            return;
        }
        running = true;
        StartCoroutine(RunRoutine());
    }

    private IEnumerator RunRoutine()
    {
        string filePath = Path.Combine(Application.dataPath, CsvFileName);
        try
        {
            BeginBenchmark();
            setup = true;
            observedGrid = Grid;
            observedGrid.CellChanged += OnGridChanged;
            scenarioChanged = false;
            writer = new StreamWriter(filePath, false, new UTF8Encoding(true));
            writer.WriteLine("Ejecución,Éxito,Nodos Expandidos (Alto Nivel),Tiempo (ms),Suma de Costos,Makespan,RAM Pico (MB)");
            Debug.Log(AlgorithmName + ": conteo de nodos = " + NodeCounting +
                (AbstractionMode.Length > 0 ? "; abstraccion = " + AbstractionMode : ""), this);

            // Same warm-up policy as the existing benchmarks, without a CSV row.
            Debug.Log(AlgorithmName + $": calentamiento de {numberOfAgents} agentes; no se registra en el CSV. " +
                $"Despues se mediran {numberOfRuns} ejecuciones de {numberOfAgents} agentes.", this);
            yield return ExecuteRun(0, new RunData());
            if (scenarioChanged)
            {
                Debug.LogError(AlgorithmName + ": el escenario cambio durante el calentamiento.", this);
                yield break;
            }
            for (int run = 1; run <= numberOfRuns; run++)
            {
                var result = new RunData();
                yield return ExecuteRun(run, result);
                WriteResult(run, result);
                writer.Flush();
                string message = string.Format(Invariant,
                    "{0} ejecucion {1}: rutas={2}; valida={3}; nodos={4}; {5:F2} ms; SOC={6}; makespan={7}. {8}",
                    AlgorithmName, run, result.success, result.valid, result.nodes,
                    result.planningMs + result.projectionMs, result.success ? result.sumOfCosts : -1,
                    result.success ? result.makespan : -1, result.reason);
                if (result.valid) Debug.Log(message, this);
                else Debug.LogWarning(message, this);
                if (scenarioChanged) yield break;
                yield return null;
            }
            Debug.Log(AlgorithmName + ": benchmark terminado. CSV: " + filePath, this);
        }
        finally
        {
            FinishBenchmark();
        }
    }

    private IEnumerator ExecuteRun(int run, RunData result)
    {
        PrepareRun();
        GC.Collect();
        result.baseMemory = GC.GetTotalMemory(true);
        result.peakMemory = result.baseMemory;
        for (int agent = 0; agent < numberOfAgents; agent++)
        {
            if (scenarioChanged)
            {
                Fail(result, "Escenario modificado durante la medicion; vuelve a ejecutar el benchmark");
                break;
            }
            Vector2Int start = GridBenchmarkPaths.StartCell(agent), goal = GridBenchmarkPaths.GoalCell;
            Vector3 startWorld = Grid.CellToWorld(start), goalWorld = Grid.CellToWorld(goal);
            List<Vector3> worldPath;
            PlannerMetrics metrics;
            string error;
            var clock = System.Diagnostics.Stopwatch.StartNew();
            bool found = TryPlan(startWorld, goalWorld, run, agent, out worldPath, out metrics, out error);
            clock.Stop();
            result.planningMs += clock.Elapsed.TotalMilliseconds;
            result.nodes += metrics.nodes;
            result.peakMemory = Math.Max(result.peakMemory, metrics.peakMemory);

            if (!found)
                Fail(result, $"Agente {agent}: " + error);
            else
            {
                clock.Restart();
                List<Vector2Int> cells;
                string reason;
                bool converted = TryConvertPath(worldPath, start, goal, out cells, out reason);
                clock.Stop();
                result.projectionMs += clock.Elapsed.TotalMilliseconds;
                if (!converted) Fail(result, $"Agente {agent}: " + reason);
                else
                {
                    result.paths.Add(cells);
                    result.sumOfCosts += cells.Count - 1;
                    result.makespan = Math.Max(result.makespan, cells.Count - 1);
                }
            }
            result.peakMemory = Math.Max(result.peakMemory, GC.GetTotalMemory(false));
            // Yield outside measured work: RRT can take seconds per query.
            yield return null;
        }
        if (scenarioChanged) Fail(result, "Escenario modificado durante la medicion");
        if (result.success && result.paths.Count == numberOfAgents)
            result.valid = GridBenchmarkPaths.ValidateJointPaths(result.paths, maxPathSteps, out result.reason);
    }

    private bool TryPlan(Vector3 start, Vector3 goal, int run, int agent,
        out List<Vector3> path, out PlannerMetrics metrics, out string error)
    {
        path = null;
        metrics = new PlannerMetrics();
        error = "Sin ruta: revisa los parametros o el presupuesto del planificador";
        try
        {
            return Plan(start, goal, run, agent, out path, out metrics);
        }
        catch (Exception exception)
        {
            error = exception.GetType().Name + ": " + exception.Message;
            return false;
        }
    }

    private bool TryConvertPath(IReadOnlyList<Vector3> worldPath, Vector2Int start, Vector2Int goal,
        out List<Vector2Int> cells, out string reason)
    {
        cells = null;
        reason = "Ruta nativa invalida";
        try
        {
            return NativePathValid(worldPath) &&
                GridBenchmarkPaths.TryProject(Grid, worldPath, start, goal, out cells, out reason);
        }
        catch (Exception exception)
        {
            reason = exception.GetType().Name + ": " + exception.Message;
            return false;
        }
    }

    private void WriteResult(int run, RunData result)
    {
        double ramMb = Math.Max(0L, result.peakMemory - result.baseMemory) / (1024d * 1024d);
        string[] values =
        {
            run.ToString(Invariant), result.success.ToString(), result.nodes.ToString(Invariant),
            (result.planningMs + result.projectionMs).ToString("F2", Invariant),
            (result.success ? result.sumOfCosts : -1).ToString(Invariant),
            (result.success ? result.makespan : -1).ToString(Invariant), ramMb.ToString("F2", Invariant)
        };
        writer.WriteLine(string.Join(",", values));
    }

    private static void Fail(RunData result, string reason)
    {
        result.success = false;
        if (result.reason.Length == 0) result.reason = reason;
    }

    protected void MarkScenarioChanged() => scenarioChanged = true;
    private void OnGridChanged(Vector2Int cell) => MarkScenarioChanged();

    protected virtual void OnDisable()
    {
        StopAllCoroutines();
        FinishBenchmark();
    }

    private void FinishBenchmark()
    {
        if (observedGrid != null) observedGrid.CellChanged -= OnGridChanged;
        observedGrid = null;
        try
        {
            if (writer != null) writer.Dispose();
        }
        finally
        {
            writer = null;
            if (setup) { setup = false; EndBenchmark(); }
            running = false;
        }
    }
}
