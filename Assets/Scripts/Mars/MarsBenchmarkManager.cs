using UnityEngine;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;

/// <summary>
/// Benchmark de MARS (RRT-Connect + optimización de ruta). Usa el MISMO laberinto
/// (MazeGridGenerator), los mismos inicios (x = 13..23, fila 13), la misma meta [x=21, y=0],
/// el mismo número de agentes/ejecuciones y las mismas 9 primeras columnas que
/// BenchmarkManager (CBS) y FlowFieldBenchmarkManager, para comparar los CSV en Excel.
///
/// MARS es un planificador de UN agente en espacio continuo: cada agente se planifica de
/// forma independiente. Para "Suma de Costos", "Makespan" y "Conflictos" la ruta continua
/// se discretiza a celdas avanzando 1 celda por paso de tiempo (mismas unidades que CBS).
/// "Solucion Valida" usa el mismo modelo de conflictos que CBS (vértice + arista,
/// el agente desaparece al llegar a la meta).
///
/// Como RRT es aleatorio, por defecto cada ejecución usa una semilla distinta
/// (semilla + ejecución) para obtener una distribución de resultados reproducible.
///
/// Uso: añadir a un GameObject de la escena de MARS y asignar el MazeNavigationGrid.
/// </summary>
public class MarsBenchmarkManager : MonoBehaviour
{
    [SerializeField] private MazeNavigationGrid grid;
    [SerializeField] private int numberOfRuns = 10;                  // igual que CBS
    [SerializeField] [Range(2, 11)] private int numberOfAgents = 10; // igual que CBS

    [Header("RRT-Connect")]
    [SerializeField] private int seed = 12345;
    [SerializeField] private bool varySeedPerRun = true;
    [SerializeField] private float stepSize = 0.75f;
    [SerializeField] private int maxIterations = 5000;

    [Header("Optimización")]
    [SerializeField] private bool optimizePath = true;
    [SerializeField] private int optimizationIterations = 200;

    private static readonly Vector2Int GoalCell = new Vector2Int(21, 0); // Salida, igual que CBS

    private class RunResult
    {
        public bool Ok = true;
        public string Reason = "";
        public List<List<Vector3>> Paths = new List<List<Vector3>>();
        public int TreeNodes, Iterations, OriginalWaypoints;
        public double PlanMs, OptimizeMs, OriginalLength;
        public long PeakBytes;
    }

    void Start()
    {
        RunBenchmark();
    }

    void RunBenchmark()
    {
        if (grid == null)
        {
            Debug.LogError("MarsBenchmarkManager: asigna el MazeNavigationGrid.");
            return;
        }

        // Mismos inicios y metas que el benchmark de CBS. Vector2Int(columna X, fila Y)
        var starts = new List<Vector2Int>();
        var goals = new List<Vector2Int>();
        for (int i = 0; i < numberOfAgents; i++)
        {
            starts.Add(new Vector2Int(13 + i, 13));
            goals.Add(GoalCell);
        }

        var inv = CultureInfo.InvariantCulture;
        string filePath = Application.dataPath + "/MARS_Benchmark_Resultados.csv";

        // Calentamiento (JIT) que NO se registra
        PlanAll(starts, goals, seed - 1);

        using (var writer = new StreamWriter(filePath))
        {
            // Las primeras 9 columnas son idénticas a las del CSV de CBS. El resto son extra.
            writer.WriteLine("Ejecucion,Exito,Nodos Expandidos (Alto Nivel),Tiempo (ms),Suma de Costos,Makespan,RAM Pico (MB),Solucion Valida,Motivo Fallo," +
                             "Semilla,Iteraciones RRT,Tiempo Planificacion (ms),Tiempo Optimizacion (ms)," +
                             "Longitud Original (unidades),Longitud Optimizada (unidades),Waypoints Originales,Waypoints Optimizados," +
                             "Segmentos Invalidos,Conflictos");

            for (int i = 1; i <= numberOfRuns; i++)
            {
                int runSeed = varySeedPerRun ? seed + i * 1000 : seed;

                System.GC.Collect();
                long startMemory = System.GC.GetTotalMemory(true);

                var sw = System.Diagnostics.Stopwatch.StartNew();
                RunResult r = PlanAll(starts, goals, runSeed);
                sw.Stop();

                bool ok = r.Ok;
                var cellPaths = ok ? r.Paths.Select(p => Discretize(p)).ToList() : null;
                int soc = ok ? cellPaths.Sum(p => p.Count - 1) : -1;
                int makespan = ok ? cellPaths.Max(p => p.Count - 1) : -1;
                int conflicts = ok ? CountConflicts(cellPaths) : -1;
                int invalidSegments = ok ? r.Paths.Sum(p => CountInvalidSegments(p)) : -1;
                bool valid = ok && conflicts == 0 && invalidSegments == 0;
                double optLength = ok ? r.Paths.Sum(p => PathLength(p)) : -1;
                int optWaypoints = ok ? r.Paths.Sum(p => p.Count) : -1;
                double ramMB = System.Math.Max(0, r.PeakBytes - startMemory) / (1024.0 * 1024.0);

                writer.WriteLine(string.Format(inv,
                    "{0},{1},{2},{3:F2},{4},{5},{6:F2},{7},{8},{9},{10},{11:F3},{12:F3},{13:F2},{14:F2},{15},{16},{17},{18}",
                    i, ok, r.TreeNodes, sw.Elapsed.TotalMilliseconds,
                    soc, makespan, ramMB, valid, ok ? "" : r.Reason,
                    runSeed, r.Iterations, r.PlanMs, r.OptimizeMs,
                    ok ? r.OriginalLength : -1, optLength, ok ? r.OriginalWaypoints : -1, optWaypoints,
                    invalidSegments, conflicts));

                Debug.Log(string.Format(inv,
                    "MARS ejecución {0}: {1} | nodos árbol={2} | {3:F2} ms | SOC={4} | makespan={5} | conflictos={6} | longitud {7:F2} -> {8:F2}",
                    i, ok ? "OK" : "FALLÓ (" + r.Reason + ")",
                    r.TreeNodes, sw.Elapsed.TotalMilliseconds, soc, makespan, conflicts, r.OriginalLength, optLength));
            }
        }

        Debug.Log("¡Benchmark de MARS terminado! Archivo guardado en: " + filePath);
    }

    /// <summary>Planifica (y optimiza) cada agente de forma independiente.</summary>
    private RunResult PlanAll(List<Vector2Int> starts, List<Vector2Int> goals, int runSeed)
    {
        var r = new RunResult { PeakBytes = System.GC.GetTotalMemory(false) };
        var sw = new System.Diagnostics.Stopwatch();

        for (int a = 0; a < starts.Count; a++)
        {
            int agentSeed = runSeed + a; // semilla distinta por agente, reproducible

            sw.Restart();
            var planner = new RRTConnectPlanner(grid, agentSeed, stepSize);
            bool found = planner.TryPlan(grid.CellToWorld(starts[a]), grid.CellToWorld(goals[a]),
                                         out List<Vector3> path, maxIterations);
            sw.Stop();
            r.PlanMs += sw.Elapsed.TotalMilliseconds;
            r.TreeNodes += planner.LastTreeNodes;
            r.Iterations += planner.LastIterations;

            if (!found)
            {
                r.Ok = false;
                r.Reason = "Agente " + a + ": RRT-Connect sin ruta en " + maxIterations + " iteraciones";
                return r;
            }

            r.OriginalLength += PathLength(path);
            r.OriginalWaypoints += path.Count;

            if (optimizePath)
            {
                sw.Restart();
                path = new MarsPathOptimizer(grid, agentSeed + 1).Optimize(path, optimizationIterations);
                sw.Stop();
                r.OptimizeMs += sw.Elapsed.TotalMilliseconds;
            }

            r.Paths.Add(path);
            r.PeakBytes = System.Math.Max(r.PeakBytes, System.GC.GetTotalMemory(false));
        }
        return r;
    }

    /// <summary>
    /// Convierte la ruta continua en una secuencia de celdas, avanzando 1 celda (CellSize)
    /// por paso de tiempo. Así SOC/makespan/conflictos están en las mismas unidades que CBS.
    /// </summary>
    private List<Vector2Int> Discretize(List<Vector3> path)
    {
        float step = grid.CellSize;
        var cells = new List<Vector2Int> { grid.WorldToCell(path[0]) };
        float carry = 0f; // distancia recorrida desde la última muestra

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

        Vector2Int goalCell = grid.WorldToCell(path[path.Count - 1]);
        if (cells[cells.Count - 1] != goalCell) cells.Add(goalCell);
        return cells;
    }

    private int CountInvalidSegments(List<Vector3> path)
    {
        var checker = new MarsCollisionChecker(grid);
        int invalid = 0;
        for (int k = 1; k < path.Count; k++)
            if (!checker.IsConnectionValid(path[k - 1], path[k])) invalid++;
        return invalid;
    }

    private static double PathLength(List<Vector3> path)
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

    // Mismo modelo que CBSManager.FindFirstConflict: vértice + arista (swap),
    // el agente DESAPARECE al llegar a su meta.
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
}
