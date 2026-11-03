using UnityEngine;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;

/// <summary>
/// Benchmark de D* Lite. Usa el MISMO laberinto (MazeGridGenerator), los mismos inicios
/// (x = 13..23, fila 13), la misma meta [x=21, y=0], el mismo número de agentes/ejecuciones
/// y las mismas 9 primeras columnas que BenchmarkManager (CBS) y FlowFieldBenchmarkManager,
/// para que los CSV se puedan comparar directamente en Excel.
///
/// D* Lite es un planificador de UN agente: cada agente se planifica de forma independiente
/// (sin coordinación). Por eso "Solucion Valida" (sin conflictos vértice/arista, mismo modelo
/// que CBS: el agente desaparece al llegar a la meta) normalmente sale False; la columna
/// "Conflictos" indica cuántos hay.
///
/// Columnas extra: prueba de replanificación con un obstáculo dinámico sobre la ruta,
/// comparando la replanificación INCREMENTAL de D* Lite contra planificar desde cero.
///
/// Uso: añadir a un GameObject de la escena de D* Lite y asignar el DStarLitePathfinder
/// (que debe tener asignado su MazeNavigationGrid y estar activo).
/// </summary>
public class DStarLiteBenchmarkManager : MonoBehaviour
{
    [SerializeField] private DStarLitePathfinder pathfinder;
    [SerializeField] private int numberOfRuns = 10;                  // igual que CBS
    [SerializeField] [Range(2, 11)] private int numberOfAgents = 10; // igual que CBS

    [Header("Prueba de replanificación (obstáculo dinámico)")]
    [SerializeField] private bool testReplanning = true;
    [SerializeField] private int randomSeed = 12345;

    private static readonly Vector2Int GoalCell = new Vector2Int(21, 0); // Salida, igual que CBS

    private MazeNavigationGrid grid;

    void Start()
    {
        RunBenchmark();
    }

    void RunBenchmark()
    {
        if (pathfinder == null || pathfinder.Grid == null)
        {
            Debug.LogError("DStarLiteBenchmarkManager: asigna un DStarLitePathfinder con su MazeNavigationGrid.");
            return;
        }
        grid = pathfinder.Grid;

        // Mismos inicios y metas que el benchmark de CBS. Vector2Int(columna X, fila Y)
        var starts = new List<Vector2Int>();
        var goals = new List<Vector2Int>();
        for (int i = 0; i < numberOfAgents; i++)
        {
            starts.Add(new Vector2Int(13 + i, 13));
            goals.Add(GoalCell);
        }

        var inv = CultureInfo.InvariantCulture;
        string filePath = Application.dataPath + "/DStarLite_Benchmark_Resultados.csv";

        // Calentamiento (JIT) que NO se registra
        PlanAll(starts, goals, out _, out _, out _, out _);

        using (var writer = new StreamWriter(filePath))
        {
            // Las primeras 9 columnas son idénticas a las del CSV de CBS. El resto son extra.
            writer.WriteLine("Ejecucion,Exito,Nodos Expandidos (Alto Nivel),Tiempo (ms),Suma de Costos,Makespan,RAM Pico (MB),Solucion Valida,Motivo Fallo," +
                             "Longitud Total (unidades),Conflictos,Rutas Sin Colision Con Muros," +
                             "Replan Exitos,Tiempo Replan Incremental (ms),Nodos Replan Incremental,Tiempo Replan Desde Cero (ms),Nodos Replan Desde Cero");

            for (int i = 1; i <= numberOfRuns; i++)
            {
                System.GC.Collect();
                long startMemory = System.GC.GetTotalMemory(true);

                var sw = System.Diagnostics.Stopwatch.StartNew();
                bool ok = PlanAll(starts, goals, out var paths, out int nodes, out long peakBytes, out string reason);
                sw.Stop();

                int soc = ok ? paths.Sum(p => p.Count - 1) : -1;
                int makespan = ok ? paths.Max(p => p.Count - 1) : -1;
                int conflicts = ok ? CountConflicts(paths) : -1;
                bool valid = ok && conflicts == 0;
                bool wallsOk = ok && paths.All(p => IsPathWalkable(p));
                double length = ok ? paths.Sum(p => PathLength(p)) : -1;
                double ramMB = System.Math.Max(0, peakBytes - startMemory) / (1024.0 * 1024.0);

                // --- Replanificación (NO cuenta en "Tiempo (ms)") ---
                int replanOk = 0, replanNodes = 0, scratchNodes = 0;
                double replanMs = 0, scratchMs = 0;
                if (ok && testReplanning)
                {
                    var rng = new System.Random(randomSeed + i);
                    for (int a = 0; a < paths.Count; a++)
                    {
                        if (TestReplan(paths[a], starts[a], goals[a], rng,
                                       out double incMs, out int incNodes, out double fullMs, out int fullNodes))
                            replanOk++;
                        replanMs += incMs; replanNodes += incNodes;
                        scratchMs += fullMs; scratchNodes += fullNodes;
                    }
                }

                writer.WriteLine(string.Format(inv,
                    "{0},{1},{2},{3:F2},{4},{5},{6:F2},{7},{8},{9:F2},{10},{11},{12},{13:F3},{14},{15:F3},{16}",
                    i, ok, nodes, sw.Elapsed.TotalMilliseconds,
                    soc, makespan, ramMB, valid, ok ? "" : reason,
                    length, conflicts, wallsOk,
                    replanOk, replanMs, replanNodes, scratchMs, scratchNodes));

                Debug.Log(string.Format(inv,
                    "D* Lite ejecución {0}: {1} | nodos={2} | {3:F2} ms | SOC={4} | makespan={5} | conflictos={6} | replan inc={7:F3} ms vs cero={8:F3} ms",
                    i, ok ? "OK" : "FALLÓ (" + reason + ")",
                    nodes, sw.Elapsed.TotalMilliseconds, soc, makespan, conflicts, replanMs, scratchMs));
            }
        }

        Debug.Log("¡Benchmark de D* Lite terminado! Archivo guardado en: " + filePath);
    }

    /// <summary>Planifica cada agente de forma independiente con D* Lite.</summary>
    private bool PlanAll(List<Vector2Int> starts, List<Vector2Int> goals,
                         out List<List<Vector2Int>> paths, out int nodes, out long peakBytes, out string reason)
    {
        paths = new List<List<Vector2Int>>();
        nodes = 0;
        peakBytes = System.GC.GetTotalMemory(false);
        reason = "";

        for (int a = 0; a < starts.Count; a++)
        {
            Vector3 startWorld = grid.CellToWorld(starts[a]);
            Vector3 goalWorld = grid.CellToWorld(goals[a]);

            if (!pathfinder.Initialize(startWorld, goalWorld))
            {
                reason = "Agente " + a + ": inicio o meta bloqueados";
                return false;
            }

            if (!pathfinder.TryGetPath(startWorld, out List<Vector3> worldPath))
            {
                nodes += pathfinder.NodesExpanded;
                reason = "Agente " + a + " no tiene ruta";
                return false;
            }

            nodes += pathfinder.NodesExpanded;
            paths.Add(worldPath.Select(p => grid.WorldToCell(p)).ToList());
            peakBytes = System.Math.Max(peakBytes, System.GC.GetTotalMemory(false));
        }
        return true;
    }

    /// <summary>
    /// Coloca un obstáculo sobre la ruta, con el agente a mitad de camino antes del obstáculo,
    /// y mide la replanificación incremental de D* Lite contra una planificación desde cero.
    /// </summary>
    private bool TestReplan(List<Vector2Int> path, Vector2Int start, Vector2Int goal, System.Random rng,
                            out double incMs, out int incNodes, out double fullMs, out int fullNodes)
    {
        incMs = fullMs = 0;
        incNodes = fullNodes = 0;
        if (path.Count < 4) return false;

        int obstacleIdx = rng.Next(2, path.Count - 1); // nunca inicio ni meta
        Vector2Int obstacle = path[obstacleIdx];
        Vector2Int agentCell = path[obstacleIdx / 2];  // el agente ya avanzó parte de la ruta
        Vector3 agentWorld = grid.CellToWorld(agentCell);
        Vector3 goalWorld = grid.CellToWorld(goal);

        // Estado previo: ruta original ya calculada
        pathfinder.Initialize(grid.CellToWorld(start), goalWorld);
        pathfinder.TryGetPath(grid.CellToWorld(start), out _);

        bool success;
        try
        {
            grid.SetWalkable(obstacle, false); // dispara CellChanged -> D* Lite actualiza vértices

            // 1) Replanificación incremental (reutiliza g/rhs)
            int before = pathfinder.NodesExpanded;
            var sw = System.Diagnostics.Stopwatch.StartNew();
            success = pathfinder.TryGetPath(agentWorld, out _);
            sw.Stop();
            incMs = sw.Elapsed.TotalMilliseconds;
            incNodes = pathfinder.NodesExpanded - before;

            // 2) Desde cero con el mismo obstáculo
            sw.Restart();
            if (pathfinder.Initialize(agentWorld, goalWorld))
                pathfinder.TryGetPath(agentWorld, out _);
            sw.Stop();
            fullMs = sw.Elapsed.TotalMilliseconds;
            fullNodes = pathfinder.NodesExpanded;
        }
        finally
        {
            grid.SetWalkable(obstacle, true); // restaurar el laberinto
        }
        return success;
    }

    private bool IsPathWalkable(List<Vector2Int> path)
    {
        for (int k = 0; k < path.Count; k++)
        {
            if (!grid.IsWalkable(path[k])) return false;
            if (k > 0 && !grid.GetNeighbours(path[k - 1]).Contains(path[k])) return false;
        }
        return true;
    }

    private double PathLength(List<Vector2Int> path)
    {
        double total = 0;
        for (int k = 1; k < path.Count; k++) total += grid.Cost(path[k - 1], path[k]);
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
