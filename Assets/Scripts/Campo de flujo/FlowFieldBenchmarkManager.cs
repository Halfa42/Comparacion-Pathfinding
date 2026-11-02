using UnityEngine;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;

/// <summary>
/// Benchmark del campo de flujo. Usa EXACTAMENTE el mismo laberinto, inicios, meta,
/// número de agentes, número de ejecuciones y columnas que BenchmarkManager (CBS),
/// para que los CSV se puedan comparar directamente.
/// </summary>
public class FlowFieldBenchmarkManager : MonoBehaviour
{
    public FlowFieldManager flowFieldManager;
    [SerializeField] private int numberOfRuns = 10;                  // igual que CBS
    [SerializeField] [Range(2, 11)] private int numberOfAgents = 10; // igual que CBS

    private readonly int[,] maze = new int[,]
    {
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1}, // Salida en [x=21, y=0]
        {1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1},
        {1,0,1,1,1,0,1,0,1,1,1,1,1,1,0,1,0,1,1,1,1,1,1,0,1},
        {1,0,1,0,0,0,1,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,1,0,1},
        {1,0,1,0,1,1,1,0,1,0,1,1,0,1,0,1,1,1,1,1,1,0,1,0,1},
        {1,0,0,0,0,0,0,0,1,0,1,1,0,0,0,0,0,0,0,0,1,0,0,0,1},
        {1,1,1,1,1,1,0,1,1,0,1,1,1,1,1,1,1,1,1,0,1,1,0,1,1},
        {1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1},
        {1,0,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,0,1},
        {1,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1},
        {1,0,1,0,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,0,1,0,1},
        {1,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,1},
        {1,1,1,0,1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,0,1,0,1,1,1},
        {1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1}, // Celdas libres x=13..23
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
    };

    void Start()
    {
        RunBenchmark();
    }

    void RunBenchmark()
    {
        // Mismos inicios y metas que el benchmark de CBS. Vector2Int(columna X, fila Y)
        var starts = new List<Vector2Int>();
        var goals = new List<Vector2Int>();
        for (int i = 0; i < numberOfAgents; i++)
        {
            starts.Add(new Vector2Int(13 + i, 13));
            goals.Add(new Vector2Int(21, 0));
        }

        var inv = CultureInfo.InvariantCulture;
        string filePath = Application.dataPath + "/FlowField_Benchmark_Resultados.csv";

        // Calentamiento (JIT) que NO se registra
        flowFieldManager.FindPaths(maze, starts, goals);

        using (var writer = new StreamWriter(filePath))
        {
            // Las primeras 9 columnas son idénticas a las del CSV de CBS. Las últimas 3 son extra.
            writer.WriteLine("Ejecucion,Exito,Nodos Expandidos (Alto Nivel),Tiempo (ms),Suma de Costos,Makespan,RAM Pico (MB),Solucion Valida,Motivo Fallo,Tiempo Campo (ms),Tiempo Simulacion (ms),Memoria Campos (KB)");

            for (int i = 1; i <= numberOfRuns; i++)
            {
                System.GC.Collect();
                long startMemory = System.GC.GetTotalMemory(true);

                var sw = System.Diagnostics.Stopwatch.StartNew();
                var resultPaths = flowFieldManager.FindPaths(maze, starts, goals);
                sw.Stop();

                bool ok = resultPaths != null && flowFieldManager.Success;
                int soc = ok ? resultPaths.Sum(p => p.Count - 1) : -1;
                int makespan = ok ? resultPaths.Max(p => p.Count - 1) : -1;
                bool valid = ok && !flowFieldManager.HasConflicts(resultPaths);
                double ramMB = System.Math.Max(0, flowFieldManager.PeakMemoryBytes - startMemory) / (1024.0 * 1024.0);
                double fieldKB = flowFieldManager.FieldMemoryBytes / 1024.0;

                writer.WriteLine(string.Format(inv, "{0},{1},{2},{3:F2},{4},{5},{6:F2},{7},{8},{9:F3},{10:F3},{11:F2}",
                    i, ok, flowFieldManager.TotalNodesExpanded, sw.Elapsed.TotalMilliseconds,
                    soc, makespan, ramMB, valid, ok ? "" : flowFieldManager.FailureReason,
                    flowFieldManager.FieldBuildMs, flowFieldManager.SimulationMs, fieldKB));

                UnityEngine.Debug.Log(string.Format(inv,
                    "FlowField ejecución {0}: {1} | celdas BFS={2} | {3:F2} ms | SOC={4} | makespan={5}",
                    i, ok ? "OK" : "FALLÓ (" + flowFieldManager.FailureReason + ")",
                    flowFieldManager.TotalNodesExpanded, sw.Elapsed.TotalMilliseconds, soc, makespan));
            }
        }

        UnityEngine.Debug.Log("¡Benchmark de Flow Field terminado! Archivo guardado en: " + filePath);
    }
}