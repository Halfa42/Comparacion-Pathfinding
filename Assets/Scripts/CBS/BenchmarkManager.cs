using UnityEngine;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;

public class BenchmarkManager : MonoBehaviour
{
    public CBSManager cbsManager;
    [SerializeField] private int numberOfRuns = 10;
    [SerializeField] [Range(2, 11)] private int numberOfAgents = 10;

    private readonly int[,] maze = new int[,]
    {
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1},
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
        {1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
    };

    void Start()
    {
        RunBenchmark();
    }

    void RunBenchmark()
    {
        var starts = new List<Vector2Int>();
        Vector2Int goal = new Vector2Int(21, 0);

        for (int i = 0; i < numberOfAgents; i++)
        {
            starts.Add(new Vector2Int(13 + i, 13));
        }

        var inv = CultureInfo.InvariantCulture;
        string filePath = Application.dataPath + "/CBS_Benchmark_Resultados.csv";

        // Ejecución de calentamiento (JIT)
        cbsManager.FindPaths(maze, starts, goal);

        using (var writer = new StreamWriter(filePath))
        {
            writer.WriteLine("Ejecucion,Exito,Nodos Expandidos (Alto Nivel),Tiempo (ms),Suma de Costos,Makespan,RAM Pico (MB),Solucion Valida,Motivo Fallo");

            for (int i = 1; i <= numberOfRuns; i++)
            {
                System.GC.Collect();
                long startMemory = System.GC.GetTotalMemory(true);

                var sw = System.Diagnostics.Stopwatch.StartNew();
                var resultPaths = cbsManager.FindPaths(maze, starts, goal);
                sw.Stop();

                bool ok = resultPaths != null && cbsManager.Success;
                int soc = ok ? resultPaths.Sum(p => p.Count - 1) : -1;
                int makespan = ok ? resultPaths.Max(p => p.Count - 1) : -1;
                bool valid = ok && !cbsManager.HasConflicts(resultPaths);
                double ramMB = System.Math.Max(0, cbsManager.PeakMemoryBytes - startMemory) / (1024.0 * 1024.0);

                writer.WriteLine(string.Format(inv, "{0},{1},{2},{3:F2},{4},{5},{6:F2},{7},{8}",
                    i, ok, cbsManager.TotalNodesExpanded, sw.Elapsed.TotalMilliseconds,
                    soc, makespan, ramMB, valid, ok ? "" : cbsManager.FailureReason));

                UnityEngine.Debug.Log(string.Format(inv,
                    "Ejecución {0}: {1} | nodos={2} | {3:F1} ms | SOC={4} | makespan={5}",
                    i, ok ? "OK" : "FALLÓ (" + cbsManager.FailureReason + ")",
                    cbsManager.TotalNodesExpanded, sw.Elapsed.TotalMilliseconds, soc, makespan));
            }
        }

        UnityEngine.Debug.Log("¡Benchmark terminado! Archivo guardado en: " + filePath);
    }
}