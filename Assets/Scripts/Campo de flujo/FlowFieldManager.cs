using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Campo de flujo (Flow Field) para múltiples agentes sobre una grilla.
///
/// Misma API y mismo modelo que CBSManager para que la comparación sea justa:
///  - FindPaths(grid, starts, goals) devuelve una lista de rutas (una posición por paso de tiempo).
///  - Movimiento discreto en 4 direcciones + esperar, un paso por unidad de tiempo.
///  - El agente DESAPARECE al llegar a su meta (no bloquea la casilla después).
///  - Conflictos prohibidos: de vértice (misma celda, mismo tiempo) y de arista (swap).
///
/// Idea: 1) BFS desde la meta calcula el costo real a cada celda (campo de integración).
///       2) Cada celda guarda la dirección hacia el vecino de menor costo (campo de flujo).
///       3) Se simula paso a paso: cada agente sigue el flujo y, si dos quieren la misma celda,
///          gana el de mayor prioridad y el otro espera. No hay replanificación.
/// </summary>
public class FlowFieldManager : MonoBehaviour, IGridPathfinder
{
    private MazeNavigationGrid navigationGrid;
    public long SearchNodes => TotalNodesExpanded;
    public long SearchPeakMemory => PeakMemoryBytes;

    public bool BindGrid(MazeNavigationGrid grid)
    {
        navigationGrid = grid;
        return grid != null;
    }

    // The shared follower handles movement, capture and benchmark collection.
    public bool TryGetPath(Vector3 start, Vector3 goal, out List<Vector3> path)
    {
        path = null;
        TotalNodesExpanded = 0;
        PeakMemoryBytes = 0;
        Success = false;
        if (navigationGrid == null) return false;
        Vector2Int startCell = navigationGrid.WorldToCell(start), goalCell = navigationGrid.WorldToCell(goal);
        if (!navigationGrid.IsWalkable(startCell) || !navigationGrid.IsWalkable(goalCell)) return false;
        var cells = new int[navigationGrid.Depth, navigationGrid.Width];
        for (int z = 0; z < navigationGrid.Depth; z++)
        for (int x = 0; x < navigationGrid.Width; x++)
            cells[z, x] = navigationGrid.IsWalkable(new Vector2Int(x, z)) ? 0 : 1;
        var routes = FindPaths(cells, new List<Vector2Int> { startCell }, goalCell);
        if (!Success || routes == null || routes.Count == 0) return false;
        path = routes[0].ConvertAll(cell => navigationGrid.CellToWorld(cell));
        return true;
    }

    public enum PriorityRule
    {
        ClosestToGoal, // gana el agente con menor costo restante (desempata por ID)
        LowestAgentID  // gana el agente con menor ID
    }

    [Header("Parámetros (mismo horizonte que el A* de CBS)")]
    [SerializeField] private int maxSimulationSteps = 200;
    [SerializeField] private PriorityRule priorityRule = PriorityRule.ClosestToGoal;

    // --- MÉTRICAS ---
    /// <summary>Celdas expandidas por el BFS (suma de todos los campos construidos).</summary>
    public int TotalNodesExpanded { get; private set; }
    public bool Success { get; private set; }
    public string FailureReason { get; private set; }
    public long PeakMemoryBytes { get; private set; }
    public int SimulationSteps { get; private set; }
    public double FieldBuildMs { get; private set; }
    public double SimulationMs { get; private set; }
    /// <summary>Memoria teórica de los campos (costo + dirección, 2 int por celda por meta distinta).</summary>
    public long FieldMemoryBytes { get; private set; }

    private class FlowField
    {
        public int[] Cost;  // distancia real a la meta (-1 = inalcanzable)
        public int[] Next;  // índice de la celda siguiente (y*W+x); -1 si es la meta o no hay camino
    }

    // Mismo orden de direcciones que CBSManager
    private static readonly Vector2Int[] Directions =
    {
        new Vector2Int(0, 1), new Vector2Int(0, -1),
        new Vector2Int(-1, 0), new Vector2Int(1, 0)
    };

    // --- SOBRECARGA PARA PERSEGUIR A UN OBJETIVO ÚNICO (PLAYER) ---
    public List<List<Vector2Int>> FindPaths(int[,] grid, List<Vector2Int> starts, Vector2Int playerGoal)
    {
        // Creamos una lista donde todos los agentes tienen exactamente la misma meta
        List<Vector2Int> goals = new List<Vector2Int>();
        for (int i = 0; i < starts.Count; i++)
        {
            goals.Add(playerGoal);
        }
        // Llamamos al método original, el cual ya está optimizado para no recalcular metas repetidas
        return FindPaths(grid, starts, goals);
    }

    public List<List<Vector2Int>> FindPaths(int[,] grid, List<Vector2Int> starts, List<Vector2Int> goals)
    {
        TotalNodesExpanded = 0;
        Success = false;
        FailureReason = "";
        PeakMemoryBytes = 0;
        SimulationSteps = 0;
        FieldBuildMs = 0;
        SimulationMs = 0;
        FieldMemoryBytes = 0;

        int H = grid.GetLength(0);
        int W = grid.GetLength(1);
        int N = W * H;
        int n = starts.Count;

        var clock = System.Diagnostics.Stopwatch.StartNew();

        // ---------- 1) Construir un campo por cada meta distinta ----------
        var fields = new Dictionary<Vector2Int, FlowField>();
        for (int i = 0; i < n; i++)
        {
            if (!fields.ContainsKey(goals[i]))
                fields[goals[i]] = BuildField(grid, goals[i], W, H);
        }
        FieldMemoryBytes = (long)fields.Count * N * 2 * sizeof(int);
        FieldBuildMs = clock.Elapsed.TotalMilliseconds;
        PeakMemoryBytes = System.Math.Max(PeakMemoryBytes, System.GC.GetTotalMemory(false));

        // ---------- 2) Preparar la simulación ----------
        int[] cur = new int[n];
        int[] target = new int[n];
        int[] goalIdx = new int[n];
        bool[] active = new bool[n];
        bool[] moving = new bool[n];
        FlowField[] myField = new FlowField[n];
        int[] order = new int[n];
        int[] occupant = new int[N];
        int[] claim = new int[N];
        for (int c = 0; c < N; c++) { occupant[c] = -1; claim[c] = -1; }

        var paths = new List<List<Vector2Int>>();
        int remaining = 0;

        for (int i = 0; i < n; i++)
        {
            myField[i] = fields[goals[i]];
            goalIdx[i] = goals[i].y * W + goals[i].x;
            cur[i] = starts[i].y * W + starts[i].x;

            if (myField[i].Cost[cur[i]] < 0)
            {
                FailureReason = "Agente " + i + " no puede alcanzar su meta";
                return null;
            }
            for (int j = 0; j < i; j++)
            {
                if (cur[j] == cur[i])
                {
                    FailureReason = "Agentes " + j + " y " + i + " comparten la celda inicial";
                    return null;
                }
            }

            paths.Add(new List<Vector2Int> { starts[i] });
            active[i] = cur[i] != goalIdx[i];
            if (active[i]) remaining++;
        }

        // ---------- 3) Simular paso a paso ----------
        int step = 0;
        while (remaining > 0)
        {
            if (step >= maxSimulationSteps)
            {
                FailureReason = "Límite de pasos de simulación alcanzado";
                SimulationSteps = step;
                return null;
            }

            // Ocupación en el tiempo t (solo agentes que siguen activos)
            int cnt = 0;
            for (int i = 0; i < n; i++)
            {
                if (!active[i]) continue;
                occupant[cur[i]] = i;
                order[cnt++] = i;
            }

            // Orden de prioridad (insertion sort; n es pequeño)
            for (int a = 1; a < cnt; a++)
            {
                int key = order[a];
                int b = a - 1;
                while (b >= 0 && HasHigherPriority(key, order[b], cur, myField))
                {
                    order[b + 1] = order[b];
                    b--;
                }
                order[b + 1] = key;
            }

            // Movimiento deseado: seguir el flujo
            for (int k = 0; k < cnt; k++)
            {
                int i = order[k];
                int nx = myField[i].Next[cur[i]];
                target[i] = nx == -1 ? cur[i] : nx;
                moving[i] = nx != -1;
            }

            // Resolver conflictos hasta que no haya cambios (los que pierden esperan)
            bool changed;
            do
            {
                changed = false;
                for (int k = 0; k < cnt; k++)
                {
                    int i = order[k];
                    claim[cur[i]] = -1;
                    claim[target[i]] = -1;
                }
                // Los que esperan ocupan su celda
                for (int k = 0; k < cnt; k++)
                {
                    int i = order[k];
                    if (!moving[i]) claim[cur[i]] = i;
                }
                // Los que se mueven reclaman su destino por orden de prioridad
                for (int k = 0; k < cnt; k++)
                {
                    int i = order[k];
                    if (!moving[i]) continue;
                    int t = target[i];

                    // Swap: yo voy a la celda de otro que viene a la mía -> ambos esperan
                    int occ = occupant[t];
                    if (occ != -1 && occ != i && moving[occ] && target[occ] == cur[i])
                    {
                        moving[i] = false;
                        moving[occ] = false;
                        changed = true;
                        continue;
                    }

                    if (claim[t] != -1)
                    {
                        moving[i] = false;
                        changed = true;
                        continue;
                    }
                    claim[t] = i;
                }
            } while (changed);

            // Aplicar movimientos
            for (int k = 0; k < cnt; k++) occupant[cur[order[k]]] = -1;

            int movedCount = 0;
            for (int k = 0; k < cnt; k++)
            {
                int i = order[k];
                if (moving[i]) { cur[i] = target[i]; movedCount++; }
            }
            step++;

            for (int k = 0; k < cnt; k++)
            {
                int i = order[k];
                paths[i].Add(new Vector2Int(cur[i] % W, cur[i] / W));
                if (cur[i] == goalIdx[i])
                {
                    active[i] = false; // desaparece al llegar
                    remaining--;
                }
            }

            if (step % 10 == 0)
                PeakMemoryBytes = System.Math.Max(PeakMemoryBytes, System.GC.GetTotalMemory(false));

            if (movedCount == 0 && remaining > 0)
            {
                FailureReason = "Deadlock: ningún agente pudo moverse";
                SimulationSteps = step;
                return null;
            }
        }

        SimulationSteps = step;
        SimulationMs = clock.Elapsed.TotalMilliseconds - FieldBuildMs;
        PeakMemoryBytes = System.Math.Max(PeakMemoryBytes, System.GC.GetTotalMemory(false));
        Success = true;
        return paths;
    }

    // true si el agente 'a' tiene mayor prioridad que 'b'
    private bool HasHigherPriority(int a, int b, int[] cur, FlowField[] fields)
    {
        if (priorityRule == PriorityRule.ClosestToGoal)
        {
            int ca = fields[a].Cost[cur[a]];
            int cb = fields[b].Cost[cur[b]];
            if (ca != cb) return ca < cb;
        }
        return a < b;
    }

    // BFS desde la meta + campo de direcciones
    private FlowField BuildField(int[,] grid, Vector2Int goal, int W, int H)
    {
        int N = W * H;
        var f = new FlowField { Cost = new int[N], Next = new int[N] };
        for (int i = 0; i < N; i++) { f.Cost[i] = -1; f.Next[i] = -1; }

        if (goal.x < 0 || goal.x >= W || goal.y < 0 || goal.y >= H || grid[goal.y, goal.x] == 1)
            return f;

        var queue = new Queue<int>();
        int g = goal.y * W + goal.x;
        f.Cost[g] = 0;
        queue.Enqueue(g);

        while (queue.Count > 0)
        {
            int c = queue.Dequeue();
            TotalNodesExpanded++;
            int cx = c % W, cy = c / W;

            for (int d = 0; d < 4; d++)
            {
                int nx = cx + Directions[d].x;
                int ny = cy + Directions[d].y;
                if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                if (grid[ny, nx] == 1) continue;
                int ni = ny * W + nx;
                if (f.Cost[ni] != -1) continue;
                f.Cost[ni] = f.Cost[c] + 1;
                queue.Enqueue(ni);
            }
        }

        // Dirección de flujo: hacia el primer vecino con costo - 1
        for (int c = 0; c < N; c++)
        {
            if (f.Cost[c] <= 0) continue;
            int cx = c % W, cy = c / W;
            for (int d = 0; d < 4; d++)
            {
                int nx = cx + Directions[d].x;
                int ny = cy + Directions[d].y;
                if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                int ni = ny * W + nx;
                if (f.Cost[ni] == f.Cost[c] - 1) { f.Next[c] = ni; break; }
            }
        }
        return f;
    }

    // Validación idéntica a la de CBSManager (vértice + arista; el agente desaparece al llegar)
    public bool HasConflicts(List<List<Vector2Int>> paths)
    {
        int maxTime = 0;
        for (int p = 0; p < paths.Count; p++) maxTime = System.Math.Max(maxTime, paths[p].Count);

        for (int t = 0; t < maxTime; t++)
        {
            for (int i = 0; i < paths.Count; i++)
            {
                if (t >= paths[i].Count) continue;
                for (int j = i + 1; j < paths.Count; j++)
                {
                    if (t >= paths[j].Count) continue;

                    if (paths[i][t] == paths[j][t]) return true;

                    if (t + 1 < paths[i].Count && t + 1 < paths[j].Count &&
                        paths[i][t] == paths[j][t + 1] && paths[i][t + 1] == paths[j][t] &&
                        paths[i][t] != paths[i][t + 1])
                        return true;
                }
            }
        }
        return false;
    }
}
