using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class CBSManager : MonoBehaviour, IGridPathfinder
{
    private MazeNavigationGrid navigationGrid;
    public long SearchNodes => TotalNodesExpanded;
    public long SearchPeakMemory => PeakMemoryBytes;

    public bool BindGrid(MazeNavigationGrid grid)
    {
        navigationGrid = grid;
        return grid != null;
    }

    // The pursuit has one controlled agent. The existing multi-agent CBS API remains available.
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
        var routes = FindPaths(cells, new List<Vector2Int> { startCell }, new List<Vector2Int> { goalCell });
        if (!Success || routes == null || routes.Count == 0) return false;
        path = routes[0].Select(cell => navigationGrid.CellToWorld(cell)).ToList();
        return true;
    }

    // --- ESTRUCTURAS DE ALTO NIVEL (CBS) ---
    public class CBSNode
    {
        public List<List<Vector2Int>> Paths = new List<List<Vector2Int>>();
        public List<Constraint> Constraints = new List<Constraint>();
        public int Cost = 0;          
        public int ConflictCount = 0; 
    }

    public struct Constraint
    {
        public int AgentID;
        public Vector2Int Position; 
        public Vector2Int From;     
        public int Time;            
        public bool IsEdge;
    }

    public class Conflict
    {
        public int Agent1, Agent2;
        public int Time;
        public bool IsEdge;
        public Vector2Int From1, Pos1, From2, Pos2;
    }

    // --- PARÁMETROS Y MÉTRICAS ---
    [Header("Límites de seguridad")]
    [SerializeField] private int maxCBSNodes = 100000;
    [SerializeField] private float timeLimitSeconds = 30f;
    [SerializeField] private int maxTimeLimit = 200; 

    public int TotalNodesExpanded { get; private set; }
    public bool Success { get; private set; }
    public string FailureReason { get; private set; }
    public long PeakMemoryBytes { get; private set; }

    private readonly Dictionary<Vector2Int, int[,]> distanceCache = new Dictionary<Vector2Int, int[,]>();

    private static readonly Vector2Int[] Directions =
    {
        new Vector2Int(0, 1), new Vector2Int(0, -1),
        new Vector2Int(-1, 0), new Vector2Int(1, 0),
        new Vector2Int(0, 0) // Esperar (clave para que los agentes hagan fila sin chocar)
    };

    // --- ALGORITMO CBS DIRIGIDO AL PLAYER ---
    public List<List<Vector2Int>> FindPaths(int[,] grid, List<Vector2Int> starts, Vector2Int playerGoal)
    {
        var goals = Enumerable.Repeat(playerGoal, starts.Count).ToList();
        return FindPaths(grid, starts, goals);
    }

    // Preserve the original benchmark API with a separate goal for each agent.
    public List<List<Vector2Int>> FindPaths(int[,] grid, List<Vector2Int> starts, List<Vector2Int> goals)
    {
        TotalNodesExpanded = 0;
        Success = false;
        FailureReason = "";
        PeakMemoryBytes = 0;
        distanceCache.Clear(); // Limpiamos el caché de heurística para el nuevo frame

        var clock = System.Diagnostics.Stopwatch.StartNew();

        var openList = new PriorityQueue<CBSNode>((a, b) =>
        {
            int cmp = a.Cost.CompareTo(b.Cost);
            return cmp != 0 ? cmp : a.ConflictCount.CompareTo(b.ConflictCount);
        });

        CBSNode root = new CBSNode();
        for (int i = 0; i < starts.Count; i++)
        {
            var path = SpaceTimeAStar(grid, starts[i], goals[i], i, root.Constraints);
            if (path == null)
            {
                FailureReason = $"Agente {i} no encuentra ruta hacia el objetivo.";
                return null;
            }
            root.Paths.Add(path);
            root.Cost += path.Count - 1;
        }
        FindFirstConflict(root.Paths, out root.ConflictCount);
        openList.Enqueue(root);

        while (openList.Count > 0)
        {
            if (TotalNodesExpanded >= maxCBSNodes)
            {
                FailureReason = "Límite de nodos CBS alcanzado";
                return null;
            }
            if (clock.Elapsed.TotalSeconds > timeLimitSeconds)
            {
                FailureReason = "Límite de tiempo alcanzado";
                return null;
            }

            TotalNodesExpanded++;
            if (TotalNodesExpanded % 25 == 0)
                PeakMemoryBytes = System.Math.Max(PeakMemoryBytes, System.GC.GetTotalMemory(false));

            CBSNode bestNode = openList.Dequeue();

            int dummy;
            Conflict conflict = FindFirstConflict(bestNode.Paths, out dummy);
            if (conflict == null)
            {
                Success = true;
                PeakMemoryBytes = System.Math.Max(PeakMemoryBytes, System.GC.GetTotalMemory(false));
                return bestNode.Paths; 
            }

            for (int i = 0; i < 2; i++)
            {
                CBSNode child = new CBSNode();
                child.Constraints.AddRange(bestNode.Constraints);
                child.Paths.AddRange(bestNode.Paths);

                Constraint newConstraint = new Constraint
                {
                    AgentID = i == 0 ? conflict.Agent1 : conflict.Agent2,
                    Position = i == 0 ? conflict.Pos1 : conflict.Pos2,
                    From = i == 0 ? conflict.From1 : conflict.From2,
                    Time = conflict.Time,
                    IsEdge = conflict.IsEdge
                };
                child.Constraints.Add(newConstraint);

                int a = newConstraint.AgentID;
                var newPath = SpaceTimeAStar(grid, starts[a], goals[a], a, child.Constraints);
                if (newPath == null) continue;

                child.Paths[a] = newPath;
                int cost = 0;
                for (int k = 0; k < child.Paths.Count; k++) cost += child.Paths[k].Count - 1;
                child.Cost = cost;
                FindFirstConflict(child.Paths, out child.ConflictCount);
                openList.Enqueue(child);
            }
        }

        FailureReason = "Sin solución (open list vacía)";
        return null;
    }

    private Conflict FindFirstConflict(List<List<Vector2Int>> paths, out int total)
    {
        total = 0;
        Conflict first = null;
        int maxTime = 0;
        for (int p = 0; p < paths.Count; p++) maxTime = System.Math.Max(maxTime, paths[p].Count);

        for (int t = 0; t < maxTime; t++)
        {
            for (int i = 0; i < paths.Count; i++)
            {
                // LÓGICA DE DESAPARICIÓN: Si el tiempo actual 't' es mayor o igual al tamaño
                // de la ruta del agente, significa que ya llegó a la meta del player.
                // Se ignora para futuros conflictos, liberando la casilla.
                if (t >= paths[i].Count) continue; 
                
                for (int j = i + 1; j < paths.Count; j++)
                {
                    if (t >= paths[j].Count) continue;

                    // Conflicto de vértice
                    if (paths[i][t] == paths[j][t])
                    {
                        total++;
                        if (first == null)
                            first = new Conflict
                            {
                                Agent1 = i, Agent2 = j, Time = t, IsEdge = false,
                                Pos1 = paths[i][t], Pos2 = paths[i][t]
                            };
                    }

                    // Conflicto de arista
                    if (t + 1 < paths[i].Count && t + 1 < paths[j].Count &&
                        paths[i][t] == paths[j][t + 1] && paths[i][t + 1] == paths[j][t] &&
                        paths[i][t] != paths[i][t + 1])
                    {
                        total++;
                        if (first == null)
                            first = new Conflict
                            {
                                Agent1 = i, Agent2 = j, Time = t + 1, IsEdge = true,
                                From1 = paths[i][t], Pos1 = paths[i][t + 1],
                                From2 = paths[j][t], Pos2 = paths[j][t + 1]
                            };
                    }
                }
            }
        }
        return first;
    }

    public bool HasConflicts(List<List<Vector2Int>> paths)
    {
        int total;
        FindFirstConflict(paths, out total);
        return total > 0;
    }

    // --- A* EN ESPACIO-TIEMPO (Bajo Nivel) ---
    private class AStarNode
    {
        public Vector2Int Position;
        public int Time;
        public int G;
        public int H;
        public int F => G + H;
        public AStarNode Parent;
    }

    private List<Vector2Int> SpaceTimeAStar(int[,] grid, Vector2Int start, Vector2Int goal, int agentID, List<Constraint> constraints)
    {
        int W = grid.GetLength(1);
        int H = grid.GetLength(0);
        long N = (long)W * H;

        int[,] dist = GetDistanceMap(grid, goal);
        if (dist[start.y, start.x] < 0) return null; 

        var vertexC = new HashSet<long>();
        var edgeC = new HashSet<long>();
        
        for (int k = 0; k < constraints.Count; k++)
        {
            var c = constraints[k];
            if (c.AgentID != agentID) continue;
            long to = (long)c.Position.y * W + c.Position.x;
            
            if (c.IsEdge)
            {
                long from = (long)c.From.y * W + c.From.x;
                edgeC.Add(((long)c.Time * N + from) * N + to);
            }
            else
            {
                vertexC.Add((long)c.Time * N + to);
            }
        }

        var openList = new PriorityQueue<AStarNode>((a, b) =>
        {
            int cmp = a.F.CompareTo(b.F);
            return cmp != 0 ? cmp : a.H.CompareTo(b.H);
        });

        var visited = new HashSet<long>();

        var startNode = new AStarNode { Position = start, Time = 0, G = 0, H = dist[start.y, start.x] };
        openList.Enqueue(startNode);
        visited.Add(((long)0 * N) + (long)start.y * W + start.x);

        while (openList.Count > 0)
        {
            AStarNode current = openList.Dequeue();

            if (current.Position == goal)
                return ReconstructPath(current);

            if (current.Time >= maxTimeLimit) continue;

            int nextTime = current.Time + 1;
            long fromCell = (long)current.Position.y * W + current.Position.x;

            for (int d = 0; d < Directions.Length; d++)
            {
                Vector2Int nextPos = current.Position + Directions[d];

                if (nextPos.x < 0 || nextPos.x >= W || nextPos.y < 0 || nextPos.y >= H) continue;
                if (grid[nextPos.y, nextPos.x] == 1) continue;

                long toCell = (long)nextPos.y * W + nextPos.x;
                if (vertexC.Contains((long)nextTime * N + toCell)) continue;
                if (edgeC.Count > 0 && edgeC.Contains(((long)nextTime * N + fromCell) * N + toCell)) continue;

                long stateKey = (long)nextTime * N + toCell;
                if (!visited.Add(stateKey)) continue;

                openList.Enqueue(new AStarNode
                {
                    Position = nextPos,
                    Time = nextTime,
                    G = current.G + 1,
                    H = dist[nextPos.y, nextPos.x],
                    Parent = current
                });
            }
        }

        return null;
    }

    private int[,] GetDistanceMap(int[,] grid, Vector2Int goal)
    {
        int[,] map;
        if (distanceCache.TryGetValue(goal, out map)) return map;

        int W = grid.GetLength(1);
        int H = grid.GetLength(0);
        map = new int[H, W];
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
                map[y, x] = -1;

        var queue = new Queue<Vector2Int>();
        map[goal.y, goal.x] = 0;
        queue.Enqueue(goal);

        while (queue.Count > 0)
        {
            var cur = queue.Dequeue();
            for (int d = 0; d < 4; d++)
            {
                var n = cur + Directions[d]; // Usamos los desplazamientos ortogonales
                if (n.x < 0 || n.x >= W || n.y < 0 || n.y >= H) continue;
                if (grid[n.y, n.x] == 1 || map[n.y, n.x] != -1) continue;
                map[n.y, n.x] = map[cur.y, cur.x] + 1;
                queue.Enqueue(n);
            }
        }

        distanceCache[goal] = map;
        return map;
    }

    private List<Vector2Int> ReconstructPath(AStarNode node)
    {
        List<Vector2Int> path = new List<Vector2Int>();
        while (node != null)
        {
            path.Add(node.Position);
            node = node.Parent;
        }
        path.Reverse();
        return path;
    }
}

// --- COLA DE PRIORIDAD (MIN-HEAP) INCLUIDA AQUÍ PARA SOLUCIONAR EL ERROR ---
public class PriorityQueue<T>
{
    private List<T> data;
    private System.Comparison<T> comparison;

    public int Count => data.Count;

    public PriorityQueue(System.Comparison<T> comparison)
    {
        this.data = new List<T>();
        this.comparison = comparison;
    }

    public void Enqueue(T item)
    {
        data.Add(item);
        int ci = data.Count - 1;
        while (ci > 0)
        {
            int pi = (ci - 1) / 2;
            if (comparison(data[ci], data[pi]) >= 0) break;
            T tmp = data[ci]; data[ci] = data[pi]; data[pi] = tmp;
            ci = pi;
        }
    }

    public T Dequeue()
    {
        int li = data.Count - 1;
        T frontItem = data[0];
        data[0] = data[li];
        data.RemoveAt(li);

        --li;
        int pi = 0;
        while (true)
        {
            int ci = pi * 2 + 1;
            if (ci > li) break;
            int rc = ci + 1;
            if (rc <= li && comparison(data[rc], data[ci]) < 0)
                ci = rc;
            if (comparison(data[pi], data[ci]) <= 0) break;
            T tmp = data[pi]; data[pi] = data[ci]; data[ci] = tmp;
            pi = ci;
        }
        return frontItem;
    }
}
