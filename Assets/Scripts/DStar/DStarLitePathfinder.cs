using System;
using System.Collections.Generic;
using UnityEngine;

public class DStarLitePathfinder : MonoBehaviour, IGridPathfinder
{
    private const float Epsilon = 0.0001f;
    private const float Infinity = float.PositiveInfinity;

    [SerializeField] private MazeNavigationGrid grid;

    [Header("Moving target")]
    [Tooltip("On: when the goal moves, reuse the previous search (MT-D* Lite style deletion). " +
             "Off: restart the search from scratch on every goal change (useful as a baseline).")]
    [SerializeField] private bool incrementalGoalUpdates = true;

    private float[,] g;
    private float[,] rhs;
    // par[x, z] = index (z * width + x) of the successor that gives rhs (the next cell toward the goal); -1 = none.
    // It defines the search tree rooted at the goal, which is what a goal change reuses.
    private int[,] par;

    private Vector2Int start;
    private Vector2Int lastStart;
    private Vector2Int goal;

    private float km;
    private bool initialized;

    // --- Métricas para benchmark ---
    /// <summary>Nodos sacados de la cola abierta desde el último Initialize (acumulativo entre replanificaciones).</summary>
    public int NodesExpanded { get; private set; }
    /// <summary>Tamaño máximo que alcanzó la cola abierta desde el último Initialize.</summary>
    public int MaxOpenSize { get; private set; }
    public MazeNavigationGrid Grid => grid;
    public long SearchNodes { get; private set; }
    public long SearchPeakMemory => 0;

    public bool BindGrid(MazeNavigationGrid navigationGrid)
    {
        if (grid != null) grid.CellChanged -= OnCellChanged;
        grid = navigationGrid;
        initialized = false;
        if (grid != null && isActiveAndEnabled) grid.CellChanged += OnCellChanged;
        return grid != null;
    }

    /// <summary>Current goal cell (changes with SetGoal).</summary>
    public Vector2Int GoalCell => goal;
    /// <summary>Number of times the goal moved to a different cell since the last Initialize.</summary>
    public int GoalChanges { get; private set; }
    /// <summary>Cells removed from the search tree by the last goal change (0 = everything was reused).</summary>
    public int LastGoalChangeDeletedCells { get; private set; }

    private readonly SortedSet<QueueEntry> open = new SortedSet<QueueEntry>(Comparer);
    private readonly Dictionary<Vector2Int, QueueEntry> entries = new();
    private long insertionOrder;

    private struct Key
    {
        public float first;
        public float second;

        public Key(float first, float second)
        {
            this.first = first;
            this.second = second;
        }
    }

    private struct QueueEntry
    {
        public Vector2Int node;
        public Key key;
        public long order;

        public QueueEntry(Vector2Int node, Key key, long order)
        {
            this.node = node;
            this.key = key;
            this.order = order;
        }
    }

    private sealed class QueueComparer : IComparer<QueueEntry>
    {
        public int Compare(QueueEntry a, QueueEntry b)
        {
            int c = a.key.first.CompareTo(b.key.first);
            if (c != 0) return c;

            c = a.key.second.CompareTo(b.key.second);
            if (c != 0) return c;

            return a.order.CompareTo(b.order);
        }
    }

    private static readonly IComparer<QueueEntry> Comparer =
        new QueueComparer();

    private void OnEnable()
    {
        if (grid != null)
            grid.CellChanged += OnCellChanged;
    }

    private void OnDisable()
    {
        if (grid != null)
            grid.CellChanged -= OnCellChanged;
    }

    public bool Initialize(Vector3 startWorld, Vector3 goalWorld)
    {
        if (grid == null)
        {
            Debug.LogError("D* Lite requires a GridManager.");
            return false;
        }

        start = grid.WorldToCell(startWorld);
        lastStart = start;
        goal = grid.WorldToCell(goalWorld);

        if (!grid.IsWalkable(start) || !grid.IsWalkable(goal))
        {
            Debug.LogWarning("Start or goal is outside the grid or blocked.");
            return false;
        }

        NodesExpanded = 0;
        MaxOpenSize = 0;
        GoalChanges = 0;
        LastGoalChangeDeletedCells = 0;
        ResetSearch();

        initialized = true;
        return true;
    }

    // Fresh search rooted at the current goal (start and metrics are kept).
    private void ResetSearch()
    {
        int width = grid.Width;
        int depth = grid.Depth;

        g = new float[width, depth];
        rhs = new float[width, depth];
        par = new int[width, depth];

        for (int x = 0; x < width; x++)
        {
            for (int z = 0; z < depth; z++)
            {
                g[x, z] = Infinity;
                rhs[x, z] = Infinity;
                par[x, z] = -1;
            }
        }

        open.Clear();
        entries.Clear();
        km = 0f;
        insertionOrder = 0;
        lastStart = start;

        rhs[goal.x, goal.y] = 0f;
        Insert(goal, CalculateKey(goal));
    }

    /// <summary>
    /// Moves the goal (moving target). Call it whenever the target moves, then TryGetPath as usual
    /// (or use the TryGetPath overload that takes the goal). Nothing happens while the target stays
    /// in the same cell. Returns false if not initialized or the new goal cell is blocked.
    ///
    /// D* Lite searches backward from the goal, so the goal is the ROOT of the search tree. Moving the
    /// root is the case MT-D* Lite handles (Sun, Yeoh &amp; Koenig, "Moving Target D* Lite", AAMAS 2010),
    /// mirrored here because our search runs goal -> agent:
    ///   1. keep the subtree hanging from the new goal (its paths to the old goal pass through the new
    ///      goal, so they are still useful);
    ///   2. delete every other cell of the old search tree (g = rhs = infinity, out of the queue);
    ///   3. the new goal becomes the root (rhs = 0);
    ///   4. deleted cells take their rhs from neighbours that were kept and go back into the queue.
    /// The next ComputeShortestPath repairs only what changed. Agent moves are still handled by km.
    /// </summary>
    public bool SetGoal(Vector3 goalWorld)
    {
        if (!initialized) return false;

        Vector2Int newGoal = grid.WorldToCell(goalWorld);
        if (newGoal == goal) return true;
        if (!grid.IsWalkable(newGoal)) return false;

        GoalChanges++;

        if (!incrementalGoalUpdates)
        {
            goal = newGoal;
            ResetSearch();
            LastGoalChangeDeletedCells = grid.Width * grid.Depth;
            return true;
        }

        OptimizedDeletion(newGoal);
        return true;
    }

    /// <summary>Moving-target version of TryGetPath: updates the goal, then plans from the current position.</summary>
    public bool TryGetPath(Vector3 currentWorldPosition, Vector3 goalWorld, out List<Vector3> worldPath)
    {
        worldPath = new List<Vector3>();
        SearchNodes = 0;
        if (grid == null) return false;
        int before = NodesExpanded;
        if (!initialized)
        {
            if (!Initialize(currentWorldPosition, goalWorld)) return false;
            before = 0;
        }
        try
        {
            if (!SetGoal(goalWorld)) return false;
            return TryGetPath(currentWorldPosition, out worldPath);
        }
        finally { SearchNodes = Math.Max(0, NodesExpanded - before); }
    }

    private void OptimizedDeletion(Vector2Int newGoal)
    {
        int width = grid.Width;
        int depth = grid.Depth;
        int cells = width * depth;

        // 1) Which cells hang from the new goal? Follow each cell's par chain (memoized).
        //    0 = unknown, 1 = in the subtree of the new goal, 2 = not.
        var mark = new byte[cells];
        mark[newGoal.y * width + newGoal.x] = 1;
        var chain = new List<int>();

        for (int index = 0; index < cells; index++)
        {
            if (mark[index] != 0) continue;

            chain.Clear();
            int current = index;
            byte result = 2;
            int steps = 0;
            while (true)
            {
                if (mark[current] != 0) { result = mark[current]; break; }
                chain.Add(current);
                int next = par[current % width, current / width];
                if (next < 0 || ++steps > cells) break; // reached the old root, or a cycle: not in the subtree
                current = next;
            }
            foreach (int c in chain) mark[c] = result;
        }

        // 2) Delete the rest of the old search tree.
        var deleted = new List<Vector2Int>();
        for (int x = 0; x < width; x++)
        {
            for (int z = 0; z < depth; z++)
            {
                if (mark[z * width + x] == 1) continue;

                var cell = new Vector2Int(x, z);
                bool inOpen = entries.ContainsKey(cell);
                if (float.IsInfinity(g[x, z]) && float.IsInfinity(rhs[x, z]) && !inOpen) continue;

                g[x, z] = Infinity;
                rhs[x, z] = Infinity;
                par[x, z] = -1;
                Remove(cell);
                deleted.Add(cell);
            }
        }

        // 2b) Every kept cell reaches the old goal THROUGH the new goal, so its distance to the new goal
        //     is its old value minus delta = old distance from the new goal to the old goal. Subtracting
        //     delta keeps the subtree consistent, so it is not re-expanded (about half the work on this
        //     maze; tested against Dijkstra with a moving target: identical path costs).
        float delta = rhs[newGoal.x, newGoal.y];
        if (float.IsInfinity(delta))
        {
            g[newGoal.x, newGoal.y] = Infinity; // the new goal was never reached: nothing to reuse
        }
        else
        {
            for (int x = 0; x < width; x++)
            {
                for (int z = 0; z < depth; z++)
                {
                    if (mark[z * width + x] != 1) continue;
                    if (!float.IsInfinity(g[x, z])) g[x, z] -= delta;
                    if (!float.IsInfinity(rhs[x, z])) rhs[x, z] -= delta;
                }
            }
        }

        // 3) The new goal is the new root.
        goal = newGoal;
        par[goal.x, goal.y] = -1;
        rhs[goal.x, goal.y] = 0f;
        Remove(goal);
        if (!Approximately(g[goal.x, goal.y], rhs[goal.x, goal.y]))
            Insert(goal, CalculateKey(goal));

        // Kept cells still in the queue need keys computed from their shifted values.
        var queued = new List<Vector2Int>(entries.Keys);
        foreach (Vector2Int cell in queued)
            if (cell != goal) Insert(cell, CalculateKey(cell));

        // 4) Deleted cells next to the kept subtree get a new rhs and go back into the queue.
        foreach (Vector2Int cell in deleted)
        {
            if (cell == goal || !grid.IsWalkable(cell)) continue;

            float best = Infinity;
            int bestPar = -1;
            foreach (Vector2Int successor in grid.GetNeighbours(cell))
            {
                float candidate = grid.Cost(cell, successor) + g[successor.x, successor.y];
                if (candidate < best)
                {
                    best = candidate;
                    bestPar = successor.y * width + successor.x;
                }
            }

            rhs[cell.x, cell.y] = best;
            par[cell.x, cell.y] = bestPar;
            if (!float.IsInfinity(best))
                Insert(cell, CalculateKey(cell));
        }

        LastGoalChangeDeletedCells = deleted.Count;
    }

    public bool TryGetPath(
        Vector3 currentWorldPosition,
        out List<Vector3> worldPath)
    {
        worldPath = new List<Vector3>();

        if (!initialized)
            return false;

        Vector2Int newStart =
            grid.WorldToCell(currentWorldPosition);

        if (!grid.IsWalkable(newStart))
            return false;

        if (newStart != start)
        {
            km += Heuristic(lastStart, newStart);
            start = newStart;
            lastStart = newStart;
        }

        ComputeShortestPath();

        if (float.IsInfinity(g[start.x, start.y]))
            return false;

        Vector2Int current = start;
        worldPath.Add(grid.CellToWorld(current));

        int maxSteps = grid.Width * grid.Depth;

        for (int i = 0; i < maxSteps && current != goal; i++)
        {
            float bestCost = Infinity;
            Vector2Int bestNext = current;

            foreach (Vector2Int next in grid.GetNeighbours(current))
            {
                float candidate =
                    grid.Cost(current, next) +
                    g[next.x, next.y];

                if (candidate < bestCost)
                {
                    bestCost = candidate;
                    bestNext = next;
                }
            }

            if (bestNext == current ||
                float.IsInfinity(bestCost))
            {
                worldPath.Clear();
                return false;
            }

            current = bestNext;
            worldPath.Add(grid.CellToWorld(current));
        }

        if (current != goal)
        {
            worldPath.Clear();
            return false;
        }

        return true;
    }

    private void ComputeShortestPath()
    {
        while (open.Count > 0)
        {
            QueueEntry top = open.Min;

            Key startKey = CalculateKey(start);

            if (CompareKeys(top.key, startKey) >= 0 &&
                Approximately(rhs[start.x, start.y],
                              g[start.x, start.y]))
                break;

            if (open.Count > MaxOpenSize) MaxOpenSize = open.Count;
            QueueEntry entry = PopMin();
            NodesExpanded++;
            Vector2Int u = entry.node;

            Key newKey = CalculateKey(u);

            if (CompareKeys(entry.key, newKey) < 0)
            {
                Insert(u, newKey);
            }
            else if (g[u.x, u.y] >
                     rhs[u.x, u.y])
            {
                g[u.x, u.y] = rhs[u.x, u.y];

                foreach (Vector2Int pred in
                         grid.GetNeighbours(u))
                    UpdateVertex(pred);
            }
            else
            {
                g[u.x, u.y] = Infinity;

                UpdateVertex(u);

                foreach (Vector2Int pred in
                         grid.GetNeighbours(u))
                    UpdateVertex(pred);
            }
        }
    }

    private void UpdateVertex(Vector2Int u)
    {
        if (!grid.IsInside(u))
            return;

        if (u != goal)
        {
            float best = Infinity;
            int bestPar = -1;

            if (grid.IsWalkable(u))
            {
                foreach (Vector2Int successor in
                         grid.GetNeighbours(u))
                {
                    float candidate =
                        grid.Cost(u, successor) +
                        g[successor.x, successor.y];

                    if (candidate < best)
                    {
                        best = candidate;
                        bestPar = successor.y * grid.Width + successor.x;
                    }
                }
            }

            rhs[u.x, u.y] = best;
            par[u.x, u.y] = bestPar;
        }
        else
        {
            par[u.x, u.y] = -1;
        }

        Remove(u);

        if (!Approximately(g[u.x, u.y], rhs[u.x, u.y]))
            Insert(u, CalculateKey(u));
    }

    private Key CalculateKey(Vector2Int u)
    {
        float min = Mathf.Min(
            g[u.x, u.y],
            rhs[u.x, u.y]);

        return new Key(
            min + Heuristic(start, u) + km,
            min
        );
    }

    private float Heuristic(Vector2Int a, Vector2Int b)
    {
        int dx = Mathf.Abs(a.x - b.x);
        int dz = Mathf.Abs(a.y - b.y);

        int diagonal = Mathf.Min(dx, dz);
        int straight = Mathf.Max(dx, dz) - diagonal;

        return grid.CellSize *
            (diagonal * Mathf.Sqrt(2f) + straight);
    }

    private void OnCellChanged(Vector2Int changed)
    {
        if (!initialized)
            return;

        // A changed cell affects its own cost-to-go and
        // the costs of adjacent nodes.
        UpdateVertex(changed);

        foreach (Vector2Int neighbour in
                 GetGeometricNeighbours(changed))
            UpdateVertex(neighbour);
    }

    private IEnumerable<Vector2Int> GetGeometricNeighbours(
        Vector2Int cell)
    {
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dz = -1; dz <= 1; dz++)
            {
                if (dx == 0 && dz == 0)
                    continue;

                Vector2Int next =
                    new Vector2Int(cell.x + dx, cell.y + dz);

                if (grid.IsInside(next))
                    yield return next;
            }
        }
    }

    private void Insert(Vector2Int node, Key key)
    {
        Remove(node);

        QueueEntry entry = new QueueEntry(
            node, key, insertionOrder++);

        open.Add(entry);
        entries[node] = entry;
    }

    private void Remove(Vector2Int node)
    {
        if (!entries.TryGetValue(node, out QueueEntry entry))
            return;

        open.Remove(entry);
        entries.Remove(node);
    }

    private QueueEntry PopMin()
    {
        QueueEntry min = open.Min;
        open.Remove(min);
        entries.Remove(min.node);
        return min;
    }

    private static int CompareKeys(Key a, Key b)
    {
        if (a.first < b.first - Epsilon) return -1;
        if (a.first > b.first + Epsilon) return 1;
        if (a.second < b.second - Epsilon) return -1;
        if (a.second > b.second + Epsilon) return 1;
        return 0;
    }

    private static bool Approximately(float a, float b)
    {
        if (float.IsInfinity(a) || float.IsInfinity(b))
            return a.Equals(b);

        return Mathf.Abs(a - b) < Epsilon;
    }
}
