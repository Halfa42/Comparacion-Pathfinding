
using System;
using System.Collections.Generic;
using UnityEngine;

public class DStarLitePathfinder : MonoBehaviour
{
    private const float Epsilon = 0.0001f;
    private const float Infinity = float.PositiveInfinity;

    [SerializeField] private MazeNavigationGrid grid;

    private float[,] g;
    private float[,] rhs;

    private Vector2Int start;
    private Vector2Int lastStart;
    private Vector2Int goal;

    private float km;
    private bool initialized;

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

        int width = grid.Width;
        int depth = grid.Depth;

        g = new float[width, depth];
        rhs = new float[width, depth];

        for (int x = 0; x < width; x++)
        {
            for (int z = 0; z < depth; z++)
            {
                g[x, z] = Infinity;
                rhs[x, z] = Infinity;
            }
        }

        open.Clear();
        entries.Clear();
        km = 0f;
        insertionOrder = 0;

        rhs[goal.x, goal.y] = 0f;
        Insert(goal, CalculateKey(goal));

        initialized = true;
        return true;
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

            QueueEntry entry = PopMin();
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

            if (grid.IsWalkable(u))
            {
                foreach (Vector2Int successor in
                         grid.GetNeighbours(u))
                {
                    float candidate =
                        grid.Cost(u, successor) +
                        g[successor.x, successor.y];

                    if (candidate < best)
                        best = candidate;
                }
            }

            rhs[u.x, u.y] = best;
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
