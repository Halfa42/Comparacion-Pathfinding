using System;
using System.Collections.Generic;
using UnityEngine;

public class RRTPathfinder : MonoBehaviour
{
    [SerializeField] private MazeNavigationGrid grid;

    [Header("Search")]
    [SerializeField, Min(1)] private int maxIterations = 12000;
    [SerializeField] private int seed = 12345;
    [SerializeField, Min(0.01f)] private float neighbourRadius = 1.5f;
    [SerializeField, Min(0.01f)] private float goalRadius = 0.7f;
    [SerializeField, Min(0f)] private float agentClearance = 0.1f;

    [Header("Terrain sampling")]
    [SerializeField, Range(0f, 1f)] private float uniformProbability = 0.2f;
    [SerializeField, Range(1, 8)] private int cornerThreshold = 3;
    [SerializeField, Min(0.01f)] private float pdfSigma = 0.75f;

    [Header("Adaptive step: Equation (3)")]
    [SerializeField, Min(0.01f)] private float maxStep = 0.7f;
    [SerializeField, Min(0.01f)] private float minStep = 0.5f;
    [SerializeField, Min(0.01f)] private float densityRadius = 2f;
    [SerializeField] private float densityMidpoint = 4f;
    [SerializeField, Min(0.01f)] private float densitySlope = 0.5f;

    public MazeNavigationGrid Grid => grid;
    public int NodeCount => nodes.Count;
    public int CornerCount => corners.Count;
    public int AdaptiveSampleCount { get; private set; }
    public float PathLength { get; private set; }
    public double PlanningMilliseconds { get; private set; }

    private const float Epsilon = 0.00001f;

    private sealed class Node
    {
        public Vector2 point;
        public int parent;
        public float cost;
        public readonly List<int> children = new List<int>();

        public Node(Vector2 point, int parent, float cost)
        {
            this.point = point;
            this.parent = parent;
            this.cost = cost;
        }
    }

    private sealed class CornerNode
    {
        public Vector2 point;
        public int axis;
        public CornerNode left, right;
    }

    private readonly List<Node> nodes = new List<Node>();
    private readonly List<Vector2> corners = new List<Vector2>();
    private readonly List<Vector2> freeCells = new List<Vector2>();
    private readonly List<double> cumulativePdf = new List<double>();
    private readonly List<int> near = new List<int>();
    private readonly Stack<int> descendants = new Stack<int>();
    private CornerNode cornerTree;
    private System.Random random;
    private Vector3 goalWorld, origin;
    private float cellSize, pathHeight;
    private int width, depth;
    private bool initialized;

    public bool Initialize(Vector3 startWorld, Vector3 targetWorld)
    {
        initialized = false;
        ResetSearch();

        if (grid == null || !ValidSettings() || !Finite(startWorld) ||
            !Finite(targetWorld) || !ReadGrid())
        {
            Debug.LogWarning("TA-RRT*: invalid grid, positions or search settings.", this);
            return false;
        }

        goalWorld = targetWorld;
        if (!PointFree(ToGrid(startWorld)) || !PointFree(ToGrid(goalWorld)))
        {
            Debug.LogWarning("TA-RRT*: start or goal is blocked, outside the grid or lacks clearance.", this);
            return false;
        }

        initialized = true;
        return true;
    }

    /// <summary>
    /// Runs a synchronous, bounded search. False means no route was found
    /// within the iteration budget, not that a route cannot exist.
    /// </summary>
    public bool TryGetPath(Vector3 currentWorldPosition, out List<Vector3> worldPath)
    {
        worldPath = new List<Vector3>();
        ResetSearch();
        if (!initialized || grid == null || !ValidSettings() || !Finite(currentWorldPosition))
            return false;

        var timer = System.Diagnostics.Stopwatch.StartNew();
        try
        {
            // Algorithm 4: rebuild terrain analysis and tree from the current
            // position on every request, including after obstacle changes.
            if (!ReadGrid()) return false;
            pathHeight = currentWorldPosition.y;
            Vector2 start = ToGrid(currentWorldPosition);
            Vector2 goal = ToGrid(goalWorld);
            if (!PointFree(start) || !PointFree(goal)) return false;

            nodes.Add(new Node(start, -1, 0f));
            if ((start - goal).sqrMagnitude < Epsilon * Epsilon)
            {
                worldPath.Add(ToWorld(goal));
                return true;
            }

            random = new System.Random(seed);
            AnalyseTerrain();
            bool initialPathFound = CanReachGoal(start, goal);

            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                Vector2 sample = Sample(initialPathFound);
                int nearest = Nearest(sample);
                Vector2 delta = sample - nodes[nearest].point;
                float distance = delta.magnitude;
                if (distance < Epsilon) continue;

                float step = AdaptiveStep(nodes[nearest].point);
                Vector2 point = nodes[nearest].point + delta * (Mathf.Min(step, distance) / distance);
                if (!SegmentFree(nodes[nearest].point, point)) continue;

                int added = AddAndRewire(point, nearest);
                if (!initialPathFound && CanReachGoal(nodes[added].point, goal))
                    initialPathFound = true;
                // Keep optimizing after the first solution (Algorithm 2).
            }

            int goalParent = -1;
            float goalCost = float.PositiveInfinity;
            for (int i = 0; i < nodes.Count; i++)
            {
                float candidate = nodes[i].cost + Vector2.Distance(nodes[i].point, goal);
                if (candidate < goalCost && CanReachGoal(nodes[i].point, goal))
                {
                    goalCost = candidate;
                    goalParent = i;
                }
            }
            if (goalParent < 0) return false;

            worldPath.Add(ToWorld(goal));
            for (int i = goalParent; i >= 0; i = nodes[i].parent)
            {
                Vector3 waypoint = ToWorld(nodes[i].point);
                if ((waypoint - worldPath[worldPath.Count - 1]).sqrMagnitude > Epsilon * Epsilon)
                    worldPath.Add(waypoint);
            }
            worldPath.Reverse();
            for (int i = 1; i < worldPath.Count; i++)
                PathLength += Vector3.Distance(worldPath[i - 1], worldPath[i]);
            return true;
        }
        finally
        {
            timer.Stop();
            PlanningMilliseconds = timer.Elapsed.TotalMilliseconds;
        }
    }

    private int AddAndRewire(Vector2 point, int nearest)
    {
        near.Clear();
        int parent = nearest;
        float bestCost = nodes[parent].cost + Vector2.Distance(nodes[parent].point, point);
        for (int i = 0; i < nodes.Count; i++)
        {
            float distance = Vector2.Distance(nodes[i].point, point);
            if (distance > neighbourRadius) continue;
            near.Add(i);
            float candidate = nodes[i].cost + distance;
            if (candidate + Epsilon < bestCost && SegmentFree(nodes[i].point, point))
            {
                parent = i;
                bestCost = candidate;
            }
        }

        int added = nodes.Count;
        nodes.Add(new Node(point, parent, bestCost));
        nodes[parent].children.Add(added);
        foreach (int i in near)
        {
            if (i == 0 || i == parent) continue;
            float candidate = bestCost + Vector2.Distance(point, nodes[i].point);
            if (candidate + Epsilon < nodes[i].cost && !IsAncestor(i, added) &&
                SegmentFree(point, nodes[i].point))
                Reparent(i, added, candidate);
        }
        return added;
    }

    private void AnalyseTerrain()
    {
        for (int x = 0; x < width; x++)
        for (int z = 0; z < depth; z++)
        {
            if (Walkable(x, z))
            {
                freeCells.Add(new Vector2(x, z));
                continue;
            }
            int discrepancies = 0;
            for (int dx = -1; dx <= 1; dx++)
            for (int dz = -1; dz <= 1; dz++)
                if ((dx != 0 || dz != 0) && Walkable(x + dx, z + dz)) discrepancies++;

            // Equation (2) prints C_th == 1, apparently a typo. We use
            // N_disp == 1 to include the concave corner shown in Figure 2c.
            // Out-of-map cells are occupied, not additional free neighbours.
            if (discrepancies > cornerThreshold || discrepancies == 1)
                corners.Add(new Vector2(x, z));
        }

        cornerTree = BuildCornerTree(new List<Vector2>(corners), 0);
        double total = 0d;
        double variance = 2d * pdfSigma * pdfSigma;
        foreach (Vector2 cell in freeCells)
        {
            // Section 3.2 does not specify the PDF kernel. Our discretization
            // sums Gaussian weights around every pairwise corner segment.
            // One corner gives a point kernel; no corners gives uniform sampling.
            double weight = 0.000001d;
            for (int i = 0; i < corners.Count; i++)
            {
                if (corners.Count == 1)
                    weight += Math.Exp(-(cell - corners[i]).sqrMagnitude / variance);
                for (int j = i + 1; j < corners.Count; j++)
                    weight += Math.Exp(-SegmentDistanceSquared(cell, corners[i], corners[j]) / variance);
            }
            total += weight;
            cumulativePdf.Add(total);
        }
    }

    private Vector2 Sample(bool initialPathFound)
    {
        int index;
        if (!initialPathFound || random.NextDouble() < uniformProbability || corners.Count == 0)
            index = random.Next(freeCells.Count);
        else
        {
            AdaptiveSampleCount++;
            double value = random.NextDouble() * cumulativePdf[cumulativePdf.Count - 1];
            int low = 0, high = cumulativePdf.Count - 1;
            while (low < high)
            {
                int middle = low + (high - low) / 2;
                if (cumulativePdf[middle] < value) low = middle + 1;
                else high = middle;
            }
            index = low;
        }
        // Uniform jitter within a chosen cell keeps the search continuous.
        return freeCells[index] + new Vector2((float)random.NextDouble() - 0.5f,
                                              (float)random.NextDouble() - 0.5f);
    }

    private float AdaptiveStep(Vector2 point)
    {
        int density = CountCorners(cornerTree, point, densityRadius * densityRadius);
        double exponent = Math.Max(-60d, Math.Min(60d, (density - densityMidpoint) / densitySlope));
        return minStep + (maxStep - minStep) / (1f + (float)Math.Exp(exponent));
    }

    private static CornerNode BuildCornerTree(List<Vector2> points, int level)
    {
        if (points.Count == 0) return null;
        int axis = level % 2;
        points.Sort((a, b) => (axis == 0 ? a.x : a.y).CompareTo(axis == 0 ? b.x : b.y));
        int mid = points.Count / 2;
        return new CornerNode
        {
            point = points[mid], axis = axis,
            left = BuildCornerTree(points.GetRange(0, mid), level + 1),
            right = BuildCornerTree(points.GetRange(mid + 1, points.Count - mid - 1), level + 1)
        };
    }

    private static int CountCorners(CornerNode node, Vector2 point, float radiusSquared)
    {
        if (node == null) return 0;
        Vector2 delta = point - node.point;
        float split = node.axis == 0 ? delta.x : delta.y;
        int count = delta.sqrMagnitude <= radiusSquared ? 1 : 0;
        count += CountCorners(split <= 0f ? node.left : node.right, point, radiusSquared);
        if (split * split <= radiusSquared)
            count += CountCorners(split <= 0f ? node.right : node.left, point, radiusSquared);
        return count;
    }

    private int Nearest(Vector2 point)
    {
        int nearest = 0;
        float distance = float.PositiveInfinity;
        for (int i = 0; i < nodes.Count; i++)
        {
            float candidate = (nodes[i].point - point).sqrMagnitude;
            if (candidate < distance) { distance = candidate; nearest = i; }
        }
        return nearest;
    }

    private bool IsAncestor(int candidate, int node)
    {
        for (int i = node; i >= 0; i = nodes[i].parent)
            if (i == candidate) return true;
        return false;
    }

    private void Reparent(int index, int parent, float cost)
    {
        Node node = nodes[index];
        nodes[node.parent].children.Remove(index);
        nodes[parent].children.Add(index);
        node.parent = parent;
        node.cost = cost;
        descendants.Clear();
        descendants.Push(index);
        while (descendants.Count > 0)
        {
            int current = descendants.Pop();
            foreach (int child in nodes[current].children)
            {
                nodes[child].cost = nodes[current].cost +
                    Vector2.Distance(nodes[current].point, nodes[child].point);
                descendants.Push(child);
            }
        }
    }

    private bool CanReachGoal(Vector2 from, Vector2 goal)
    {
        return (from - goal).sqrMagnitude <= goalRadius * goalRadius && SegmentFree(from, goal);
    }

    private bool PointFree(Vector2 point) => SegmentFree(point, point);

    // Segment-versus-cell tests with conservative square clearance. Closed
    // boxes reject corner touching and prevent diagonal corner cutting.
    private bool SegmentFree(Vector2 a, Vector2 b)
    {
        float padding = agentClearance + Epsilon;
        if (Mathf.Min(a.x, b.x) <= -0.5f + padding ||
            Mathf.Min(a.y, b.y) <= -0.5f + padding ||
            Mathf.Max(a.x, b.x) >= width - 0.5f - padding ||
            Mathf.Max(a.y, b.y) >= depth - 0.5f - padding) return false;

        int xMin = Mathf.Max(0, Mathf.CeilToInt(Mathf.Min(a.x, b.x) - 0.5f - padding));
        int xMax = Mathf.Min(width - 1, Mathf.FloorToInt(Mathf.Max(a.x, b.x) + 0.5f + padding));
        int zMin = Mathf.Max(0, Mathf.CeilToInt(Mathf.Min(a.y, b.y) - 0.5f - padding));
        int zMax = Mathf.Min(depth - 1, Mathf.FloorToInt(Mathf.Max(a.y, b.y) + 0.5f + padding));
        for (int x = xMin; x <= xMax; x++)
        for (int z = zMin; z <= zMax; z++)
        {
            if (Walkable(x, z)) continue;
            float tMin = 0f, tMax = 1f;
            float half = 0.5f + padding;
            if (ClipAxis(a.x, b.x - a.x, x - half, x + half, ref tMin, ref tMax) &&
                ClipAxis(a.y, b.y - a.y, z - half, z + half, ref tMin, ref tMax)) return false;
        }
        return true;
    }

    private static bool ClipAxis(float origin, float direction, float min, float max,
                                 ref float tMin, ref float tMax)
    {
        if (Mathf.Abs(direction) < 0.0000001f) return origin >= min && origin <= max;
        float a = (min - origin) / direction, b = (max - origin) / direction;
        tMin = Mathf.Max(tMin, Mathf.Min(a, b));
        tMax = Mathf.Min(tMax, Mathf.Max(a, b));
        return tMin <= tMax;
    }

    private static float SegmentDistanceSquared(Vector2 point, Vector2 a, Vector2 b)
    {
        Vector2 delta = b - a;
        float t = delta.sqrMagnitude > 0f
            ? Mathf.Clamp01(Vector2.Dot(point - a, delta) / delta.sqrMagnitude) : 0f;
        return (point - (a + delta * t)).sqrMagnitude;
    }

    private bool Walkable(int x, int z) => grid.IsWalkable(new Vector2Int(x, z));
    private Vector2 ToGrid(Vector3 point) => new Vector2((point.x - origin.x) / cellSize,
                                                        (point.z - origin.z) / cellSize);
    private Vector3 ToWorld(Vector2 point) => new Vector3(origin.x + point.x * cellSize,
                                                         pathHeight, origin.z + point.y * cellSize);

    private bool ReadGrid()
    {
        // IsInside checks the grid's Maze reference before accessing dimensions.
        if (!grid.IsInside(Vector2Int.zero)) return false;
        width = grid.Width;
        depth = grid.Depth;
        cellSize = grid.CellSize;
        origin = grid.CellToWorld(Vector2Int.zero);
        return width > 0 && depth > 0 && Finite(origin) &&
            !float.IsNaN(cellSize) && !float.IsInfinity(cellSize) && cellSize > 0f;
    }

    private void ResetSearch()
    {
        nodes.Clear();
        corners.Clear();
        freeCells.Clear();
        cumulativePdf.Clear();
        cornerTree = null;
        PathLength = 0f;
        PlanningMilliseconds = 0d;
        AdaptiveSampleCount = 0;
    }

    private bool ValidSettings()
    {
        float[] settings = { neighbourRadius, goalRadius, agentClearance, uniformProbability,
            pdfSigma, maxStep, minStep, densityRadius, densityMidpoint, densitySlope };
        foreach (float value in settings)
            if (float.IsNaN(value) || float.IsInfinity(value)) return false;
        return maxIterations > 0 && neighbourRadius > 0f && goalRadius > 0f &&
            agentClearance >= 0f && uniformProbability >= 0f && uniformProbability <= 1f &&
            cornerThreshold >= 1 && cornerThreshold <= 8 && pdfSigma > 0f &&
            minStep > 0f && maxStep >= minStep && densityRadius > 0f && densitySlope > 0f;
    }

    private static bool Finite(Vector3 point)
    {
        return !float.IsNaN(point.x) && !float.IsInfinity(point.x) &&
            !float.IsNaN(point.y) && !float.IsInfinity(point.y) &&
            !float.IsNaN(point.z) && !float.IsInfinity(point.z);
    }
}
