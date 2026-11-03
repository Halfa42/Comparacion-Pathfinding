using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// DTA with A* for abstract search and low-level refinement.
/// Sturtevant et al., AIIDE 2019, doi:10.1609/aiide.v15i1.5228, sections 3-4.2.
/// The abstract route is approximate; no global optimality guarantee is claimed.
/// </summary>
public class DTAPathfinder : MonoBehaviour
{
    [Serializable]
    private struct TerrainCost
    {
        [Min(0)] public int terrainType;
        [Min(0.0001f)] public float cost;
    }

    [SerializeField] private DTATerrainMap terrainMap;
    [Header("Agent terrain costs")]
    [SerializeField, Min(0.0001f)] private float defaultTerrainCost = 1f;
    [Tooltip("Overrides by terrain ID. IDs without an override use Default Terrain Cost.")]
    [SerializeField] private TerrainCost[] terrainCosts =
    {
        new TerrainCost { terrainType = 1, cost = 1f },
        new TerrainCost { terrainType = 2, cost = 3f },
        new TerrainCost { terrainType = 3, cost = 5f }
    };

    public DTATerrainMap TerrainMap => terrainMap;
    public MazeNavigationGrid Grid => terrainMap != null ? terrainMap.Grid : null;
    public int ExpandedAbstractNodes { get; private set; }
    public int ExpandedRefinementNodes { get; private set; }
    public float PathLength { get; private set; }
    public double PathCost { get; private set; }
    public double PlanningMilliseconds { get; private set; }
    public long PeakMemoryBytes { get; private set; }
    public bool CollectBenchmarkMetrics { get; set; }
    public IReadOnlyList<Vector3> AbstractWorldPath => abstractWorldPath;
    public event Action CostsChanged;

    private readonly Dictionary<int, double> costByType = new Dictionary<int, double>();
    private readonly List<Vector3> abstractWorldPath = new List<Vector3>();
    private Vector3 goalWorld;
    private double minimumTerrainCost;
    private bool initialized;

    public bool Initialize(Vector3 startWorld, Vector3 targetWorld)
    {
        initialized = false;
        ResetMetrics();
        if (!Finite(startWorld) || !Finite(targetWorld) || !ReadyGrid() || !BuildCostProfile())
        {
            Debug.LogWarning("DTA: assign a Terrain Map and Grid, and use finite positive terrain costs.", this);
            return false;
        }
        if (!Grid.IsWalkable(Grid.WorldToCell(startWorld)) || !Grid.IsWalkable(Grid.WorldToCell(targetWorld)))
        {
            Debug.LogWarning("DTA: start or goal is blocked or outside the grid.", this);
            return false;
        }
        goalWorld = targetWorld;
        initialized = true;
        return true;
    }

    public bool TryGetPath(Vector3 currentWorldPosition, out List<Vector3> worldPath)
    {
        worldPath = new List<Vector3>();
        ResetMetrics();
        if (!initialized || !Finite(currentWorldPosition) || !ReadyGrid()) return false;

        var timer = System.Diagnostics.Stopwatch.StartNew();
        try
        {
            // Initial abstraction construction and pending local repairs are
            // included in this measurement. Other agents may reuse this map.
            if (!BuildCostProfile() || !terrainMap.EnsureBuilt()) return false;
            SampleBenchmarkMemory();
            Vector2Int start = Grid.WorldToCell(currentWorldPosition);
            Vector2Int goal = Grid.WorldToCell(goalWorld);
            if (!Grid.IsWalkable(start) || !Grid.IsWalkable(goal)) return false;
            int startRegion = terrainMap.GetRegionId(start), goalRegion = terrainMap.GetRegionId(goal);
            if (startRegion < 0 || goalRegion < 0) return false;

            List<int> abstractPath;
            int expansions;
            bool found = Search(startRegion, goalRegion, AbstractNeighbours,
                AbstractCost, AbstractHeuristic, out abstractPath, out expansions);
            ExpandedAbstractNodes = expansions;
            if (!found) return false;

            foreach (int id in abstractPath)
            {
                DTATerrainMap.Region region;
                terrainMap.TryGetRegion(id, out region);
                Vector3 point = Grid.CellToWorld(region.Representative);
                point.y = currentWorldPosition.y;
                abstractWorldPath.Add(point);
            }

            List<Vector2Int> cells;
            if (!Refine(abstractPath, start, goal, out cells)) return false;
            for (int i = 0; i < cells.Count; i++)
            {
                Vector3 point = Grid.CellToWorld(cells[i]);
                point.y = currentWorldPosition.y;
                worldPath.Add(point);
                if (i == 0) continue;
                PathLength += Grid.Cost(cells[i - 1], cells[i]);
                PathCost += CellCost(cells[i - 1], cells[i]);
            }
            return true;
        }
        finally
        {
            SampleBenchmarkMemory();
            timer.Stop();
            PlanningMilliseconds = timer.Elapsed.TotalMilliseconds;
        }
    }

    private IEnumerable<int> AbstractNeighbours(int id)
    {
        DTATerrainMap.Region region;
        if (!terrainMap.TryGetRegion(id, out region)) yield break;
        foreach (int neighbour in region.Neighbours) yield return neighbour;
    }

    private double AbstractCost(int a, int b)
    {
        DTATerrainMap.Region from, to;
        terrainMap.TryGetRegion(a, out from);
        terrainMap.TryGetRegion(b, out to);
        // Section 4: representative distance split equally between terrains.
        // Costs are evaluated per request, never cached in the shared graph.
        return Euclidean(from.Representative, to.Representative) *
            (CostForType(from.TerrainType) + CostForType(to.TerrainType)) * 0.5d;
    }

    private double AbstractHeuristic(int a, int b)
    {
        DTATerrainMap.Region from, to;
        terrainMap.TryGetRegion(a, out from);
        terrainMap.TryGetRegion(b, out to);
        // Ignore the current regions' weights. Scaling by the profile's minimum
        // preserves admissibility even if an agent assigns a cost below one.
        return Euclidean(from.Representative, to.Representative) * minimumTerrainCost;
    }

    private bool Refine(List<int> abstractPath, Vector2Int start, Vector2Int goal,
                        out List<Vector2Int> path)
    {
        path = new List<Vector2Int>();
        Vector2Int current = start;
        int segmentCount = Mathf.Max(1, abstractPath.Count - 1);
        for (int i = 0; i < segmentCount; i++)
        {
            int fromRegion = abstractPath[i];
            int toRegion = abstractPath[Mathf.Min(i + 1, abstractPath.Count - 1)];
            Vector2Int target = goal;
            if (i < segmentCount - 1)
            {
                DTATerrainMap.Region next;
                terrainMap.TryGetRegion(toRegion, out next);
                target = next.Representative;
            }

            List<Vector2Int> segment;
            int expansions;
            // Section 4.1: successive representatives are subgoals, except
            // the first and last, which are replaced by the actual start/goal.
            // Refinement stays in the two adjacent regions of this segment;
            // this is the corridor policy chosen for this implementation.
            bool found = Search(current, target,
                cell => RefinementNeighbours(cell, fromRegion, toRegion),
                CellCost, CellHeuristic, out segment, out expansions);
            ExpandedRefinementNodes += expansions;
            if (!found)
            {
                path.Clear();
                return false;
            }
            for (int j = path.Count == 0 ? 0 : 1; j < segment.Count; j++) path.Add(segment[j]);
            current = target;
        }
        return true;
    }

    private IEnumerable<Vector2Int> RefinementNeighbours(Vector2Int cell, int fromRegion, int toRegion)
    {
        foreach (Vector2Int next in Grid.GetNeighbours(cell))
        {
            int region = terrainMap.GetRegionId(next);
            if (region == fromRegion || region == toRegion) yield return next;
        }
    }

    private double CellCost(Vector2Int a, Vector2Int b)
    {
        // Section 3: c(a,b) = d(a,b) * (tc(a) + tc(b)) / 2.
        return Grid.Cost(a, b) * (CostForType(terrainMap.GetTerrainType(a)) +
                                  CostForType(terrainMap.GetTerrainType(b))) * 0.5d;
    }

    private double CellHeuristic(Vector2Int a, Vector2Int b)
    {
        int dx = Mathf.Abs(a.x - b.x), dz = Mathf.Abs(a.y - b.y);
        int diagonal = Mathf.Min(dx, dz), straight = Mathf.Max(dx, dz) - diagonal;
        // Match the grid's float diagonal cost exactly, rather than using a
        // slightly larger double sqrt(2) in an otherwise admissible heuristic.
        double diagonalCost = Grid.CellSize * Mathf.Sqrt(2f);
        return (straight * (double)Grid.CellSize + diagonal * diagonalCost) * minimumTerrainCost;
    }

    private double Euclidean(Vector2Int a, Vector2Int b)
    {
        double dx = (double)a.x - b.x, dz = (double)a.y - b.y;
        return Math.Sqrt(dx * dx + dz * dz) * Grid.CellSize;
    }

    private bool BuildCostProfile()
    {
        costByType.Clear();
        if (!PositiveFinite(defaultTerrainCost)) return false;
        minimumTerrainCost = defaultTerrainCost;
        if (terrainCosts == null) return true;
        foreach (TerrainCost entry in terrainCosts)
        {
            if (entry.terrainType < 0 || !PositiveFinite(entry.cost) || costByType.ContainsKey(entry.terrainType))
                return false;
            costByType.Add(entry.terrainType, entry.cost);
            minimumTerrainCost = Math.Min(minimumTerrainCost, entry.cost);
        }
        return true;
    }

    private double CostForType(int type)
    {
        double value;
        return costByType.TryGetValue(type, out value) ? value : defaultTerrainCost;
    }

    public bool SetTerrainCost(int terrainType, float cost)
    {
        if (terrainType < 0 || !PositiveFinite(cost)) return false;
        if (terrainCosts == null) terrainCosts = new TerrainCost[0];
        for (int i = 0; i < terrainCosts.Length; i++)
        {
            if (terrainCosts[i].terrainType != terrainType) continue;
            if (terrainCosts[i].cost == cost) return true;
            terrainCosts[i] = new TerrainCost { terrainType = terrainType, cost = cost };
            CostsChanged?.Invoke();
            return true;
        }
        Array.Resize(ref terrainCosts, terrainCosts.Length + 1);
        terrainCosts[terrainCosts.Length - 1] = new TerrainCost { terrainType = terrainType, cost = cost };
        CostsChanged?.Invoke();
        return true;
    }

    public bool SetDefaultTerrainCost(float cost)
    {
        if (!PositiveFinite(cost)) return false;
        if (defaultTerrainCost == cost) return true;
        defaultTerrainCost = cost;
        CostsChanged?.Invoke();
        return true;
    }

    private struct OpenEntry<T>
    {
        public T node;
        public double g, f;
        public long order;
    }

    private sealed class EntryComparer<T> : IComparer<OpenEntry<T>>
    {
        public int Compare(OpenEntry<T> a, OpenEntry<T> b)
        {
            int result = a.f.CompareTo(b.f);
            if (result != 0) return result;
            result = b.g.CompareTo(a.g);
            return result != 0 ? result : a.order.CompareTo(b.order);
        }
    }

    // The same A* implementation searches the abstract graph and refines each
    // segment. Stale entries are discarded; improved nodes can be reopened.
    private bool Search<T>(T start, T goal, Func<T, IEnumerable<T>> neighbours,
                                  Func<T, T, double> cost, Func<T, T, double> heuristic,
                                  out List<T> path, out int expansions)
    {
        path = new List<T>();
        expansions = 0;
        var open = new SortedSet<OpenEntry<T>>(new EntryComparer<T>());
        var best = new Dictionary<T, double>();
        var parent = new Dictionary<T, T>();
        var equality = EqualityComparer<T>.Default;
        long order = 0;
        best[start] = 0d;
        open.Add(new OpenEntry<T> { node = start, g = 0d, f = heuristic(start, goal), order = order++ });
        while (open.Count > 0)
        {
            OpenEntry<T> current = open.Min;
            open.Remove(current);
            if (current.g != best[current.node]) continue;
            expansions++;
            if (expansions % 128 == 0) SampleBenchmarkMemory();
            if (equality.Equals(current.node, goal))
            {
                T step = goal;
                path.Add(step);
                while (!equality.Equals(step, start))
                {
                    step = parent[step];
                    path.Add(step);
                }
                path.Reverse();
                SampleBenchmarkMemory();
                return true;
            }
            foreach (T next in neighbours(current.node))
            {
                double candidate = current.g + cost(current.node, next), known;
                if (best.TryGetValue(next, out known) && candidate >= known) continue;
                best[next] = candidate;
                parent[next] = current.node;
                open.Add(new OpenEntry<T>
                {
                    node = next, g = candidate, f = candidate + heuristic(next, goal), order = order++
                });
            }
        }
        return false;
    }

    private bool ReadyGrid()
    {
        return Grid != null && Grid.IsInside(Vector2Int.zero) && PositiveFinite(Grid.CellSize) &&
            Finite(Grid.CellToWorld(Vector2Int.zero));
    }

    private void ResetMetrics()
    {
        ExpandedAbstractNodes = 0;
        ExpandedRefinementNodes = 0;
        PathLength = 0f;
        PathCost = 0d;
        PlanningMilliseconds = 0d;
        PeakMemoryBytes = 0;
        abstractWorldPath.Clear();
    }

    public void ClearSearch()
    {
        initialized = false;
        ResetMetrics();
    }

    private void SampleBenchmarkMemory()
    {
        if (CollectBenchmarkMetrics)
            PeakMemoryBytes = Math.Max(PeakMemoryBytes, GC.GetTotalMemory(false));
    }

    private static bool PositiveFinite(float value) => value > 0f && !float.IsInfinity(value);

    private static bool Finite(Vector3 point)
    {
        return !float.IsNaN(point.x) && !float.IsInfinity(point.x) &&
            !float.IsNaN(point.y) && !float.IsInfinity(point.y) &&
            !float.IsNaN(point.z) && !float.IsInfinity(point.z);
    }
}
