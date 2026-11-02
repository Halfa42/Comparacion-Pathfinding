using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Shared, cost-independent Dynamic Terrain Abstraction (Sturtevant et al., 2019).
/// Terrain types and regions belong to the map; costs belong to each agent.
/// </summary>
public class DTATerrainMap : MonoBehaviour
{
    [Serializable]
    private struct TerrainPatch
    {
        [Tooltip("Grid coordinates: X and Z (shown as Y in this rectangle).")]
        public RectInt area;
        [Min(0)] public int terrainType;
    }

    public sealed class Region
    {
        public int Id { get; internal set; }
        public int Sector { get; internal set; }
        public int TerrainType { get; internal set; }
        public Vector2Int Representative { get; internal set; }
        public int CellCount { get; internal set; }
        internal readonly SortedSet<int> adjacent = new SortedSet<int>();
        public IEnumerable<int> Neighbours => adjacent;
    }

    [SerializeField] private MazeNavigationGrid grid;
    [SerializeField, Min(1)] private int sectorSize = 8;
    [SerializeField, Min(0)] private int defaultTerrainType;
    [Tooltip("Applied in order at initialization; later patches override earlier ones.")]
    [SerializeField] private TerrainPatch[] terrainPatches = new TerrainPatch[0];
    [SerializeField] private bool drawAbstraction = true;

    public MazeNavigationGrid Grid => grid;
    public int RegionCount => regions.Count;
    public int LastRebuiltSectorCount { get; private set; }
    public int Revision { get; private set; }
    public IEnumerable<Region> Regions => regions.Values;
    public event Action Changed;

    private readonly Dictionary<int, Region> regions = new Dictionary<int, Region>();
    private readonly SortedSet<int> dirtySectors = new SortedSet<int>();
    private readonly Queue<Vector2Int> frontier = new Queue<Vector2Int>();
    private readonly List<Vector2Int> component = new List<Vector2Int>();
    private List<int>[] sectorRegions;
    private int[,] terrainTypes;
    private int[,] regionIds;
    private int width, depth, sectorsX, sectorsZ, builtSectorSize, nextRegionId;
    private bool built;
    private MazeNavigationGrid subscribedGrid;
    private MazeNavigationGrid terrainGrid;

    private void OnEnable()
    {
        UpdateSubscription();
        built = false;
    }

    private void OnDisable()
    {
        if (subscribedGrid != null) subscribedGrid.CellChanged -= OnCellChanged;
        subscribedGrid = null;
        // Events may be missed while disabled: rebuild on the next request.
        built = false;
    }

    private void UpdateSubscription()
    {
        if (subscribedGrid == grid) return;
        if (subscribedGrid != null) subscribedGrid.CellChanged -= OnCellChanged;
        subscribedGrid = grid;
        if (subscribedGrid != null) subscribedGrid.CellChanged += OnCellChanged;
        built = false;
    }

    /// <summary>Builds once, then repairs only changed sectors and their edges.</summary>
    public bool EnsureBuilt()
    {
        UpdateSubscription();
        if (grid == null || !grid.IsInside(Vector2Int.zero) || sectorSize < 1 ||
            defaultTerrainType < 0 || !PositiveFinite(grid.CellSize)) return false;
        if (terrainPatches != null)
            foreach (TerrainPatch patch in terrainPatches)
                if (patch.terrainType < 0 || patch.area.width < 0 || patch.area.height < 0) return false;

        if (terrainGrid != grid || terrainTypes == null || width != grid.Width || depth != grid.Depth)
        {
            terrainGrid = grid;
            width = grid.Width;
            depth = grid.Depth;
            terrainTypes = new int[width, depth];
            ApplyInitialTerrain();
            built = false;
        }

        if (!built || builtSectorSize != sectorSize)
            BuildAll();
        else if (dirtySectors.Count > 0)
            RepairDirtySectors();
        return true;
    }

    private void ApplyInitialTerrain()
    {
        for (int x = 0; x < width; x++)
        for (int z = 0; z < depth; z++)
            terrainTypes[x, z] = defaultTerrainType;
        if (terrainPatches == null) return;
        foreach (TerrainPatch patch in terrainPatches)
        {
            int xMin = Mathf.Max(0, patch.area.xMin), xMax = Mathf.Min(width, patch.area.xMax);
            int zMin = Mathf.Max(0, patch.area.yMin), zMax = Mathf.Min(depth, patch.area.yMax);
            for (int x = xMin; x < xMax; x++)
            for (int z = zMin; z < zMax; z++)
                terrainTypes[x, z] = patch.terrainType;
        }
    }

    public bool SetTerrainType(Vector2Int cell, int terrainType)
    {
        if (terrainType < 0 || !EnsureBuilt() || !grid.IsInside(cell)) return false;
        if (terrainTypes[cell.x, cell.y] == terrainType) return true;
        terrainTypes[cell.x, cell.y] = terrainType;
        dirtySectors.Add(SectorOf(cell));
        Changed?.Invoke();
        return true;
    }

    public int GetTerrainType(Vector2Int cell) => terrainTypes[cell.x, cell.y];

    public int GetRegionId(Vector2Int cell)
    {
        return built && grid.IsInside(cell) ? regionIds[cell.x, cell.y] : -1;
    }

    public bool TryGetRegion(int id, out Region region) => regions.TryGetValue(id, out region);

    // Call after rebuilding the whole generator or changing Inspector patches
    // during Play. Runtime SetTerrainType changes are intentionally reset here.
    [ContextMenu("Reload DTA Terrain From Inspector")]
    public void ReloadTerrain()
    {
        terrainTypes = null;
        built = false;
        Changed?.Invoke();
    }

    private void OnCellChanged(Vector2Int cell)
    {
        if (built && grid.IsInside(cell)) dirtySectors.Add(SectorOf(cell));
        Changed?.Invoke();
    }

    private void BuildAll()
    {
        builtSectorSize = sectorSize;
        sectorsX = (width - 1) / sectorSize + 1;
        sectorsZ = (depth - 1) / sectorSize + 1;
        sectorRegions = new List<int>[sectorsX * sectorsZ];
        regionIds = new int[width, depth];
        regions.Clear();
        dirtySectors.Clear();
        nextRegionId = 0;
        for (int x = 0; x < width; x++)
        for (int z = 0; z < depth; z++) regionIds[x, z] = -1;
        for (int sector = 0; sector < sectorRegions.Length; sector++)
        {
            sectorRegions[sector] = new List<int>();
            BuildSector(sector);
        }
        for (int sector = 0; sector < sectorRegions.Length; sector++) BuildEdges(sector);
        built = true;
        LastRebuiltSectorCount = sectorRegions.Length;
        Revision++;
    }

    private void RepairDirtySectors()
    {
        // Rebuild the changed sectors. Their eight neighbours also need edge
        // repair because the grid permits diagonal movement across sectors.
        var affectedEdges = new SortedSet<int>();
        foreach (int sector in dirtySectors)
        {
            int sx = sector % sectorsX, sz = sector / sectorsX;
            for (int dx = -1; dx <= 1; dx++)
            for (int dz = -1; dz <= 1; dz++)
                if (sx + dx >= 0 && sx + dx < sectorsX && sz + dz >= 0 && sz + dz < sectorsZ)
                    affectedEdges.Add((sz + dz) * sectorsX + sx + dx);
        }

        foreach (int sector in affectedEdges)
        foreach (int id in sectorRegions[sector])
        {
            Region region = regions[id];
            foreach (int neighbour in region.adjacent) regions[neighbour].adjacent.Remove(id);
            region.adjacent.Clear();
        }
        foreach (int sector in dirtySectors)
        {
            foreach (int id in sectorRegions[sector]) regions.Remove(id);
            sectorRegions[sector].Clear();
            RectInt bounds = SectorBounds(sector);
            for (int x = bounds.xMin; x < bounds.xMax; x++)
            for (int z = bounds.yMin; z < bounds.yMax; z++) regionIds[x, z] = -1;
            BuildSector(sector);
        }
        foreach (int sector in affectedEdges) BuildEdges(sector);
        LastRebuiltSectorCount = dirtySectors.Count;
        dirtySectors.Clear();
        Revision++;
    }

    private void BuildSector(int sector)
    {
        RectInt bounds = SectorBounds(sector);
        for (int x = bounds.xMin; x < bounds.xMax; x++)
        for (int z = bounds.yMin; z < bounds.yMax; z++)
        {
            Vector2Int seed = new Vector2Int(x, z);
            if (!grid.IsWalkable(seed) || regionIds[x, z] >= 0) continue;
            var region = new Region { Id = nextRegionId++, Sector = sector, TerrainType = terrainTypes[x, z] };
            frontier.Clear();
            component.Clear();
            frontier.Enqueue(seed);
            regionIds[x, z] = region.Id;
            Vector2 sum = Vector2.zero;
            while (frontier.Count > 0)
            {
                Vector2Int cell = frontier.Dequeue();
                component.Add(cell);
                sum += (Vector2)cell;
                foreach (Vector2Int next in grid.GetNeighbours(cell))
                {
                    if (!bounds.Contains(next) || regionIds[next.x, next.y] >= 0 ||
                        terrainTypes[next.x, next.y] != region.TerrainType) continue;
                    regionIds[next.x, next.y] = region.Id;
                    frontier.Enqueue(next);
                }
            }

            // Section 4: representative is an actual member nearest the mean,
            // not a centroid that could fall inside a wall or a different region.
            Vector2 mean = sum / component.Count;
            float bestDistance = float.PositiveInfinity;
            foreach (Vector2Int cell in component)
            {
                float distance = ((Vector2)cell - mean).sqrMagnitude;
                if (distance < bestDistance) { bestDistance = distance; region.Representative = cell; }
            }
            region.CellCount = component.Count;
            regions.Add(region.Id, region);
            sectorRegions[sector].Add(region.Id);
        }
    }

    private void BuildEdges(int sector)
    {
        RectInt bounds = SectorBounds(sector);
        for (int x = bounds.xMin; x < bounds.xMax; x++)
        for (int z = bounds.yMin; z < bounds.yMax; z++)
        {
            Vector2Int cell = new Vector2Int(x, z);
            int from = regionIds[x, z];
            if (from < 0) continue;
            foreach (Vector2Int next in grid.GetNeighbours(cell))
            {
                int to = regionIds[next.x, next.y];
                if (to < 0 || from == to) continue;
                regions[from].adjacent.Add(to);
                regions[to].adjacent.Add(from);
            }
        }
    }

    private int SectorOf(Vector2Int cell) => (cell.y / builtSectorSize) * sectorsX + cell.x / builtSectorSize;

    private RectInt SectorBounds(int sector)
    {
        int x = (sector % sectorsX) * builtSectorSize, z = (sector / sectorsX) * builtSectorSize;
        return new RectInt(x, z, Mathf.Min(builtSectorSize, width - x), Mathf.Min(builtSectorSize, depth - z));
    }

    private static bool PositiveFinite(float value) => value > 0f && !float.IsInfinity(value);

    private void OnDrawGizmosSelected()
    {
        if (!drawAbstraction || !built || grid == null || dirtySectors.Count > 0) return;
        foreach (Region region in regions.Values)
        {
            Vector3 point = grid.CellToWorld(region.Representative) + Vector3.up * 0.15f;
            Gizmos.color = Color.HSVToRGB(Mathf.Repeat(region.TerrainType * 0.17f, 1f), 0.8f, 1f);
            Gizmos.DrawSphere(point, grid.CellSize * 0.12f);
            Gizmos.color = Color.green;
            foreach (int neighbour in region.adjacent)
                if (neighbour > region.Id)
                    Gizmos.DrawLine(point, grid.CellToWorld(regions[neighbour].Representative) + Vector3.up * 0.15f);
        }
    }
}
