using System;
using System.Collections.Generic;
using UnityEngine;

public class MazeNavigationGrid : MonoBehaviour
{
    [SerializeField] private MazeGridGenerator maze;
    [SerializeField, Min(0.0001f)]
    private float diagonalCornerTolerance = 0f;

    public int Width => maze.Width;
    public int Depth => maze.Height;
    public float CellSize => maze.CellSize;

    public event Action<Vector2Int> CellChanged;

    /// <summary>
    /// Increases every time a cell changes. Planners that cache collision checks (MARS)
    /// compare against it to know when a cached result is stale.
    /// </summary>
    public int Revision { get; private set; }

    private void Awake()
    {
        if (maze == null) maze = FindFirstObjectByType<MazeGridGenerator>();
        if(maze == null)
            Debug.LogError("MazeNavigationGrid needs a MazeGridGenerator.");
    }

    public bool IsInside(Vector2Int cell)
    {
        return maze != null && 
                cell.x >= 0 && cell.x < Width &&
                cell.y >= 0 && cell.y < Depth;
    }

    public bool IsWalkable(Vector2Int cell)
    {
        return IsInside(cell) && maze.IsWalkable(cell.x, cell.y);
    }

    public Vector3 CellToWorld(Vector2Int cell)
    {
        return maze.GridToWorldPosition(cell.x, cell.y);
    }

    public Vector2Int WorldToCell(Vector3 position)
    {
        return maze.WorldToGridPositionUnclamped(position);
    }

    public float Cost(Vector2Int a, Vector2Int b)
    {
        bool diagonal = a.x != b.x && a.y != b.y;

        return CellSize * (diagonal ? Mathf.Sqrt(2f) : 1f);
    }

    public IEnumerable<Vector2Int> GetNeighbours(Vector2Int cell)
    {
        for(int dx = -1; dx <= 1; dx++)
        {
            for(int dy = -1; dy <= 1; dy++)
            {
                if(dx == 0 && dy == 0) continue;

                Vector2Int next = new Vector2Int(cell.x + dx, cell.y + dy);

                if(!IsWalkable(next)) continue;

                if(dx != 0 && dy != 0)
                {
                    Vector2Int sideA = new Vector2Int(cell.x + dx, cell.y);
                    Vector2Int sideB = new Vector2Int(cell.x, cell.y + dy);

                    if(!IsWalkable(sideA) || !IsWalkable(sideB)) continue;
                }

                yield return next;
            }
        }
    }

    public bool SetWalkable(Vector2Int cell, bool walkable)
    {
        if(!IsInside(cell)) return false;

        if(maze.IsWalkable(cell.x, cell.y) == walkable) return true;

        maze.SetWalkable(cell.x, cell.y, walkable);
        Revision++;
        CellChanged?.Invoke(cell);
        return true;
    }

    public Vector3 GetRandomWorldPosition(System.Random random)
    {
        // The grid positions are at integer multiples of CellSize.
        // The half-cell margins define the complete grid area.
        float x = (float)(random.NextDouble() * Width - 0.5f);
        float z = (float)(random.NextDouble() * Depth - 0.5f);

        Vector3 origin = maze.transform.position;

        return origin + new Vector3(
            x * CellSize,
            0f,
            z * CellSize
        );
    }

    public bool IsValidWorldPosition(Vector3 position)
    {
        return IsWalkable(WorldToCell(position));
    }

    // Exact XZ segment test against blocked cells.
    // Sampling can miss a short crossing near a corner or between two samples.
    public bool IsSegmentWalkable(Vector3 from, Vector3 to)
    {
        if (!IsInside(Vector2Int.zero) || !(CellSize > 0f) || float.IsInfinity(CellSize) || !Finite(from) || !Finite(to) ||
            !IsWalkable(WorldToCell(from)) || !IsWalkable(WorldToCell(to))) return false;
        Vector3 origin = CellToWorld(Vector2Int.zero);
        Vector2 a = new Vector2((from.x - origin.x) / CellSize, (from.z - origin.z) / CellSize);
        Vector2 b = new Vector2((to.x - origin.x) / CellSize, (to.z - origin.z) / CellSize);
        if (Mathf.Min(a.x, b.x) < -0.5f || Mathf.Min(a.y, b.y) < -0.5f ||
            Mathf.Max(a.x, b.x) > Width - 0.5f || Mathf.Max(a.y, b.y) > Depth - 0.5f)
            return false;
        int xMin = Mathf.Max(0, Mathf.CeilToInt(Mathf.Min(a.x, b.x) - 0.5f));
        int xMax = Mathf.Min(Width - 1, Mathf.FloorToInt(Mathf.Max(a.x, b.x) + 0.5f));
        int zMin = Mathf.Max(0, Mathf.CeilToInt(Mathf.Min(a.y, b.y) - 0.5f));
        int zMax = Mathf.Min(Depth - 1, Mathf.FloorToInt(Mathf.Max(a.y, b.y) + 0.5f));
        for (int x = xMin; x <= xMax; x++)
        for (int z = zMin; z <= zMax; z++)
        {
            if (IsWalkable(new Vector2Int(x, z))) continue;
            float entry = 0f, exit = 1f, half = 0.5f;
            if (ClipSegmentAxis(a.x, b.x - a.x, x - half, x + half, ref entry, ref exit) &&
                ClipSegmentAxis(a.y, b.y - a.y, z - half, z + half, ref entry, ref exit)) return false;
        }
        return true;
    }

    private static bool ClipSegmentAxis(float origin, float direction, float min, float max,
        ref float entry, ref float exit)
    {
        if (Mathf.Abs(direction) < 0.0000001f) return origin >= min && origin <= max;
        float a = (min - origin) / direction, b = (max - origin) / direction;
        entry = Mathf.Max(entry, Mathf.Min(a, b));
        exit = Mathf.Min(exit, Mathf.Max(a, b));
        return entry <= exit;
    }

    private static bool Finite(Vector3 value) =>
        !float.IsNaN(value.x) && !float.IsInfinity(value.x) &&
        !float.IsNaN(value.y) && !float.IsInfinity(value.y) &&
        !float.IsNaN(value.z) && !float.IsInfinity(value.z);

}
