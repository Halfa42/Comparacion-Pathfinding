using UnityEngine;

/// <summary>
/// Collision checks for the continuous planners (RRT-Connect, MARS).
///
/// A segment is valid if it stays at least Clearance (in cells) away from every wall cell. Each wall
/// cell is a square of side 1 + 2*Clearance, and the segment is tested EXACTLY against it (slab
/// clipping, same method as RRTPathfinder). The previous version only tested points every 0.2 cells,
/// so a segment could cut the corner of a wall between two test points; an agent walking that piece
/// was inside a wall cell for a moment and could not replan from there.
/// Corner cutting between diagonal cells is rejected automatically (the enlarged squares overlap).
/// </summary>
public class MarsCollisionChecker
{
    private readonly MazeNavigationGrid grid;

    /// <summary>Safety margin around walls, in cells.</summary>
    public float Clearance { get; }

    public MarsCollisionChecker(MazeNavigationGrid grid, float clearance = 0.05f)
    {
        this.grid = grid;
        Clearance = Mathf.Max(0f, clearance);
    }

    /// <summary>The position is in a free cell.</summary>
    public bool IsPositionValid(Vector3 position)
    {
        return grid.IsValidWorldPosition(position);
    }

    /// <summary>The position is in a free cell AND at least Clearance away from every wall cell.</summary>
    public bool IsPositionClear(Vector3 position)
    {
        if (!IsPositionValid(position)) return false;

        Vector2 p = ToGrid(position);
        float half = 0.5f + Clearance;
        int x0 = Mathf.FloorToInt(p.x - half) , x1 = Mathf.CeilToInt(p.x + half);
        int z0 = Mathf.FloorToInt(p.y - half) , z1 = Mathf.CeilToInt(p.y + half);

        for (int x = x0; x <= x1; x++)
        for (int z = z0; z <= z1; z++)
        {
            if (grid.IsWalkable(new Vector2Int(x, z))) continue;
            if (Mathf.Abs(p.x - x) <= half && Mathf.Abs(p.y - z) <= half) return false;
        }
        return true;
    }

    public bool IsConnectionValid(Vector3 from, Vector3 to)
    {
        if (CalculateDistance(from, to) < 0.0001f)
            return IsPositionValid(from);

        if (!IsPositionValid(from) || !IsPositionValid(to))
            return false;

        Vector2 a = ToGrid(from), b = ToGrid(to);
        float half = 0.5f + Clearance;

        int xMin = Mathf.CeilToInt(Mathf.Min(a.x, b.x) - half);
        int xMax = Mathf.FloorToInt(Mathf.Max(a.x, b.x) + half);
        int zMin = Mathf.CeilToInt(Mathf.Min(a.y, b.y) - half);
        int zMax = Mathf.FloorToInt(Mathf.Max(a.y, b.y) + half);

        for (int x = xMin; x <= xMax; x++)
        for (int z = zMin; z <= zMax; z++)
        {
            if (grid.IsWalkable(new Vector2Int(x, z))) continue; // outside the maze counts as a wall

            float tMin = 0f, tMax = 1f;
            if (ClipAxis(a.x, b.x - a.x, x - half, x + half, ref tMin, ref tMax) &&
                ClipAxis(a.y, b.y - a.y, z - half, z + half, ref tMin, ref tMax))
                return false;
        }

        return true;
    }

    public float CalculateDistance(Vector3 a, Vector3 b)
    {
        a.y = 0f;
        b.y = 0f;

        return Vector3.Distance(a, b);
    }

    // World position -> continuous grid coordinates (cell (x, z) is centred on integer (x, z)).
    private Vector2 ToGrid(Vector3 world)
    {
        Vector3 origin = grid.CellToWorld(Vector2Int.zero);
        float size = grid.CellSize;
        return new Vector2((world.x - origin.x) / size, (world.z - origin.z) / size);
    }

    private static bool ClipAxis(float origin, float direction, float min, float max, ref float tMin, ref float tMax)
    {
        if (Mathf.Abs(direction) < 0.0000001f) return origin >= min && origin <= max;
        float t0 = (min - origin) / direction, t1 = (max - origin) / direction;
        tMin = Mathf.Max(tMin, Mathf.Min(t0, t1));
        tMax = Mathf.Min(tMax, Mathf.Max(t0, t1));
        return tMin <= tMax;
    }
}
