using System;
using System.Collections.Generic;
using UnityEngine;

// Common scenario and validation for RRT/DTA. This does not plan or coordinate
// agents: projection only enumerates the cells crossed by their native paths.
public static class GridBenchmarkPaths
{
    private static readonly int[,] ReferenceMaze =
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

    public static Vector2Int StartCell(int agent) => new Vector2Int(13 + agent, 13);
    public static Vector2Int GoalCell => new Vector2Int(21, 0);

    public static bool MatchesReferenceMaze(MazeNavigationGrid grid)
    {
        if (grid == null || !grid.IsInside(Vector2Int.zero) ||
            grid.Width != ReferenceMaze.GetLength(1) || grid.Depth != ReferenceMaze.GetLength(0) ||
            !(grid.CellSize > 0f) || float.IsInfinity(grid.CellSize)) return false;
        for (int x = 0; x < grid.Width; x++)
        for (int z = 0; z < grid.Depth; z++)
            if (grid.IsWalkable(new Vector2Int(x, z)) != (ReferenceMaze[z, x] == 0)) return false;
        return true;
    }

    public static bool TryProject(MazeNavigationGrid grid, IReadOnlyList<Vector3> worldPath,
        Vector2Int start, Vector2Int goal, out List<Vector2Int> path, out string reason)
    {
        path = new List<Vector2Int>();
        reason = "";
        if (worldPath == null || worldPath.Count == 0) { reason = "Ruta vacia"; return false; }
        foreach (Vector3 point in worldPath)
            if (!Finite(point)) { reason = "Waypoint no finito"; return false; }
        if (grid.WorldToCell(worldPath[0]) != start || grid.WorldToCell(worldPath[worldPath.Count - 1]) != goal)
        { reason = "Inicio o meta incorrectos"; return false; }
        if (!AppendCell(grid, path, start)) { reason = "Inicio bloqueado"; return false; }

        Vector3 origin = grid.CellToWorld(Vector2Int.zero);
        for (int i = 1; i < worldPath.Count; i++)
        {
            Vector3 a = worldPath[i - 1], b = worldPath[i];
            double x = ((double)a.x - origin.x) / grid.CellSize;
            double z = ((double)a.z - origin.z) / grid.CellSize;
            double dx = ((double)b.x - a.x) / grid.CellSize;
            double dz = ((double)b.z - a.z) / grid.CellSize;
            int sx = Math.Sign(dx), sz = Math.Sign(dz);
            Vector2Int current = path[path.Count - 1], end = grid.WorldToCell(b);
            int steps = 0;
            while (current != end)
            {
                if (steps++ > grid.Width + grid.Depth + 2)
                { reason = "Proyeccion fuera de limites"; return false; }
                double tx = sx == 0 ? double.PositiveInfinity : (current.x + sx * 0.5d - x) / dx;
                double tz = sz == 0 ? double.PositiveInfinity : (current.y + sz * 0.5d - z) / dz;
                if (Math.Min(tx, tz) > 1d + 0.000001d)
                { reason = "Segmento no coincide con su celda final"; return false; }
                if (Math.Abs(tx - tz) < 0.000000001d)
                {
                    // A diagonal corner becomes X then Z. Both side cells must
                    // be free; projection must never invent a detour past a wall.
                    if (!grid.IsWalkable(current + new Vector2Int(0, sz)) ||
                        !AppendCell(grid, path, current + new Vector2Int(sx, 0)))
                    { reason = "Segmento corta una esquina bloqueada"; return false; }
                    current += new Vector2Int(sx, sz);
                }
                else current += tx < tz ? new Vector2Int(sx, 0) : new Vector2Int(0, sz);
                if (!AppendCell(grid, path, current))
                { reason = "Segmento atraviesa una pared o tiene un salto invalido"; return false; }
            }
        }
        return true;
    }

    private static bool AppendCell(MazeNavigationGrid grid, List<Vector2Int> path, Vector2Int next)
    {
        if (!grid.IsWalkable(next)) return false;
        if (path.Count > 0)
        {
            Vector2Int previous = path[path.Count - 1];
            if (previous == next) return true;
            if (Mathf.Abs(previous.x - next.x) + Mathf.Abs(previous.y - next.y) != 1) return false;
        }
        path.Add(next);
        return true;
    }

    public static bool ValidateJointPaths(List<List<Vector2Int>> paths, int maxSteps, out string reason)
    {
        reason = "";
        int lastTime = 0;
        for (int i = 0; i < paths.Count; i++)
        {
            if (paths[i].Count - 1 > maxSteps)
            { reason = $"Agente {i}: supera el horizonte de {maxSteps} pasos"; return false; }
            lastTime = Math.Max(lastTime, paths[i].Count);
        }
        // Same disappearance model as CBS: the arrival vertex is still checked;
        // a path stops occupying cells on the following time step.
        for (int t = 0; t < lastTime; t++)
        for (int i = 0; i < paths.Count; i++)
        {
            if (t >= paths[i].Count) continue;
            for (int j = i + 1; j < paths.Count; j++)
            {
                if (t >= paths[j].Count) continue;
                if (paths[i][t] == paths[j][t])
                { reason = $"Conflicto de vertice: agentes {i} y {j}; t={t}"; return false; }
                if (t + 1 < paths[i].Count && t + 1 < paths[j].Count &&
                    paths[i][t] == paths[j][t + 1] && paths[i][t + 1] == paths[j][t] &&
                    paths[i][t] != paths[i][t + 1])
                { reason = $"Intercambio de arista: agentes {i} y {j}; t={t + 1}"; return false; }
            }
        }
        return true;
    }

    private static bool Finite(Vector3 point)
    {
        return !float.IsNaN(point.x) && !float.IsInfinity(point.x) &&
            !float.IsNaN(point.y) && !float.IsInfinity(point.y) &&
            !float.IsNaN(point.z) && !float.IsInfinity(point.z);
    }
}
