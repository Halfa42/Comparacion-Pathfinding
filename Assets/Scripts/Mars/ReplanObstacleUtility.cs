using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Picks where a "suddenly appearing" obstacle goes: a cell on the robot's remaining path,
/// between minAhead and maxAhead (world units, measured along the path), that is not the
/// robot's cell or the goal, and that still leaves the goal reachable. In a maze many corridors
/// are dead ends or single passages; blocking those would make the goal unreachable for every
/// algorithm, which tests nothing, so those cells are rejected.
/// </summary>
public static class ReplanObstacleUtility
{
    public static bool TryPickCellAhead(MazeNavigationGrid grid, Vector3 robot, IList<Vector3> waypoints, int fromIndex,
                                        float minAhead, float maxAhead, Vector2Int goalCell, System.Random random,
                                        out Vector2Int cell)
    {
        cell = new Vector2Int(-1, -1);
        if (waypoints == null) return false;

        Vector2Int robotCell = grid.WorldToCell(robot);
        var candidates = new List<Vector2Int>();
        float spacing = grid.CellSize * 0.25f;
        float travelled = 0f;
        Vector3 previous = robot;

        for (int i = Mathf.Max(0, fromIndex); i < waypoints.Count && travelled <= maxAhead; i++)
        {
            Vector3 next = waypoints[i];
            float length = Flat(previous, next);
            for (float d = 0f; d < length; d += spacing)
            {
                float at = travelled + d;
                if (at > maxAhead) break;
                if (at < minAhead) continue;

                Vector2Int c = grid.WorldToCell(Vector3.Lerp(previous, next, d / length));
                if (c == robotCell || c == goalCell || !grid.IsWalkable(c) || candidates.Contains(c)) continue;
                candidates.Add(c);
            }
            travelled += length;
            previous = next;
        }

        // Shuffle so the obstacle is not always the first cell.
        for (int i = candidates.Count - 1; i > 0; i--)
        {
            int j = random.Next(i + 1);
            Vector2Int tmp = candidates[i];
            candidates[i] = candidates[j];
            candidates[j] = tmp;
        }

        foreach (Vector2Int c in candidates)
        {
            if (GoalReachable(grid, robotCell, goalCell, c))
            {
                cell = c;
                return true;
            }
        }
        return false;
    }

    /// <summary>BFS with the grid's own movement rule (8 neighbours, no corner cutting), treating 'blocked' as a wall.</summary>
    public static bool GoalReachable(MazeNavigationGrid grid, Vector2Int from, Vector2Int goal, Vector2Int blocked)
    {
        if (from == blocked || goal == blocked || !grid.IsWalkable(from) || !grid.IsWalkable(goal)) return false;

        var visited = new HashSet<Vector2Int> { from };
        var queue = new Queue<Vector2Int>();
        queue.Enqueue(from);

        while (queue.Count > 0)
        {
            Vector2Int c = queue.Dequeue();
            if (c == goal) return true;

            for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0) continue;
                var n = new Vector2Int(c.x + dx, c.y + dy);
                if (!Free(grid, n, blocked) || visited.Contains(n)) continue;
                if (dx != 0 && dy != 0 &&
                    (!Free(grid, new Vector2Int(c.x + dx, c.y), blocked) || !Free(grid, new Vector2Int(c.x, c.y + dy), blocked)))
                    continue;
                visited.Add(n);
                queue.Enqueue(n);
            }
        }
        return false;
    }

    private static bool Free(MazeNavigationGrid grid, Vector2Int c, Vector2Int blocked) => c != blocked && grid.IsWalkable(c);

    private static float Flat(Vector3 a, Vector3 b)
    {
        float dx = a.x - b.x, dz = a.z - b.z;
        return Mathf.Sqrt(dx * dx + dz * dz);
    }
}
