using System.Collections.Generic;
using UnityEngine;
using System;

public class MarsPathOptimizer
{
    private readonly MarsCollisionChecker collisionChecker;
    private readonly System.Random random;

    public MarsPathOptimizer(
        MazeNavigationGrid grid,
        int seed)
    {
        collisionChecker = new MarsCollisionChecker(grid);
        random = new System.Random(seed);
    }

    public List<Vector3> Optimize(
        List<Vector3> originalPath,
        int iterations = 200)
    {
        if (originalPath == null || originalPath.Count < 3)
            return originalPath;

        // Work on a copy. Never modify the original path.
        List<Vector3> optimizedPath =
            new List<Vector3>(originalPath);

        for (int i = 0; i < iterations; i++)
        {
            if (optimizedPath.Count < 3)
                break;

            // Select two different, non-adjacent indices.
            int first = random.Next(0, optimizedPath.Count - 2);

            int second = random.Next(
                first + 2,
                optimizedPath.Count
            );

            Vector3 from = optimizedPath[first];
            Vector3 to = optimizedPath[second];

            // Can we bypass all intermediate points?
            if (collisionChecker.IsConnectionValid(from, to))
            {
                int pointsToRemove = second - first - 1;

                optimizedPath.RemoveRange(
                    first + 1,
                    pointsToRemove
                );
            }
        }

        return optimizedPath;
    }

    /// <summary>
    /// Deterministic "wrapping" (the paper optimizes its initial path with Informed RRT*
    /// followed by a wrapping step, Kim &amp; Song): the path is pulled tight around the
    /// walls like a string.
    ///  1) string pulling: from each node jump to the farthest node it can see;
    ///  2) subdivide long segments so nodes can slide around corners;
    ///  3) repeatedly move every interior node toward the midpoint of its neighbours
    ///     (or delete it) while both segments stay collision-free and the path gets shorter;
    ///  4) final string pulling to drop the redundant nodes.
    /// </summary>
    public List<Vector3> Wrap(List<Vector3> originalPath, int passes = 20, float maxSegment = 1f)
    {
        if (originalPath == null || originalPath.Count < 3)
            return originalPath;

        List<Vector3> path = PullString(originalPath);
        path = Subdivide(path, maxSegment);

        for (int pass = 0; pass < passes; pass++)
        {
            bool changed = false;

            for (int i = 1; i < path.Count - 1; i++)
            {
                Vector3 a = path[i - 1], c = path[i], b = path[i + 1];

                if (collisionChecker.IsConnectionValid(a, b))
                {
                    path.RemoveAt(i);
                    i--;
                    changed = true;
                    continue;
                }

                Vector3 mid = (a + b) * 0.5f;
                mid.y = c.y;
                float current = Flat(a, c) + Flat(c, b);

                for (float t = 1f; t >= 0.124f; t *= 0.5f)
                {
                    Vector3 candidate = Vector3.Lerp(c, mid, t);
                    if (Flat(a, candidate) + Flat(candidate, b) < current - 0.0001f &&
                        collisionChecker.IsConnectionValid(a, candidate) &&
                        collisionChecker.IsConnectionValid(candidate, b))
                    {
                        path[i] = candidate;
                        changed = true;
                        break;
                    }
                }
            }

            if (!changed) break;
        }

        return PullString(path);
    }

    private List<Vector3> PullString(List<Vector3> path)
    {
        var result = new List<Vector3> { path[0] };
        int i = 0;
        while (i < path.Count - 1)
        {
            int j = path.Count - 1;
            while (j > i + 1 && !collisionChecker.IsConnectionValid(path[i], path[j])) j--;
            result.Add(path[j]);
            i = j;
        }
        return result;
    }

    private static List<Vector3> Subdivide(List<Vector3> path, float maxSegment)
    {
        var result = new List<Vector3> { path[0] };
        for (int k = 1; k < path.Count; k++)
        {
            Vector3 a = path[k - 1], b = path[k];
            int pieces = Mathf.Max(1, Mathf.CeilToInt(Flat(a, b) / maxSegment));
            for (int s = 1; s <= pieces; s++)
                result.Add(Vector3.Lerp(a, b, (float)s / pieces));
        }
        return result;
    }

    private static float Flat(Vector3 a, Vector3 b)
    {
        a.y = b.y = 0f;
        return Vector3.Distance(a, b);
    }
}