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
}