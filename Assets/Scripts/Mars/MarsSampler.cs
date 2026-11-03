
using UnityEngine;
using System;

public class MarsSampler
{
    private readonly MazeNavigationGrid grid;
    private readonly System.Random random;

    public MarsSampler(MazeNavigationGrid grid, int seed)
    {
        this.grid = grid;
        random = new System.Random(seed);
    }

    public bool TrySample(out Vector3 sample, int maxAttempts = 100)
    {
        for (int i = 0; i < maxAttempts; i++)
        {
            Vector3 candidate = grid.GetRandomWorldPosition(random);

            if (grid.IsValidWorldPosition(candidate))
            {
                sample = candidate;
                return true;
            }
        }

        sample = Vector3.zero;
        return false;
    }
}
