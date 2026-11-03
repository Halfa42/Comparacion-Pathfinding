
using UnityEngine;

public class MarsCollisionChecker
{
    private readonly MazeNavigationGrid grid;

    private const float SampleSpacingFactor = 0.2f;

    public MarsCollisionChecker(MazeNavigationGrid grid)
    {
        this.grid = grid;
    }

    public bool IsPositionValid(Vector3 position)
    {
        return grid.IsValidWorldPosition(position);
    }

    public bool IsConnectionValid(Vector3 from, Vector3 to)
    {
        Vector3 difference = to - from;
        difference.y = 0f;

        float distance = difference.magnitude;

        if (distance < 0.0001f)
            return IsPositionValid(from);

        float spacing = grid.CellSize * SampleSpacingFactor;

        int steps = Mathf.Max(
            1,
            Mathf.CeilToInt(distance / spacing)
        );

        Vector2Int previousCell = grid.WorldToCell(from);

        if (!grid.IsWalkable(previousCell))
            return false;

        for (int i = 1; i <= steps; i++)
        {
            float t = (float)i / steps;

            Vector3 point = Vector3.Lerp(from, to, t);
            Vector2Int currentCell = grid.WorldToCell(point);

            if (!grid.IsWalkable(currentCell))
                return false;

            // Reject transitions between diagonal cells
            // if either adjacent side cell is blocked.
            int dx = currentCell.x - previousCell.x;
            int dz = currentCell.y - previousCell.y;

            if (dx != 0 && dz != 0)
            {
                Vector2Int sideA = new Vector2Int(
                    previousCell.x + dx,
                    previousCell.y
                );

                Vector2Int sideB = new Vector2Int(
                    previousCell.x,
                    previousCell.y + dz
                );

                if (!grid.IsWalkable(sideA) ||
                    !grid.IsWalkable(sideB))
                {
                    return false;
                }
            }

            previousCell = currentCell;
        }

        return true;
    }

    public float CalculateDistance(Vector3 a, Vector3 b)
    {
        a.y = 0f;
        b.y = 0f;

        return Vector3.Distance(a, b);
    }
}
