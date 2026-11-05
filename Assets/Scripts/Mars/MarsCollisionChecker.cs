using UnityEngine;

public class MarsCollisionChecker
{
    private readonly MazeNavigationGrid grid;

    public MarsCollisionChecker(MazeNavigationGrid grid)
    {
        this.grid = grid;
    }

    public bool IsPositionValid(Vector3 position) =>
        grid.IsSegmentWalkable(position, position);

    public bool IsConnectionValid(Vector3 from, Vector3 to) =>
        grid.IsSegmentWalkable(from, to);

    public float CalculateDistance(Vector3 a, Vector3 b)
    {
        a.y = 0f;
        b.y = 0f;
        return Vector3.Distance(a, b);
    }
}
