
using UnityEngine;
using System.Collections.Generic;

public class BenchmarkObstacleManager : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private MazeNavigationGrid grid;
    [SerializeField] private GameObject obstaclePrefab;

    [Header("Experiment")]
    [SerializeField] private int randomSeed = 12345;

    private readonly List<GameObject> spawnedObstacles = new();

    private System.Random random;
    private readonly List<Vector2Int> blockedCells = new();

    private void Awake()
    {
        random = new System.Random(randomSeed);
    }

    public MazeNavigationGrid Grid => grid;
    public IReadOnlyList<Vector2Int> BlockedCells => blockedCells;

    public void ResetRandom()
    {
        random = new System.Random(randomSeed);
    }

    public bool AddObstacle(Vector2Int cell)
    {
        if (!grid.IsInside(cell) || !grid.IsWalkable(cell))
            return false;

        Vector3 position = grid.CellToWorld(cell);

        // The prefab is only visual; without one the cell is still blocked.
        if (obstaclePrefab != null)
        {
            GameObject obstacle = Instantiate(
                obstaclePrefab,
                position + Vector3.up * 0.5f,
                Quaternion.identity
            );

            obstacle.name = $"DynamicObstacle_{cell.x}_{cell.y}";
            spawnedObstacles.Add(obstacle);
        }

        blockedCells.Add(cell);

        grid.SetWalkable(cell, false);

        return true;
    }

    public void ClearObstacles()
    {
        foreach (Vector2Int cell in blockedCells)
        {
            grid.SetWalkable(cell, true);
        }

        foreach (GameObject obstacle in spawnedObstacles)
        {
            if (obstacle != null)
                Destroy(obstacle);
        }

        blockedCells.Clear();
        spawnedObstacles.Clear();
    }

    public Vector2Int SelectRandomCell(
        IReadOnlyList<Vector2Int> currentPath,
        Vector2Int start,
        Vector2Int goal)
    {
        if (currentPath == null || currentPath.Count < 3)
            return new Vector2Int(-1, -1);

        // Avoid the first and last path positions.
        int index = random.Next(1, currentPath.Count - 1);

        Vector2Int selectedCell = currentPath[index];

        if (selectedCell == start || selectedCell == goal)
            return new Vector2Int(-1, -1);

        return selectedCell;
    }
}
