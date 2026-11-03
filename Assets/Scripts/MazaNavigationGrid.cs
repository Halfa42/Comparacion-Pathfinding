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
}
