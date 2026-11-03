
using System.Collections.Generic;
using UnityEngine;

public class DStarLiteTest : MonoBehaviour
{
    [SerializeField] private DStarLitePathfinder pathfinder;
    [SerializeField] private Transform goal;
    [SerializeField] private GridAgentMovement movement;

    private List<Vector3> path;

    private void Start()
    {
        if (pathfinder.Initialize(transform.position, goal.position))
            Recalculate();
    }

    [ContextMenu("Recalculate Path")]
    public void Recalculate()
    {
        if (pathfinder.TryGetPath(transform.position, out path))
        {
            movement.SetPath(path);
            Debug.Log($"Path found: {path.Count} waypoints.");
        }
        else
            Debug.LogWarning("No valid path found.");
    }

    private void OnDrawGizmos()
    {
        if (path == null || path.Count < 2)
            return;

        Gizmos.color = Color.cyan;

        for (int i = 0; i < path.Count - 1; i++)
            Gizmos.DrawLine(path[i], path[i + 1]);
    }
}
