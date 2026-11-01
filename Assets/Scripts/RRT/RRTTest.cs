using System.Collections.Generic;
using UnityEngine;

public class RRTTest : MonoBehaviour
{
    [SerializeField] private RRTPathfinder pathfinder;
    [SerializeField] private Transform goal;
    [SerializeField] private GridAgentMovement movement;

    private MazeNavigationGrid subscribedGrid;
    private List<Vector3> path;
    private bool started;
    private bool replanRequested;

    private void OnEnable()
    {
        subscribedGrid = pathfinder != null ? pathfinder.Grid : null;
        if (subscribedGrid != null) subscribedGrid.CellChanged += OnCellChanged;
        if (started) replanRequested = true;
    }

    private void Start()
    {
        started = true;
        Recalculate();
    }

    private void OnDisable()
    {
        if (subscribedGrid != null) subscribedGrid.CellChanged -= OnCellChanged;
        subscribedGrid = null;
        if (movement != null) movement.Stop();
        path = null;
    }

    private void OnCellChanged(Vector2Int cell)
    {
        if (movement != null) movement.Stop();
        path = null;
        replanRequested = true;
    }

    private void LateUpdate()
    {
        if (replanRequested) Recalculate();
    }

    [ContextMenu("Recalculate TA-RRT* Path")]
    public void Recalculate()
    {
        replanRequested = false;
        path = null;
        if (movement != null) movement.Stop();
        if (!Application.isPlaying || pathfinder == null || goal == null || movement == null)
        {
            Debug.LogWarning("TA-RRT*: assign Pathfinder, Goal and Movement, then enter Play mode.", this);
            return;
        }

        if (pathfinder.Initialize(transform.position, goal.position) &&
            pathfinder.TryGetPath(transform.position, out path))
        {
            movement.SetPath(path);
            Debug.Log($"TA-RRT*: {path.Count} waypoints, {pathfinder.NodeCount} nodes, " +
                $"length {pathfinder.PathLength:F3}, planning {pathfinder.PlanningMilliseconds:F2} ms.", this);
        }
        else
            Debug.LogWarning("TA-RRT*: no route found; check endpoints, clearance and iteration budget.", this);
    }

    private void OnDrawGizmos()
    {
        if (path == null) return;
        Gizmos.color = Color.magenta;
        for (int i = 1; i < path.Count; i++) Gizmos.DrawLine(path[i - 1], path[i]);
    }
}
