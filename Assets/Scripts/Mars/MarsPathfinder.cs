using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// MARS as a component, with the same API as DStarLitePathfinder / RRTPathfinder / DTAPathfinder:
///   Initialize(start, goal)  -> offline phase (builds the path set)
///   TryGetPath(position)     -> remaining current path if it is still valid, otherwise MARS replanning.
/// </summary>
public class MarsPathfinder : MonoBehaviour, IGridPathfinder
{
    [SerializeField] private MazeNavigationGrid grid;
    [SerializeField] private MarsPlanner.Settings settings = new MarsPlanner.Settings();

    [Header("Gizmos")]
    [SerializeField] private bool drawPathSet = true;
    [SerializeField] private float gizmoHeight = 0.3f;

    private MarsPlanner planner;

    public MazeNavigationGrid Grid => grid;
    public MarsPlanner Planner => planner;
    public MarsPlanner.Settings Settings => settings;
    public MarsPlanner.ReplanResult LastReplan => planner != null ? planner.LastReplan : null;
    public long SearchNodes { get; private set; }
    public long SearchPeakMemory => 0;
    private Vector2Int movingGoalCell;

    public bool BindGrid(MazeNavigationGrid navigationGrid)
    {
        grid = navigationGrid;
        planner = null;
        return grid != null;
    }

    public bool TryGetPath(Vector3 currentWorldPosition, Vector3 targetWorldPosition, out List<Vector3> worldPath)
    {
        worldPath = null;
        SearchNodes = 0;
        if (grid == null) return false;
        Vector2Int targetCell = grid.WorldToCell(targetWorldPosition);
        if (planner == null || planner.CurrentPath == null || movingGoalCell != targetCell)
        {
            movingGoalCell = targetCell;
            bool initialized = Initialize(currentWorldPosition, targetWorldPosition);
            SearchNodes = planner != null ? planner.OfflineTreeNodes : 0;
            if (!initialized) return false;
        }
        MarsPlanner.ReplanResult previous = LastReplan;
        bool found = TryGetPath(currentWorldPosition, out worldPath);
        // Existing MARS counters: offline tree nodes plus online connection/subtree attempts.
        if (LastReplan != null && LastReplan != previous)
            SearchNodes += LastReplan.ConnectionAttempts + LastReplan.SubtreeIterations;
        return found;
    }

    public bool Initialize(Vector3 startWorld, Vector3 goalWorld)
    {
        if (grid == null)
        {
            Debug.LogError("MARS: assign the MazeNavigationGrid.", this);
            return false;
        }

        planner = new MarsPlanner(grid, settings);
        bool ok = planner.Initialize(startWorld, goalWorld);
        if (ok)
            Debug.Log($"MARS: path set with {planner.PathSet.Count} paths built in {planner.OfflineMilliseconds:F1} ms.", this);
        else
            Debug.LogWarning("MARS: " + planner.FailureReason, this);
        return ok;
    }

    public bool TryGetPath(Vector3 currentWorldPosition, out List<Vector3> worldPath)
    {
        worldPath = new List<Vector3>();
        if (planner == null || planner.CurrentPath == null) return false;

        if (planner.IsValidFrom(currentWorldPosition))
        {
            worldPath = planner.RemainingWaypoints(currentWorldPosition);
            return true;
        }

        MarsPlanner.ReplanResult result = planner.Replan(currentWorldPosition);
        if (!result.Success) return false;
        worldPath = result.Waypoints;
        return true;
    }

    public bool IsPathValidFrom(Vector3 position) => planner != null && planner.IsValidFrom(position);

    private void OnDrawGizmos()
    {
        if (!drawPathSet || planner == null) return;
        Vector3 up = Vector3.up * gizmoHeight;

        foreach (MarsPlanner.Path path in planner.PathSet)
        {
            if (path == planner.CurrentPath) continue;
            Gizmos.color = path.Origin == "replan" ? new Color(1f, 0.8f, 0f, 0.6f) : new Color(0.6f, 0.6f, 0.6f, 0.8f);
            for (int i = 1; i < path.Nodes.Count; i++)
                Gizmos.DrawLine(path.Nodes[i - 1].Position + up, path.Nodes[i].Position + up);
        }

        if (planner.CurrentPath == null) return;
        Gizmos.color = Color.green;
        List<MarsPlanner.Node> current = planner.CurrentPath.Nodes;
        for (int i = 1; i < current.Count; i++)
        {
            Gizmos.DrawLine(current[i - 1].Position + up * 1.1f, current[i].Position + up * 1.1f);
            Gizmos.DrawSphere(current[i].Position + up * 1.1f, 0.06f);
        }
    }
}
