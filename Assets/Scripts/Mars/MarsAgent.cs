using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Live MARS demo (replaces RRTConnectTest for MARS). Mirrors the paper's architecture in one
/// component:
///   - execution: GridAgentMovement follows the current path;
///   - collision checking: every 1/checkRate s the remaining path is checked (30 Hz in the paper);
///   - replanning: if it is blocked, MARS replans from the agent's position within its time budget.
///     If replanning fails the agent stays stopped (safety stop) and retries on the next check.
/// Obstacles: assign a BenchmarkObstacleManager and use the ⋮ menu "Drop Obstacle Ahead",
/// or enable Auto Drop Obstacles.
/// </summary>
public class MarsAgent : MonoBehaviour
{
    [SerializeField] private MarsPathfinder pathfinder;
    [SerializeField] private Transform goal;
    [SerializeField] private GridAgentMovement movement;

    [Tooltip("Collision checks per second (30 Hz in the paper).")]
    [SerializeField, Min(1f)] private float checkRate = 30f;

    [Header("Obstacles appearing on the path (optional)")]
    [SerializeField] private BenchmarkObstacleManager obstacles;
    [SerializeField] private bool autoDropObstacles = false;
    [SerializeField, Min(0.1f)] private float dropEverySeconds = 3f;
    [SerializeField] private float minAhead = 1.5f;
    [SerializeField] private float maxAhead = 5f;
    [SerializeField] private int seed = 7;

    private System.Random random;
    private float nextCheck, nextDrop;
    private int replans, safetyStops;

    private void Start()
    {
        random = new System.Random(seed);
        if (pathfinder == null || goal == null || movement == null)
        {
            Debug.LogWarning("MarsAgent: assign Pathfinder, Goal and Movement.", this);
            enabled = false;
            return;
        }

        if (pathfinder.Initialize(transform.position, goal.position) &&
            pathfinder.TryGetPath(transform.position, out List<Vector3> path))
            movement.SetPath(path);
        else
            Debug.LogWarning("MarsAgent: no initial path.", this);

        nextDrop = Time.time + dropEverySeconds;
    }

    private void Update()
    {
        if (pathfinder.Planner == null) return;

        if (autoDropObstacles && obstacles != null && movement.HasPath && Time.time >= nextDrop)
        {
            DropObstacleAhead();
            nextDrop = Time.time + dropEverySeconds;
        }

        if (Time.time < nextCheck) return;
        nextCheck = Time.time + 1f / checkRate;

        if (pathfinder.IsPathValidFrom(transform.position)) return;

        // Blocked: stop, replan, resume.
        movement.Stop();
        replans++;
        if (pathfinder.TryGetPath(transform.position, out List<Vector3> newPath))
        {
            movement.SetPath(newPath);
            MarsPlanner.ReplanResult r = pathfinder.LastReplan;
            if (r != null)
                Debug.Log($"MARS replan #{replans}: {r.Milliseconds:F2} ms, cost {r.Cost:F2}, " +
                          $"attempts {r.ConnectionAttempts} (direct {r.DirectConnections}, reused {r.ReusedConnections}), " +
                          $"subtree iterations {r.SubtreeIterations}, improved by graph search: {r.ImprovedByGraphSearch}.", this);
        }
        else
        {
            safetyStops++;
            if (safetyStops % 30 == 1)
                Debug.LogWarning($"MARS: safety stop ({safetyStops}). {pathfinder.LastReplan?.Reason}", this);
        }
    }

    [ContextMenu("Drop Obstacle Ahead")]
    public void DropObstacleAhead()
    {
        if (obstacles == null || pathfinder.Planner == null)
        {
            Debug.LogWarning("MarsAgent: assign a BenchmarkObstacleManager and enter Play mode.", this);
            return;
        }

        List<Vector3> remaining = pathfinder.Planner.RemainingWaypoints(transform.position);
        Vector2Int goalCell = pathfinder.Grid.WorldToCell(goal.position);
        if (ReplanObstacleUtility.TryPickCellAhead(pathfinder.Grid, transform.position, remaining, 1,
                                                   minAhead, maxAhead, goalCell, random, out Vector2Int cell))
            obstacles.AddObstacle(cell);
        else
            Debug.Log("MarsAgent: no cell ahead can be blocked without sealing the goal.", this);
    }
}
