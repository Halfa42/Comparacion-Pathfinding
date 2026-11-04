using System.Collections.Generic;
using UnityEngine;

public class RRTTest : MonoBehaviour
{
    [SerializeField] private RRTPathfinder pathfinder;
    [Tooltip("Target to follow, or an AStarSingleManager whose spawned player should be followed.")]
    [SerializeField] private Transform goal;
    [SerializeField] private GridAgentMovement movement;
    [SerializeField, Min(0.02f)] private float replanInterval = 0.5f;
    [Tooltip("Horizontal target displacement in world units that requests a new route.")]
    [SerializeField, Min(0.01f)] private float goalMovementThreshold = 0.25f;

    private MazeNavigationGrid subscribedGrid;
    private List<Vector3> path;
    private bool started;
    private bool replanRequested;
    private Transform trackedGoal;
    private Vector3 plannedGoalPosition;
    private bool hasGoalSample;
    private float nextReplanTime;
    private bool pathSearchFailed;
    private Transform goalSource;
    private Transform spawnedGoal;
    private bool reportedTargetAmbiguity;
    private bool goalCaught;
    private Transform collisionGoal;
    private Collider[] agentColliders;
    private Collider[] goalColliders;

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
        hasGoalSample = false;
        trackedGoal = null;
        goalSource = null;
        spawnedGoal = null;
        pathSearchFailed = false;
    }

    private void OnCellChanged(Vector2Int cell)
    {
        if (goalCaught) return;
        if (movement != null) movement.Stop();
        path = null;
        replanRequested = true;
    }

    private void LateUpdate()
    {
        if (goalCaught) return;
        if (WaitForBenchmark()) return;

        Transform currentGoal = ResolveGoal();
        if (pathfinder == null || pathfinder.Grid == null || currentGoal == null || movement == null)
        {
            if (movement != null) movement.Stop();
            path = null;
            hasGoalSample = false;
            return;
        }
        if (TryCatchGoal(currentGoal)) return;

        Vector3 displacement = currentGoal.position - plannedGoalPosition;
        displacement.y = 0f;
        float threshold = Mathf.Max(0.01f, goalMovementThreshold);
        bool goalChanged = !hasGoalSample || trackedGoal != currentGoal ||
            displacement.sqrMagnitude >= threshold * threshold;
        Vector3 remaining = currentGoal.position - transform.position;
        remaining.y = 0f;
        bool routeExhausted = !movement.HasPath && remaining.sqrMagnitude > 0.0001f;

        if (replanRequested ||
            ((goalChanged || pathSearchFailed || routeExhausted) && Time.time >= nextReplanTime))
            Recalculate();
    }

    private bool WaitForBenchmark()
    {
        if (pathfinder == null || !pathfinder.IsBenchmarkRunning) return false;
        if (movement != null) movement.Stop();
        path = null;
        replanRequested = true;
        return true;
    }

    private Transform ResolveGoal()
    {
        if (goalSource != goal)
        {
            goalSource = goal;
            spawnedGoal = null;
            reportedTargetAmbiguity = false;
        }
        if (goal == null) return null;

        AStarSingleManager manager = goal.GetComponent<AStarSingleManager>();
        if (manager == null) return goal;
        if (spawnedGoal != null) return spawnedGoal;
        if (manager.mazeGenerator == null) return null;

        // The manager spawns the player at the scene root during Start.
        PlayerAStarController match = null;
        foreach (PlayerAStarController player in
                 FindObjectsByType<PlayerAStarController>(FindObjectsSortMode.None))
        {
            if (player.maze != manager.mazeGenerator || player.gameObject.scene != manager.gameObject.scene)
                continue;
            if (match != null)
            {
                if (!reportedTargetAmbiguity)
                    Debug.LogWarning("TA-RRT*: multiple players match Goal's manager. Assign the actual player Transform to Goal.", this);
                reportedTargetAmbiguity = true;
                return null;
            }
            match = player;
        }
        if (match != null)
        {
            spawnedGoal = match.transform;
            reportedTargetAmbiguity = false;
        }
        return spawnedGoal;
    }

    private bool TryCatchGoal(Transform currentGoal)
    {
        if (agentColliders == null) agentColliders = GetComponentsInChildren<Collider>();
        if (collisionGoal != currentGoal)
        {
            collisionGoal = currentGoal;
            goalColliders = currentGoal.GetComponentsInChildren<Collider>();
        }

        // Transform-driven agents do not require a Rigidbody to detect contact.
        foreach (Collider agentCollider in agentColliders)
        {
            if (agentCollider == null || !agentCollider.enabled ||
                !agentCollider.gameObject.activeInHierarchy) continue;
            foreach (Collider goalCollider in goalColliders)
            {
                if (goalCollider == null || goalCollider == agentCollider || !goalCollider.enabled ||
                    !goalCollider.gameObject.activeInHierarchy) continue;
                Vector3 separation;
                float depth;
                if (!Physics.ComputePenetration(
                    agentCollider, agentCollider.transform.position, agentCollider.transform.rotation,
                    goalCollider, goalCollider.transform.position, goalCollider.transform.rotation,
                    out separation, out depth)) continue;

                goalCaught = true;
                replanRequested = false;
                pathSearchFailed = false;
                path = null;
                movement.Stop();

                PlayerAStarController targetController = currentGoal.GetComponent<PlayerAStarController>();
                if (targetController != null)
                {
                    targetController.CancelInvoke();
                    targetController.enabled = false;
                }
                GridAgentMovement targetMovement = currentGoal.GetComponent<GridAgentMovement>();
                if (targetMovement != null) targetMovement.Stop();
                Debug.Log("TA-RRT*: target caught; both agents stopped.", this);
                return true;
            }
        }
        return false;
    }

    [ContextMenu("Recalculate TA-RRT* Path")]
    public void Recalculate()
    {
        if (goalCaught || WaitForBenchmark()) return;
        replanRequested = false;
        path = null;
        if (!Application.isPlaying || pathfinder == null || pathfinder.Grid == null ||
            goal == null || movement == null)
        {
            if (movement != null) movement.Stop();
            hasGoalSample = false;
            Debug.LogWarning("TA-RRT*: assign Pathfinder, Goal and Movement, then enter Play mode.", this);
            return;
        }

        Transform currentGoal = ResolveGoal();
        if (currentGoal == null)
        {
            movement.Stop();
            hasGoalSample = false;
            return;
        }
        if (TryCatchGoal(currentGoal)) return;

        trackedGoal = currentGoal;
        plannedGoalPosition = currentGoal.position;
        hasGoalSample = true;
        nextReplanTime = Time.time + Mathf.Max(0.02f, replanInterval);

        if (pathfinder.TryGetPath(transform.position, plannedGoalPosition, out path))
        {
            pathSearchFailed = false;
            movement.SetPath(path);
            Debug.Log($"TA-RRT*: {path.Count} waypoints, {pathfinder.NodeCount} nodes, " +
                $"length {pathfinder.PathLength:F3}, planning {pathfinder.PlanningMilliseconds:F2} ms.", this);
        }
        else
        {
            pathSearchFailed = true;
            movement.Stop();
            path = null;
            Debug.LogWarning("TA-RRT*: no route found; check endpoints, clearance and iteration budget.", this);
        }
    }

    private void OnDrawGizmos()
    {
        if (path == null) return;
        Gizmos.color = Color.magenta;
        for (int i = 1; i < path.Count; i++) Gizmos.DrawLine(path[i - 1], path[i]);
    }
}
