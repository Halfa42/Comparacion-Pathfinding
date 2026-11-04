using System.Collections.Generic;
using UnityEngine;

// Attach to the agent, alongside GridAgentMovement, in the DTA scene.
public class DTATest : MonoBehaviour
{
    [SerializeField] private DTAPathfinder pathfinder;
    [Tooltip("Target to follow, or an AStarSingleManager whose spawned player should be followed.")]
    [SerializeField] private Transform goal;
    [SerializeField] private GridAgentMovement movement;
    [SerializeField] private bool drawAbstractPath = true;
    [SerializeField, Min(0.02f)] private float replanInterval = 0.2f;

    private DTATerrainMap subscribedMap;
    private DTAPathfinder subscribedPathfinder;
    private List<Vector3> path;
    private bool started;
    private bool replanRequested;
    private Transform trackedGoal;
    private Vector2Int plannedGoalCell;
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
        subscribedPathfinder = pathfinder;
        if (subscribedPathfinder != null)
        {
            subscribedMap = subscribedPathfinder.TerrainMap;
            subscribedPathfinder.CostsChanged += RequestReplan;
        }
        if (subscribedMap != null) subscribedMap.Changed += RequestReplan;
        if (started) replanRequested = true;
    }

    private void Start()
    {
        started = true;
        Recalculate();
    }

    private void OnDisable()
    {
        if (subscribedMap != null) subscribedMap.Changed -= RequestReplan;
        if (subscribedPathfinder != null) subscribedPathfinder.CostsChanged -= RequestReplan;
        subscribedMap = null;
        subscribedPathfinder = null;
        if (movement != null) movement.Stop();
        path = null;
        hasGoalSample = false;
        trackedGoal = null;
        goalSource = null;
        spawnedGoal = null;
        pathSearchFailed = false;
    }

    private void RequestReplan()
    {
        if (goalCaught) return;
        if (movement != null) movement.Stop();
        path = null;
        replanRequested = true;
    }

    private void LateUpdate()
    {
        if (goalCaught) return;
        Transform currentGoal = ResolveGoal();
        if (pathfinder == null || pathfinder.Grid == null || currentGoal == null || movement == null)
        {
            if (movement != null) movement.Stop();
            path = null;
            hasGoalSample = false;
            return;
        }

        if (TryCatchGoal(currentGoal)) return;

        Vector2Int goalCell = pathfinder.Grid.WorldToCell(currentGoal.position);
        bool goalChanged = !hasGoalSample || trackedGoal != currentGoal || plannedGoalCell != goalCell;

        // Terrain/cost changes remain immediate; throttle moving-goal updates.
        if (replanRequested || ((goalChanged || pathSearchFailed) && Time.time >= nextReplanTime))
            Recalculate();
    }

    private bool TryCatchGoal(Transform currentGoal)
    {
        if (agentColliders == null) agentColliders = GetComponentsInChildren<Collider>();
        if (collisionGoal != currentGoal)
        {
            collisionGoal = currentGoal;
            goalColliders = currentGoal.GetComponentsInChildren<Collider>();
        }

        // These agents move their Transforms directly and need not have a
        // Rigidbody. Test their actual collider shapes rather than relying
        // on collision callbacks or on both occupying the same grid cell.
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

                // Stopping the queue alone would make the target's A* driver
                // choose another destination on its next Update.
                PlayerAStarController targetController = currentGoal.GetComponent<PlayerAStarController>();
                if (targetController != null)
                {
                    targetController.CancelInvoke();
                    targetController.enabled = false;
                }
                GridAgentMovement targetMovement = currentGoal.GetComponent<GridAgentMovement>();
                if (targetMovement != null) targetMovement.Stop();
                Debug.Log("DTA: target caught; both agents stopped.", this);
                return true;
            }
        }
        return false;
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

        // AStarSingleManager spawns the player at the scene root, not as its
        // child. Wait for its Start method rather than chasing the manager.
        PlayerAStarController match = null;
        foreach (PlayerAStarController player in
                 FindObjectsByType<PlayerAStarController>(FindObjectsSortMode.None))
        {
            if (player.maze != manager.mazeGenerator || player.gameObject.scene != manager.gameObject.scene)
                continue;
            if (match != null)
            {
                if (!reportedTargetAmbiguity)
                    Debug.LogWarning("DTA: multiple players match Goal's manager. Assign the actual player Transform to Goal.", this);
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

    [ContextMenu("Recalculate DTA Path")]
    public void Recalculate()
    {
        if (goalCaught) return;
        replanRequested = false;
        path = null;
        if (!Application.isPlaying || pathfinder == null || pathfinder.Grid == null ||
            goal == null || movement == null)
        {
            if (movement != null) movement.Stop();
            hasGoalSample = false;
            Debug.LogWarning("DTA: assign Pathfinder, Goal and Movement, then enter Play mode.", this);
            return;
        }

        Transform currentGoal = ResolveGoal();
        if (currentGoal == null)
        {
            // The assigned manager may not have spawned its player yet.
            movement.Stop();
            hasGoalSample = false;
            return;
        }

        if (TryCatchGoal(currentGoal)) return;

        Vector3 startPosition = transform.position;
        Vector3 goalPosition = currentGoal.position;
        trackedGoal = currentGoal;
        plannedGoalCell = pathfinder.Grid.WorldToCell(goalPosition);
        hasGoalSample = true;
        nextReplanTime = Time.time + Mathf.Max(0.02f, replanInterval);

        if (pathfinder.TryGetPath(startPosition, goalPosition, out path))
        {
            pathSearchFailed = false;
            movement.SetPath(path);
            Debug.Log($"DTA: {path.Count} waypoints; {pathfinder.TerrainMap.RegionCount} regions; " +
                $"expanded {pathfinder.ExpandedAbstractNodes} abstract / {pathfinder.ExpandedRefinementNodes} grid nodes; " +
                $"length {pathfinder.PathLength:F3}; cost {pathfinder.PathCost:F3}; " +
                $"planning {pathfinder.PlanningMilliseconds:F2} ms.", this);
        }
        else
        {
            pathSearchFailed = true;
            movement.Stop();
            path = null;
            Debug.LogWarning("DTA: no valid route; check references, endpoints and terrain settings.", this);
        }
    }

    private void OnDrawGizmos()
    {
        if (path == null) return;
        Gizmos.color = Color.cyan;
        for (int i = 1; i < path.Count; i++) Gizmos.DrawLine(path[i - 1], path[i]);
        if (!drawAbstractPath || pathfinder == null) return;
        IReadOnlyList<Vector3> abstractPath = pathfinder.AbstractWorldPath;
        Gizmos.color = Color.yellow;
        for (int i = 1; i < abstractPath.Count; i++)
            Gizmos.DrawLine(abstractPath[i - 1], abstractPath[i]);
    }

}
