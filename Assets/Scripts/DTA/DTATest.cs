using System.Collections.Generic;
using UnityEngine;

// Attach to the agent, alongside GridAgentMovement, in the DTA scene.
public class DTATest : MonoBehaviour
{
    [SerializeField] private DTAPathfinder pathfinder;
    [SerializeField] private Transform goal;
    [SerializeField] private GridAgentMovement movement;
    [SerializeField] private bool drawAbstractPath = true;

    private DTATerrainMap subscribedMap;
    private DTAPathfinder subscribedPathfinder;
    private List<Vector3> path;
    private bool started;
    private bool replanRequested;

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
    }

    private void RequestReplan()
    {
        if (movement != null) movement.Stop();
        path = null;
        replanRequested = true;
    }

    private void LateUpdate()
    {
        // Batch changes within the frame; never resume a stale path on failure.
        if (replanRequested) Recalculate();
    }

    [ContextMenu("Recalculate DTA Path")]
    public void Recalculate()
    {
        replanRequested = false;
        path = null;
        if (movement != null) movement.Stop();
        if (!Application.isPlaying || pathfinder == null || goal == null || movement == null)
        {
            Debug.LogWarning("DTA: assign Pathfinder, Goal and Movement, then enter Play mode.", this);
            return;
        }
        if (pathfinder.Initialize(transform.position, goal.position) &&
            pathfinder.TryGetPath(transform.position, out path))
        {
            movement.SetPath(path);
            Debug.Log($"DTA: {path.Count} waypoints; {pathfinder.TerrainMap.RegionCount} regions; " +
                $"expanded {pathfinder.ExpandedAbstractNodes} abstract / {pathfinder.ExpandedRefinementNodes} grid nodes; " +
                $"length {pathfinder.PathLength:F3}; cost {pathfinder.PathCost:F3}; " +
                $"planning {pathfinder.PlanningMilliseconds:F2} ms.", this);
        }
        else
            Debug.LogWarning("DTA: no valid route; check references, endpoints and terrain settings.", this);
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
