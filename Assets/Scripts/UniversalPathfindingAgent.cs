using System;
using System.Collections.Generic;
using UnityEngine;

[DisallowMultipleComponent, DefaultExecutionOrder(-75)]
public sealed class UniversalPathfindingAgent : MonoBehaviour,
    AlgorithmSequenceController.IAlgorithmStrategy, IPathfindingMetrics
{
    [SerializeField] private Transform goal;
    [SerializeField] private MazeNavigationGrid grid;
    [SerializeField, Min(0.02f)] private float replanInterval = 0.5f;
    public bool IsSolved { get; private set; }
    public bool CollectBenchmarkMetrics { get; set; }
    public event Action<long, double, long> SearchCompleted;

    private IGridPathfinder planner;
    private GridAgentMovement movement;
    private Collider[] agentColliders, goalColliders;
    private List<Vector3> path;
    private bool started;
    private float nextPlan;
    private Vector3 lastSafePosition;
    private bool reportedInvalidMotion;

    public static IGridPathfinder FindPlanner(GameObject actor)
    {
        IGridPathfinder result = null;
        foreach (MonoBehaviour component in actor.GetComponentsInChildren<MonoBehaviour>(true))
            if (component is IGridPathfinder candidate)
            {
                if (result != null) return null;
                result = candidate;
            }
        return result;
    }

    public bool Prepare(MazeNavigationGrid navigationGrid)
    {
        planner = FindPlanner(gameObject);
        if (planner == null || navigationGrid == null) return false;
        // Existing demo controllers remain available in their own scenes.
        // A sequence uses this one follower so two controllers never drive the same agent.
        foreach (MonoBehaviour component in GetComponentsInChildren<MonoBehaviour>(true))
            if (component is DTATest || component is RRTTest || component is DStarLiteTest ||
                component is AgentPathFollower || component is MarsAgent)
            {
                component.CancelInvoke();
                component.enabled = false;
            }
        movement = GetComponentInChildren<GridAgentMovement>();
        if (movement == null) movement = gameObject.AddComponent<GridAgentMovement>();
        grid = navigationGrid;
        return planner.BindGrid(grid);
    }

    private void Start()
    {
        if (!started && goal != null) BeginRun(goal, grid);
    }

    public bool BeginRun(Transform target, MazeNavigationGrid navigationGrid)
    {
        EndRun();
        if (target == null || !Prepare(navigationGrid)) return false;
        Vector2Int startCell = grid.WorldToCell(movement.transform.position);
        if (!grid.IsWalkable(startCell))
        {
            Debug.LogError("Universal agent: spawn is blocked at cell " + startCell + ".", this);
            return false;
        }
        goal = target;
        IsSolved = false;
        started = true;
        agentColliders = GetComponentsInChildren<Collider>();
        goalColliders = goal.GetComponentsInChildren<Collider>();
        if (agentColliders.Length == 0 || goalColliders.Length == 0)
        {
            Debug.LogError("Universal agent: both agent and goal need a Collider to detect capture.", this);
            return false;
        }
        path = null;
        nextPlan = 0f;
        lastSafePosition = movement.transform.position;
        reportedInvalidMotion = false;
        grid.CellChanged += OnGridChanged;
        enabled = true;
        return true;
    }

    private void LateUpdate()
    {
        if (IsSolved) return;
        if (goal == null || movement == null || grid == null) { EndRun(); return; }
        Vector3 positionNow = movement.transform.position;
        if (positionNow != lastSafePosition && !grid.IsSegmentWalkable(lastSafePosition, positionNow))
        {
            // GridAgentMovement moves Transforms directly. Undo an invalid
            // corner crossing before capture detection and benchmark sampling.
            movement.transform.position = lastSafePosition;
            movement.Stop();
            nextPlan = 0f;
            if (!reportedInvalidMotion)
                Debug.LogWarning("Universal agent: blocked wall crossing; stopped and requested a new route.", this);
            reportedInvalidMotion = true;
        }
        lastSafePosition = movement.transform.position;
        if (TryCapture()) return;
        if (Time.time < nextPlan) return;
        nextPlan = Time.time + Mathf.Max(0.02f, replanInterval);
        IPathfindingMetrics nativeMetrics = planner as IPathfindingMetrics;
        bool originalCollect = nativeMetrics != null && nativeMetrics.CollectBenchmarkMetrics;
        if (nativeMetrics != null) nativeMetrics.CollectBenchmarkMetrics = CollectBenchmarkMetrics;
        var clock = System.Diagnostics.Stopwatch.StartNew();
        bool found;
        try { found = planner.TryGetPath(movement.transform.position, goal.position, out path); }
        finally
        {
            clock.Stop();
            if (nativeMetrics != null) nativeMetrics.CollectBenchmarkMetrics = originalCollect;
            long peak = CollectBenchmarkMetrics ? Math.Max(planner.SearchPeakMemory, GC.GetTotalMemory(false)) : 0;
            SearchCompleted?.Invoke(planner.SearchNodes, clock.Elapsed.TotalMilliseconds, peak);
        }
        if (!found || path == null || path.Count == 0)
        {
            movement.Stop();
            return;
        }
        // Grid planners return cell centers. Keep the actor's height, and skip
        // the start center only when already advancing on the same first edge.
        Vector3 position = movement.transform.position;
        for (int i = 0; i < path.Count; i++) path[i] = new Vector3(path[i].x, position.y, path[i].z);
        if (path.Count > 1)
        {
            Vector3 edge = path[1] - path[0], offset = position - path[0];
            float progress = edge.sqrMagnitude > 0f ? Vector3.Dot(offset, edge) / edge.sqrMagnitude : -1f;
            float tolerance = grid.CellSize * 0.0001f;
            if (progress >= 0f && progress <= 1f && (offset - edge * progress).sqrMagnitude <= tolerance * tolerance)
                path.RemoveAt(0);
        }
        movement.SetPath(path);
    }

    private bool TryCapture()
    {
        foreach (Collider actor in agentColliders)
        foreach (Collider target in goalColliders)
        {
            if (actor == null || target == null || actor == target || !actor.enabled || !target.enabled ||
                !actor.gameObject.activeInHierarchy || !target.gameObject.activeInHierarchy) continue;
            if (!Physics.ComputePenetration(actor, actor.transform.position, actor.transform.rotation,
                target, target.transform.position, target.transform.rotation, out _, out _)) continue;
            IsSolved = true;
            EndRun();
            PlayerAStarController targetController = goal.GetComponent<PlayerAStarController>();
            if (targetController != null)
            {
                targetController.CancelInvoke();
                targetController.enabled = false;
            }
            GridAgentMovement targetMovement = goal.GetComponent<GridAgentMovement>();
            if (targetMovement != null) targetMovement.Stop();
            return true;
        }
        return false;
    }

    private void OnGridChanged(Vector2Int cell)
    {
        movement.Stop();
        nextPlan = 0f;
    }

    public void EndRun()
    {
        enabled = false;
        if (movement != null) movement.Stop();
        if (grid != null) grid.CellChanged -= OnGridChanged;
    }

    private void OnDisable() => EndRun();

    private void OnDrawGizmos()
    {
        if (path == null) return;
        Gizmos.color = Color.cyan;
        for (int i = 1; i < path.Count; i++) Gizmos.DrawLine(path[i - 1], path[i]);
    }
}
