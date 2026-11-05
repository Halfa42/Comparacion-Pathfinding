using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>Replaces the pursuing prefab after each capture, using the same starting scenario.</summary>
public class AlgorithmSequenceController : MonoBehaviour
{
    public interface IAlgorithmStrategy
    {
        bool IsSolved { get; }
        bool BeginRun(Transform target, MazeNavigationGrid grid);
        void EndRun();
    }

    [Tooltip("Ordered prefabs. Each needs one planner implementing IGridPathfinder; the shared follower is added automatically.")]
    [SerializeField] private List<GameObject> agentPrefabs = new List<GameObject>();
    [SerializeField] private Transform agentStart;
    [SerializeField] private MazeNavigationGrid grid;
    [Tooltip("Moving target, or the AStarSingleManager that spawns it.")]
    [SerializeField] private Transform goal;
    [SerializeField] private int targetSeed = 12345;
    [SerializeField, Min(1)] private int runsPerAlgorithm = 3;
    [SerializeField, Min(0f)] private float secondsBetweenRuns = 1f;
    [SerializeField] private bool runOnStart = true;
    [Tooltip("Optional shared benchmarker. Also found automatically on this GameObject.")]
    [SerializeField] private AlgorithmRunBenchmark benchmark;

    public bool IsRunning => sequence != null;
    private Coroutine sequence;
    private GameObject currentAgent;
    private IAlgorithmStrategy currentStrategy;
    private Transform actualGoal;
    private Pose initialGoalPose;
    private PlayerAStarController targetController;
    private GridAgentMovement targetMovement;

    private void Start()
    {
        if (runOnStart) RunSequence();
    }

    [ContextMenu("Run Algorithm Sequence")]
    public void RunSequence()
    {
        if (!Application.isPlaying || !isActiveAndEnabled || IsRunning) return;
        if (benchmark == null) benchmark = GetComponent<AlgorithmRunBenchmark>();
        if (grid == null || agentStart == null || goal == null || agentPrefabs.Count == 0)
        {
            Debug.LogError("Algorithm sequence: assign Grid, Agent Start, Goal and Agent Prefabs.", this);
            return;
        }
        foreach (GameObject prefab in agentPrefabs)
            if (prefab == null || UniversalPathfindingAgent.FindPlanner(prefab) == null)
            {
                Debug.LogError("Algorithm sequence: '" + (prefab != null ? prefab.name : "empty entry") +
                    "' needs exactly one planner implementing IGridPathfinder. Add the planner component to that prefab.", prefab);
                return;
            }
        sequence = StartCoroutine(RunRoutine());
    }

    private IEnumerator RunRoutine()
    {
        // Let the target spawner and grid finish initialization.
        yield return null;
        AStarSingleManager spawner = goal.GetComponent<AStarSingleManager>();
        Transform target = spawner != null ? spawner.SpawnedPlayer : goal;
        if (target == null)
        {
            Debug.LogError("Algorithm sequence: the target has not been spawned.", this);
            sequence = null;
            yield break;
        }
        if (actualGoal != target) initialGoalPose = new Pose(target.position, target.rotation);
        actualGoal = target;
        targetController = target.GetComponent<PlayerAStarController>();
        targetMovement = target.GetComponent<GridAgentMovement>();
        if (!TryGetStartPose(out Pose startPose))
        {
            Debug.LogError("Algorithm sequence: no valid spawn cell. Check Agent Start and the grid.", this);
            sequence = null;
            yield break;
        }
        AlgorithmRunBenchmark recorder = benchmark != null && benchmark.isActiveAndEnabled ? benchmark : null;
        int repetitions = Mathf.Max(1, runsPerAlgorithm);
        try
        {
            recorder?.BeginSession();
            foreach (GameObject prefab in agentPrefabs)
            for (int execution = 1; execution <= repetitions; execution++)
            {
                ReleaseAgent();
                FreezeTarget();
                actualGoal.SetPositionAndRotation(initialGoalPose.position, initialGoalPose.rotation);
                if (targetController != null && !targetController.RestartNavigation(targetSeed))
                {
                    Debug.LogError("Algorithm sequence: place the target on a walkable grid cell.", this);
                    yield break;
                }

                currentAgent = Instantiate(prefab, startPose.position, startPose.rotation);
                currentAgent.SetActive(true);
                UniversalPathfindingAgent follower = currentAgent.GetComponent<UniversalPathfindingAgent>();
                if (follower == null) follower = currentAgent.AddComponent<UniversalPathfindingAgent>();
                currentStrategy = follower;
                recorder?.BeginRun(currentAgent, grid, prefab.name, execution);
                if (currentStrategy == null || !currentStrategy.BeginRun(actualGoal, grid))
                {
                    Debug.LogError("Algorithm sequence: check '" + prefab.name + "' planner, colliders and terrain references.", this);
                    yield break;
                }
                Debug.Log($"Algorithm sequence: {prefab.name} ({execution}/{repetitions}).", this);
                yield return new WaitUntil(() => currentAgent == null || actualGoal == null || currentStrategy.IsSolved);
                if (currentAgent == null || actualGoal == null) yield break;
                recorder?.EndRun(true);
                currentStrategy.EndRun();
                FreezeTarget();
                yield return new WaitForSeconds(Mathf.Max(0f, secondsBetweenRuns));
            }
            Debug.Log("Algorithm sequence completed.", this);
        }
        finally
        {
            try
            {
                if (currentAgent != null) currentStrategy?.EndRun();
                FreezeTarget();
            }
            finally
            {
                sequence = null;
                recorder?.EndSession();
            }
        }
    }

    private bool TryGetStartPose(out Pose pose)
    {
        pose = default;
        Vector3 requested = agentStart.position;
        if (!grid.IsInside(Vector2Int.zero) ||
            float.IsNaN(requested.x) || float.IsInfinity(requested.x) ||
            float.IsNaN(requested.y) || float.IsInfinity(requested.y) ||
            float.IsNaN(requested.z) || float.IsInfinity(requested.z)) return false;
        Vector2Int cell = grid.WorldToCell(requested);
        if (!grid.IsWalkable(cell))
        {
            float nearest = float.PositiveInfinity;
            for (int x = 0; x < grid.Width; x++)
            for (int z = 0; z < grid.Depth; z++)
            {
                var candidate = new Vector2Int(x, z);
                if (!grid.IsWalkable(candidate)) continue;
                Vector3 offset = grid.CellToWorld(candidate) - requested;
                float distance = offset.x * offset.x + offset.z * offset.z;
                if (distance >= nearest) continue;
                nearest = distance;
                cell = candidate;
            }
            if (float.IsPositiveInfinity(nearest)) return false;
            Debug.LogWarning("Algorithm sequence: Agent Start is blocked or outside the grid; using free cell " + cell + ".", this);
        }
        Vector3 position = grid.CellToWorld(cell);
        position.y = requested.y;
        pose = new Pose(position, agentStart.rotation);
        Debug.Log($"Algorithm sequence: spawn cell {cell}, position {position}; shared by all algorithms and runs.", this);
        return true;
    }

    private void FreezeTarget()
    {
        if (targetController != null)
        {
            targetController.CancelInvoke();
            targetController.enabled = false;
        }
        if (targetMovement != null) targetMovement.Stop();
    }

    private void ReleaseAgent()
    {
        if (currentAgent != null) currentStrategy?.EndRun();
        currentStrategy = null;
        if (currentAgent == null) return;
        currentAgent.SetActive(false);
        Destroy(currentAgent);
        currentAgent = null;
    }

    [ContextMenu("Stop Algorithm Sequence")]
    public void StopSequence()
    {
        if (sequence != null) StopCoroutine(sequence);
        sequence = null;
        if (benchmark != null) benchmark.EndSession();
        FreezeTarget();
        ReleaseAgent();
    }

    private void OnDisable() => StopSequence();
}
