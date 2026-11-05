using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Makes an agent chase a (moving) target using D* Lite OR MARS.
/// Assign ONE pathfinder: Pathfinder (D* Lite) or Mars Pathfinder.
/// Target: drag a Transform, or leave it empty to follow the player spawned at runtime by
/// AStarSingleManager (the object with PlayerAStarController).
///
/// Every 1/Updates Per Second seconds it sends the target's current cell to the pathfinder
/// (TryGetPath(agent, goal)), which reuses its previous work when the goal moves.
/// </summary>
public class AgentPathFollower : MonoBehaviour
{
    [Header("Pathfinder (assign one)")]
    [SerializeField] private DStarLitePathfinder pathfinder;
    [SerializeField] private MarsPathfinder marsPathfinder;

    [Header("References")]
    [Tooltip("Optional: taken from the pathfinder if empty.")]
    [SerializeField] private MazeNavigationGrid grid;
    [SerializeField] private GridAgentMovement movement;

    [Header("Target")]
    [Tooltip("Empty = find the player (PlayerAStarController) when it spawns.")]
    [SerializeField] private Transform target;
    [SerializeField, Min(1f)] private float updatesPerSecond = 10f;
    [Tooltip("The agent stops when it is this close to the target (world units).")]
    [SerializeField] private float catchDistance = 0.6f;
    [Tooltip("If the agent starts inside a wall or outside the maze, move it to the nearest free cell.")]
    [SerializeField] private bool snapStartToFreeCell = true;

    private bool initialized;
    private bool caught;
    private float nextUpdate;
    private int failures;

    private bool UseMars => marsPathfinder != null;

    private void Start()
    {
        if (movement == null) movement = GetComponent<GridAgentMovement>();
        if (grid == null) grid = UseMars ? marsPathfinder.Grid : (pathfinder != null ? pathfinder.Grid : null);

        if ((pathfinder == null && marsPathfinder == null) || grid == null || movement == null)
        {
            Debug.LogError("AgentPathFollower: assign a pathfinder (D* Lite or MARS) whose Grid is set, and a GridAgentMovement.", this);
            enabled = false;
        }
    }

    private void Update()
    {
        if (target == null)
        {
            // The player is spawned in Start by AStarSingleManager, so it may not exist yet.
            PlayerAStarController player = FindObjectOfType<PlayerAStarController>();
            if (player == null) return;
            target = player.transform;
        }

        if (Time.time < nextUpdate) return;
        nextUpdate = Time.time + 1f / updatesPerSecond;

        // The goal is the target's cell, on the floor (the player capsule is higher than the grid).
        Vector3 goal = grid.CellToWorld(grid.WorldToCell(target.position));

        if (!initialized && !TryInitialize(goal)) return;

        if (Flat(transform.position, target.position) <= catchDistance)
        {
            if (!caught) Debug.Log($"AgentPathFollower ({Name}): target caught.", this);
            caught = true;
            movement.Stop();
            return;
        }
        caught = false;

        List<Vector3> path;
        bool found = UseMars
            ? marsPathfinder.TryGetPath(transform.position, goal, out path)
            : pathfinder.TryGetPath(transform.position, goal, out path);

        if (!found || path.Count == 0)
        {
            if (failures++ % 20 == 0)
            {
                string reason = UseMars && marsPathfinder.LastReplan != null ? " Reason: " + marsPathfinder.LastReplan.Reason : "";
                Debug.LogWarning($"AgentPathFollower ({Name}): no path from {grid.WorldToCell(transform.position)} to " +
                                 $"{grid.WorldToCell(target.position)}.{reason}", this);
            }
            return;
        }
        failures = 0;

        // D* Lite starts the path at the centre of the agent's cell, which may be behind the agent.
        // Skip it if the agent is already on its way to the next waypoint (avoids stepping back and forth).
        // Not for MARS: its path already starts at the agent, and skipping a point would create an
        // unchecked segment that can cut a wall corner.
        if (!UseMars && path.Count > 1 && Flat(transform.position, path[1]) < Flat(path[0], path[1]))
            path.RemoveAt(0);

        // Keep the agent's own height (the grid is at floor level).
        for (int i = 0; i < path.Count; i++)
        {
            Vector3 p = path[i];
            p.y = transform.position.y;
            path[i] = p;
        }

        movement.SetPath(path);
    }

    private bool TryInitialize(Vector3 goal)
    {
        Vector2Int startCell = grid.WorldToCell(transform.position);
        if (!grid.IsWalkable(startCell))
        {
            if (!snapStartToFreeCell || !TryFindNearestFreeCell(startCell, out Vector2Int free))
            {
                Debug.LogError($"AgentPathFollower ({Name}): the agent starts in cell {startCell}, which is a wall or outside the maze. Move it onto a free cell.", this);
                enabled = false;
                return false;
            }

            Vector3 snapped = grid.CellToWorld(free);
            snapped.y = transform.position.y;
            transform.position = snapped;
            Debug.LogWarning($"AgentPathFollower ({Name}): start cell {startCell} is a wall/outside the maze; moved the agent to {free}.", this);
        }

        Vector3 start = grid.CellToWorld(grid.WorldToCell(transform.position));
        bool ok = UseMars ? marsPathfinder.Initialize(start, goal) : pathfinder.Initialize(start, goal);
        if (!ok)
        {
            if (failures++ % 20 == 0)
                Debug.LogWarning($"AgentPathFollower ({Name}): could not initialize (start {grid.WorldToCell(start)}, goal {grid.WorldToCell(goal)}).", this);
            return false;
        }

        initialized = true;
        failures = 0;
        return true;
    }

    private bool TryFindNearestFreeCell(Vector2Int from, out Vector2Int result)
    {
        result = from;
        float best = float.PositiveInfinity;
        for (int x = 0; x < grid.Width; x++)
        {
            for (int y = 0; y < grid.Depth; y++)
            {
                var c = new Vector2Int(x, y);
                if (!grid.IsWalkable(c)) continue;
                float d = (c - from).sqrMagnitude;
                if (d < best) { best = d; result = c; }
            }
        }
        return !float.IsInfinity(best);
    }

    private string Name => UseMars ? "MARS" : "D* Lite";

    private static float Flat(Vector3 a, Vector3 b)
    {
        float dx = a.x - b.x, dz = a.z - b.z;
        return Mathf.Sqrt(dx * dx + dz * dz);
    }
}
