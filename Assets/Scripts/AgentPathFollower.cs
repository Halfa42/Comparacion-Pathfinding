
using System.Collections.Generic;
using UnityEngine;

public class AgentPathFollower : MonoBehaviour
{
    [SerializeField] private DStarLitePathfinder pathfinder;
    [SerializeField] private MazeNavigationGrid grid;
    [SerializeField] private GridAgentMovement movement;

    private void Start()
    {

        if (pathfinder.TryGetPath(
            transform.position,
            out List<Vector3> path))
        {
            movement.SetPath(path);
        }
        else
        {
            Debug.LogWarning("No path found.");
        }
    }
}
