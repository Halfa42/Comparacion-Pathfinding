
using System.Collections.Generic;
using UnityEngine;

public class GridAgentMovement : MonoBehaviour
{
    [Header("Movement")]
    [SerializeField] private float moveSpeed = 3f;
    [SerializeField] private float rotationSpeed = 8f;
    [SerializeField] private float stoppingDistance = 0.05f;

    private Queue<Vector3> waypoints = new Queue<Vector3>();
    private bool hasPath;

    public bool HasPath => hasPath;
    public bool IsMoving => hasPath && waypoints.Count > 0;

    public void SetPath(IReadOnlyList<Vector3> path)
    {
        waypoints.Clear();

        if (path == null || path.Count == 0)
        {
            hasPath = false;
            return;
        }

        foreach (Vector3 waypoint in path)
        {
            waypoints.Enqueue(waypoint);
        }

        hasPath = true;
    }

    private void Update()
    {
        if (!hasPath || waypoints.Count == 0)
            return;

        Vector3 target = waypoints.Peek();

        Vector3 direction = target - transform.position;
        direction.y = 0f;

        if (direction.sqrMagnitude <= stoppingDistance * stoppingDistance)
        {
            waypoints.Dequeue();

            if (waypoints.Count == 0)
                hasPath = false;

            return;
        }

        Vector3 movement = direction.normalized;

        transform.position = Vector3.MoveTowards(
            transform.position,
            target,
            moveSpeed * Time.deltaTime
        );

        if (movement.sqrMagnitude > 0.001f)
        {
            Quaternion targetRotation = Quaternion.LookRotation(movement);

            transform.rotation = Quaternion.Slerp(
                transform.rotation,
                targetRotation,
                rotationSpeed * Time.deltaTime
            );
        }
    }

    public void Stop()
    {
        waypoints.Clear();
        hasPath = false;
    }
}
