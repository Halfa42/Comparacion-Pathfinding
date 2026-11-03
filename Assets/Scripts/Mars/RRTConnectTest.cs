using System.Collections.Generic;
using UnityEngine;

public class RRTConnectTest : MonoBehaviour
{
    [Header("Optimization")]
    [SerializeField] private bool optimizePath = true;
    [SerializeField] private int optimizationIterations = 200;

    private List<Vector3> originalPath;
    private List<Vector3> optimizedPath;

    [Header("References")]
    [SerializeField] private MazeNavigationGrid grid;
    [SerializeField] private Transform startPoint;
    [SerializeField] private Transform goalPoint;

    [Header("RRT-Connect Settings")]
    [SerializeField] private int seed = 12345;
    [SerializeField] private float stepSize = 0.75f;
    [SerializeField] private int maxIterations = 5000;

    [Header("Visualization")]
    [SerializeField] private float lineHeight = 0.3f;
    [SerializeField] private float pointRadius = 0.12f;

    private List<Vector3> generatedPath;

    private void Start()
    {
        if (grid == null || startPoint == null || goalPoint == null)
        {
            Debug.LogError("RRTConnectTest: Assign the grid, start and goal.");
            return;
        }

        RRTConnectPlanner planner = new RRTConnectPlanner(
            grid,
            seed,
            stepSize
        );

        float startTime = Time.realtimeSinceStartup;

        bool success = planner.TryPlan(
            startPoint.position,
            goalPoint.position,
            out generatedPath,
            maxIterations
        );

        float elapsedMs =
            (Time.realtimeSinceStartup - startTime) * 1000f;

        if (success)
        {
            originalPath = new List<Vector3>(generatedPath);

            MarsPathOptimizer optimizer =
                new MarsPathOptimizer(grid, seed + 1);

            if (optimizePath)
            {
                optimizedPath = optimizer.Optimize(
                    originalPath,
                    optimizationIterations
                );

                generatedPath = optimizedPath;
            }

            ValidatePath();

            Debug.Log(
                $"Original points: {originalPath.Count}\n" +
                $"Optimized points: {generatedPath.Count}\n" +
                $"Original length: {CalculatePathLength(originalPath):F2}\n" +
                $"Optimized length: {CalculatePathLength(generatedPath):F2}"
            );
}
    }

    private float CalculatePathLength(List<Vector3> path)
    {
        float totalDistance = 0f;

        for (int i = 1; i < path.Count; i++)
        {
            totalDistance += Vector3.Distance(
                path[i - 1],
                path[i]
            );
        }

        return totalDistance;
    }

    private void OnDrawGizmos()
    {
        if (generatedPath == null || generatedPath.Count == 0)
            return;

        Gizmos.color = Color.green;

        for (int i = 0; i < generatedPath.Count; i++)
        {
            Vector3 point = generatedPath[i] +
                Vector3.up * lineHeight;

            Gizmos.DrawSphere(point, pointRadius);

            if (i > 0)
            {
                Vector3 previous = generatedPath[i - 1] +
                    Vector3.up * lineHeight;

                Gizmos.DrawLine(previous, point);
            }
        }

        Gizmos.color = Color.blue;
        Gizmos.DrawSphere(
            generatedPath[0] + Vector3.up * lineHeight,
            pointRadius * 1.5f
        );

        Gizmos.color = Color.red;
        Gizmos.DrawSphere(
            generatedPath[generatedPath.Count - 1] +
            Vector3.up * lineHeight,
            pointRadius * 1.5f
        );
    }
    private void ValidatePath()
    {
        MarsCollisionChecker checker =
            new MarsCollisionChecker(grid);

        int invalidSegments = 0;

        for (int i = 1; i < generatedPath.Count; i++)
        {
            bool valid = checker.IsConnectionValid(
                generatedPath[i - 1],
                generatedPath[i]
            );

            if (!valid)
                invalidSegments++;
        }

        Debug.Log(
            $"Path validation:\n" +
            $"Total segments: {generatedPath.Count - 1}\n" +
            $"Invalid segments: {invalidSegments}"
        );
}
}