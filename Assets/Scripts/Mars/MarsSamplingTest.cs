
using UnityEngine;
using System.Collections.Generic;

public class MarsSamplingTest : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private MazeNavigationGrid grid;

    [Header("Sampling")]
    [SerializeField] private int seed = 12345;
    [SerializeField] private int numberOfSamples = 30;
    [SerializeField] private float connectionLength = 3f;

    [Header("Visualization")]
    [SerializeField] private float lineHeight = 0.25f;
    [SerializeField] private float sampleRadius = 0.12f;

    private MarsSampler sampler;
    private MarsCollisionChecker collisionChecker;
    private System.Random directionRandom;

    private readonly List<Vector3> samples = new();
    private readonly List<(Vector3 from, Vector3 to, bool valid)> connections = new();

    private void Start()
    {
        sampler = new MarsSampler(grid, seed);
        collisionChecker = new MarsCollisionChecker(grid);
        directionRandom = new System.Random(seed + 1);

        TestSampling();
    }

    private void TestSampling()
    {
        samples.Clear();
        connections.Clear();

        int successfulSamples = 0;
        int validConnections = 0;
        int invalidConnections = 0;

        for (int i = 0; i < numberOfSamples; i++)
        {
            if (!sampler.TrySample(out Vector3 sample))
                continue;

            successfulSamples++;
            samples.Add(sample);

            // Seeded, so the test is reproducible (UnityEngine.Random was not seeded).
            double angle = directionRandom.NextDouble() * 2.0 * System.Math.PI;
            Vector3 direction = new Vector3((float)System.Math.Cos(angle), 0f, (float)System.Math.Sin(angle));

            if (direction.sqrMagnitude < 0.001f)
                continue;

            direction.Normalize();

            Vector3 target = sample + direction * connectionLength;

            bool valid = collisionChecker.IsConnectionValid(
                sample,
                target
            );

            connections.Add((sample, target, valid));

            if (valid)
                validConnections++;
            else
                invalidConnections++;
        }

        Debug.Log(
            $"MARS Sampling Test\n" +
            $"Samples generated: {successfulSamples}\n" +
            $"Valid connections: {validConnections}\n" +
            $"Invalid connections: {invalidConnections}"
        );
    }

    private void OnDrawGizmos()
    {
        if (samples == null || connections == null)
            return;

        // Draw sample positions
        Gizmos.color = Color.blue;

        foreach (Vector3 sample in samples)
        {
            Gizmos.DrawSphere(
                sample + Vector3.up * lineHeight,
                sampleRadius
            );
        }

        // Draw connections
        foreach (var connection in connections)
        {
            Gizmos.color = connection.valid
                ? Color.green
                : Color.red;

            Vector3 from = connection.from +
                           Vector3.up * lineHeight;

            Vector3 to = connection.to +
                         Vector3.up * lineHeight;

            Gizmos.DrawLine(from, to);

            Gizmos.DrawSphere(to, sampleRadius * 0.5f);
        }
    }
}
