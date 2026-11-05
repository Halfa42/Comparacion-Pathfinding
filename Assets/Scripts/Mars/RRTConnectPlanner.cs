using System.Collections.Generic;
using UnityEngine;

public class RRTConnectPlanner
{
    private enum ExtendStatus
    {
        Trapped,
        Advanced,
        Reached
    }

    private class Node
    {
        public Vector3 Position;
        public Node Parent;

        public Node(Vector3 position, Node parent = null)
        {
            Position = position;
            Parent = parent;
        }
    }

    private class Tree
    {
        public Node Root { get; }

        private readonly List<Node> nodes = new();

        public int Count => nodes.Count;

        public Tree(Vector3 rootPosition)
        {
            Root = new Node(rootPosition);
            nodes.Add(Root);
        }

        public Node FindNearest(Vector3 target)
        {
            Node nearest = null;
            float bestDistance = float.MaxValue;

            foreach (Node node in nodes)
            {
                float distance = (node.Position - target).sqrMagnitude;

                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    nearest = node;
                }
            }

            return nearest;
        }

        public Node AddNode(Vector3 position, Node parent)
        {
            Node node = new Node(position, parent);
            nodes.Add(node);
            return node;
        }

        public List<Vector3> GetPathFromRoot(Node node)
        {
            List<Vector3> path = new();

            Node current = node;

            while (current != null)
            {
                path.Add(current.Position);
                current = current.Parent;
            }

            path.Reverse();
            return path;
        }
    }

    private readonly MarsSampler sampler;
    private readonly MarsCollisionChecker collisionChecker;
    private readonly float stepSize;

    // --- Métricas para benchmark (de la última llamada a TryPlan) ---
    public int LastIterations { get; private set; }
    public int LastTreeNodes { get; private set; }

    private const float PositionTolerance = 0.001f;
    private const int MaxConnectSteps = 10000;

    public RRTConnectPlanner(
        MazeNavigationGrid grid,
        int seed,
        float stepSize = 0.75f)
    {
        sampler = new MarsSampler(grid, seed);
        collisionChecker = new MarsCollisionChecker(grid);
        this.stepSize = Mathf.Max(0.01f, stepSize);
    }

    public bool TryPlan(
        Vector3 start,
        Vector3 goal,
        out List<Vector3> path,
        int maxIterations = 5000)
    {
        path = null;
        LastIterations = 0;
        LastTreeNodes = 0;

        if (!collisionChecker.IsPositionValid(start) ||
            !collisionChecker.IsPositionValid(goal))
        {
            Debug.LogWarning("RRT-Connect: Start or goal is not walkable.");
            return false;
        }

        // The two trees grow independently from their respective roots.
        Tree treeA = new Tree(start);
        Tree treeB = new Tree(goal);

        // Tracks which tree is rooted at the start.
        bool treeAIsStart = true;

        for (int iteration = 0; iteration < maxIterations; iteration++)
        {
            LastIterations = iteration + 1;
            LastTreeNodes = treeA.Count + treeB.Count;

            if (!sampler.TrySample(out Vector3 randomPosition))
                continue;

            // Extend the active tree toward the random sample.
            ExtendStatus statusA = Extend(
                treeA,
                randomPosition,
                out Node newNodeA
            );

            if (statusA != ExtendStatus.Trapped)
            {
                // Attempt to connect the other tree to the new node.
                ExtendStatus statusB = Connect(
                    treeB,
                    newNodeA.Position,
                    out Node newNodeB
                );

                if (statusB == ExtendStatus.Reached)
                {
                    Node startConnection = treeAIsStart
                        ? newNodeA
                        : newNodeB;

                    Node goalConnection = treeAIsStart
                        ? newNodeB
                        : newNodeA;

                    path = BuildFinalPath(
                        treeAIsStart ? treeA : treeB,
                        startConnection,
                        treeAIsStart ? treeB : treeA,
                        goalConnection
                    );

                    LastTreeNodes = treeA.Count + treeB.Count;
                    return true;
                }
            }

            // Alternate which tree grows first.
            (treeA, treeB) = (treeB, treeA);
            treeAIsStart = !treeAIsStart;
        }

        LastTreeNodes = treeA.Count + treeB.Count;
        return false;
    }

    private ExtendStatus Extend(
        Tree tree,
        Vector3 target,
        out Node newNode)
    {
        newNode = null;

        Node nearest = tree.FindNearest(target);

        Vector3 difference = target - nearest.Position;
        difference.y = 0f;

        float distance = difference.magnitude;

        if (distance < PositionTolerance)
        {
            newNode = nearest;
            return ExtendStatus.Reached;
        }

        float travelDistance = Mathf.Min(stepSize, distance);

        Vector3 newPosition = nearest.Position +
            difference.normalized * travelDistance;

        // Keep the new position on the same navigation plane.
        newPosition.y = nearest.Position.y;

        if (!collisionChecker.IsConnectionValid(
            nearest.Position,
            newPosition))
        {
            return ExtendStatus.Trapped;
        }

        newNode = tree.AddNode(newPosition, nearest);

        if (distance <= stepSize + PositionTolerance)
            return ExtendStatus.Reached;

        return ExtendStatus.Advanced;
    }

    private ExtendStatus Connect(
        Tree tree,
        Vector3 target,
        out Node connectedNode)
    {
        connectedNode = null;

        for (int i = 0; i < MaxConnectSteps; i++)
        {
            ExtendStatus status = Extend(
                tree,
                target,
                out connectedNode
            );

            if (status == ExtendStatus.Reached ||
                status == ExtendStatus.Trapped)
            {
                return status;
            }
        }

        return ExtendStatus.Trapped;
    }

    private List<Vector3> BuildFinalPath(
        Tree startTree,
        Node startConnection,
        Tree goalTree,
        Node goalConnection)
    {
        List<Vector3> result =
            startTree.GetPathFromRoot(startConnection);

        // This path is goal -> connection.
        List<Vector3> goalPath =
            goalTree.GetPathFromRoot(goalConnection);

        // Append connection -> goal, skipping the duplicate connection.
        for (int i = goalPath.Count - 2; i >= 0; i--)
        {
            result.Add(goalPath[i]);
        }

        return result;
    }
}
