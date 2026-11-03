using System;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

/// <summary>
/// Informed RRT* on the XZ plane of the maze. MARS uses it in two places:
///  - informedPlan (online): grow a subtree from a node of the current path (x_n) until it
///    reaches a node of another path (x_j), sampling only inside the informed ellipse whose
///    foci are x_n and x_j and whose major axis is the cost still allowed. If no solution
///    exists yet the allowed cost is infinite, so samples cover the whole map.
///  - offline optimization of the first path (Informed RRT* bounded by that path's cost).
///
/// The tree is persistent: calling TryConnect again with another target reuses it, so a
/// subtree grown from x_n for one x_j also serves the next x_j (subtree reuse in the paper).
/// </summary>
public class MarsInformedPlanner
{
    public delegate bool SegmentValidity(Vector3 from, Vector3 to);

    private sealed class TreeNode
    {
        public Vector3 Position;
        public int Parent;
        public float Cost;
        public readonly List<int> Children = new List<int>();
    }

    private const float Eps = 0.00001f;

    private readonly MazeNavigationGrid grid;
    private readonly SegmentValidity isFree;
    private readonly System.Random random;
    private readonly List<TreeNode> tree = new List<TreeNode>();
    private readonly List<int> near = new List<int>();
    private readonly Stack<int> stack = new Stack<int>();
    private readonly float height;

    public float StepSize = 0.75f;
    public float RewireRadius = 1.5f;
    public float GoalBias = 0.05f;

    public int TreeSize => tree.Count;
    public int Iterations { get; private set; }   // cumulative over all TryConnect calls

    public MarsInformedPlanner(MazeNavigationGrid grid, Vector3 root, SegmentValidity isFree, int seed)
    {
        this.grid = grid;
        this.isFree = isFree;
        random = new System.Random(seed);
        height = root.y;
        tree.Add(new TreeNode { Position = root, Parent = -1, Cost = 0f });
    }

    /// <summary>
    /// Finds a path root -> target cheaper than costBound (use float.PositiveInfinity for "any").
    /// stopAtFirst = true returns the first connection (MARS informedPlan); false keeps
    /// optimizing for maxIterations (anytime Informed RRT*).
    /// clock/deadlineMs (optional) stop the search when the time budget is spent.
    /// </summary>
    public bool TryConnect(Vector3 target, float costBound, int maxIterations, Stopwatch clock, double deadlineMs,
                           bool stopAtFirst, out List<Vector3> path, out float cost)
    {
        path = null;
        cost = float.PositiveInfinity;
        target.y = height;

        int bestNode = -1;
        float best = costBound;

        // Reuse: does any node already in the tree see the target directly?
        for (int i = 0; i < tree.Count; i++)
        {
            float c = tree[i].Cost + Flat(tree[i].Position, target);
            if (c < best && isFree(tree[i].Position, target))
            {
                best = c;
                bestNode = i;
            }
        }
        if (bestNode >= 0 && stopAtFirst) return Build(bestNode, target, out path, out cost);

        Vector3 root = tree[0].Position;
        float cMin = Flat(root, target);

        for (int it = 0; it < maxIterations; it++)
        {
            if (clock != null && clock.Elapsed.TotalMilliseconds > deadlineMs) break;
            Iterations++;

            if (!Sample(root, target, cMin, best, out Vector3 sample)) continue;

            int nearest = Nearest(sample);
            Vector3 from = tree[nearest].Position;
            float d = Flat(from, sample);
            if (d < Eps) continue;

            Vector3 point = from + (sample - from) * (Mathf.Min(StepSize, d) / d);
            point.y = height;
            if (!isFree(from, point)) continue;

            // Choose the cheapest valid parent among the near nodes (RRT*).
            near.Clear();
            int parent = nearest;
            float pointCost = tree[nearest].Cost + Flat(from, point);
            for (int i = 0; i < tree.Count; i++)
            {
                float di = Flat(tree[i].Position, point);
                if (di > RewireRadius) continue;
                near.Add(i);
                float c = tree[i].Cost + di;
                if (c + Eps < pointCost && isFree(tree[i].Position, point))
                {
                    parent = i;
                    pointCost = c;
                }
            }

            // Informed pruning: a node that cannot beat the bound is useless.
            if (pointCost + Flat(point, target) >= best) continue;

            int added = tree.Count;
            tree.Add(new TreeNode { Position = point, Parent = parent, Cost = pointCost });
            tree[parent].Children.Add(added);

            // Rewire the neighbours through the new node.
            for (int k = 0; k < near.Count; k++)
            {
                int i = near[k];
                if (i == 0 || i == parent) continue;
                float c = pointCost + Flat(point, tree[i].Position);
                if (c + Eps < tree[i].Cost && !IsAncestor(i, added) && isFree(point, tree[i].Position))
                    Reparent(i, added, c);
            }

            // Connect to the target.
            float toTarget = Flat(point, target);
            if (toTarget <= StepSize && pointCost + toTarget < best && isFree(point, target))
            {
                best = pointCost + toTarget;
                bestNode = added;
                if (stopAtFirst) break;
            }
        }

        if (bestNode < 0) return false;
        return Build(bestNode, target, out path, out cost);
    }

    private bool Build(int node, Vector3 target, out List<Vector3> path, out float cost)
    {
        path = new List<Vector3>();
        for (int i = node; i >= 0; i = tree[i].Parent) path.Add(tree[i].Position);
        path.Reverse();
        if (Flat(path[path.Count - 1], target) > Eps) path.Add(target);

        cost = 0f;
        for (int k = 1; k < path.Count; k++) cost += Flat(path[k - 1], path[k]);
        return true;
    }

    // Uniform sample over the map if the bound is infinite, otherwise uniform inside the
    // ellipse { x : |x - root| + |x - target| <= cMax }.
    private bool Sample(Vector3 root, Vector3 target, float cMin, float cMax, out Vector3 sample)
    {
        if (random.NextDouble() < GoalBias)
        {
            sample = target;
            return true;
        }

        if (float.IsInfinity(cMax))
        {
            sample = grid.GetRandomWorldPosition(random);
            sample.y = height;
            return grid.IsValidWorldPosition(sample);
        }

        if (cMax <= cMin + Eps)
        {
            sample = target;
            return false;
        }

        float a = cMax * 0.5f;
        float b = Mathf.Sqrt(Mathf.Max(0f, cMax * cMax - cMin * cMin)) * 0.5f;
        float dirX = 1f, dirZ = 0f;
        if (cMin > Eps)
        {
            dirX = (target.x - root.x) / cMin;
            dirZ = (target.z - root.z) / cMin;
        }

        double r = Math.Sqrt(random.NextDouble());
        double theta = random.NextDouble() * 2.0 * Math.PI;
        float ux = (float)(r * Math.Cos(theta)) * a;
        float uz = (float)(r * Math.Sin(theta)) * b;

        sample = new Vector3(
            (root.x + target.x) * 0.5f + ux * dirX - uz * dirZ,
            height,
            (root.z + target.z) * 0.5f + ux * dirZ + uz * dirX);
        return grid.IsValidWorldPosition(sample);
    }

    private int Nearest(Vector3 point)
    {
        int nearest = 0;
        float best = float.PositiveInfinity;
        for (int i = 0; i < tree.Count; i++)
        {
            float dx = tree[i].Position.x - point.x, dz = tree[i].Position.z - point.z;
            float d = dx * dx + dz * dz;
            if (d < best) { best = d; nearest = i; }
        }
        return nearest;
    }

    private bool IsAncestor(int candidate, int node)
    {
        for (int i = node; i >= 0; i = tree[i].Parent)
            if (i == candidate) return true;
        return false;
    }

    private void Reparent(int index, int newParent, float newCost)
    {
        TreeNode node = tree[index];
        tree[node.Parent].Children.Remove(index);
        tree[newParent].Children.Add(index);
        node.Parent = newParent;
        node.Cost = newCost;

        stack.Clear();
        stack.Push(index);
        while (stack.Count > 0)
        {
            int current = stack.Pop();
            foreach (int child in tree[current].Children)
            {
                tree[child].Cost = tree[current].Cost + Flat(tree[current].Position, tree[child].Position);
                stack.Push(child);
            }
        }
    }

    public static float Flat(Vector3 a, Vector3 b)
    {
        float dx = a.x - b.x, dz = a.z - b.z;
        return Mathf.Sqrt(dx * dx + dz * dz);
    }
}
