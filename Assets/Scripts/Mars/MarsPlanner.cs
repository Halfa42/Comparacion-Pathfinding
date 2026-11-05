using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using UnityEngine;

/// <summary>
/// MARS — Multi-pAth Replanning Strategy.
/// C. Tonola, M. Faroni, M. Beschi, N. Pedrocchi, "Anytime informed multi-path replanning
/// strategy for complex environments", IEEE Access 11, 4105-4116, 2023.
///
/// 2D adaptation for the maze (the robot moves on the XZ plane of MazeNavigationGrid, cost = length).
///
/// OFFLINE  (Initialize)
///   - Path 1: RRT-Connect, then optimized (Informed RRT* bounded by its cost + wrapping).
///   - Paths 2..N: alternative routes with the same start/goal (the paper uses 3 paths in total).
///     The paper does not say how they are produced; here the middle part of every previous path is
///     temporarily forbidden so the new path must take a different corridor.
///   - Every path is stored in one directed graph G (nodes + edges), shared start and goal.
///
/// ONLINE   (Replan, when an obstacle invalidates the current path)
///   1. Project the robot on the current path. Q1 = path nodes between the robot and the obstacle.
///   2. P = every stored path; for each node the valid cost-to-go along its path (lazy, cached checks).
///   3. connectToPaths: for each x_n in Q1 (closest first), for each x_j in P (sorted by distance to x_n):
///        prune if  g(x_n) + |x_n x_j| + ctg(x_j) >= best;
///        informedPlan(x_n, x_j): reuse an existing connection in G, else a direct segment, else grow an
///        Informed RRT* subtree from x_n inside the ellipse allowed by the current best cost.
///   4. searchBetterPath: with the time left, depth-first branch-and-bound over G (lazy collision checks)
///      looking for a cheaper route robot -> goal.
///   5. The new path becomes the current path and is added to the path set (reused by later replans).
///   Everything runs inside a time budget (200 ms in the paper).
/// </summary>
public class MarsPlanner
{
    [Serializable]
    public class Settings
    {
        [Header("Path set (offline)")]
        [Min(1)] public int pathSetSize = 3;
        public int seed = 12345;
        public float stepSize = 0.75f;
        public int rrtConnectMaxIterations = 5000;
        [Tooltip("Shortcut iterations of MarsPathOptimizer.Optimize (0 = off).")]
        public int shortcutIterations = 200;
        [Tooltip("Informed RRT* iterations used to improve the first path (0 = off).")]
        public int informedOptimizationIterations = 1500;
        public bool wrapPaths = true;
        [Tooltip("Alternative paths: cells within ±band (fraction of length) around the middle of each previous path are forbidden.")]
        [Range(0.05f, 0.5f)] public float diversityBand = 0.2f;
        [Tooltip("Skip an alternative if this fraction of its cells is already used by another path.")]
        [Range(0.5f, 1f)] public float maxOverlap = 0.9f;

        [Header("Replanning (online)")]
        [Tooltip("Time budget per replan, ms (200 in the paper).")]
        public float replanBudgetMs = 200f;
        [Tooltip("Max Informed RRT* iterations per informedPlan call.")]
        public int connectionMaxIterations = 400;
        public float rewireRadius = 1.5f;
        public bool anytimeImprovement = true;
        [Tooltip("Paths kept in the set (initial paths are always kept; oldest replanned paths are dropped).")]
        [Min(2)] public int maxStoredPaths = 10;

        public Settings Clone() => (Settings)MemberwiseClone();
    }

    public sealed class Node
    {
        public int Id;
        public Vector3 Position;
        public readonly List<Edge> Out = new List<Edge>();
    }

    public sealed class Edge
    {
        public Node From, To;
        public float Cost;
        public bool IsConnection;          // created by replanning (second-order connection)
        internal int CheckedRevision = int.MinValue;
        internal bool Valid;
    }

    public sealed class Path
    {
        public readonly List<Node> Nodes = new List<Node>();
        public readonly List<Edge> Edges = new List<Edge>();
        public float Cost;
        public string Origin = "";
        public List<Vector3> Waypoints() => Nodes.Select(n => n.Position).ToList();
    }

    public class ReplanResult
    {
        public bool Success;
        public bool WasBlocked;
        public float Cost = float.PositiveInfinity;
        public double Milliseconds;
        public int ConnectionAttempts;
        public int ReusedConnections;
        public int DirectConnections;
        public int SubtreeIterations;
        public bool ImprovedByGraphSearch;
        public int CollisionChecks;
        public bool GoalChanged;          // moving target: this call moved the goal
        public int RetargetedPaths;       // stored paths extended to the new goal
        public int DroppedPaths;          // stored paths that could not reach the new goal
        public string Reason = "";
        public List<Vector3> Waypoints;
    }

    private struct Target
    {
        public Node Node;
        public float CostToGo;
        public Path Path;
        public int Index;
    }

    private readonly MazeNavigationGrid grid;
    private readonly MarsCollisionChecker checker;
    private readonly Settings s;
    private readonly List<Node> nodes = new List<Node>();
    private readonly List<Path> paths = new List<Path>();
    private readonly Dictionary<long, List<Edge>> connections = new Dictionary<long, List<Edge>>();
    private Node startNode, goalNode;
    private int initialPathCount;

    public Path CurrentPath { get; private set; }
    public IReadOnlyList<Path> PathSet => paths;
    public int InitialPathCount => initialPathCount;
    public int GraphNodeCount => nodes.Count;
    public double OfflineMilliseconds { get; private set; }
    public int OfflineTreeNodes { get; private set; }
    public int CollisionChecks { get; private set; }
    public ReplanResult LastReplan { get; private set; }
    public string FailureReason { get; private set; } = "";
    public Vector3 GoalPosition => goalNode != null ? goalNode.Position : Vector3.zero;
    /// <summary>Number of goal changes (moving target) since Initialize.</summary>
    public int GoalChanges { get; private set; }

    // False after a goal change if the current path could not be extended to the new goal yet.
    private bool currentReachesGoal;

    public MarsPlanner(MazeNavigationGrid grid, Settings settings)
    {
        this.grid = grid;
        s = settings ?? new Settings();
        checker = new MarsCollisionChecker(grid);
    }

    // =====================================================================  OFFLINE

    public bool Initialize(Vector3 start, Vector3 goal)
    {
        var clock = Stopwatch.StartNew();
        nodes.Clear(); paths.Clear(); connections.Clear();
        CurrentPath = null; LastReplan = null; FailureReason = "";
        OfflineTreeNodes = 0; CollisionChecks = 0; initialPathCount = 0;
        GoalChanges = 0; currentReachesGoal = true;

        try
        {
            if (!checker.IsPositionValid(start) || !checker.IsPositionValid(goal))
            {
                FailureReason = "inicio o meta bloqueados";
                return false;
            }

            startNode = NewNode(start);
            goalNode = NewNode(goal);

            List<Vector3> first = PlanFirstPath(start, goal);
            if (first == null)
            {
                FailureReason = "RRT-Connect sin ruta inicial";
                return false;
            }
            CurrentPath = AddPath(first, "inicial");

            for (int k = 1; k < s.pathSetSize; k++)
            {
                List<Vector3> alt = PlanAlternative(start, goal, k);
                if (alt != null && !TooSimilar(alt)) AddPath(alt, "alternativa " + k);
            }

            initialPathCount = paths.Count;
            return true;
        }
        finally
        {
            OfflineMilliseconds = clock.Elapsed.TotalMilliseconds;
        }
    }

    private List<Vector3> PlanFirstPath(Vector3 start, Vector3 goal)
    {
        var rrt = new RRTConnectPlanner(grid, s.seed, s.stepSize);
        if (!rrt.TryPlan(start, goal, out List<Vector3> path, s.rrtConnectMaxIterations)) return null;
        OfflineTreeNodes += rrt.LastTreeNodes;

        var optimizer = new MarsPathOptimizer(grid, s.seed + 1);
        if (s.shortcutIterations > 0) path = optimizer.Optimize(path, s.shortcutIterations);

        if (s.informedOptimizationIterations > 0)
        {
            var informed = new MarsInformedPlanner(grid, start, checker.IsConnectionValid, s.seed + 2)
            {
                StepSize = s.stepSize,
                RewireRadius = s.rewireRadius
            };
            float cost = PathCost(path);
            if (informed.TryConnect(goal, cost, s.informedOptimizationIterations, null, 0, false,
                                    out List<Vector3> better, out float betterCost) && betterCost < cost)
                path = better;
            OfflineTreeNodes += informed.TreeSize;
        }

        if (s.wrapPaths) path = optimizer.Wrap(path);
        return path;
    }

    private List<Vector3> PlanAlternative(Vector3 start, Vector3 goal, int k)
    {
        float[] bands = { s.diversityBand, s.diversityBand * 0.5f };
        Vector2Int startCell = grid.WorldToCell(start), goalCell = grid.WorldToCell(goal);

        for (int b = 0; b < bands.Length; b++)
        {
            var forbidden = new HashSet<Vector2Int>();
            foreach (Path p in paths) AddMiddleCells(p.Waypoints(), bands[b], forbidden);
            forbidden.Remove(startCell);
            forbidden.Remove(goalCell);

            MarsInformedPlanner.SegmentValidity valid =
                (a, c) => checker.IsConnectionValid(a, c) && AvoidsCells(a, c, forbidden);

            var planner = new MarsInformedPlanner(grid, start, valid, s.seed + 100 * k + 10 * b)
            {
                StepSize = s.stepSize,
                RewireRadius = s.rewireRadius,
                GoalBias = 0.1f
            };
            bool found = planner.TryConnect(goal, float.PositiveInfinity, s.rrtConnectMaxIterations, null, 0, true,
                                            out List<Vector3> path, out float _);
            OfflineTreeNodes += planner.TreeSize;
            if (!found) continue;

            var optimizer = new MarsPathOptimizer(grid, s.seed + 100 * k + 7);
            if (s.shortcutIterations > 0) path = optimizer.Optimize(path, s.shortcutIterations);
            if (s.wrapPaths) path = optimizer.Wrap(path);
            return path;
        }
        return null;
    }

    private Path AddPath(List<Vector3> points, string origin)
    {
        var path = new Path { Origin = origin };
        path.Nodes.Add(startNode);
        for (int i = 1; i < points.Count - 1; i++) path.Nodes.Add(NewNode(points[i]));
        path.Nodes.Add(goalNode);

        for (int i = 1; i < path.Nodes.Count; i++)
        {
            Edge e = NewEdge(path.Nodes[i - 1], path.Nodes[i], false, true);
            path.Edges.Add(e);
            path.Cost += e.Cost;
        }
        paths.Add(path);
        return path;
    }

    // =====================================================================  ONLINE

    /// <summary>Collision check of the current path from the robot to the goal (the paper's checking thread).</summary>
    public bool IsValidFrom(Vector3 robot)
    {
        if (CurrentPath == null || !currentReachesGoal) return false;
        robot = PlanningPosition(robot);
        int k = Project(CurrentPath, robot);
        if (!checker.IsConnectionValid(robot, CurrentPath.Nodes[k + 1].Position)) return false;
        for (int i = k + 1; i < CurrentPath.Edges.Count; i++)
            if (!IsValid(CurrentPath.Edges[i])) return false;
        return true;
    }

    /// <summary>Remaining waypoints of the current path: robot position, then the next nodes to the goal.</summary>
    public List<Vector3> RemainingWaypoints(Vector3 robot)
    {
        if (CurrentPath == null) return new List<Vector3> { robot };
        robot = PlanningPosition(robot);
        var result = new List<Vector3> { robot };
        int k = Project(CurrentPath, robot);
        for (int i = k + 1; i < CurrentPath.Nodes.Count; i++) result.Add(CurrentPath.Nodes[i].Position);
        return result;
    }

    /// <summary>Replans from the robot's position because the current path is blocked (or to improve it).</summary>
    public ReplanResult Replan(Vector3 robot) => Run(robot, false, Vector3.zero);

    /// <summary>
    /// Moving target: moves the goal and replans from the robot's position, within the same time budget.
    /// Extension of MARS (the paper assumes a fixed goal). The path set and the graph are reused:
    ///   1. every stored path is re-targeted: it is cut at its cheapest node that sees the new goal and a
    ///      segment to the new goal is added; if no node sees it, the path is extended from the old goal
    ///      with informedPlan (old goal -> new goal, computed once and shared by all paths);
    ///   2. paths that cannot be re-targeted are dropped from the set;
    ///   3. the normal MARS replanning runs toward the new goal: if the re-targeted current path is valid it
    ///      is the incumbent and searchBetterPath looks for shortcuts in G; if it is blocked or could not be
    ///      re-targeted, connectToPaths connects the robot side to the re-targeted paths / the new goal.
    /// Every call with a different goal moves it; MarsPathfinder.TryGetPath(position, goal, ...) only calls
    /// this when the target moved at least its Goal Update Distance.
    /// </summary>
    public ReplanResult SetGoal(Vector3 newGoal, Vector3 robot) => Run(robot, true, newGoal);

    private ReplanResult Run(Vector3 robot, bool changeGoal, Vector3 newGoal)
    {
        var res = new ReplanResult();
        var clock = Stopwatch.StartNew();
        double deadline = s.replanBudgetMs;
        int checksBefore = CollisionChecks;

        try
        {
            if (CurrentPath == null) { res.Reason = "MARS no inicializado"; return res; }
            robot = PlanningPosition(robot);
            if (!checker.IsPositionValid(robot)) { res.Reason = "el robot está en una celda bloqueada"; return res; }

            if (changeGoal)
            {
                newGoal.y = startNode.Position.y; // stay on the navigation plane (the target may be taller)
                if (!checker.IsPositionValid(newGoal)) { res.Reason = "la nueva meta está bloqueada"; return res; }
                if (Flat(newGoal, goalNode.Position) > 0.0001f)
                {
                    res.GoalChanged = true;
                    RetargetPaths(newGoal, robot, clock, deadline, res);
                }
            }

            // 1) Project the robot on the current path and build robot -> ... -> goal.
            int k = Project(CurrentPath, robot);
            Node robotNode = NewNode(robot);
            var remNodes = new List<Node> { robotNode, CurrentPath.Nodes[k + 1] };
            var remEdges = new List<Edge> { NewEdge(robotNode, CurrentPath.Nodes[k + 1], false, false) };
            for (int i = k + 1; i < CurrentPath.Edges.Count; i++)
            {
                remEdges.Add(CurrentPath.Edges[i]);
                remNodes.Add(CurrentPath.Edges[i].To);
            }

            int blocked = -1;
            for (int i = 0; i < remEdges.Count; i++)
                if (!IsValid(remEdges[i])) { blocked = i; break; }

            // The current path ends somewhere that is no longer the goal: every node up to its end can
            // be used to connect toward the new goal (as if it were blocked after its last node).
            if (blocked < 0 && !currentReachesGoal) blocked = remEdges.Count;

            float best = float.PositiveInfinity;
            List<Edge> bestRoute = null;

            if (blocked < 0)
            {
                // Not blocked: the current remainder is the incumbent solution.
                best = remEdges.Sum(e => e.Cost);
                bestRoute = remEdges;
            }
            else
            {
                res.WasBlocked = true;

                // Q1: nodes between the robot and the obstacle, closest first, with their cost from the robot.
                var q1 = new List<Node>();
                var g1 = new List<float>();
                float g = 0f;
                for (int i = 0; i <= blocked; i++)
                {
                    q1.Add(remNodes[i]);
                    g1.Add(g);
                    if (i < blocked) g += remEdges[i].Cost;
                }

                // P: every node with a valid cost-to-go along its path.
                List<Target> targets = CollectTargets();
                var subtrees = new Dictionary<Node, MarsInformedPlanner>();

                for (int qi = 0; qi < q1.Count && clock.Elapsed.TotalMilliseconds < deadline; qi++)
                {
                    Node xn = q1[qi];
                    float gn = g1[qi];
                    if (gn >= best) break; // nodes further along cost even more

                    // Q2: candidates sorted by distance from x_n.
                    List<Target> q2 = targets.OrderBy(x => Flat(xn.Position, x.Node.Position)).ToList();

                    foreach (Target t in q2)
                    {
                        if (clock.Elapsed.TotalMilliseconds >= deadline) break;
                        if (gn + Flat(xn.Position, t.Node.Position) + t.CostToGo >= best) continue;

                        List<Edge> connection;
                        float connectionCost;

                        if (t.Node == xn)
                        {
                            // x_n already lies on another valid path: no connection needed.
                            connection = new List<Edge>();
                            connectionCost = 0f;
                        }
                        else
                        {
                            float allowed = float.IsInfinity(best) ? float.PositiveInfinity : best - gn - t.CostToGo;
                            res.ConnectionAttempts++;
                            if (!InformedPlan(xn, t.Node, allowed, subtrees, clock, deadline, res,
                                              out connection, out connectionCost))
                                continue;
                        }

                        float candidate = gn + connectionCost + t.CostToGo;
                        if (candidate < best)
                        {
                            best = candidate;
                            bestRoute = new List<Edge>();
                            for (int i = 0; i < qi; i++) bestRoute.Add(remEdges[i]);            // robot -> x_n
                            bestRoute.AddRange(connection);                                     // x_n -> x_j
                            if (t.Path != null)                                                  // x_j -> goal
                                for (int i = t.Index; i < t.Path.Edges.Count; i++) bestRoute.Add(t.Path.Edges[i]);
                        }
                    }
                }
            }

            // 4) Anytime improvement with the time left.
            if (bestRoute != null && s.anytimeImprovement && clock.Elapsed.TotalMilliseconds < deadline)
            {
                // Candidate shortcut straight to the goal (checked lazily by the search, only if it helps).
                NewEdge(robotNode, goalNode, true, false);
                float before = best;
                SearchBetterPath(robotNode, ref best, ref bestRoute, clock, deadline);
                res.ImprovedByGraphSearch = best < before - 0.0001f;
            }

            if (bestRoute == null)
            {
                res.Reason = "sin conexión dentro del presupuesto de " + s.replanBudgetMs + " ms";
                return res;
            }

            // 5) The new path becomes current and joins the path set.
            var newPath = new Path { Origin = "replan", Cost = best };
            newPath.Nodes.Add(robotNode);
            foreach (Edge e in bestRoute)
            {
                newPath.Edges.Add(e);
                newPath.Nodes.Add(e.To);
            }
            AddReplannedPath(newPath);
            CurrentPath = newPath;
            currentReachesGoal = true;

            res.Success = true;
            res.Cost = best;
            res.Waypoints = newPath.Waypoints();
            return res;
        }
        finally
        {
            res.Milliseconds = clock.Elapsed.TotalMilliseconds;
            res.CollisionChecks = CollisionChecks - checksBefore;
            LastReplan = res;
        }
    }

    // Valid cost-to-go of every node along its own path (lazy checks, cached per map revision).
    private List<Target> CollectTargets()
    {
        var best = new Dictionary<Node, Target>();
        // The goal itself is always a target (needed after a goal change, when few paths reach it yet).
        Offer(best, new Target { Node = goalNode, CostToGo = 0f, Path = null, Index = 0 });
        foreach (Path p in paths)
        {
            int last = p.Nodes.Count - 1;
            float ctg = 0f;
            Offer(best, new Target { Node = p.Nodes[last], CostToGo = 0f, Path = p, Index = last });
            for (int i = last - 1; i >= 0; i--)
            {
                if (!IsValid(p.Edges[i])) break;
                ctg += p.Edges[i].Cost;
                Offer(best, new Target { Node = p.Nodes[i], CostToGo = ctg, Path = p, Index = i });
            }
        }
        return best.Values.ToList();
    }

    // ---------------------------------------------------------------- moving target

    private void RetargetPaths(Vector3 newGoal, Vector3 robot, Stopwatch clock, double deadline, ReplanResult res)
    {
        int k = Project(CurrentPath, robot); // before any path is modified
        Node oldGoal = goalNode;
        goalNode = NewNode(newGoal);
        GoalChanges++;

        var subtrees = new Dictionary<Node, MarsInformedPlanner>();
        List<Edge> bridge = null;  // old goal -> new goal, shared by every path that needs it
        bool bridgeTried = false;

        // Current path first: it matters most. Others follow while there is time.
        var order = new List<Path>(paths);
        if (order.Remove(CurrentPath)) order.Insert(0, CurrentPath);
        bool currentInSet = paths.Contains(CurrentPath);
        if (!currentInSet) order.Insert(0, CurrentPath);

        foreach (Path p in order)
        {
            bool isCurrent = p == CurrentPath;
            bool ok = clock.Elapsed.TotalMilliseconds < deadline || isCurrent;
            if (ok) ok = Retarget(p, isCurrent ? k + 1 : 0, oldGoal, ref bridge, ref bridgeTried, subtrees, clock, deadline, res);

            if (ok)
            {
                res.RetargetedPaths++;
                continue;
            }

            res.DroppedPaths++;
            if (isCurrent) currentReachesGoal = false;
            int index = paths.IndexOf(p);
            if (index >= 0)
            {
                paths.RemoveAt(index);
                if (index < initialPathCount) initialPathCount--;
            }
        }
    }

    // Cut the path at its cheapest node (index >= minIndex) that sees the new goal and add that segment;
    // otherwise append the old goal -> new goal bridge.
    private bool Retarget(Path p, int minIndex, Node oldGoal, ref List<Edge> bridge, ref bool bridgeTried,
                          Dictionary<Node, MarsInformedPlanner> subtrees, Stopwatch clock, double deadline,
                          ReplanResult res)
    {
        int last = p.Nodes.Count - 1;
        if (p.Nodes[last] != oldGoal) return false; // already re-targeted elsewhere: should not happen
        minIndex = Mathf.Clamp(minIndex, 0, last);

        var costFromStart = new float[p.Nodes.Count];
        for (int i = 1; i < p.Nodes.Count; i++) costFromStart[i] = costFromStart[i - 1] + p.Edges[i - 1].Cost;

        Vector3 goal = goalNode.Position;
        List<int> candidates = Enumerable.Range(minIndex, last - minIndex + 1)
            .OrderBy(j => costFromStart[j] + Flat(p.Nodes[j].Position, goal))
            .ToList();

        foreach (int i in candidates)
        {
            CollisionChecks++;
            if (!checker.IsConnectionValid(p.Nodes[i].Position, goal)) continue;

            p.Nodes.RemoveRange(i + 1, p.Nodes.Count - (i + 1));
            p.Edges.RemoveRange(i, p.Edges.Count - i);
            Edge e = NewEdge(p.Nodes[i], goalNode, true, true);
            p.Edges.Add(e);
            p.Nodes.Add(goalNode);
            p.Cost = costFromStart[i] + e.Cost;
            return true;
        }

        // No node of this path sees the new goal: extend it from the old goal.
        if (!bridgeTried)
        {
            bridgeTried = true;
            res.ConnectionAttempts++;
            if (!InformedPlan(oldGoal, goalNode, float.PositiveInfinity, subtrees, clock, deadline, res,
                              out bridge, out float _))
                bridge = null;
        }
        if (bridge == null) return false;

        foreach (Edge e in bridge)
        {
            p.Edges.Add(e);
            p.Nodes.Add(e.To);
        }
        p.Cost = costFromStart[last] + bridge.Sum(x => x.Cost);
        return true;
    }

    private static void Offer(Dictionary<Node, Target> best, Target t)
    {
        if (!best.TryGetValue(t.Node, out Target old) || t.CostToGo < old.CostToGo) best[t.Node] = t;
    }

    // informedPlan: existing connection in G -> direct segment -> Informed RRT* subtree from x_n.
    private bool InformedPlan(Node xn, Node xj, float allowed, Dictionary<Node, MarsInformedPlanner> subtrees,
                              Stopwatch clock, double deadline, ReplanResult res,
                              out List<Edge> connection, out float cost)
    {
        connection = null;
        cost = float.PositiveInfinity;
        long key = ((long)xn.Id << 32) | (uint)xj.Id;

        if (connections.TryGetValue(key, out List<Edge> cached))
        {
            float c = cached.Sum(e => e.Cost);
            if (c < allowed && cached.All(e => IsValid(e)))
            {
                connection = cached;
                cost = c;
                res.ReusedConnections++;
                return true;
            }
        }

        float direct = Flat(xn.Position, xj.Position);
        if (direct >= allowed) return false;

        CollisionChecks++;
        if (checker.IsConnectionValid(xn.Position, xj.Position))
        {
            connection = new List<Edge> { NewEdge(xn, xj, true, true) };
            connections[key] = connection;
            cost = direct;
            res.DirectConnections++;
            return true;
        }

        if (!subtrees.TryGetValue(xn, out MarsInformedPlanner planner))
        {
            planner = new MarsInformedPlanner(grid, xn.Position, checker.IsConnectionValid,
                                              s.seed + xn.Id * 7919 + grid.Revision)
            {
                StepSize = s.stepSize,
                RewireRadius = s.rewireRadius
            };
            subtrees[xn] = planner;
        }

        int iterationsBefore = planner.Iterations;
        bool found = planner.TryConnect(xj.Position, allowed, s.connectionMaxIterations, clock, deadline, true,
                                        out List<Vector3> points, out float pathCost);
        res.SubtreeIterations += planner.Iterations - iterationsBefore;
        if (!found) return false;

        // points[0] = x_n, points[last] = x_j. Store the subtree branch in G as a connection.
        connection = new List<Edge>();
        Node previous = xn;
        for (int i = 1; i < points.Count; i++)
        {
            Node next = i == points.Count - 1 ? xj : NewNode(points[i]);
            connection.Add(NewEdge(previous, next, true, true));
            previous = next;
        }
        connections[key] = connection;
        cost = pathCost;
        return true;
    }

    // searchBetterPath: depth-first branch-and-bound over G with lazy collision checking.
    private void SearchBetterPath(Node robotNode, ref float best, ref List<Edge> bestRoute,
                                  Stopwatch clock, double deadline)
    {
        var stack = new List<Edge>();
        var onStack = new HashSet<Node>();
        var bestG = new Dictionary<Node, float>();
        Dfs(robotNode, 0f, stack, onStack, bestG, ref best, ref bestRoute, clock, deadline);
    }

    private void Dfs(Node n, float g, List<Edge> stack, HashSet<Node> onStack, Dictionary<Node, float> bestG,
                     ref float best, ref List<Edge> bestRoute, Stopwatch clock, double deadline)
    {
        if (clock.Elapsed.TotalMilliseconds >= deadline) return;

        if (n == goalNode)
        {
            if (g < best - 0.0001f)
            {
                best = g;
                bestRoute = new List<Edge>(stack);
            }
            return;
        }

        if (bestG.TryGetValue(n, out float known) && g >= known - 0.0001f) return;
        bestG[n] = g;
        onStack.Add(n);

        foreach (Edge e in n.Out.OrderBy(x => x.Cost + Flat(x.To.Position, goalNode.Position)))
        {
            if (onStack.Contains(e.To)) continue;
            float g2 = g + e.Cost;
            if (g2 + Flat(e.To.Position, goalNode.Position) >= best) continue;
            if (!IsValid(e)) continue; // lazy: only edges that could improve are checked
            stack.Add(e);
            Dfs(e.To, g2, stack, onStack, bestG, ref best, ref bestRoute, clock, deadline);
            stack.RemoveAt(stack.Count - 1);
        }

        onStack.Remove(n);
    }

    private void AddReplannedPath(Path path)
    {
        paths.Add(path);
        int keep = Mathf.Max(s.maxStoredPaths, initialPathCount + 1);
        while (paths.Count > keep) paths.RemoveAt(initialPathCount); // oldest replanned path
    }

    // =====================================================================  helpers

    /// <summary>
    /// The agent follows the path with a small tolerance (GridAgentMovement's stopping distance), so it can
    /// be a few centimetres off it, inside the safety margin of a wall, where every segment would be
    /// rejected. In that case plan from the closest point of the current path (clear by construction),
    /// or, failing that, from the centre of the agent's cell.
    /// </summary>
    private Vector3 PlanningPosition(Vector3 robot)
    {
        if (checker.IsPositionClear(robot) || CurrentPath == null) return robot;

        int k = Project(CurrentPath, robot);
        Vector3 onPath = ClosestPoint(robot, CurrentPath.Nodes[k].Position, CurrentPath.Nodes[k + 1].Position);
        onPath.y = robot.y;
        if (Flat(onPath, robot) < 0.5f * grid.CellSize && checker.IsPositionClear(onPath)) return onPath;

        Vector3 centre = grid.CellToWorld(grid.WorldToCell(robot));
        centre.y = robot.y;
        return checker.IsPositionValid(centre) ? centre : robot;
    }

    private static Vector3 ClosestPoint(Vector3 p, Vector3 a, Vector3 b)
    {
        float abx = b.x - a.x, abz = b.z - a.z;
        float len2 = abx * abx + abz * abz;
        float t = len2 > 0f ? Mathf.Clamp01(((p.x - a.x) * abx + (p.z - a.z) * abz) / len2) : 0f;
        return new Vector3(a.x + abx * t, p.y, a.z + abz * t);
    }

    private bool IsValid(Edge e)
    {
        int revision = grid.Revision;
        if (e.CheckedRevision != revision)
        {
            e.Valid = checker.IsConnectionValid(e.From.Position, e.To.Position);
            e.CheckedRevision = revision;
            CollisionChecks++;
        }
        return e.Valid;
    }

    private Node NewNode(Vector3 position)
    {
        var n = new Node { Id = nodes.Count, Position = position };
        nodes.Add(n);
        return n;
    }

    private Edge NewEdge(Node from, Node to, bool isConnection, bool knownValid)
    {
        foreach (Edge existing in from.Out)
        {
            if (existing.To != to) continue;
            if (knownValid)
            {
                existing.Valid = true;
                existing.CheckedRevision = grid.Revision;
            }
            return existing;
        }

        var e = new Edge { From = from, To = to, Cost = Flat(from.Position, to.Position), IsConnection = isConnection };
        if (knownValid)
        {
            e.Valid = true;
            e.CheckedRevision = grid.Revision;
        }
        from.Out.Add(e);
        return e;
    }

    // Index of the path edge closest to the robot (ties -> the later edge, i.e. more progress).
    private static int Project(Path path, Vector3 p)
    {
        int bestIndex = 0;
        float bestDistance = float.PositiveInfinity;
        for (int i = 0; i < path.Edges.Count; i++)
        {
            float d = SegmentDistance(p, path.Nodes[i].Position, path.Nodes[i + 1].Position);
            if (d <= bestDistance + 0.000001f)
            {
                bestDistance = Mathf.Min(bestDistance, d);
                bestIndex = i;
            }
        }
        return bestIndex;
    }

    private static float SegmentDistance(Vector3 p, Vector3 a, Vector3 b)
    {
        float abx = b.x - a.x, abz = b.z - a.z;
        float len2 = abx * abx + abz * abz;
        float t = len2 > 0f ? Mathf.Clamp01(((p.x - a.x) * abx + (p.z - a.z) * abz) / len2) : 0f;
        float dx = p.x - (a.x + abx * t), dz = p.z - (a.z + abz * t);
        return Mathf.Sqrt(dx * dx + dz * dz);
    }

    private void AddMiddleCells(List<Vector3> path, float band, HashSet<Vector2Int> cells)
    {
        float total = PathCost(path);
        float from = total * (0.5f - band), to = total * (0.5f + band);
        float spacing = grid.CellSize * 0.25f, travelled = 0f;

        for (int k = 1; k < path.Count; k++)
        {
            float len = Flat(path[k - 1], path[k]);
            for (float d = 0f; d < len; d += spacing)
            {
                float at = travelled + d;
                if (at >= from && at <= to) cells.Add(grid.WorldToCell(Vector3.Lerp(path[k - 1], path[k], d / len)));
            }
            travelled += len;
        }
    }

    private bool AvoidsCells(Vector3 a, Vector3 b, HashSet<Vector2Int> forbidden)
    {
        if (forbidden.Count == 0) return true;
        float len = Flat(a, b);
        int steps = Mathf.Max(1, Mathf.CeilToInt(len / (grid.CellSize * 0.2f)));
        for (int i = 0; i <= steps; i++)
            if (forbidden.Contains(grid.WorldToCell(Vector3.Lerp(a, b, (float)i / steps)))) return false;
        return true;
    }

    private bool TooSimilar(List<Vector3> candidate)
    {
        HashSet<Vector2Int> mine = CellsOf(candidate);
        if (mine.Count == 0) return true;
        foreach (Path p in paths)
        {
            HashSet<Vector2Int> other = CellsOf(p.Waypoints());
            int shared = mine.Count(c => other.Contains(c));
            if ((float)shared / mine.Count >= s.maxOverlap) return true;
        }
        return false;
    }

    private HashSet<Vector2Int> CellsOf(List<Vector3> path)
    {
        var cells = new HashSet<Vector2Int>();
        for (int k = 1; k < path.Count; k++)
        {
            float len = Flat(path[k - 1], path[k]);
            int steps = Mathf.Max(1, Mathf.CeilToInt(len / (grid.CellSize * 0.25f)));
            for (int i = 0; i <= steps; i++) cells.Add(grid.WorldToCell(Vector3.Lerp(path[k - 1], path[k], (float)i / steps)));
        }
        return cells;
    }

    public static float PathCost(List<Vector3> path)
    {
        float total = 0f;
        for (int k = 1; k < path.Count; k++) total += Flat(path[k - 1], path[k]);
        return total;
    }

    private static float Flat(Vector3 a, Vector3 b) => MarsInformedPlanner.Flat(a, b);
}
