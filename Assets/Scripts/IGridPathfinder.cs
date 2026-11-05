using System.Collections.Generic;
using UnityEngine;

// Contract for planning only. Movement, capture, repetition and CSV belong to common components.
public interface IGridPathfinder
{
    bool BindGrid(MazeNavigationGrid grid);
    bool TryGetPath(Vector3 start, Vector3 goal, out List<Vector3> path);
    // Counters for the last query; -1 means that the planner does not expose a node count.
    long SearchNodes { get; }
    long SearchPeakMemory { get; }
}
