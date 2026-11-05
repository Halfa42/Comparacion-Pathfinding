using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(GridAgentMovement))]
public class PlayerAStarController : MonoBehaviour
{
    [Header("Referencias")]
    public MazeGridGenerator maze;
    
    private GridAgentMovement movement;
    private Vector2Int currentGridPos;
    private Vector2Int currentGoal;
    private System.Random scenarioRandom;
    private bool navigationStarted;

    public bool RestartNavigation(int randomSeed)
    {
        CancelInvoke();
        enabled = false;
        movement = GetComponent<GridAgentMovement>();
        if (movement != null) movement.Stop();
        if (maze == null) maze = FindObjectOfType<MazeGridGenerator>();
        if (movement == null || maze == null) return false;
        currentGridPos = maze.WorldToGridPositionUnclamped(transform.position);
        if (!maze.IsWalkable(currentGridPos.x, currentGridPos.y)) return false;
        currentGoal = currentGridPos;
        scenarioRandom = new System.Random(randomSeed);
        navigationStarted = true;
        enabled = true;
        SetNewRandomGoal();
        return true;
    }

    private void Start()
    {
        if (navigationStarted) return;
        navigationStarted = true;
        movement = GetComponent<GridAgentMovement>();
        
        // Autodetectar el laberinto si no se asignó en el inspector
        if (maze == null) 
            maze = FindObjectOfType<MazeGridGenerator>();

        // Obtener la posición inicial del jugador en la grilla
        currentGridPos = maze.WorldToGridPosition(transform.position); 
        
        // Iniciar el ciclo de búsqueda
        SetNewRandomGoal();
    }

    private void Update()
    {
        // El "Trigger": GridAgentMovement pone HasPath en false al llegar
        if (!movement.HasPath)
        {
            // --- LOG DE LLEGADA ---
            Debug.Log($"[Agente A*] ¡Llegué al objetivo en la celda {currentGoal}!");
            
            // Actualizamos la posición actual a la meta que acabamos de alcanzar
            currentGridPos = currentGoal;
            
            // Asignar una nueva meta aleatoria
            SetNewRandomGoal();
        }
    }

    private void SetNewRandomGoal()
    {
        currentGoal = GetRandomWalkableCell();
        
        // Calcular ruta con A*
        List<Vector2Int> gridPath = FindPathAStar(currentGridPos, currentGoal);
        
        if (gridPath != null && gridPath.Count > 0)
        {
            // --- LOG DE NUEVO OBJETIVO ---
            Debug.Log($"[Agente A*] Nuevo objetivo asignado: {currentGoal}. Distancia de ruta: {gridPath.Count} pasos.");

            List<Vector3> worldPath = new List<Vector3>();
            
            // Convertir la ruta 2D a posiciones 3D para el movimiento
            foreach (Vector2Int p in gridPath)
            {
                Vector3 wPos = maze.GridToWorldPosition(p.x, p.y);
                wPos.y = transform.position.y; // Mantener la altura original de la cápsula
                worldPath.Add(wPos);
            }
            
            // Enviarle la ruta al motor de movimiento
            movement.SetPath(worldPath);
        }
        else
        {
            // Failsafe: Si eligió una casilla aislada sin camino, intenta de nuevo
            Invoke(nameof(SetNewRandomGoal), 0.1f);
        }
    }

    private Vector2Int GetRandomWalkableCell()
    {
        int maxAttempts = 100;
        for (int i = 0; i < maxAttempts; i++)
        {
            int rx = scenarioRandom != null ? scenarioRandom.Next(maze.Width) : Random.Range(0, maze.Width);
            int ry = scenarioRandom != null ? scenarioRandom.Next(maze.Height) : Random.Range(0, maze.Height);
            
            // Validar que sea transitable y no sea la casilla donde ya estamos
            if (maze.IsWalkable(rx, ry) && (rx != currentGridPos.x || ry != currentGridPos.y))
            {
                return new Vector2Int(rx, ry);
            }
        }
        return currentGridPos;
    }

    // ==========================================
    // ALGORITMO A* CLÁSICO (Búsqueda en Grilla 2D)
    // ==========================================
    private class Node
    {
        public Vector2Int Position;
        public int G; 
        public int H; 
        public int F => G + H; 
        public Node Parent;
    }

    private List<Vector2Int> FindPathAStar(Vector2Int start, Vector2Int goal)
    {
        List<Node> openList = new List<Node>();
        HashSet<Vector2Int> closedSet = new HashSet<Vector2Int>();

        openList.Add(new Node { Position = start, G = 0, H = GetManhattan(start, goal) });

        Vector2Int[] dirs = { Vector2Int.up, Vector2Int.down, Vector2Int.left, Vector2Int.right };

        while (openList.Count > 0)
        {
            openList.Sort((a, b) => {
                int cmp = a.F.CompareTo(b.F);
                if (cmp == 0) return a.H.CompareTo(b.H);
                return cmp;
            });

            Node current = openList[0];
            openList.RemoveAt(0);

            if (current.Position == goal)
            {
                return ReconstructPath(current);
            }

            closedSet.Add(current.Position);

            foreach (var d in dirs)
            {
                Vector2Int next = current.Position + d;
                
                if (next.x < 0 || next.x >= maze.Width || next.y < 0 || next.y >= maze.Height) continue;
                if (!maze.IsWalkable(next.x, next.y)) continue;
                if (closedSet.Contains(next)) continue;

                int newG = current.G + 1;
                Node existing = openList.Find(n => n.Position == next);

                if (existing != null)
                {
                    if (newG < existing.G)
                    {
                        existing.G = newG;
                        existing.Parent = current;
                    }
                }
                else
                {
                    openList.Add(new Node {
                        Position = next,
                        G = newG,
                        H = GetManhattan(next, goal),
                        Parent = current
                    });
                }
            }
        }
        return null;
    }

    private int GetManhattan(Vector2Int a, Vector2Int b)
    {
        return Mathf.Abs(a.x - b.x) + Mathf.Abs(a.y - b.y);
    }

    private List<Vector2Int> ReconstructPath(Node node)
    {
        List<Vector2Int> path = new List<Vector2Int>();
        while (node != null)
        {
            path.Add(node.Position);
            node = node.Parent;
        }
        path.Reverse();
        
        if (path.Count > 0) path.RemoveAt(0); 
        
        return path;
    }
}
