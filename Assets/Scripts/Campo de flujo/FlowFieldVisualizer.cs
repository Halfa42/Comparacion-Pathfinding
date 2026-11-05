using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FlowFieldVisualizer : MonoBehaviour
{
    [Header("Referencias")]
    public FlowFieldManager flowFieldManager;
    public MazeGridGenerator mazeGenerator;
    public Transform playerTransform; 
    public GameObject agentPrefab;

    [Header("Configuración")]
    [Range(1, 50)] public int numberOfAgents = 10;
    [Tooltip("Cada cuántos segundos recalculan la ruta los agentes hacia el jugador")]
    public float recalculateInterval = 0.5f; // Flow Field es tan rápido que podemos recalcular más seguido

    private List<GridAgentMovement> activeAgents = new List<GridAgentMovement>();
    private int[,] grid;

    void Start()
    {
        if (mazeGenerator == null) mazeGenerator = FindObjectOfType<MazeGridGenerator>();
        if (flowFieldManager == null) flowFieldManager = GetComponent<FlowFieldManager>();

        int width = mazeGenerator.Width;
        int height = mazeGenerator.Height;
        grid = new int[height, width];
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                grid[y, x] = mazeGenerator.IsWalkable(x, y) ? 0 : 1; 
            }
        }

        // Instanciar los agentes
        for (int i = 0; i < numberOfAgents; i++)
        {
            // Inician en la zona libre (x=13..23) de la fila 13
            Vector2Int startPos = new Vector2Int(13 + i, 13); 
            Vector3 worldPos = mazeGenerator.GridToWorldPosition(startPos.x, startPos.y);
            worldPos.y = 0.5f;

            GameObject agentObj = Instantiate(agentPrefab, worldPos, Quaternion.identity);
            agentObj.name = "Agente_FlowField_" + i;
            
            if (agentObj.TryGetComponent(out GridAgentMovement movement))
            {
                activeAgents.Add(movement);
            }
        }

        StartCoroutine(ChasePlayerRoutine());
    }

    IEnumerator ChasePlayerRoutine()
    {
        yield return new WaitForSeconds(0.5f);

        while (true)
        {
            if (playerTransform == null)
            {
                GameObject playerObj = GameObject.Find("Player_AStar_Unico");
                if (playerObj != null) playerTransform = playerObj.transform;
            }

            if (playerTransform != null)
            {
                RecalculatePaths();
            }

            yield return new WaitForSeconds(recalculateInterval);
        }
    }

    void RecalculatePaths()
    {
        List<Vector2Int> currentStarts = new List<Vector2Int>();
        
        foreach (var agent in activeAgents)
        {
            currentStarts.Add(mazeGenerator.WorldToGridPosition(agent.transform.position));
        }

        Vector2Int playerGoal = mazeGenerator.WorldToGridPosition(playerTransform.position);

        // Usamos la nueva sobrecarga de FlowFieldManager
        var paths = flowFieldManager.FindPaths(grid, currentStarts, playerGoal);

        if (paths != null && flowFieldManager.Success)
        {
            for (int i = 0; i < paths.Count; i++)
            {
                List<Vector3> worldPath = new List<Vector3>();
                foreach (Vector2Int p in paths[i])
                {
                    Vector3 worldPos = mazeGenerator.GridToWorldPosition(p.x, p.y);
                    worldPos.y = 0.5f;
                    worldPath.Add(worldPos);
                }
                activeAgents[i].SetPath(worldPath);
            }
        }
    }
}