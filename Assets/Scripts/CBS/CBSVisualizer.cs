using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CBSVisualizer : MonoBehaviour
{
    [Header("Referencias")]
    public CBSManager cbsManager;
    public MazeGridGenerator mazeGenerator;
    public Transform playerTransform; 
    public GameObject agentPrefab;

    [Header("Configuración")]
    [Range(1, 11)] public int numberOfAgents = 1;
    [Tooltip("Cada cuántos segundos recalculan la ruta los agentes hacia el jugador")]
    public float recalculateInterval = 1.0f; 

    private List<GridAgentMovement> activeAgents = new List<GridAgentMovement>();
    private int[,] grid;

    void Start()
    {
        if (mazeGenerator == null) mazeGenerator = FindObjectOfType<MazeGridGenerator>();
        if (cbsManager == null) cbsManager = GetComponent<CBSManager>();

        // 1. Extraer la matriz de la grilla
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

        // 2. Instanciar los agentes físicos al inicio (aún sin ruta)
        for (int i = 0; i < numberOfAgents; i++)
        {
            Vector2Int startPos = new Vector2Int(13 + i, 13);
            Vector3 worldPos = mazeGenerator.GridToWorldPosition(startPos.x, startPos.y);
            worldPos.y = 0.5f;

            GameObject agentObj = Instantiate(agentPrefab, worldPos, Quaternion.identity);
            agentObj.name = "Agente_CBS_" + i;
            
            if (agentObj.TryGetComponent(out GridAgentMovement movement))
            {
                activeAgents.Add(movement);
            }
            else
            {
                Debug.LogWarning($"El prefab {agentPrefab.name} no tiene GridAgentMovement.");
            }
        }

        // 3. Iniciar el bucle de persecución dinámica
        StartCoroutine(ChasePlayerRoutine());
    }

    IEnumerator ChasePlayerRoutine()
    {
        // Esperar 0.5s para asegurar que AStarSingleManager ya instanció al jugador
        yield return new WaitForSeconds(0.5f);

        while (true)
        {
            // Buscar al jugador dinámico por su nombre exacto si aún no lo tenemos
            if (playerTransform == null)
            {
                GameObject playerObj = GameObject.Find("Player_AStar_Unico");
                if (playerObj != null) playerTransform = playerObj.transform;
            }

            // Si el jugador existe, recalcular rutas hacia su posición actual
            if (playerTransform != null)
            {
                RecalculatePaths();
            }

            // Esperar el intervalo indicado antes de volver a recalcular (para no saturar el CPU)
            yield return new WaitForSeconds(recalculateInterval);
        }
    }

    void RecalculatePaths()
    {
        List<Vector2Int> currentStarts = new List<Vector2Int>();
        
        // Registrar desde qué celda va a salir cada agente en este instante
        foreach (var agent in activeAgents)
        {
            currentStarts.Add(mazeGenerator.WorldToGridPosition(agent.transform.position));
        }

        // Registrar en qué celda está pisando el jugador ahora mismo
        Vector2Int playerGoal = mazeGenerator.WorldToGridPosition(playerTransform.position);

        // Ejecutar el CBS centralizado
        var paths = cbsManager.FindPaths(grid, currentStarts, playerGoal);

        // Si encontró una solución sin choques, actualizar la ruta de todos los modelos 3D
        if (paths != null && cbsManager.Success)
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
                // Enviar la nueva ruta fresca al componente de movimiento
                activeAgents[i].SetPath(worldPath);
            }
        }
    }
}