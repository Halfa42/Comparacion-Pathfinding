using System.Collections.Generic;
using UnityEngine;

public class CBSVisualizer : MonoBehaviour
{
    [Header("Referencias")]
    public CBSManager cbsManager;
    public MazeGridGenerator mazeGenerator;
    public GameObject agentPrefab;

    [Header("Configuración")]
    [Range(1, 11)] public int numberOfAgents = 10;

    void Start()
    {
        // 1. Extraer la matriz del MazeGridGenerator para que CBS la entienda
        int width = mazeGenerator.Width;
        int height = mazeGenerator.Height;
        int[,] grid = new int[height, width];

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                // 0 = Transitable, 1 = Pared
                grid[y, x] = mazeGenerator.IsWalkable(x, y) ? 0 : 1; 
            }
        }

        // 2. Definir los inicios y la meta (Igual que en tus benchmarks)
        List<Vector2Int> starts = new List<Vector2Int>();
        List<Vector2Int> goals = new List<Vector2Int>();
        for (int i = 0; i < numberOfAgents; i++)
        {
            starts.Add(new Vector2Int(13 + i, 13));
            goals.Add(new Vector2Int(21, 0)); // El cuello de botella
        }

        // 3. Ejecutar CBS
        var paths = cbsManager.FindPaths(grid, starts, goals);

        // 4. Procesar resultados e instanciar agentes visuales
        if (paths != null && cbsManager.Success)
        {
            Debug.Log("¡Rutas encontradas! Instanciando agentes...");

            for (int i = 0; i < paths.Count; i++)
            {
                List<Vector3> worldPath = new List<Vector3>();
                
                // Convertir coordenadas de grilla a posiciones reales en 3D
                foreach (Vector2Int p in paths[i])
                {
                    Vector3 worldPos = mazeGenerator.GridToWorldPosition(p.x, p.y);
                    worldPos.y = 0.5f; // Elevar ligeramente para que la cápsula toque el suelo
                    worldPath.Add(worldPos);
                }

                // Instanciar el prefab en el primer punto de la ruta
                GameObject agent = Instantiate(agentPrefab, worldPath[0], Quaternion.identity);
                agent.name = "Agente_CBS_" + i;

                // Asignarle la ruta al componente GridAgentMovement
                if (agent.TryGetComponent(out GridAgentMovement movement))
                {
                    movement.SetPath(worldPath);
                }
                else
                {
                    Debug.LogWarning("El prefab no tiene el script GridAgentMovement adjunto.");
                }
            }
        }
        else
        {
            Debug.LogError("CBS falló: " + cbsManager.FailureReason);
        }
    }
}