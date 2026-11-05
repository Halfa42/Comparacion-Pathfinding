using UnityEngine;

public class AStarSingleManager : MonoBehaviour
{
    [Header("Referencias")]
    public MazeGridGenerator mazeGenerator;
    public GameObject playerPrefab; // Tu prefab de la cápsula azul

    // CAMBIO AQUÍ: Regresamos a Start() para esperar a que el laberinto se construya primero
    void Start()
    {
        if (mazeGenerator == null)
            mazeGenerator = FindObjectOfType<MazeGridGenerator>();

        SpawnSinglePlayer();
    }

    private void SpawnSinglePlayer()
    {
        int rx = 0, ry = 0;
        int attempts = 0;
        
        // Buscar una posición inicial aleatoria transitable
        do
        {
            rx = Random.Range(0, mazeGenerator.Width);
            ry = Random.Range(0, mazeGenerator.Height);
            attempts++;
        } 
        while (!mazeGenerator.IsWalkable(rx, ry) && attempts < 100);

        // Convertir coordenada de grilla a posición 3D real
        Vector3 spawnPos = mazeGenerator.GridToWorldPosition(rx, ry);
        spawnPos.y = 0.5f; 

        // Instanciar el único jugador
        GameObject player = Instantiate(playerPrefab, spawnPos, Quaternion.identity);
        player.name = "Player_AStar_Unico";
        
        // Mantenemos la etiqueta para que el CBSVisualizer lo encuentre
        player.tag = "Player"; 
        
        // Inyectarle la referencia del laberinto al controlador
        if (player.TryGetComponent(out PlayerAStarController controller))
        {
            controller.maze = mazeGenerator;
        }
        else
        {
            Debug.LogWarning("El prefab no tiene el script PlayerAStarController asignado.");
        }
    }
}