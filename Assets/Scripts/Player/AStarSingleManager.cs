using UnityEngine;

public class AStarSingleManager : MonoBehaviour
{
    [Header("Referencias")]
    public MazeGridGenerator mazeGenerator;
    public GameObject playerPrefab; // Tu prefab de la cápsula azul

    void Start()
    {
        // Autodetectar el laberinto si no se asignó en el inspector
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
        spawnPos.y = 0.5f; // Ajustar altura para que toque el suelo

        // Instanciar el único jugador
        GameObject player = Instantiate(playerPrefab, spawnPos, Quaternion.identity);
        player.name = "Player_AStar_Unico";
        
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