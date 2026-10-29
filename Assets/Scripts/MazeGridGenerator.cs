using UnityEngine;

public class MazeGridGenerator : MonoBehaviour
{
    [Header("Prefabs de Entorno")]
    [SerializeField] private GameObject wallPrefab;
    [SerializeField] private GameObject floorPrefab;

    [Header("Configuración de Grilla")]
    [SerializeField] private float cellSize = 1.0f;

    // Matriz 2D determinista (0 = Transitable, 1 = Pared)
    // Diseñada específicamente para benchs de MAPF:
    // Incluye pasillos estrechos, múltiples rutas alternativas y zonas abiertas.
    private readonly int[,] defaultMazeLayout = new int[,]
    {
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1},
        {1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1},
        {1,0,1,1,1,0,1,0,1,1,1,1,1,1,0,1,0,1,1,1,1,1,1,0,1},
        {1,0,1,0,0,0,1,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,1,0,1},
        {1,0,1,0,1,1,1,0,1,0,1,1,0,1,0,1,1,1,1,1,1,0,1,0,1},
        {1,0,0,0,0,0,0,0,1,0,1,1,0,0,0,0,0,0,0,0,1,0,0,0,1},
        {1,1,1,1,1,1,0,1,1,0,1,1,1,1,1,1,1,1,1,0,1,1,0,1,1},
        {1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1},
        {1,0,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,0,1},
        {1,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1},
        {1,0,1,0,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1,1,0,1,0,1},
        {1,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,1},
        {1,1,1,0,1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,0,1,0,1,1,1},
        {1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
    };

    // Estructura de datos pública expuesta para los algoritmos
    public bool[,] WalkableGrid { get; private set; }
    public int Width { get; private set; }
    public int Height { get; private set; }
    public float CellSize => cellSize;

    private void Awake()
    {
        BuildEnvironment();
    }

    public void BuildEnvironment()
    {
        // Limpiar elementos existentes si se ejecuta en editor
        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            DestroyImmediate(transform.GetChild(i).gameObject);
        }

        Height = defaultMazeLayout.GetLength(0);
        Width = defaultMazeLayout.GetLength(1);
        WalkableGrid = new bool[Width, Height];

        for (int y = 0; y < Height; y++)
        {
            for (int x = 0; x < Width; x++)
            {
                bool isWall = defaultMazeLayout[y, x] == 1;
                WalkableGrid[x, y] = !isWall;

                Vector3 cellPos = GridToWorldPosition(x, y);

                // Instanciar Suelo
                if (floorPrefab != null)
                {
                    GameObject floor = Instantiate(floorPrefab, cellPos, Quaternion.identity, transform);
                    floor.name = $"Floor_{x}_{y}";
                }

                // Instanciar Pared
                if (isWall && wallPrefab != null)
                {
                    GameObject wall = Instantiate(wallPrefab, cellPos + new Vector3(0, 0.5f, 0), Quaternion.identity, transform);
                    wall.name = $"Wall_{x}_{y}";
                }
            }
        }
    }

    // Conversiones de coordenadas Grilla <-> Mundo
    public Vector3 GridToWorldPosition(int x, int y)
    {
        return transform.position + new Vector3(x * cellSize, 0, y * cellSize);
    }

    public Vector2Int WorldToGridPosition(Vector3 worldPosition)
    {
        Vector3 localPos = worldPosition - transform.position;
        int x = Mathf.Clamp(Mathf.RoundToInt(localPos.x / cellSize), 0, Width - 1);
        int y = Mathf.Clamp(Mathf.RoundToInt(localPos.z / cellSize), 0, Height - 1);
        return new Vector2Int(x, y);
    }

    public bool IsWalkable(int x, int y)
    {
        if (x < 0 || x >= Width || y < 0 || y >= Height) return false;
        return WalkableGrid[x, y];
    }
}