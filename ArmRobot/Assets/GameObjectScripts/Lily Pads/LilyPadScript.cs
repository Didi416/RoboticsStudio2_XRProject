using UnityEngine;
using System.Collections.Generic;

public class LilyPadGrid : MonoBehaviour
{
    [Header("Grid Settings")]
    public int gridWidth = 5;
    public int gridHeight = 5;
    public float cellSize = 1f;

    [Header("Prefabs")]
    public GameObject lilyPadPrefab;    // visible lily pad
    public GameObject emptyTilePrefab;  // empty water tile

    [Header("Path Settings")]
    public int minPathLength = 5;
    public int maxPathLength = 12;

    // Stores the generated path as grid coordinates
    public List<Vector2Int> currentPath = new List<Vector2Int>();

    private GameObject[,] tiles = new GameObject[5, 5];

    void Start()
    {
        GenerateGrid();
        GeneratePath();
    }

    void GenerateGrid()
    {
        // Spawn empty tiles for the whole grid
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                Vector3 pos = new Vector3(x * cellSize, 0, y * cellSize) + transform.position;
                tiles[x, y] = Instantiate(emptyTilePrefab, pos, Quaternion.identity, transform);
            }
        }
    }

    void GeneratePath()
    {
        currentPath.Clear();

        // Start from a random cell on the left column
        Vector2Int current = new Vector2Int(0, Random.Range(0, gridHeight));
        currentPath.Add(current);

        int pathLength = Random.Range(minPathLength, maxPathLength);

        for (int i = 0; i < pathLength; i++)
        {
            List<Vector2Int> neighbours = GetValidNeighbours(current);
            if (neighbours.Count == 0) break;

            current = neighbours[Random.Range(0, neighbours.Count)];
            currentPath.Add(current);
        }

        // Show lily pads on path tiles
        foreach (Vector2Int cell in currentPath)
        {
            Destroy(tiles[cell.x, cell.y]);
            Vector3 pos = new Vector3(cell.x * cellSize, 0, cell.y * cellSize) + transform.position;
            tiles[cell.x, cell.y] = Instantiate(lilyPadPrefab, pos, Quaternion.identity, transform);
        }

        Debug.Log($"Path generated with {currentPath.Count} steps");
    }

    List<Vector2Int> GetValidNeighbours(Vector2Int cell)
    {
        List<Vector2Int> neighbours = new List<Vector2Int>();
        Vector2Int[] directions = {
            Vector2Int.up, Vector2Int.down,
            Vector2Int.left, Vector2Int.right
        };

        foreach (var dir in directions)
        {
            Vector2Int next = cell + dir;
            // Stay within grid and don't revisit
            if (next.x >= 0 && next.x < gridWidth &&
                next.y >= 0 && next.y < gridHeight &&
                !currentPath.Contains(next))
            {
                neighbours.Add(next);
            }
        }
        return neighbours;
    }
}