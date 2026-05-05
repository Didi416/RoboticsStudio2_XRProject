// using UnityEngine;
// using System.Collections.Generic;

// public class LilyPadGrid : MonoBehaviour
// {
//     [Header("Grid Settings")]
//     public int gridWidth = 5;
//     public int gridHeight = 5;
//     public float cellSize = 1f;

//     [Header("Prefabs")]
//     public GameObject lilyPadPrefab;    // visible lily pad
//     public GameObject emptyTilePrefab;  // empty water tile

//     [Header("Path Settings")]
//     public int minPathLength = 5;
//     public int maxPathLength = 12;

//     // Stores the generated path as grid coordinates
//     public List<Vector2Int> currentPath = new List<Vector2Int>();

//     private GameObject[,] tiles = new GameObject[5, 5];

//     void Start()
//     {
//         GenerateGrid();
//         GeneratePath();
//     }

//     void GenerateGrid()
//     {
//         for (int x = 0; x < gridWidth; x++)
//         {
//             for (int y = 0; y < gridHeight; y++)
//             {
//                 // Uses the parent object's rotation so tiles align correctly
//                 Vector3 localPos = new Vector3(x * cellSize, 0, y * cellSize);
//                 Vector3 worldPos = transform.TransformPoint(localPos);
//                 tiles[x, y] = Instantiate(emptyTilePrefab, worldPos, transform.rotation, transform);
//             }
//         }
//     }

//     void GeneratePath()
//     {
//         currentPath.Clear();

//         // Start from a random cell on the left column
//         Vector2Int current = new Vector2Int(0, Random.Range(0, gridHeight));
//         currentPath.Add(current);

//         int pathLength = Random.Range(minPathLength, maxPathLength);

//         for (int i = 0; i < pathLength; i++)
//         {
//             List<Vector2Int> neighbours = GetValidNeighbours(current);
//             if (neighbours.Count == 0) break;

//             current = neighbours[Random.Range(0, neighbours.Count)];
//             currentPath.Add(current);
//         }

//         // Show lily pads on path tiles
//         foreach (Vector2Int cell in currentPath)
//         {
//             Destroy(tiles[cell.x, cell.y]);
//             Vector3 localPos = new Vector3(cell.x * cellSize, 0, cell.y * cellSize);
//             Vector3 worldPos = transform.TransformPoint(localPos);
//             tiles[cell.x, cell.y] = Instantiate(lilyPadPrefab, worldPos, transform.rotation, transform);
//         }

//         Debug.Log($"Path generated with {currentPath.Count} steps");
//     }

//     List<Vector2Int> GetValidNeighbours(Vector2Int cell)
//     {
//         List<Vector2Int> neighbours = new List<Vector2Int>();
//         Vector2Int[] directions = {
//             Vector2Int.up, Vector2Int.down,
//             Vector2Int.left, Vector2Int.right
//         };

//         foreach (var dir in directions)
//         {
//             Vector2Int next = cell + dir;
//             // Stay within grid and don't revisit
//             if (next.x >= 0 && next.x < gridWidth &&
//                 next.y >= 0 && next.y < gridHeight &&
//                 !currentPath.Contains(next))
//             {
//                 neighbours.Add(next);
//             }
//         }
//         return neighbours;
//     }
// }

using UnityEngine;
using System.Collections.Generic;

public class LilyPadGrid : MonoBehaviour
{
    [Header("Grid Settings")]
    public int gridWidth = 5;
    public int gridHeight = 5;
    public float cellSize = 1f;

    [Header("Prefabs")]
    public GameObject lilyPadPrefab;
    public GameObject emptyTilePrefab;

    public List<Vector2Int> currentPath = new List<Vector2Int>();
    private GameObject[,] tiles = new GameObject[5, 5];

    // Fixed start - top left
    private Vector2Int startCell = new Vector2Int(0, 0);

    // Four possible end points
    private Vector2Int[] possibleEndCells = new Vector2Int[]
    {
        new Vector2Int(1, 4), // 2nd column bottom row
        new Vector2Int(3, 4), // 4th column bottom row
        new Vector2Int(4, 3), // 2nd row last column
        new Vector2Int(4, 1)  // 4th row last column
    };

    void Start()
    {
        GenerateGrid();
        GeneratePath();
    }

    void GenerateGrid()
    {
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                Vector3 localPos = new Vector3(x * cellSize, 0, y * cellSize);
                Vector3 worldPos = transform.TransformPoint(localPos);
                tiles[x, y] = Instantiate(emptyTilePrefab, worldPos, transform.rotation, transform);
            }
        }
    }

    void GeneratePath()
    {
        // Pick a random end point
        Vector2Int endCell = possibleEndCells[Random.Range(0, possibleEndCells.Length)];
        Debug.Log($"Path: start {startCell} → end {endCell}");

        // Try to generate a valid path up to 100 attempts
        for (int attempt = 0; attempt < 100; attempt++)
        {
            List<Vector2Int> path = TryGeneratePath(startCell, endCell);
            if (path != null)
            {
                currentPath = path;
                SpawnLilyPads();
                return;
            }
        }

        Debug.LogError("Could not generate a valid path after 100 attempts!");
    }

    List<Vector2Int> TryGeneratePath(Vector2Int start, Vector2Int end)
    {
        List<Vector2Int> path = new List<Vector2Int>();
        HashSet<Vector2Int> visited = new HashSet<Vector2Int>();

        Vector2Int current = start;
        path.Add(current);
        visited.Add(current);

        int maxSteps = 50;

        for (int step = 0; step < maxSteps; step++)
        {
            // If we reached the end, success!
            if (current == end)
            {
                Debug.Log($"Path generated with {path.Count} steps");
                return path;
            }

            List<Vector2Int> neighbours = GetValidNeighbours(current, visited);

            if (neighbours.Count == 0)
                return null; // dead end, try again

            // Bias towards the end point
            neighbours.Sort((a, b) =>
            {
                int distA = Mathf.Abs(a.x - end.x) + Mathf.Abs(a.y - end.y);
                int distB = Mathf.Abs(b.x - end.x) + Mathf.Abs(b.y - end.y);
                return distA.CompareTo(distB);
            });

            // 70% chance to move towards end, 30% chance to wander
            Vector2Int next;
            if (Random.value < 0.7f)
                next = neighbours[0]; // closest to end
            else
                next = neighbours[Random.Range(0, neighbours.Count)]; // random

            current = next;
            path.Add(current);
            visited.Add(current);
        }

        return null; // exceeded max steps
    }

    List<Vector2Int> GetValidNeighbours(Vector2Int cell, HashSet<Vector2Int> visited)
    {
        List<Vector2Int> neighbours = new List<Vector2Int>();
        Vector2Int[] directions = {
            Vector2Int.up, Vector2Int.down,
            Vector2Int.left, Vector2Int.right
        };

        foreach (var dir in directions)
        {
            Vector2Int next = cell + dir;
            if (next.x >= 0 && next.x < gridWidth &&
                next.y >= 0 && next.y < gridHeight &&
                !visited.Contains(next))
            {
                neighbours.Add(next);
            }
        }
        return neighbours;
    }

    void SpawnLilyPads()
    {
        foreach (Vector2Int cell in currentPath)
        {
            Destroy(tiles[cell.x, cell.y]);
            Vector3 localPos = new Vector3(cell.x * cellSize, 0, cell.y * cellSize);
            Vector3 worldPos = transform.TransformPoint(localPos);
            tiles[cell.x, cell.y] = Instantiate(lilyPadPrefab, worldPos, transform.rotation, transform);
        }
    }
}