
// using UnityEngine;
// using System.Collections.Generic;

// public class LilyPadGrid : MonoBehaviour
// {
//     [Header("Grid Settings")]
//     public int gridWidth = 4;
//     public int gridHeight = 4;
//     public float cellSize = 1f;

//     [Header("Lily Pad Prefab")]
//     public GameObject lilyPadPrefab;

//     [Header("Grid Line Settings")]
//     public Material lineMaterial;
//     public Color lineColor = Color.white;
//     public float lineWidth = 0.02f;
//     public float lineHeightOffset = 0.01f;

//     [Header("Path Settings")]
//     public int minPathLength = 5;
//     public int maxPathLength = 12;

//     public List<Vector2Int> currentPath = new List<Vector2Int>();

//     private GameObject[,] lilyPads;
//     private Vector2Int startCell = new Vector2Int(0, 0);
//     private Vector2Int[] possibleEndCells = new Vector2Int[]
//     {
//         new Vector2Int(1, 4),
//         new Vector2Int(3, 4),
//         new Vector2Int(4, 1),
//         new Vector2Int(4, 3)
//     };

//     void Start()
//     {
//         lilyPads = new GameObject[gridWidth + 1, gridHeight + 1];
//         DrawGrid();
//         GeneratePath();
//     }

//     // ─────────────────────────────────────────
//     // GRID LINES
//     // ─────────────────────────────────────────

//     void DrawGrid()
//     {
//         // Horizontal lines
//         for (int y = 0; y <= gridHeight; y++)
//         {
//             Vector3 start = transform.TransformPoint(new Vector3(0, lineHeightOffset, y * cellSize));
//             Vector3 end = transform.TransformPoint(new Vector3(gridWidth * cellSize, lineHeightOffset, y * cellSize));
//             CreateLine($"HLine_{y}", start, end);
//         }

//         // Vertical lines
//         for (int x = 0; x <= gridWidth; x++)
//         {
//             Vector3 start = transform.TransformPoint(new Vector3(x * cellSize, lineHeightOffset, 0));
//             Vector3 end = transform.TransformPoint(new Vector3(x * cellSize, lineHeightOffset, gridHeight * cellSize));
//             CreateLine($"VLine_{x}", start, end);
//         }
//     }

//     void CreateLine(string lineName, Vector3 start, Vector3 end)
//     {
//         GameObject lineObj = new GameObject(lineName);
//         lineObj.transform.parent = transform;

//         LineRenderer lr = lineObj.AddComponent<LineRenderer>();

//         lr.positionCount = 2;
//         lr.SetPosition(0, start);
//         lr.SetPosition(1, end);

//         lr.startWidth = lineWidth;
//         lr.endWidth = lineWidth;

//         lr.material = lineMaterial != null
//             ? lineMaterial
//             : new Material(Shader.Find("Sprites/Default"));

//         lr.startColor = lineColor;
//         lr.endColor = lineColor;

//         lr.useWorldSpace = true;
//         lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
//         lr.receiveShadows = false;
//     }

//     // ─────────────────────────────────────────
//     // PATH GENERATION
//     // ─────────────────────────────────────────

//     void GeneratePath()
//     {
//         Vector2Int endCell = possibleEndCells[Random.Range(0, possibleEndCells.Length)];
//         Debug.Log($"Path: start {startCell} → end {endCell}");

//         for (int attempt = 0; attempt < 100; attempt++)
//         {
//             List<Vector2Int> path = TryGeneratePath(startCell, endCell);
//             if (path != null)
//             {
//                 currentPath = path;
//                 SpawnLilyPads();
//                 return;
//             }
//         }

//         Debug.LogError("Could not generate a valid path after 100 attempts!");
//     }

//     List<Vector2Int> TryGeneratePath(Vector2Int start, Vector2Int end)
//     {
//         List<Vector2Int> path = new List<Vector2Int>();
//         HashSet<Vector2Int> visited = new HashSet<Vector2Int>();

//         Vector2Int current = start;
//         path.Add(current);
//         visited.Add(current);

//         int maxSteps = 50;

//         for (int step = 0; step < maxSteps; step++)
//         {
//             if (current == end)
//             {
//                 Debug.Log($"Path generated with {path.Count} steps");
//                 return path;
//             }

//             List<Vector2Int> neighbours = GetValidNeighbours(current, visited);
//             if (neighbours.Count == 0) return null;

//             neighbours.Sort((a, b) =>
//             {
//                 int distA = Mathf.Abs(a.x - end.x) + Mathf.Abs(a.y - end.y);
//                 int distB = Mathf.Abs(b.x - end.x) + Mathf.Abs(b.y - end.y);
//                 return distA.CompareTo(distB);
//             });

//             Vector2Int next = Random.value < 0.7f
//                 ? neighbours[0]
//                 : neighbours[Random.Range(0, neighbours.Count)];

//             current = next;
//             path.Add(current);
//             visited.Add(current);
//         }

//         return null;
//     }

//     List<Vector2Int> GetValidNeighbours(Vector2Int cell, HashSet<Vector2Int> visited)
//     {
//         List<Vector2Int> neighbours = new List<Vector2Int>();
//         Vector2Int[] directions = {
//             Vector2Int.up, Vector2Int.down,
//             Vector2Int.left, Vector2Int.right
//         };

//         foreach (var dir in directions)
//         {
//             Vector2Int next = cell + dir;
//             if (next.x >= 0 && next.x <= gridWidth &&
//                 next.y >= 0 && next.y <= gridHeight &&
//                 !visited.Contains(next))
//             {
//                 neighbours.Add(next);
//             }
//         }
//         return neighbours;
//     }

//     // ─────────────────────────────────────────
//     // LILY PAD SPAWNING
//     // ─────────────────────────────────────────

//     void SpawnLilyPads()
//     {
//         foreach (Vector2Int cell in currentPath)
//         {
//             if (lilyPads[cell.x, cell.y] != null)
//                 Destroy(lilyPads[cell.x, cell.y]);

//             Vector3 localPos = new Vector3(cell.x * cellSize, 0, cell.y * cellSize);
//             Vector3 worldPos = transform.TransformPoint(localPos);
//             lilyPads[cell.x, cell.y] = Instantiate(lilyPadPrefab, worldPos, transform.rotation, transform);
//         }
//     }
// }

using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class LilyPadGrid : MonoBehaviour
{
    [Header("Grid Settings")]
    public int gridWidth = 4;
    public int gridHeight = 4;
    public float cellSize = 1f;

    [Header("Lily Pad Prefab")]
    public GameObject lilyPadPrefab;

    [Header("Grid Line Settings")]
    public Material lineMaterial;
    public Color lineColor = Color.white;
    public float lineWidth = 0.02f;
    public float lineHeightOffset = 0.01f;

    [Header("Path Settings")]
    public int minPathLength = 5;
    public int maxPathLength = 12;

    [Header("Highlight Settings")]
    public Color defaultColor = Color.green;       // normal lily pad color
    public Color highlightColor = Color.yellow;    // currently highlighted pad
    public Color completedColor = Color.cyan;      // already shown pads
    public float highlightDuration = 0.6f;         // how long each pad glows
    public float delayBetweenPads = 0.3f;          // gap between each highlight
    public bool autoPlaySequence = true;           // play sequence on start
    public KeyCode replayKey = KeyCode.R;          // press R to replay sequence

    public List<Vector2Int> currentPath = new List<Vector2Int>();

    private GameObject[,] lilyPads;
    private Renderer[,] lilyPadRenderers;

    private Vector2Int startCell = new Vector2Int(0, 0);
    private Vector2Int[] possibleEndCells = new Vector2Int[]
    {
        new Vector2Int(1, 4),
        new Vector2Int(3, 4),
        new Vector2Int(4, 1),
        new Vector2Int(4, 3)
    };

    void Start()
    {
        lilyPads = new GameObject[gridWidth + 1, gridHeight + 1];
        lilyPadRenderers = new Renderer[gridWidth + 1, gridHeight + 1];
        DrawGrid();
        GeneratePath();

        if (autoPlaySequence)
            StartCoroutine(PlayHighlightSequence());
    }

    // void Update()
    // {
    //     // Press R to replay the highlight sequence
    //     if (Keyboard.current != null &&
    //         UnityEngine.InputSystem.Keyboard.current[UnityEngine.InputSystem.Key.R].wasPressedThisFrame)
    //     {
    //         ReplaySequence();
    //     }
    // }

    public void ReplaySequence()
    {
        StopAllCoroutines();
        ResetAllColors();
        StartCoroutine(PlayHighlightSequence());
    }

    // ─────────────────────────────────────────
    // HIGHLIGHT SEQUENCE
    // ─────────────────────────────────────────

    IEnumerator PlayHighlightSequence()
    {
        // Wait a moment before starting
        yield return new WaitForSeconds(1f);

        for (int i = 0; i < currentPath.Count; i++)
        {
            Vector2Int cell = currentPath[i];
            Renderer rend = lilyPadRenderers[cell.x, cell.y];

            if (rend == null) continue;

            // Pulse the current pad
            yield return StartCoroutine(PulsePad(rend, i));

            // Mark as completed color after highlight
            rend.material.color = completedColor;

            yield return new WaitForSeconds(delayBetweenPads);
        }

        // After full sequence, reset all to default color
        yield return new WaitForSeconds(0.5f);
        ResetAllColors();

        Debug.Log("Highlight sequence complete!");
    }

    IEnumerator PulsePad(Renderer rend, int index)
    {
        float elapsed = 0f;
        bool isStart = index == 0;
        bool isEnd = index == currentPath.Count - 1;

        // Start and end pads pulse faster to stand out
        float duration = (isStart || isEnd) ? highlightDuration * 1.5f : highlightDuration;

        // Pulse between highlight and default color
        while (elapsed < duration)
        {
            float t = Mathf.PingPong(elapsed * 4f, 1f);
            rend.material.color = Color.Lerp(defaultColor, highlightColor, t);
            elapsed += Time.deltaTime;
            yield return null;
        }

        rend.material.color = highlightColor;
    }

    void ResetAllColors()
    {
        foreach (Vector2Int cell in currentPath)
        {
            Renderer rend = lilyPadRenderers[cell.x, cell.y];
            if (rend != null)
                rend.material.color = defaultColor;
        }
    }

    // ─────────────────────────────────────────
    // GRID LINES
    // ─────────────────────────────────────────

    void DrawGrid()
    {
        for (int y = 0; y <= gridHeight; y++)
        {
            Vector3 start = transform.TransformPoint(new Vector3(0, lineHeightOffset, y * cellSize));
            Vector3 end = transform.TransformPoint(new Vector3(gridWidth * cellSize, lineHeightOffset, y * cellSize));
            CreateLine($"HLine_{y}", start, end);
        }

        for (int x = 0; x <= gridWidth; x++)
        {
            Vector3 start = transform.TransformPoint(new Vector3(x * cellSize, lineHeightOffset, 0));
            Vector3 end = transform.TransformPoint(new Vector3(x * cellSize, lineHeightOffset, gridHeight * cellSize));
            CreateLine($"VLine_{x}", start, end);
        }
    }

    void CreateLine(string lineName, Vector3 start, Vector3 end)
    {
        GameObject lineObj = new GameObject(lineName);
        lineObj.transform.parent = transform;

        LineRenderer lr = lineObj.AddComponent<LineRenderer>();
        lr.positionCount = 2;
        lr.SetPosition(0, start);
        lr.SetPosition(1, end);
        lr.startWidth = lineWidth;
        lr.endWidth = lineWidth;
        lr.material = lineMaterial != null
            ? lineMaterial
            : new Material(Shader.Find("Sprites/Default"));
        lr.startColor = lineColor;
        lr.endColor = lineColor;
        lr.useWorldSpace = true;
        lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lr.receiveShadows = false;
    }

    // ─────────────────────────────────────────
    // PATH GENERATION
    // ─────────────────────────────────────────

    void GeneratePath()
    {
        Vector2Int endCell = possibleEndCells[Random.Range(0, possibleEndCells.Length)];
        Debug.Log($"Path: start {startCell} → end {endCell}");

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
            if (current == end)
            {
                Debug.Log($"Path generated with {path.Count} steps");
                return path;
            }

            List<Vector2Int> neighbours = GetValidNeighbours(current, visited);
            if (neighbours.Count == 0) return null;

            neighbours.Sort((a, b) =>
            {
                int distA = Mathf.Abs(a.x - end.x) + Mathf.Abs(a.y - end.y);
                int distB = Mathf.Abs(b.x - end.x) + Mathf.Abs(b.y - end.y);
                return distA.CompareTo(distB);
            });

            Vector2Int next = Random.value < 0.7f
                ? neighbours[0]
                : neighbours[Random.Range(0, neighbours.Count)];

            current = next;
            path.Add(current);
            visited.Add(current);
        }

        return null;
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
            if (next.x >= 0 && next.x <= gridWidth &&
                next.y >= 0 && next.y <= gridHeight &&
                !visited.Contains(next))
            {
                neighbours.Add(next);
            }
        }
        return neighbours;
    }

    // ─────────────────────────────────────────
    // LILY PAD SPAWNING
    // ─────────────────────────────────────────

    void SpawnLilyPads()
    {
        foreach (Vector2Int cell in currentPath)
        {
            if (lilyPads[cell.x, cell.y] != null)
                Destroy(lilyPads[cell.x, cell.y]);

            Vector3 localPos = new Vector3(cell.x * cellSize, 0, cell.y * cellSize);
            Vector3 worldPos = transform.TransformPoint(localPos);
            GameObject pad = Instantiate(lilyPadPrefab, worldPos, transform.rotation, transform);
            lilyPads[cell.x, cell.y] = pad;

            // Store renderer for color changes
            Renderer rend = pad.GetComponentInChildren<Renderer>();
            if (rend != null)
            {
                lilyPadRenderers[cell.x, cell.y] = rend;
                rend.material.color = defaultColor;
            }
        }
    }
}