// using UnityEngine;
// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine.XR.Interaction.Toolkit;
// using UnityEngine.XR.Interaction.Toolkit.Interactables;

// public class LilyPadGrid : MonoBehaviour
// {
//     [Header("Grid Settings")]
//     public int gridWidth = 5;
//     public int gridHeight = 5;
//     public float cellSize = 1f;

//     [Header("Lily Pad Prefab")]
//     public GameObject lilyPadPrefab;

//     [Header("Grid Line Settings")]
//     public Material lineMaterial;
//     public Color lineColor = Color.white;
//     public float lineWidth = 0.02f;
//     public float lineHeightOffset = 0.01f;

//     [Header("Lily Pad Offset")]
//     public float lilyPadHeightOffset = 0.05f;

//     [Header("Path Settings")]
//     public int minPathLength = 5;
//     public int maxPathLength = 10;

//     [Header("Highlight Settings")]
//     public Color defaultColor = Color.green;
//     public Color highlightColor = Color.yellow;
//     public Color completedColor = Color.cyan;
//     public float highlightDuration = 0.6f;
//     public float delayBetweenPads = 0.3f;

//     public List<Vector2Int> currentPath = new List<Vector2Int>();

//     private GameObject[,] lilyPads;
//     private Renderer[,] lilyPadRenderers;
//     private bool isPlaying = false;

//     private Vector2Int startCell = new Vector2Int(0, 0);
//     private Vector2Int[] possibleEndCells = new Vector2Int[]
//     {
//         new Vector2Int(1, 5),
//         new Vector2Int(3, 5),
//         new Vector2Int(5, 1),
//         new Vector2Int(5, 3)
//     };

//     void Start()
//     {
//         lilyPads = new GameObject[gridWidth + 1, gridHeight + 1];
//         lilyPadRenderers = new Renderer[gridWidth + 1, gridHeight + 1];
//         DrawGrid();
//         GeneratePath();       // generate path first
//         SpawnAllLilyPads();   // then spawn all pads
//     }

//     // ─────────────────────────────────────────
//     // SPAWN ALL LILY PADS AT EVERY INTERSECTION
//     // ─────────────────────────────────────────

//     void SpawnAllLilyPads()
//     {
//         for (int x = 0; x <= gridWidth; x++)
//         {
//             for (int y = 0; y <= gridHeight; y++)
//             {
//                 Vector3 localPos = new Vector3(
//                     x * cellSize, lilyPadHeightOffset, y * cellSize);
//                 Vector3 worldPos = transform.TransformPoint(localPos);

//                 GameObject pad = Instantiate(
//                     lilyPadPrefab, worldPos, transform.rotation, transform);
//                 lilyPads[x, y] = pad;

//                 // Store renderer and set default colour
//                 Renderer rend = pad.GetComponentInChildren<Renderer>();
//                 if (rend != null)
//                 {
//                     lilyPadRenderers[x, y] = rend;
//                     rend.material.color = defaultColor;
//                 }

//                 // Add collider if missing
//                 if (pad.GetComponent<Collider>() == null)
//                     pad.AddComponent<SphereCollider>();

//                 // Add XR Simple Interactable
//                 XRSimpleInteractable interactable =
//                     pad.AddComponent<XRSimpleInteractable>();

//                 // Add trigger script pointing back to this grid
//                 LilyPadTrigger trigger = pad.AddComponent<LilyPadTrigger>();
//                 trigger.grid = this;
//             }
//         }
//     }

//     // ─────────────────────────────────────────
//     // CALLED BY ANY LILY PAD WHEN TRIGGERED
//     // ─────────────────────────────────────────

//     public void OnLilyPadTriggered()
//     {
//         if (!isPlaying)
//         {
//             StopAllCoroutines();
//             ResetAllColors();
//             StartCoroutine(PlayHighlightSequence());
//         }
//     }

//     // ─────────────────────────────────────────
//     // HIGHLIGHT SEQUENCE - same logic as before
//     // ─────────────────────────────────────────

//     IEnumerator PlayHighlightSequence()
//     {
//         isPlaying = true;
//         yield return new WaitForSeconds(0.5f);

//         for (int i = 0; i < currentPath.Count; i++)
//         {
//             Vector2Int cell = currentPath[i];
//             Renderer rend = lilyPadRenderers[cell.x, cell.y];
//             if (rend == null) continue;

//             bool isStart = i == 0;
//             bool isEnd = i == currentPath.Count - 1;
//             float duration = (isStart || isEnd)
//                 ? highlightDuration * 1.5f
//                 : highlightDuration;

//             yield return StartCoroutine(PulsePad(rend, duration));

//             rend.material.color = completedColor;
//             yield return new WaitForSeconds(delayBetweenPads);
//         }

//         yield return new WaitForSeconds(0.5f);
//         ResetAllColors();

//         isPlaying = false;
//         Debug.Log("Highlight sequence complete!");
//     }

//     IEnumerator PulsePad(Renderer rend, float duration)
//     {
//         float elapsed = 0f;
//         while (elapsed < duration)
//         {
//             float t = Mathf.PingPong(elapsed * 4f, 1f);
//             rend.material.color = Color.Lerp(defaultColor, highlightColor, t);
//             elapsed += Time.deltaTime;
//             yield return null;
//         }
//         rend.material.color = highlightColor;
//     }

//     void ResetAllColors()
//     {
//         // Reset ALL pads back to default green
//         for (int x = 0; x <= gridWidth; x++)
//         {
//             for (int y = 0; y <= gridHeight; y++)
//             {
//                 Renderer rend = lilyPadRenderers[x, y];
//                 if (rend != null)
//                     rend.material.color = defaultColor;
//             }
//         }
//     }

//     // ─────────────────────────────────────────
//     // GRID LINES
//     // ─────────────────────────────────────────

//     void DrawGrid()
//     {
//         for (int y = 0; y <= gridHeight; y++)
//         {
//             Vector3 start = transform.TransformPoint(
//                 new Vector3(0, lineHeightOffset, y * cellSize));
//             Vector3 end = transform.TransformPoint(
//                 new Vector3(gridWidth * cellSize, lineHeightOffset, y * cellSize));
//             CreateLine($"HLine_{y}", start, end);
//         }

//         for (int x = 0; x <= gridWidth; x++)
//         {
//             Vector3 start = transform.TransformPoint(
//                 new Vector3(x * cellSize, lineHeightOffset, 0));
//             Vector3 end = transform.TransformPoint(
//                 new Vector3(x * cellSize, lineHeightOffset, gridHeight * cellSize));
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
//         Vector2Int endCell =
//             possibleEndCells[Random.Range(0, possibleEndCells.Length)];
//         Debug.Log($"Path: start {startCell} → end {endCell}");

//         for (int attempt = 0; attempt < 100; attempt++)
//         {
//             List<Vector2Int> path = TryGeneratePath(startCell, endCell);
//             if (path != null)
//             {
//                 currentPath = path;
//                 Debug.Log($"Path generated with {currentPath.Count} steps");
//                 return;
//             }
//         }

//         Debug.LogError("Could not generate a valid path after 100 attempts!");
//         Debug.Log($"LilyPadGrid path ready with {currentPath.Count} points");
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

//     // Add this method to LilyPadGrid.cs
//     public void SetEndPoint(int x, int y)
//     {
//         // Clamp to valid grid range
//         x = Mathf.Clamp(x, 0, gridWidth);
//         y = Mathf.Clamp(y, 0, gridHeight);

//         Debug.Log($"LilyPadGrid end point updated to ({x},{y})");

//         // Regenerate path with new end point
//         currentPath.Clear();

//         Vector2Int newEnd = new Vector2Int(x, y);

//         for (int attempt = 0; attempt < 100; attempt++)
//         {
//             List<Vector2Int> path = TryGeneratePath(startCell, newEnd);
//             if (path != null)
//             {
//                 currentPath = path;
//                 Debug.Log($"Path regenerated with {currentPath.Count} steps");
//                 return;
//             }
//         }

//         Debug.LogError("Could not regenerate path!");
//     }
// }

using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;

public class LilyPadGrid : MonoBehaviour
{
    [Header("Grid Settings")]
    public int gridWidth = 5;
    public int gridHeight = 5;
    public float cellSize = 1f;

    [Header("Lily Pad Prefab")]
    public GameObject lilyPadPrefab;

    [Header("Grid Line Settings")]
    public Material lineMaterial;
    public Color lineColor = Color.white;
    public float lineWidth = 0.02f;
    public float lineHeightOffset = 0.01f;

    [Header("Lily Pad Offset")]
    public float lilyPadHeightOffset = 0.05f;

    [Header("Highlight Settings")]
    public Color defaultColor = Color.green;
    public Color highlightColor = Color.yellow;
    public Color completedColor = Color.cyan;
    public float highlightDuration = 0.6f;
    public float delayBetweenPads = 0.3f;

    public List<Vector2Int> currentPath = new List<Vector2Int>();

    private GameObject[,] lilyPads;
    private Renderer[,] lilyPadRenderers;
    private bool isPlaying = false;

    // CHANGED: startCell fixed at (0,0) — matches Arduino maze start (1,1)
    private Vector2Int startCell = new Vector2Int(0, 0);

    void Start()
    {
        lilyPads = new GameObject[gridWidth + 1, gridHeight + 1];
        lilyPadRenderers = new Renderer[gridWidth + 1, gridHeight + 1];
        DrawGrid();
        GenerateFallbackPath();  // CHANGED: safe fallback until ROS maze arrives
        SpawnAllLilyPads();
    }

    // ─────────────────────────────────────────
    // SPAWN ALL LILY PADS
    // Unchanged
    // ─────────────────────────────────────────

    void SpawnAllLilyPads()
    {
        for (int x = 0; x <= gridWidth; x++)
        {
            for (int y = 0; y <= gridHeight; y++)
            {
                Vector3 localPos = new Vector3(x * cellSize, lilyPadHeightOffset, y * cellSize);
                Vector3 worldPos = transform.TransformPoint(localPos);

                GameObject pad = Instantiate(lilyPadPrefab, worldPos, transform.rotation, transform);
                lilyPads[x, y] = pad;

                Renderer rend = pad.GetComponentInChildren<Renderer>();
                if (rend != null)
                {
                    lilyPadRenderers[x, y] = rend;
                    rend.material.color = defaultColor;
                }

                if (pad.GetComponent<Collider>() == null)
                    pad.AddComponent<SphereCollider>();

                XRSimpleInteractable interactable = pad.AddComponent<XRSimpleInteractable>();

                LilyPadTrigger trigger = pad.AddComponent<LilyPadTrigger>();
                trigger.grid = this;
            }
        }
    }

    // ─────────────────────────────────────────
    // TRIGGERED BY ANY LILY PAD
    // Unchanged
    // ─────────────────────────────────────────

    public void OnLilyPadTriggered()
    {
        if (!isPlaying)
        {
            StopAllCoroutines();
            ResetAllColors();
            StartCoroutine(PlayHighlightSequence());
        }
    }

    // ─────────────────────────────────────────
    // HIGHLIGHT SEQUENCE
    // Unchanged
    // ─────────────────────────────────────────

    IEnumerator PlayHighlightSequence()
    {
        isPlaying = true;
        yield return new WaitForSeconds(0.5f);

        for (int i = 0; i < currentPath.Count; i++)
        {
            Vector2Int cell = currentPath[i];
            Renderer rend = lilyPadRenderers[cell.x, cell.y];
            if (rend == null) continue;

            bool isStart = i == 0;
            bool isEnd = i == currentPath.Count - 1;
            float duration = (isStart || isEnd) ? highlightDuration * 1.5f : highlightDuration;

            yield return StartCoroutine(PulsePad(rend, duration));

            rend.material.color = completedColor;
            yield return new WaitForSeconds(delayBetweenPads);
        }

        yield return new WaitForSeconds(0.5f);
        ResetAllColors();
        isPlaying = false;
        Debug.Log("Highlight sequence complete!");
    }

    IEnumerator PulsePad(Renderer rend, float duration)
    {
        float elapsed = 0f;
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
        for (int x = 0; x <= gridWidth; x++)
            for (int y = 0; y <= gridHeight; y++)
                if (lilyPadRenderers[x, y] != null)
                    lilyPadRenderers[x, y].material.color = defaultColor;
    }

    // ─────────────────────────────────────────
    // GRID LINES
    // Unchanged
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
        lr.material = lineMaterial != null ? lineMaterial : new Material(Shader.Find("Sprites/Default"));
        lr.startColor = lineColor;
        lr.endColor = lineColor;
        lr.useWorldSpace = true;
        lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lr.receiveShadows = false;
    }

    // ─────────────────────────────────────────
    // SET PATH FROM MAZE
    // ADDED: called by ROSPuzzleBridge with the parsed 8x8 maze grid.
    // Replaces the old random path generation entirely.
    // Extracts the exact ordered waypoints from Arduino's maze grid
    // so the VR lily pad path matches the physical maze exactly.
    //
    // Coordinate mapping:
    //   Arduino maze grid [row][col], inner playable area = rows/cols 1-6
    //   Lily pad grid: 0-5 on each axis
    //   Conversion: lilyX = mazeCol - 1, lilyY = mazeRow - 1
    // ─────────────────────────────────────────

    public void SetPathFromMaze(bool[,] mazeGrid)
    {
        List<Vector2Int> path = ExtractOrderedPath(mazeGrid);

        if (path == null || path.Count == 0)
        {
            Debug.LogError("LilyPadGrid: could not extract path from maze — keeping current path");
            return;
        }

        currentPath = path;

        string log = "LilyPadGrid: maze path set — ";
        foreach (var p in currentPath) log += $"({p.x},{p.y}) ";
        Debug.Log(log);
    }

    // ─────────────────────────────────────────
    // EXTRACT ORDERED PATH FROM MAZE GRID
    // ADDED: walks the 8x8 bool grid from start (mazeCol=1, mazeRow=1)
    // following connected 1-cells in order.
    // Returns lily pad coords (mazeCoord - 1).
    // ─────────────────────────────────────────

    List<Vector2Int> ExtractOrderedPath(bool[,] mazeGrid)
    {
        List<Vector2Int> path = new List<Vector2Int>();
        HashSet<Vector2Int> visited = new HashSet<Vector2Int>();

        // Arduino start is always maze (col=1, row=1) → lily pad (0,0)
        Vector2Int current = new Vector2Int(0, 0);
        path.Add(current);
        visited.Add(current);

        Vector2Int[] directions = {
            Vector2Int.right, Vector2Int.up,
            Vector2Int.left, Vector2Int.down
        };

        for (int step = 0; step < 64; step++)
        {
            bool moved = false;
            foreach (var dir in directions)
            {
                Vector2Int next = current + dir;

                // Lily pad bounds: 0 to gridWidth/gridHeight
                if (next.x < 0 || next.x > gridWidth  ||
                    next.y < 0 || next.y > gridHeight) continue;

                if (visited.Contains(next)) continue;

                // Convert lily coords back to maze coords to check the grid
                // lily (x,y) → maze (col = x+1, row = y+1)
                int mazeCol = next.x + 1;
                int mazeRow = next.y + 1;

                if (mazeRow < 8 && mazeCol < 8 && mazeGrid[mazeRow, mazeCol])
                {
                    path.Add(next);
                    visited.Add(next);
                    current = next;
                    moved = true;
                    break;
                }
            }

            if (!moved) break;
        }

        if (path.Count < 2)
        {
            Debug.LogError($"ExtractOrderedPath: only found {path.Count} cell(s) — check maze data");
            return null;
        }

        Debug.Log($"ExtractOrderedPath: found {path.Count} waypoints");
        return path;
    }

    // ─────────────────────────────────────────
    // FALLBACK PATH GENERATION
    // CHANGED: replaces the old random GeneratePath().
    // Used only at startup before ROS maze data arrives.
    // A simple straight path so the grid isn't empty.
    // ─────────────────────────────────────────

    void GenerateFallbackPath()
    {
        currentPath = new List<Vector2Int>
        {
            new Vector2Int(0, 0),
            new Vector2Int(1, 0),
            new Vector2Int(2, 0),
            new Vector2Int(2, 1),
            new Vector2Int(2, 2),
            new Vector2Int(2, 3)
        };
        Debug.Log("LilyPadGrid: using fallback path — waiting for ROS maze data");
    }

    // ─────────────────────────────────────────
    // SET END POINT (kept as fallback)
    // CHANGED: now a no-op since path comes from SetPathFromMaze.
    // Keeping the signature so nothing breaks if still called.
    // ─────────────────────────────────────────

    public void SetEndPoint(int x, int y)
    {
        // Path is now set by SetPathFromMaze via ROS maze data.
        // This method is kept only as a fallback when no maze string arrives.
        Debug.Log($"SetEndPoint({x},{y}) — use SetPathFromMaze for exact path sync");
    }
}