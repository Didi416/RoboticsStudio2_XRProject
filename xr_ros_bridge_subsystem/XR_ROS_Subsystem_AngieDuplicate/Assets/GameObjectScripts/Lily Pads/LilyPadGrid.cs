
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
//     public int maxPathLength = 12;

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
//         GeneratePath();
//     }

//     // ─────────────────────────────────────────
//     // CALLED BY EACH LILY PAD WHEN SELECTED
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
//     // HIGHLIGHT SEQUENCE
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
//         foreach (Vector2Int cell in currentPath)
//         {
//             Renderer rend = lilyPadRenderers[cell.x, cell.y];
//             if (rend != null)
//                 rend.material.color = defaultColor;
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

//             Vector3 localPos = new Vector3(cell.x * cellSize, lilyPadHeightOffset, cell.y * cellSize);
//             Vector3 worldPos = transform.TransformPoint(localPos);
//             GameObject pad = Instantiate(
//                 lilyPadPrefab, worldPos, transform.rotation, transform);
//             lilyPads[cell.x, cell.y] = pad;

//             // Add collider if missing
//             if (pad.GetComponent<Collider>() == null)
//                 pad.AddComponent<SphereCollider>();

//             // Add XR Simple Interactable so ray can select it
//             XRSimpleInteractable interactable =
//                 pad.AddComponent<XRSimpleInteractable>();

//             // Store reference to grid for callback
//             LilyPadTrigger trigger = pad.AddComponent<LilyPadTrigger>();
//             trigger.grid = this;

//             // Store renderer
//             Renderer rend = pad.GetComponentInChildren<Renderer>();
//             if (rend != null)
//             {
//                 lilyPadRenderers[cell.x, cell.y] = rend;
//                 rend.material.color = defaultColor;
//             }
//         }
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

    [Header("Path Settings")]
    public int minPathLength = 5;
    public int maxPathLength = 10;

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

    private Vector2Int startCell = new Vector2Int(0, 0);
    private Vector2Int[] possibleEndCells = new Vector2Int[]
    {
        new Vector2Int(1, 5),
        new Vector2Int(3, 5),
        new Vector2Int(5, 1),
        new Vector2Int(5, 3)
    };

    void Start()
    {
        lilyPads = new GameObject[gridWidth + 1, gridHeight + 1];
        lilyPadRenderers = new Renderer[gridWidth + 1, gridHeight + 1];
        DrawGrid();
        GeneratePath();       // generate path first
        SpawnAllLilyPads();   // then spawn all pads
    }

    // ─────────────────────────────────────────
    // SPAWN ALL LILY PADS AT EVERY INTERSECTION
    // ─────────────────────────────────────────

    void SpawnAllLilyPads()
    {
        for (int x = 0; x <= gridWidth; x++)
        {
            for (int y = 0; y <= gridHeight; y++)
            {
                Vector3 localPos = new Vector3(
                    x * cellSize, lilyPadHeightOffset, y * cellSize);
                Vector3 worldPos = transform.TransformPoint(localPos);

                GameObject pad = Instantiate(
                    lilyPadPrefab, worldPos, transform.rotation, transform);
                lilyPads[x, y] = pad;

                // Store renderer and set default colour
                Renderer rend = pad.GetComponentInChildren<Renderer>();
                if (rend != null)
                {
                    lilyPadRenderers[x, y] = rend;
                    rend.material.color = defaultColor;
                }

                // Add collider if missing
                if (pad.GetComponent<Collider>() == null)
                    pad.AddComponent<SphereCollider>();

                // Add XR Simple Interactable
                XRSimpleInteractable interactable =
                    pad.AddComponent<XRSimpleInteractable>();

                // Add trigger script pointing back to this grid
                LilyPadTrigger trigger = pad.AddComponent<LilyPadTrigger>();
                trigger.grid = this;
            }
        }
    }

    // ─────────────────────────────────────────
    // CALLED BY ANY LILY PAD WHEN TRIGGERED
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
    // HIGHLIGHT SEQUENCE - same logic as before
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
            float duration = (isStart || isEnd)
                ? highlightDuration * 1.5f
                : highlightDuration;

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
        // Reset ALL pads back to default green
        for (int x = 0; x <= gridWidth; x++)
        {
            for (int y = 0; y <= gridHeight; y++)
            {
                Renderer rend = lilyPadRenderers[x, y];
                if (rend != null)
                    rend.material.color = defaultColor;
            }
        }
    }

    // ─────────────────────────────────────────
    // GRID LINES
    // ─────────────────────────────────────────

    void DrawGrid()
    {
        for (int y = 0; y <= gridHeight; y++)
        {
            Vector3 start = transform.TransformPoint(
                new Vector3(0, lineHeightOffset, y * cellSize));
            Vector3 end = transform.TransformPoint(
                new Vector3(gridWidth * cellSize, lineHeightOffset, y * cellSize));
            CreateLine($"HLine_{y}", start, end);
        }

        for (int x = 0; x <= gridWidth; x++)
        {
            Vector3 start = transform.TransformPoint(
                new Vector3(x * cellSize, lineHeightOffset, 0));
            Vector3 end = transform.TransformPoint(
                new Vector3(x * cellSize, lineHeightOffset, gridHeight * cellSize));
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
        Vector2Int endCell =
            possibleEndCells[Random.Range(0, possibleEndCells.Length)];
        Debug.Log($"Path: start {startCell} → end {endCell}");

        for (int attempt = 0; attempt < 100; attempt++)
        {
            List<Vector2Int> path = TryGeneratePath(startCell, endCell);
            if (path != null)
            {
                currentPath = path;
                Debug.Log($"Path generated with {currentPath.Count} steps");
                return;
            }
        }

        Debug.LogError("Could not generate a valid path after 100 attempts!");
        Debug.Log($"LilyPadGrid path ready with {currentPath.Count} points");
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
}