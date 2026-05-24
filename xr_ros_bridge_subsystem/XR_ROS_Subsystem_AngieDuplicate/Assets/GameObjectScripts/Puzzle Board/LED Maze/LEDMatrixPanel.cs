// using UnityEngine;
// using UnityEngine.UI;
// using TMPro;
// using System.Collections;
// using System.Collections.Generic;

// public class LEDMatrixPanel : MonoBehaviour
// {
//     [Header("Grid Settings")]
//     public int gridSize = 6;

//     [Header("LED Colors")]
//     public Color offColor = new Color(0.1f, 0.1f, 0.1f);
//     public Color onColor = new Color(0f, 1f, 0f);
//     public Color cursorColor = new Color(1f, 1f, 0f);
//     public Color wrongColor = new Color(1f, 0f, 0f);
//     public Color borderColor = new Color(0.05f, 0.05f, 0.05f); // darker for border

//     [Header("UI References")]
//     public GameObject ledPrefab;
//     public Transform gridParent;
//     public Image statusLight;
//     public TextMeshProUGUI statusText;
//     public float blinkSpeed = 0.5f;

//     [Header("Lily Pad Grid Reference")]
//     public LilyPadGrid lilyPadGrid;

//     [Header("Puzzle Board Manager")]
//     public PuzzleBoardManager puzzleBoardManager;

//     private Image[,] leds;
//     private Vector2Int cursorPos;
//     private List<Vector2Int> playerPath = new List<Vector2Int>();
//     private List<Vector2Int> mappedPath = new List<Vector2Int>();
//     private bool puzzleSolved = false;
//     private Coroutine blinkCoroutine;

//     void Start()
//     {
//         GenerateLEDGrid();
//         ResetMatrix();
//     }

//     // ─────────────────────────────────────────
//     // GENERATE 6x6 LED GRID
//     // ─────────────────────────────────────────

//     void GenerateLEDGrid()
//     {
//         leds = new Image[gridSize, gridSize];

//         for (int y = gridSize - 1; y >= 0; y--)
//         {
//             for (int x = 0; x < gridSize; x++)
//             {
//                 GameObject led = Instantiate(ledPrefab, gridParent);
//                 led.name = $"LED_{x}_{y}";
//                 Image img = led.GetComponent<Image>();
//                 leds[x, y] = img;

//                 // Border LEDs darker
//                 bool isBorder = x == 0 || x == gridSize - 1 ||
//                                 y == 0 || y == gridSize - 1;
//                 img.color = isBorder ? borderColor : offColor;
//             }
//         }

//         Debug.Log($"Generated {gridSize}x{gridSize} LED grid");
//     }

//     // ─────────────────────────────────────────
//     // MAP LILY PAD PATH TO INNER LED GRID
//     // Lily pad (x,y) → LED (x+1, gridSize-2-y)
//     // +1 offset to skip border
//     // Y flipped because UI Y is inverted
//     // ─────────────────────────────────────────

//     void BuildMappedPath()
//     {
//         mappedPath.Clear();

//         if (lilyPadGrid == null || lilyPadGrid.currentPath == null
//             || lilyPadGrid.currentPath.Count == 0)
//         {
//             Debug.LogWarning("No lily pad path available!");
//             return;
//         }

//         foreach (Vector2Int p in lilyPadGrid.currentPath)
//         {
//             Vector2Int mapped = new Vector2Int(
//                 p.x + 1,            // offset X by 1 to skip left border
//                 (gridSize - 2) - p.y // flip Y and offset to skip top border
//             );
//             mappedPath.Add(mapped);
//         }

//         // Debug both paths
//         string lily = "Lily pad path: ";
//         string led = "LED mapped path: ";
//         foreach (Vector2Int p in lilyPadGrid.currentPath)
//             lily += $"({p.x},{p.y}) ";
//         foreach (Vector2Int p in mappedPath)
//             led += $"({p.x},{p.y}) ";

//         Debug.Log(lily);
//         Debug.Log(led);
//     }

//     // ─────────────────────────────────────────
//     // MOVEMENT - called by VirtualJoystick
//     // ─────────────────────────────────────────

//     public void MoveUp()    => TryMove(Vector2Int.up);
//     public void MoveDown()  => TryMove(Vector2Int.down);
//     public void MoveLeft()  => TryMove(Vector2Int.left);
//     public void MoveRight() => TryMove(Vector2Int.right);

//     void TryMove(Vector2Int direction)
//     {
//         if (puzzleSolved) return;

//         Vector2Int newPos = cursorPos + direction;

//         // Clamp to INNER grid only - no border movement
//         newPos.x = Mathf.Clamp(newPos.x, 1, gridSize - 2);
//         newPos.y = Mathf.Clamp(newPos.y, 1, gridSize - 2);

//         if (newPos == cursorPos) return;

//         // Light up trail
//         leds[cursorPos.x, cursorPos.y].color = onColor;

//         cursorPos = newPos;
//         playerPath.Add(cursorPos);
//         leds[cursorPos.x, cursorPos.y].color = cursorColor;

//         Debug.Log($"LED cursor at: {cursorPos}");
//         CheckPath();
//     }

//     // ─────────────────────────────────────────
//     // PATH VALIDATION
//     // ─────────────────────────────────────────

//     void CheckPath()
//     {
//         if (mappedPath == null || mappedPath.Count == 0)
//         {
//             Debug.LogWarning("No mapped path to validate!");
//             return;
//         }

//         int step = playerPath.Count - 1;

//         if (step < mappedPath.Count)
//         {
//             if (playerPath[step] != mappedPath[step])
//             {
//                 // Wrong step
//                 leds[cursorPos.x, cursorPos.y].color = wrongColor;
//                 SetStatusLight(false);
//                 SetStatusText("✗ Wrong Path!\nTry Again");
//                 Debug.Log($"Wrong! Expected {mappedPath[step]} got {playerPath[step]}");

//                 if (puzzleBoardManager != null)
//                     puzzleBoardManager.OnLEDMatrixFailed();

//                 Invoke("ResetMatrix", 2f);
//                 return;
//             }
//         }

//         // Check if complete
//         if (playerPath.Count >= mappedPath.Count)
//         {
//             puzzleSolved = true;

//             // Light all path LEDs green
//             foreach (Vector2Int p in mappedPath)
//                 leds[p.x, p.y].color = onColor;

//             SetStatusLight(true);
//             SetStatusText("✓ Correct Path!\nPuzzle Solved!");
//             Debug.Log("LED Matrix SOLVED!");

//             if (puzzleBoardManager != null)
//                 puzzleBoardManager.OnLEDMatrixSolved();
//         }
//         else
//         {
//             SetStatusLight(true);
//             SetStatusText($"Step {playerPath.Count}/{mappedPath.Count}");
//         }
//     }

//     // ─────────────────────────────────────────
//     // STATUS LIGHT + TEXT
//     // ─────────────────────────────────────────

//     void SetStatusLight(bool correct)
//     {
//         if (blinkCoroutine != null)
//             StopCoroutine(blinkCoroutine);
//         if (statusLight != null)
//             blinkCoroutine = StartCoroutine(BlinkLight(correct));
//     }

//     IEnumerator BlinkLight(bool correct)
//     {
//         Color blinkColor = correct ? Color.green : Color.red;
//         while (true)
//         {
//             statusLight.color = blinkColor;
//             yield return new WaitForSeconds(blinkSpeed);
//             statusLight.color = offColor;
//             yield return new WaitForSeconds(blinkSpeed);
//         }
//     }

//     void SetStatusText(string message)
//     {
//         if (statusText != null)
//             statusText.text = message;
//     }

//     // ─────────────────────────────────────────
//     // RESET
//     // ─────────────────────────────────────────

//     public void ResetMatrix()
//     {
//         puzzleSolved = false;
//         playerPath.Clear();

//         // Start at second column second row from top
//         // (1, gridSize-2) = inner top left
//         cursorPos = new Vector2Int(1, gridSize - 2);

//         if (blinkCoroutine != null)
//             StopCoroutine(blinkCoroutine);

//         if (leds == null) return;

//         // Reset all LEDs
//         for (int x = 0; x < gridSize; x++)
//         {
//             for (int y = 0; y < gridSize; y++)
//             {
//                 if (leds[x, y] == null) continue;

//                 bool isBorder = x == 0 || x == gridSize - 1 ||
//                                 y == 0 || y == gridSize - 1;
//                 leds[x, y].color = isBorder ? borderColor : offColor;
//             }
//         }

//         if (statusLight != null)
//             statusLight.color = offColor;

//         SetStatusText("Move joystick to trace path");

//         // Build mapped path from lily pad
//         BuildMappedPath();

//         // Show cursor at start position
//         leds[cursorPos.x, cursorPos.y].color = cursorColor;

//         Debug.Log($"LED reset - cursor at {cursorPos}");
//     }
// }

using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections;
using System.Collections.Generic;

public class LEDMatrixPanel : MonoBehaviour
{
    [Header("Grid Settings")]
    public int gridSize = 8;

    [Header("LED Colors")]
    public Color offColor = new Color(0.1f, 0.1f, 0.1f);
    public Color onColor = new Color(0f, 1f, 0f);
    public Color cursorColor = new Color(1f, 1f, 0f);
    public Color wrongColor = new Color(1f, 0f, 0f);
    public Color borderColor = new Color(0.02f, 0.02f, 0.02f);
    public Color tickColor = new Color(0f, 1f, 0f);
    public Color crossColor = new Color(1f, 0f, 0f);

    [Header("UI References")]
    public GameObject ledPrefab;
    public Transform gridParent;
    public TextMeshProUGUI statusText;

    [Header("Lily Pad Grid Reference")]
    public LilyPadGrid lilyPadGrid;

    [Header("Puzzle Board Manager")]
    public PuzzleBoardManager puzzleBoardManager;

    [Header("Reset Settings")]
    public float wrongResetDelay = 1.5f;

    private Image[,] leds;
    private Vector2Int cursorPos;
    private Vector2Int startPos;
    private List<Vector2Int> playerPath = new List<Vector2Int>();
    private List<Vector2Int> mappedPath = new List<Vector2Int>();
    private bool puzzleSolved = false;
    private bool isResetting = false;
    private Coroutine blinkCoroutine;

    private int innerMin = 1;
    private int innerMax; // = gridSize - 2 = 6 for 8x8

    void Start()
    {
        innerMin = 1;
        innerMax = gridSize - 2; // 8-2 = 6
        GenerateLEDGrid();
        StartCoroutine(DelayedReset());
    }

    IEnumerator DelayedReset()
    {
        yield return new WaitForSeconds(0.5f);
        while (lilyPadGrid == null || lilyPadGrid.currentPath == null
               || lilyPadGrid.currentPath.Count == 0)
        {
            Debug.Log("Waiting for lily pad path...");
            yield return new WaitForSeconds(0.1f);
        }
        Debug.Log($"Path ready: {lilyPadGrid.currentPath.Count} points");
        ResetMatrix();
    }

    // ─────────────────────────────────────────
    // GENERATE 8x8 GRID
    // Border = row/col 0 and 7
    // Inner = rows/cols 1-6
    // Spawns top to bottom so Y=7 is top Y=0 is bottom
    // ─────────────────────────────────────────

    void GenerateLEDGrid()
    {
        leds = new Image[gridSize, gridSize];

        for (int row = 0; row < gridSize; row++)
        {
            int y = gridSize - 1 - row; // row 0 = y7 (top), row 7 = y0 (bottom)
            for (int x = 0; x < gridSize; x++)
            {
                GameObject led = Instantiate(ledPrefab, gridParent);
                led.name = $"LED_{x}_{y}";
                Image img = led.GetComponent<Image>();
                leds[x, y] = img;

                bool isBorder = x == 0 || x == gridSize - 1 ||
                                y == 0 || y == gridSize - 1;
                img.color = isBorder ? borderColor : offColor;
            }
        }

        Debug.Log($"Generated {gridSize}x{gridSize} LED grid");
        Debug.Log($"Inner area: col {innerMin}-{innerMax}, row {innerMin}-{innerMax}");
    }

    // ─────────────────────────────────────────
    // MAP LILY PAD PATH TO LED GRID
    //
    // Lily pad grid: 6x6 points (0-5 both axes)
    // LED inner grid: 6x6 playable (1-6 both axes)
    //
    // Lily pad Y=0 = visual top
    // LED Y=7 = visual top (spawned top to bottom)
    // LED Y=1 = visual bottom inner
    //
    // So lily (0,0) top left → LED (1, innerMax) = (1,6) top left inner
    // ledX = lilyX + 1
    // ledY = innerMax - lilyY  (flip because lily Y increases down, LED Y increases up)
    // ─────────────────────────────────────────

    void BuildMappedPath()
    {
        mappedPath.Clear();

        if (lilyPadGrid == null || lilyPadGrid.currentPath == null
            || lilyPadGrid.currentPath.Count == 0)
        {
            Debug.LogWarning("No lily pad path!");
            return;
        }

        foreach (Vector2Int p in lilyPadGrid.currentPath)
        {
            int ledX = Mathf.Clamp(p.x + 1, innerMin, innerMax);
            int ledY = Mathf.Clamp(innerMax - p.y, innerMin, innerMax);
            mappedPath.Add(new Vector2Int(ledX, ledY));
        }

        string lily = "Lily: ";
        string led = "LED:  ";
        for (int i = 0; i < lilyPadGrid.currentPath.Count; i++)
        {
            lily += $"({lilyPadGrid.currentPath[i].x}," +
                    $"{lilyPadGrid.currentPath[i].y}) ";
            led += $"({mappedPath[i].x},{mappedPath[i].y}) ";
        }
        Debug.Log(lily);
        Debug.Log(led);
        Debug.Log($"Start point: Lily(0,0) → LED({mappedPath[0].x},{mappedPath[0].y})");
    }

    // ─────────────────────────────────────────
    // MOVEMENT
    // ─────────────────────────────────────────

    public void MoveUp()    => TryMove(Vector2Int.up);
    public void MoveDown()  => TryMove(Vector2Int.down);
    public void MoveLeft()  => TryMove(Vector2Int.left);
    public void MoveRight() => TryMove(Vector2Int.right);

    void TryMove(Vector2Int direction)
    {
        if (puzzleSolved || isResetting) return;

        Vector2Int newPos = cursorPos + direction;

        // Clamp to inner grid only - never touch borders
        newPos.x = Mathf.Clamp(newPos.x, innerMin, innerMax);
        newPos.y = Mathf.Clamp(newPos.y, innerMin, innerMax);

        if (newPos == cursorPos) return;

        // Light up trail
        leds[cursorPos.x, cursorPos.y].color = onColor;

        cursorPos = newPos;
        playerPath.Add(cursorPos);
        leds[cursorPos.x, cursorPos.y].color = cursorColor;

        Debug.Log($"Cursor → {cursorPos}");
        CheckPath();
    }

    // ─────────────────────────────────────────
    // PATH VALIDATION
    // ─────────────────────────────────────────

    void CheckPath()
    {
        if (mappedPath == null || mappedPath.Count == 0)
        {
            Debug.LogWarning("No mapped path!");
            BuildMappedPath();
            return;
        }

        int step = playerPath.Count - 1;

        // Skip validation for step 0 - cursor starts there already
        if (step == 0)
        {
            SetStatusText($"Step {playerPath.Count}/{mappedPath.Count}");
            return;
        }

        Debug.Log($"Step {step}: at {playerPath[step]} expected " +
                $"{(step < mappedPath.Count ? mappedPath[step].ToString() : "end")}");

        if (step < mappedPath.Count && playerPath[step] != mappedPath[step])
        {
            StartCoroutine(WrongStepSequence());
            return;
        }

        if (playerPath.Count >= mappedPath.Count)
        {
            puzzleSolved = true;
            ShowPixelTick();
            SetStatusText("✓ Correct!\nPuzzle Solved!");
            Debug.Log("LED Matrix SOLVED!");

            if (puzzleBoardManager != null)
                puzzleBoardManager.OnLEDMatrixSolved();
        }
        else
        {
            SetStatusText($"Step {playerPath.Count}/{mappedPath.Count}");
        }
    }

    // ─────────────────────────────────────────
    // WRONG STEP - flash red then reset to start
    // ─────────────────────────────────────────

    
    IEnumerator WrongStepSequence()
    {
        isResetting = true;

        leds[cursorPos.x, cursorPos.y].color = wrongColor;
        SetStatusText("✗ Wrong!\nResetting...");
        Debug.Log($"WRONG at {cursorPos}! Resetting...");

        if (puzzleBoardManager != null)
            puzzleBoardManager.OnLEDMatrixFailed();

        yield return new WaitForSeconds(wrongResetDelay);

        ClearInnerGrid();

        // Reset path and add start point
        playerPath.Clear();
        playerPath.Add(startPos); // ← add this

        cursorPos = startPos;

        if (InBounds(cursorPos))
            leds[cursorPos.x, cursorPos.y].color = cursorColor;

        SetStatusText("Move joystick to trace path");
        Debug.Log($"Reset to start: {startPos}");

        isResetting = false;
    }

    // ─────────────────────────────────────────
    // PIXEL TICK - drawn on inner 6x6 (1-6)
    // ─────────────────────────────────────────

    void ShowPixelTick()
    {
        ClearInnerGrid();
        Vector2Int[] pixels = new Vector2Int[]
        {
            new Vector2Int(1, 3),
            new Vector2Int(2, 2),
            new Vector2Int(3, 3),
            new Vector2Int(4, 4),
            new Vector2Int(5, 5),
            new Vector2Int(6, 6),
        };
        foreach (Vector2Int p in pixels)
            if (InBounds(p)) leds[p.x, p.y].color = tickColor;
    }

    // ─────────────────────────────────────────
    // PIXEL CROSS - drawn on inner 6x6 (1-6)
    // ─────────────────────────────────────────

    void ShowPixelCross()
    {
        ClearInnerGrid();
        Vector2Int[] pixels = new Vector2Int[]
        {
            new Vector2Int(1, 6),
            new Vector2Int(2, 5),
            new Vector2Int(3, 4),
            new Vector2Int(4, 3),
            new Vector2Int(5, 2),
            new Vector2Int(6, 1),
            new Vector2Int(1, 1),
            new Vector2Int(2, 2),
            new Vector2Int(3, 3),
            new Vector2Int(4, 4),
            new Vector2Int(5, 5),
            new Vector2Int(6, 6),
        };
        foreach (Vector2Int p in pixels)
            if (InBounds(p)) leds[p.x, p.y].color = crossColor;
    }

    void ClearInnerGrid()
    {
        for (int x = innerMin; x <= innerMax; x++)
            for (int y = innerMin; y <= innerMax; y++)
                leds[x, y].color = offColor;
    }

    bool InBounds(Vector2Int p)
    {
        return p.x >= innerMin && p.x <= innerMax &&
               p.y >= innerMin && p.y <= innerMax;
    }

    void SetStatusText(string message)
    {
        if (statusText != null)
            statusText.text = message;
    }

    // ─────────────────────────────────────────
    // RESET
    // ─────────────────────────────────────────

    public void ResetMatrix()
    {
        puzzleSolved = false;
        isResetting = false;
        //playerPath.Clear();

        if (blinkCoroutine != null)
            StopCoroutine(blinkCoroutine);

        if (leds == null) return;

        // Reset all LEDs
        for (int x = 0; x < gridSize; x++)
        {
            for (int y = 0; y < gridSize; y++)
            {
                if (leds[x, y] == null) continue;
                bool isBorder = x == 0 || x == gridSize - 1 ||
                                y == 0 || y == gridSize - 1;
                leds[x, y].color = isBorder ? borderColor : offColor;
            }
        }

        SetStatusText("Move joystick to trace path");

        // Build mapped path
        BuildMappedPath();

        // Start at first mapped point
        // Lily (0,0) → LED (1, innerMax) = (1,6) = top left inner
        if (mappedPath != null && mappedPath.Count > 0)
            startPos = mappedPath[0];
        else
            startPos = new Vector2Int(innerMin, innerMax);

        cursorPos = startPos;

        if (InBounds(cursorPos))
            leds[cursorPos.x, cursorPos.y].color = cursorColor;

        Debug.Log($"Reset - cursor at {cursorPos}");
        Debug.Log($"Inner grid: ({innerMin},{innerMin}) to ({innerMax},{innerMax})");
        
        playerPath.Clear();
        playerPath.Add(cursorPos); // add start point so step 0 is already counted
    }
}