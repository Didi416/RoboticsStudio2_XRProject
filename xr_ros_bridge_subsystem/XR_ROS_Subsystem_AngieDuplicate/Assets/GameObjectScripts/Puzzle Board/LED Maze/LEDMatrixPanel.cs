using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections;
using System.Collections.Generic;

public class LEDMatrixPanel : MonoBehaviour
{
    [Header("Grid Settings")]
    public int gridSize = 6;

    [Header("LED Colors")]
    public Color offColor = new Color(0.1f, 0.1f, 0.1f);
    public Color onColor = new Color(0f, 1f, 0f);
    public Color cursorColor = new Color(1f, 1f, 0f);
    public Color wrongColor = new Color(1f, 0f, 0f);
    public Color borderColor = new Color(0.05f, 0.05f, 0.05f); // darker for border

    [Header("UI References")]
    public GameObject ledPrefab;
    public Transform gridParent;
    public Image statusLight;
    public TextMeshProUGUI statusText;
    public float blinkSpeed = 0.5f;

    [Header("Lily Pad Grid Reference")]
    public LilyPadGrid lilyPadGrid;

    [Header("Puzzle Board Manager")]
    public PuzzleBoardManager puzzleBoardManager;

    private Image[,] leds;
    private Vector2Int cursorPos;
    private List<Vector2Int> playerPath = new List<Vector2Int>();
    private List<Vector2Int> mappedPath = new List<Vector2Int>();
    private bool puzzleSolved = false;
    private Coroutine blinkCoroutine;

    void Start()
    {
        GenerateLEDGrid();
        ResetMatrix();
    }

    // ─────────────────────────────────────────
    // GENERATE 6x6 LED GRID
    // ─────────────────────────────────────────

    void GenerateLEDGrid()
    {
        leds = new Image[gridSize, gridSize];

        for (int y = gridSize - 1; y >= 0; y--)
        {
            for (int x = 0; x < gridSize; x++)
            {
                GameObject led = Instantiate(ledPrefab, gridParent);
                led.name = $"LED_{x}_{y}";
                Image img = led.GetComponent<Image>();
                leds[x, y] = img;

                // Border LEDs darker
                bool isBorder = x == 0 || x == gridSize - 1 ||
                                y == 0 || y == gridSize - 1;
                img.color = isBorder ? borderColor : offColor;
            }
        }

        Debug.Log($"Generated {gridSize}x{gridSize} LED grid");
    }

    // ─────────────────────────────────────────
    // MAP LILY PAD PATH TO INNER LED GRID
    // Lily pad (x,y) → LED (x+1, gridSize-2-y)
    // +1 offset to skip border
    // Y flipped because UI Y is inverted
    // ─────────────────────────────────────────

    void BuildMappedPath()
    {
        mappedPath.Clear();

        if (lilyPadGrid == null || lilyPadGrid.currentPath == null
            || lilyPadGrid.currentPath.Count == 0)
        {
            Debug.LogWarning("No lily pad path available!");
            return;
        }

        foreach (Vector2Int p in lilyPadGrid.currentPath)
        {
            Vector2Int mapped = new Vector2Int(
                p.x + 1,            // offset X by 1 to skip left border
                (gridSize - 2) - p.y // flip Y and offset to skip top border
            );
            mappedPath.Add(mapped);
        }

        // Debug both paths
        string lily = "Lily pad path: ";
        string led = "LED mapped path: ";
        foreach (Vector2Int p in lilyPadGrid.currentPath)
            lily += $"({p.x},{p.y}) ";
        foreach (Vector2Int p in mappedPath)
            led += $"({p.x},{p.y}) ";

        Debug.Log(lily);
        Debug.Log(led);
    }

    // ─────────────────────────────────────────
    // MOVEMENT - called by VirtualJoystick
    // ─────────────────────────────────────────

    public void MoveUp()    => TryMove(Vector2Int.up);
    public void MoveDown()  => TryMove(Vector2Int.down);
    public void MoveLeft()  => TryMove(Vector2Int.left);
    public void MoveRight() => TryMove(Vector2Int.right);

    void TryMove(Vector2Int direction)
    {
        if (puzzleSolved) return;

        Vector2Int newPos = cursorPos + direction;

        // Clamp to INNER grid only - no border movement
        newPos.x = Mathf.Clamp(newPos.x, 1, gridSize - 2);
        newPos.y = Mathf.Clamp(newPos.y, 1, gridSize - 2);

        if (newPos == cursorPos) return;

        // Light up trail
        leds[cursorPos.x, cursorPos.y].color = onColor;

        cursorPos = newPos;
        playerPath.Add(cursorPos);
        leds[cursorPos.x, cursorPos.y].color = cursorColor;

        Debug.Log($"LED cursor at: {cursorPos}");
        CheckPath();
    }

    // ─────────────────────────────────────────
    // PATH VALIDATION
    // ─────────────────────────────────────────

    void CheckPath()
    {
        if (mappedPath == null || mappedPath.Count == 0)
        {
            Debug.LogWarning("No mapped path to validate!");
            return;
        }

        int step = playerPath.Count - 1;

        if (step < mappedPath.Count)
        {
            if (playerPath[step] != mappedPath[step])
            {
                // Wrong step
                leds[cursorPos.x, cursorPos.y].color = wrongColor;
                SetStatusLight(false);
                SetStatusText("✗ Wrong Path!\nTry Again");
                Debug.Log($"Wrong! Expected {mappedPath[step]} got {playerPath[step]}");

                if (puzzleBoardManager != null)
                    puzzleBoardManager.OnLEDMatrixFailed();

                Invoke("ResetMatrix", 2f);
                return;
            }
        }

        // Check if complete
        if (playerPath.Count >= mappedPath.Count)
        {
            puzzleSolved = true;

            // Light all path LEDs green
            foreach (Vector2Int p in mappedPath)
                leds[p.x, p.y].color = onColor;

            SetStatusLight(true);
            SetStatusText("✓ Correct Path!\nPuzzle Solved!");
            Debug.Log("LED Matrix SOLVED!");

            if (puzzleBoardManager != null)
                puzzleBoardManager.OnLEDMatrixSolved();
        }
        else
        {
            SetStatusLight(true);
            SetStatusText($"Step {playerPath.Count}/{mappedPath.Count}");
        }
    }

    // ─────────────────────────────────────────
    // STATUS LIGHT + TEXT
    // ─────────────────────────────────────────

    void SetStatusLight(bool correct)
    {
        if (blinkCoroutine != null)
            StopCoroutine(blinkCoroutine);
        if (statusLight != null)
            blinkCoroutine = StartCoroutine(BlinkLight(correct));
    }

    IEnumerator BlinkLight(bool correct)
    {
        Color blinkColor = correct ? Color.green : Color.red;
        while (true)
        {
            statusLight.color = blinkColor;
            yield return new WaitForSeconds(blinkSpeed);
            statusLight.color = offColor;
            yield return new WaitForSeconds(blinkSpeed);
        }
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
        playerPath.Clear();

        // Start at second column second row from top
        // (1, gridSize-2) = inner top left
        cursorPos = new Vector2Int(1, gridSize - 2);

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

        if (statusLight != null)
            statusLight.color = offColor;

        SetStatusText("Move joystick to trace path");

        // Build mapped path from lily pad
        BuildMappedPath();

        // Show cursor at start position
        leds[cursorPos.x, cursorPos.y].color = cursorColor;

        Debug.Log($"LED reset - cursor at {cursorPos}");
    }
}