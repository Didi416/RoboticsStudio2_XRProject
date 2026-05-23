using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;

public class LEDMatrixPanel : MonoBehaviour
{
    [Header("Grid Settings")]
    public int gridSize = 8;

    [Header("LED Colors")]
    public Color offColor = new Color(0.1f, 0.1f, 0.1f);   // dark grey
    public Color onColor = new Color(0f, 1f, 0f);           // bright green
    public Color cursorColor = new Color(1f, 1f, 0f);       // yellow cursor
    public Color wrongColor = new Color(1f, 0f, 0f);        // red wrong

    [Header("UI References")]
    public GameObject ledButtonPrefab;   // single LED prefab (UI Button)
    public Transform gridParent;         // parent for LED grid
    public Image statusLight;            // centre blink light
    public float blinkSpeed = 0.5f;

    [Header("Lily Pad Grid Reference")]
    public LilyPadGrid lilyPadGrid;      // reference to get correct path

    [Header("Mouse Testing")]
    public bool enableMouseTesting = true;

    // State
    private Image[,] leds;
    private Vector2Int cursorPos = new Vector2Int(0, 7); // start top left
    private List<Vector2Int> playerPath = new List<Vector2Int>();
    private bool puzzleSolved = false;
    private Coroutine blinkCoroutine;

    void Start()
    {
        GenerateLEDGrid();
        ResetMatrix();
    }

    // ─────────────────────────────────────────
    // GENERATE 8x8 LED GRID
    // ─────────────────────────────────────────

    void GenerateLEDGrid()
    {
        leds = new Image[gridSize, gridSize];

        for (int y = gridSize - 1; y >= 0; y--)
        {
            for (int x = 0; x < gridSize; x++)
            {
                GameObject led = Instantiate(ledButtonPrefab, gridParent);
                led.name = $"LED_{x}_{y}";
                Image img = led.GetComponent<Image>();
                leds[x, y] = img;
                img.color = offColor;

                // Mouse click testing
                if (enableMouseTesting)
                {
                    int captureX = x;
                    int captureY = y;
                    Button btn = led.GetComponent<Button>();
                    if (btn != null)
                        btn.onClick.AddListener(() => OnLEDClicked(captureX, captureY));
                }
            }
        }

        // Set cursor at start
        UpdateCursorVisual();
    }

    // ─────────────────────────────────────────
    // ROBOT CALLS THESE TO MOVE
    // Teammate hooks into these from robot script
    // ─────────────────────────────────────────

    public void MoveUp()    => TryMove(Vector2Int.up);
    public void MoveDown()  => TryMove(Vector2Int.down);
    public void MoveLeft()  => TryMove(Vector2Int.left);
    public void MoveRight() => TryMove(Vector2Int.right);

    void TryMove(Vector2Int direction)
    {
        if (puzzleSolved) return;

        Vector2Int newPos = cursorPos + direction;

        // Clamp to grid
        if (newPos.x < 0 || newPos.x >= gridSize ||
            newPos.y < 0 || newPos.y >= gridSize)
            return;

        cursorPos = newPos;
        playerPath.Add(cursorPos);

        // Light up this LED
        leds[cursorPos.x, cursorPos.y].color = onColor;

        UpdateCursorVisual();
        CheckPathCorrectness();
    }

    void UpdateCursorVisual()
    {
        // Flash cursor position yellow
        if (leds != null)
            leds[cursorPos.x, cursorPos.y].color = cursorColor;
    }

    // ─────────────────────────────────────────
    // PATH VALIDATION
    // Compares player path against lily pad path
    // ─────────────────────────────────────────

    void CheckPathCorrectness()
    {
        if (lilyPadGrid == null || lilyPadGrid.currentPath == null) return;

        List<Vector2Int> correctPath = lilyPadGrid.currentPath;

        // Check current step matches
        int step = playerPath.Count - 1;

        if (step < correctPath.Count)
        {
            // Scale lily pad coords (0-5) to LED grid (0-7)
            Vector2Int scaledCorrect = ScaleToLED(correctPath[step]);

            if (playerPath[step] != scaledCorrect)
            {
                // Wrong step
                leds[cursorPos.x, cursorPos.y].color = wrongColor;
                SetStatusLight(false);
                Debug.Log($"Wrong step at {cursorPos}!");
                return;
            }
        }

        // Check if path complete
        if (playerPath.Count >= correctPath.Count)
        {
            puzzleSolved = true;
            SetStatusLight(true);
            Debug.Log("LED Matrix puzzle SOLVED!");
        }
        else
        {
            SetStatusLight(true); // green while correct so far
        }
    }

    // Scale 5x5 lily pad grid coords to 8x8 LED grid
    Vector2Int ScaleToLED(Vector2Int lilyCoord)
    {
        int x = Mathf.RoundToInt((float)lilyCoord.x / 5f * 7f);
        int y = Mathf.RoundToInt((float)lilyCoord.y / 5f * 7f);
        return new Vector2Int(x, y);
    }

    // ─────────────────────────────────────────
    // STATUS LIGHT
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

    // ─────────────────────────────────────────
    // MOUSE TESTING
    // ─────────────────────────────────────────

    void OnLEDClicked(int x, int y)
    {
        if (!enableMouseTesting) return;

        // Simulate moving to clicked LED
        Vector2Int clicked = new Vector2Int(x, y);
        Vector2Int diff = clicked - cursorPos;

        // Move step by step towards clicked LED
        if (Mathf.Abs(diff.x) > 0)
            TryMove(diff.x > 0 ? Vector2Int.right : Vector2Int.left);
        else if (Mathf.Abs(diff.y) > 0)
            TryMove(diff.y > 0 ? Vector2Int.up : Vector2Int.down);
    }

    // ─────────────────────────────────────────
    // RESET
    // ─────────────────────────────────────────

    public void ResetMatrix()
    {
        puzzleSolved = false;
        playerPath.Clear();
        cursorPos = new Vector2Int(0, 7);

        if (blinkCoroutine != null)
            StopCoroutine(blinkCoroutine);

        if (leds == null) return;

        for (int x = 0; x < gridSize; x++)
            for (int y = 0; y < gridSize; y++)
                if (leds[x, y] != null)
                    leds[x, y].color = offColor;

        if (statusLight != null)
            statusLight.color = offColor;

        UpdateCursorVisual();
    }
}