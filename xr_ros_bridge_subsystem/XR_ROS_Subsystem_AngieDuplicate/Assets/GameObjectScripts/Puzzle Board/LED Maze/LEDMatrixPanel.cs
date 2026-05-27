

// // using UnityEngine;
// // using UnityEngine.UI;
// // using TMPro;
// // using System.Collections;
// // using System.Collections.Generic;

// // public class LEDMatrixPanel : MonoBehaviour
// // {
// //     [Header("Grid Settings")]
// //     public int gridSize = 8;

// //     [Header("LED Colors")]
// //     public Color offColor = new Color(0.1f, 0.1f, 0.1f);
// //     public Color onColor = new Color(0f, 1f, 0f);
// //     public Color cursorColor = new Color(1f, 1f, 0f);
// //     public Color wrongColor = new Color(1f, 0f, 0f);
// //     public Color borderColor = new Color(0.02f, 0.02f, 0.02f);
// //     public Color tickColor = new Color(0f, 1f, 0f);
// //     public Color crossColor = new Color(1f, 0f, 0f);

// //     [Header("UI References")]
// //     public GameObject ledPrefab;
// //     public Transform gridParent;
// //     public TextMeshProUGUI statusText;

// //     [Header("Lily Pad Grid Reference")]
// //     public LilyPadGrid lilyPadGrid;

// //     [Header("Puzzle Board Manager")]
// //     public PuzzleBoardManager puzzleBoardManager;

// //     [Header("Reset Settings")]
// //     public float wrongResetDelay = 1.5f;

// //     private Image[,] leds;
// //     private Vector2Int cursorPos;
// //     private Vector2Int startPos;
// //     private List<Vector2Int> playerPath = new List<Vector2Int>();
// //     private List<Vector2Int> mappedPath = new List<Vector2Int>();
// //     private bool puzzleSolved = false;
// //     private bool isResetting = false;
// //     private Coroutine blinkCoroutine;

// //     private int innerMin = 1;
// //     private int innerMax; // = gridSize - 2 = 6 for 8x8

// //     void Start()
// //     {
// //         innerMin = 1;
// //         innerMax = gridSize - 2; // 8-2 = 6
// //         GenerateLEDGrid();
// //         StartCoroutine(DelayedReset());
// //     }

// //     IEnumerator DelayedReset()
// //     {
// //         yield return new WaitForSeconds(0.5f);
// //         while (lilyPadGrid == null || lilyPadGrid.currentPath == null
// //                || lilyPadGrid.currentPath.Count == 0)
// //         {
// //             Debug.Log("Waiting for lily pad path...");
// //             yield return new WaitForSeconds(0.1f);
// //         }
// //         Debug.Log($"Path ready: {lilyPadGrid.currentPath.Count} points");
// //         ResetMatrix();
// //     }

// //     // ─────────────────────────────────────────
// //     // GENERATE 8x8 GRID
// //     // Border = row/col 0 and 7
// //     // Inner = rows/cols 1-6
// //     // Spawns top to bottom so Y=7 is top Y=0 is bottom
// //     // ─────────────────────────────────────────

// //     void GenerateLEDGrid()
// //     {
// //         leds = new Image[gridSize, gridSize];

// //         for (int row = 0; row < gridSize; row++)
// //         {
// //             int y = gridSize - 1 - row; // row 0 = y7 (top), row 7 = y0 (bottom)
// //             for (int x = 0; x < gridSize; x++)
// //             {
// //                 GameObject led = Instantiate(ledPrefab, gridParent);
// //                 led.name = $"LED_{x}_{y}";
// //                 Image img = led.GetComponent<Image>();
// //                 leds[x, y] = img;

// //                 bool isBorder = x == 0 || x == gridSize - 1 ||
// //                                 y == 0 || y == gridSize - 1;
// //                 img.color = isBorder ? borderColor : offColor;
// //             }
// //         }

// //         Debug.Log($"Generated {gridSize}x{gridSize} LED grid");
// //         Debug.Log($"Inner area: col {innerMin}-{innerMax}, row {innerMin}-{innerMax}");
// //     }

// //     // ─────────────────────────────────────────
// //     // MAP LILY PAD PATH TO LED GRID
// //     //
// //     // Lily pad grid: 6x6 points (0-5 both axes)
// //     // LED inner grid: 6x6 playable (1-6 both axes)
// //     //
// //     // Lily pad Y=0 = visual top
// //     // LED Y=7 = visual top (spawned top to bottom)
// //     // LED Y=1 = visual bottom inner
// //     //
// //     // So lily (0,0) top left → LED (1, innerMax) = (1,6) top left inner
// //     // ledX = lilyX + 1
// //     // ledY = innerMax - lilyY  (flip because lily Y increases down, LED Y increases up)
// //     // ─────────────────────────────────────────

// //     void BuildMappedPath()
// //     {
// //         mappedPath.Clear();

// //         if (lilyPadGrid == null || lilyPadGrid.currentPath == null
// //             || lilyPadGrid.currentPath.Count == 0)
// //         {
// //             Debug.LogWarning("No lily pad path!");
// //             return;
// //         }
        
// //         // foreach (Vector2Int p in lilyPadGrid.currentPath)
// //         // {
// //         //     int ledX = Mathf.Clamp(p.x + 1, innerMin, innerMax);
// //         //     // Change this line - remove the flip
// //         //     int ledY = Mathf.Clamp(p.y + 1, innerMin, innerMax);
// //         //     mappedPath.Add(new Vector2Int(ledX, ledY));
// //         // }
// //         // foreach (Vector2Int p in lilyPadGrid.currentPath)
// //         // {
// //         //     int ledX = Mathf.Clamp(p.x + 1, innerMin, innerMax);
// //         //     int ledY = Mathf.Clamp(innerMax - p.y, innerMin, innerMax);
// //         //     mappedPath.Add(new Vector2Int(ledX, ledY));
// //         // }
// //         foreach (Vector2Int p in lilyPadGrid.currentPath)
// //         {
// //             // Swap: lily X maps to LED Y, lily Y maps to LED X
// //             int ledX = Mathf.Clamp(p.y + 1, innerMin, innerMax);
// //             int ledY = Mathf.Clamp(innerMax - p.x, innerMin, innerMax);
// //             mappedPath.Add(new Vector2Int(ledX, ledY));
// //         }

// //         // foreach (Vector2Int p in lilyPadGrid.currentPath)
// //         // {
// //         //     int ledX = Mathf.Clamp(p.x + 1, innerMin, innerMax);
// //         //     int ledY = Mathf.Clamp(innerMax - p.y, innerMin, innerMax);
// //         //     mappedPath.Add(new Vector2Int(ledX, ledY));
// //         // }

// //         string lily = "Lily: ";
// //         string led = "LED:  ";
// //         for (int i = 0; i < lilyPadGrid.currentPath.Count; i++)
// //         {
// //             lily += $"({lilyPadGrid.currentPath[i].x}," +
// //                     $"{lilyPadGrid.currentPath[i].y}) ";
// //             led += $"({mappedPath[i].x},{mappedPath[i].y}) ";
// //         }
// //         Debug.Log(lily);
// //         Debug.Log(led);
// //         Debug.Log($"Start point: Lily(0,0) → LED({mappedPath[0].x},{mappedPath[0].y})");
// //     }

// //     // ─────────────────────────────────────────
// //     // MOVEMENT
// //     // ─────────────────────────────────────────

// //     public void MoveUp()    => TryMove(Vector2Int.up);
// //     public void MoveDown()  => TryMove(Vector2Int.down);
// //     public void MoveLeft()  => TryMove(Vector2Int.left);
// //     public void MoveRight() => TryMove(Vector2Int.right);

// //     void TryMove(Vector2Int direction)
// //     {
// //         if (puzzleSolved || isResetting) return;

// //         Vector2Int newPos = cursorPos + direction;

// //         // Clamp to inner grid only - never touch borders
// //         newPos.x = Mathf.Clamp(newPos.x, innerMin, innerMax);
// //         newPos.y = Mathf.Clamp(newPos.y, innerMin, innerMax);

// //         if (newPos == cursorPos) return;

// //         // Light up trail
// //         leds[cursorPos.x, cursorPos.y].color = onColor;

// //         cursorPos = newPos;
// //         playerPath.Add(cursorPos);
// //         leds[cursorPos.x, cursorPos.y].color = cursorColor;

// //         Debug.Log($"Cursor → {cursorPos}");
// //         CheckPath();
// //     }

// //     // ─────────────────────────────────────────
// //     // PATH VALIDATION
// //     // ─────────────────────────────────────────

// //     void CheckPath()
// //     {
// //         if (mappedPath == null || mappedPath.Count == 0)
// //         {
// //             Debug.LogWarning("No mapped path!");
// //             BuildMappedPath();
// //             return;
// //         }

// //         int step = playerPath.Count - 1;

// //         // Skip validation for step 0 - cursor starts there already
// //         if (step == 0)
// //         {
// //             SetStatusText($"Step {playerPath.Count}/{mappedPath.Count}");
// //             return;
// //         }

// //         Debug.Log($"Step {step}: at {playerPath[step]} expected " +
// //                 $"{(step < mappedPath.Count ? mappedPath[step].ToString() : "end")}");

// //         if (step < mappedPath.Count && playerPath[step] != mappedPath[step])
// //         {
// //             StartCoroutine(WrongStepSequence());
// //             return;
// //         }

// //         if (playerPath.Count >= mappedPath.Count)
// //         {
// //             puzzleSolved = true;
// //             ShowPixelTick();
// //             SetStatusText("✓ Correct!\nPuzzle Solved!");
// //             Debug.Log("LED Matrix SOLVED!");

// //             if (puzzleBoardManager != null)
// //                 puzzleBoardManager.OnLEDMatrixSolved();
// //         }
// //         else
// //         {
// //             SetStatusText($"Step {playerPath.Count}/{mappedPath.Count}");
// //         }
// //     }

// //     // ─────────────────────────────────────────
// //     // WRONG STEP - flash red then reset to start
// //     // ─────────────────────────────────────────

    
// //     IEnumerator WrongStepSequence()
// //     {
// //         isResetting = true;

// //         leds[cursorPos.x, cursorPos.y].color = wrongColor;
// //         SetStatusText("✗ Wrong!\nResetting...");
// //         Debug.Log($"WRONG at {cursorPos}! Resetting...");

// //         if (puzzleBoardManager != null)
// //             puzzleBoardManager.OnLEDMatrixFailed();

// //         yield return new WaitForSeconds(wrongResetDelay);

// //         ClearInnerGrid();

// //         // Reset path and add start point
// //         playerPath.Clear();
// //         playerPath.Add(startPos); // ← add this

// //         cursorPos = startPos;

// //         if (InBounds(cursorPos))
// //             leds[cursorPos.x, cursorPos.y].color = cursorColor;

// //         SetStatusText("Move joystick to trace path");
// //         Debug.Log($"Reset to start: {startPos}");

// //         isResetting = false;
// //     }

// //     // ─────────────────────────────────────────
// //     // PIXEL TICK - drawn on inner 6x6 (1-6)
// //     // ─────────────────────────────────────────

// //     void ShowPixelTick()
// //     {
// //         ClearInnerGrid();
// //         Vector2Int[] pixels = new Vector2Int[]
// //         {
// //             new Vector2Int(1, 3),
// //             new Vector2Int(2, 2),
// //             new Vector2Int(3, 3),
// //             new Vector2Int(4, 4),
// //             new Vector2Int(5, 5),
// //             new Vector2Int(6, 6),
// //         };
// //         foreach (Vector2Int p in pixels)
// //             if (InBounds(p)) leds[p.x, p.y].color = tickColor;
// //     }

// //     // ─────────────────────────────────────────
// //     // PIXEL CROSS - drawn on inner 6x6 (1-6)
// //     // ─────────────────────────────────────────

// //     void ShowPixelCross()
// //     {
// //         ClearInnerGrid();
// //         Vector2Int[] pixels = new Vector2Int[]
// //         {
// //             new Vector2Int(1, 6),
// //             new Vector2Int(2, 5),
// //             new Vector2Int(3, 4),
// //             new Vector2Int(4, 3),
// //             new Vector2Int(5, 2),
// //             new Vector2Int(6, 1),
// //             new Vector2Int(1, 1),
// //             new Vector2Int(2, 2),
// //             new Vector2Int(3, 3),
// //             new Vector2Int(4, 4),
// //             new Vector2Int(5, 5),
// //             new Vector2Int(6, 6),
// //         };
// //         foreach (Vector2Int p in pixels)
// //             if (InBounds(p)) leds[p.x, p.y].color = crossColor;
// //     }

// //     void ClearInnerGrid()
// //     {
// //         for (int x = innerMin; x <= innerMax; x++)
// //             for (int y = innerMin; y <= innerMax; y++)
// //                 leds[x, y].color = offColor;
// //     }

// //     bool InBounds(Vector2Int p)
// //     {
// //         return p.x >= innerMin && p.x <= innerMax &&
// //                p.y >= innerMin && p.y <= innerMax;
// //     }

// //     void SetStatusText(string message)
// //     {
// //         if (statusText != null)
// //             statusText.text = message;
// //     }

// //     // ─────────────────────────────────────────
// //     // RESET
// //     // ─────────────────────────────────────────

// //     public void ResetMatrix()
// //     {
// //         puzzleSolved = false;
// //         isResetting = false;
// //         //playerPath.Clear();

// //         if (blinkCoroutine != null)
// //             StopCoroutine(blinkCoroutine);

// //         if (leds == null) return;

// //         // Reset all LEDs
// //         for (int x = 0; x < gridSize; x++)
// //         {
// //             for (int y = 0; y < gridSize; y++)
// //             {
// //                 if (leds[x, y] == null) continue;
// //                 bool isBorder = x == 0 || x == gridSize - 1 ||
// //                                 y == 0 || y == gridSize - 1;
// //                 leds[x, y].color = isBorder ? borderColor : offColor;
// //             }
// //         }

// //         SetStatusText("Move joystick to trace path");

// //         // Build mapped path
// //         BuildMappedPath();

// //         // Start at first mapped point
// //         // Lily (0,0) → LED (1, innerMax) = (1,6) = top left inner
// //         if (mappedPath != null && mappedPath.Count > 0)
// //             startPos = mappedPath[0];
// //         else
// //             startPos = new Vector2Int(innerMin, innerMax);

// //         cursorPos = startPos;

// //         if (InBounds(cursorPos))
// //             leds[cursorPos.x, cursorPos.y].color = cursorColor;

// //         Debug.Log($"Reset - cursor at {cursorPos}");
// //         Debug.Log($"Inner grid: ({innerMin},{innerMin}) to ({innerMax},{innerMax})");
        
// //         playerPath.Clear();
// //         playerPath.Add(cursorPos); // add start point so step 0 is already counted
// //     }

// //     // Called by ROS bridge to mirror Arduino LED states
// //     public void SetLEDStatesFromROS(byte[] ledData)
// //     {
// //         if (leds == null || ledData == null) return;
// //         if (ledData.Length < gridSize * gridSize) return;

// //         for (int row = 0; row < gridSize; row++)
// //         {
// //             for (int col = 0; col < gridSize; col++)
// //             {
// //                 int index = row * gridSize + col;
// //                 int x = col;
// //                 int y = row;
// //                 //int y = (gridSize - 1) - row; // flip
// //                 //int y = gridSize - 1 - row; // flip Y for Unity UI

// //                 if (leds[x, y] == null) continue;

// //                 bool isBorder = x == 0 || x == gridSize - 1 ||
// //                                 y == 0 || y == gridSize - 1;

// //                 if (isBorder)
// //                 {
// //                     leds[x, y].color = borderColor;
// //                     continue;
// //                 }

// //                 leds[x, y].color = ledData[index] == 1 ? onColor : offColor;
// //             }
// //         }

// //         // Update cursor position from Arduino posX/posY
// //         // These come through joystick topic so no extra work needed
// //     }
// // }

// // using UnityEngine;
// // using UnityEngine.UI;
// // using TMPro;
// // using System.Collections;
// // using System.Collections.Generic;
 
// // public class LEDMatrixPanel : MonoBehaviour
// // {
// //     [Header("Grid Settings")]
// //     public int gridSize = 8;
 
// //     [Header("LED Colors")]
// //     public Color offColor = new Color(0.1f, 0.1f, 0.1f);
// //     public Color onColor = new Color(0f, 1f, 0f);
// //     public Color cursorColor = new Color(1f, 1f, 0f);
// //     public Color wrongColor = new Color(1f, 0f, 0f);
// //     public Color borderColor = new Color(0.02f, 0.02f, 0.02f);
// //     public Color tickColor = new Color(0f, 1f, 0f);
// //     public Color crossColor = new Color(1f, 0f, 0f);
// //     public Color idleWarningColor = new Color(1f, 0.5f, 0f);
 
// //     [Header("UI References")]
// //     public GameObject ledPrefab;
// //     public Transform gridParent;
// //     public TextMeshProUGUI statusText;
 
// //     [Header("Lily Pad Grid Reference")]
// //     public LilyPadGrid lilyPadGrid;
 
// //     [Header("Puzzle Board Manager")]
// //     public PuzzleBoardManager puzzleBoardManager;
 
// //     [Header("Reset Settings")]
// //     public float wrongResetDelay = 1.5f;
 
// //     [Header("Idle Settings")]
// //     public float idleResetTime = 5f;
 
// //     private Image[,] leds;
// //     private Vector2Int cursorPos;
// //     private Vector2Int startPos;
// //     private List<Vector2Int> playerPath = new List<Vector2Int>();
// //     private List<Vector2Int> mappedPath = new List<Vector2Int>();
// //     private bool puzzleSolved = false;
// //     private bool isResetting = false;
// //     private Coroutine blinkCoroutine;
// //     private float lastMoveTime;
// //     private int innerMin = 1;
// //     private int innerMax;
 
// //     void Start()
// //     {
// //         innerMin = 1;
// //         innerMax = gridSize - 2;
// //         GenerateLEDGrid();
// //         StartCoroutine(DelayedReset());
// //     }
 
// //     void Update()
// //     {
// //         if (!puzzleSolved && !isResetting && playerPath.Count > 1)
// //         {
// //             if (Time.time - lastMoveTime > idleResetTime)
// //             {
// //                 Debug.Log("Idle timeout - resetting!");
// //                 StartCoroutine(IdleResetSequence());
// //             }
// //         }
// //     }
 
// //     IEnumerator DelayedReset()
// //     {
// //         yield return new WaitForSeconds(0.5f);
// //         while (lilyPadGrid == null || lilyPadGrid.currentPath == null
// //                || lilyPadGrid.currentPath.Count == 0)
// //         {
// //             Debug.Log("Waiting for lily pad path...");
// //             yield return new WaitForSeconds(0.1f);
// //         }
// //         Debug.Log($"Path ready: {lilyPadGrid.currentPath.Count} points");
// //         ResetMatrix();
// //     }
 
// //     // ─────────────────────────────────────────
// //     // GENERATE 8x8 GRID
// //     // ─────────────────────────────────────────
 
// //     void GenerateLEDGrid()
// //     {
// //         leds = new Image[gridSize, gridSize];
 
// //         for (int row = 0; row < gridSize; row++)
// //         {
// //             int y = gridSize - 1 - row;
// //             for (int x = 0; x < gridSize; x++)
// //             {
// //                 GameObject led = Instantiate(ledPrefab, gridParent);
// //                 led.name = $"LED_{x}_{y}";
// //                 Image img = led.GetComponent<Image>();
// //                 leds[x, y] = img;
 
// //                 bool isBorder = x == 0 || x == gridSize - 1 ||
// //                                 y == 0 || y == gridSize - 1;
// //                 img.color = isBorder ? borderColor : offColor;
// //             }
// //         }
 
// //         Debug.Log($"Generated {gridSize}x{gridSize} LED grid");
// //     }
 
// //     // ─────────────────────────────────────────
// //     // MAP LILY PAD PATH TO LED GRID
// //     // ─────────────────────────────────────────
 
// //     void BuildMappedPath()
// //     {
// //         mappedPath.Clear();
 
// //         if (lilyPadGrid == null || lilyPadGrid.currentPath == null
// //             || lilyPadGrid.currentPath.Count == 0)
// //         {
// //             Debug.LogWarning("No lily pad path!");
// //             return;
// //         }
 
// //         foreach (Vector2Int p in lilyPadGrid.currentPath)
// //         {
// //             int ledX = Mathf.Clamp(innerMax - p.x, innerMin, innerMax);
// //             int ledY = Mathf.Clamp(innerMax - p.y, innerMin, innerMax);
// //             mappedPath.Add(new Vector2Int(ledX, ledY));
// //         }
 
// //         string lily = "Lily: ";
// //         string led = "LED:  ";
// //         for (int i = 0; i < lilyPadGrid.currentPath.Count; i++)
// //         {
// //             lily += $"({lilyPadGrid.currentPath[i].x},{lilyPadGrid.currentPath[i].y}) ";
// //             led += $"({mappedPath[i].x},{mappedPath[i].y}) ";
// //         }
// //         Debug.Log(lily);
// //         Debug.Log(led);
// //         Debug.Log($"Start: Lily(0,0) → LED({mappedPath[0].x},{mappedPath[0].y})");
// //         Debug.Log($"End: LED({mappedPath[mappedPath.Count-1].x},{mappedPath[mappedPath.Count-1].y})");
// //     }
 
// //     // ─────────────────────────────────────────
// //     // MOVEMENT
// //     // ─────────────────────────────────────────
 
// //     public void MoveUp()    => TryMove(Vector2Int.up);
// //     public void MoveDown()  => TryMove(Vector2Int.down);
// //     public void MoveLeft()  => TryMove(Vector2Int.left);
// //     public void MoveRight() => TryMove(Vector2Int.right);
 
// //     void TryMove(Vector2Int direction)
// //     {
// //         if (puzzleSolved || isResetting) return;
 
// //         Vector2Int newPos = cursorPos + direction;
 
// //         newPos.x = Mathf.Clamp(newPos.x, innerMin, innerMax);
// //         newPos.y = Mathf.Clamp(newPos.y, innerMin, innerMax);
 
// //         if (newPos == cursorPos) return;
 
// //         lastMoveTime = Time.time;
 
// //         leds[cursorPos.x, cursorPos.y].color = onColor;
// //         cursorPos = newPos;
// //         playerPath.Add(cursorPos);
// //         leds[cursorPos.x, cursorPos.y].color = cursorColor;
 
// //         Debug.Log($"Cursor → {cursorPos}");
// //         CheckPath();
// //     }
 
// //     // ─────────────────────────────────────────
// //     // PATH VALIDATION - only at end edge
// //     // ─────────────────────────────────────────
 
// //     // void CheckPath()
// //     // {
// //     //     if (mappedPath == null || mappedPath.Count == 0)
// //     //     {
// //     //         Debug.LogWarning("No mapped path!");
// //     //         BuildMappedPath();
// //     //         return;
// //     //     }
 
// //     //     int step = playerPath.Count - 1;
 
// //     //     if (step == 0)
// //     //     {
// //     //         SetStatusText("Move joystick to trace path");
// //     //         return;
// //     //     }
 
// //     //     SetStatusText($"Step {playerPath.Count}/{mappedPath.Count}");
 
// //     //     // Only validate at end edge of inner grid
// //     //     bool reachedEnd = cursorPos.x == innerMax ||
// //     //                       cursorPos.y == innerMin ||
// //     //                       cursorPos.x == innerMin ||
// //     //                       cursorPos.y == innerMax;
 
// //     //     if (!reachedEnd) return;
 
// //     //     Debug.Log($"Reached edge {cursorPos} - validating...");
 
// //     //     Vector2Int correctEnd = mappedPath[mappedPath.Count - 1];
 
// //     //     if (cursorPos == correctEnd && playerPath.Count == mappedPath.Count)
// //     //     {
// //     //         puzzleSolved = true;
// //     //         ShowPixelTick();
// //     //         SetStatusText("✓ Correct Path!\nPuzzle Solved!");
// //     //         Debug.Log("LED Matrix SOLVED!");
 
// //     //         if (puzzleBoardManager != null)
// //     //             puzzleBoardManager.OnLEDMatrixSolved();
// //     //     }
// //     //     else
// //     //     {
// //     //         Debug.Log($"Wrong! Got {cursorPos} expected {correctEnd} " +
// //     //                   $"steps {playerPath.Count}/{mappedPath.Count}");
// //     //         StartCoroutine(WrongStepSequence());
// //     //     }
// //     // }
 
// //     // // ─────────────────────────────────────────
// //     // // WRONG - show cross + text then reset
// //     // // ─────────────────────────────────────────
 
// //     // IEnumerator WrongStepSequence()
// //     // {
// //     //     isResetting = true;
 
// //     //     ShowPixelCross();
// //     //     SetStatusText("✗ Wrong Path!\nTry Again");
// //     //     Debug.Log($"WRONG at {cursorPos}!");
 
// //     //     if (puzzleBoardManager != null)
// //     //         puzzleBoardManager.OnLEDMatrixFailed();
 
// //     //     yield return new WaitForSeconds(wrongResetDelay);
 
// //     //     ClearInnerGrid();
// //     //     playerPath.Clear();
// //     //     playerPath.Add(startPos);
// //     //     cursorPos = startPos;
 
// //     //     if (InBounds(cursorPos))
// //     //         leds[cursorPos.x, cursorPos.y].color = cursorColor;
 
// //     //     SetStatusText("Move joystick to trace path");
// //     //     lastMoveTime = Time.time;
// //     //     Debug.Log($"Reset to start: {startPos}");
 
// //     //     isResetting = false;
// //     // }
// //     // ─────────────────────────────────────────
// // // PATH VALIDATION
// // // ─────────────────────────────────────────
// //     void CheckPath()
// //     {
// //         if (mappedPath == null || mappedPath.Count == 0)
// //         {
// //             Debug.LogWarning("No mapped path!");
// //             BuildMappedPath();
// //             return;
// //         }

// //         // Check if the player has reached ANY of the possible end positions
// //         // (mapped from LilyPadGrid's possibleEndCells)
// //         if (!IsEndPosition(cursorPos)) 
// //         {
// //             // Just update status, no validation yet
// //             SetStatusText($"Step {playerPath.Count}");
// //             return;
// //         }

// //         // Player reached an end cell — now check if the full path is correct
// //         Debug.Log($"Reached end position {cursorPos}. Checking path...");

// //         bool pathCorrect = IsPathCorrect();

// //         if (pathCorrect)
// //         {
// //             puzzleSolved = true;
// //             ShowPixelTick();
// //             SetStatusText("✓ Correct!\nPuzzle Solved!");
// //             Debug.Log("LED Matrix SOLVED!");

// //             if (puzzleBoardManager != null)
// //                 puzzleBoardManager.OnLEDMatrixSolved();
// //         }
// //         else
// //         {
// //             StartCoroutine(WrongEndSequence());
// //         }
// //     }

// //     // Returns true if cursorPos matches one of the mapped end positions
// //     bool IsEndPosition(Vector2Int pos)
// //     {
// //         // The last point in mappedPath is always the correct end
// //         // But we also want to trigger on ANY of the 4 possible ends
// //         // mapped from LilyPadGrid.possibleEndCells
// //         if (mappedPath.Count > 0 && pos == mappedPath[mappedPath.Count - 1])
// //             return true;

// //         // Map all possibleEndCells to LED coords and check
// //         // (same mapping as BuildMappedPath)
// //         Vector2Int[] possibleEnds = new Vector2Int[]
// //         {
// //             new Vector2Int(1, 5),
// //             new Vector2Int(3, 5),
// //             new Vector2Int(5, 1),
// //             new Vector2Int(5, 3)
// //         };

// //         foreach (Vector2Int p in possibleEnds)
// //         {
// //             //int ledX = Mathf.Clamp(p.y + 1, innerMin, innerMax);
// //             //int ledY = Mathf.Clamp(innerMax - p.x, innerMin, innerMax);
// //             int ledX = Mathf.Clamp(innerMax - p.x, innerMin, innerMax);
// //             int ledY = Mathf.Clamp(innerMax - p.y, innerMin, innerMax);
// //             //int ledX = Mathf.Clamp(p.y + 1, innerMin, innerMax);
// //             //int ledY = Mathf.Clamp(innerMax - p.x, innerMin, innerMax);
// //             if (pos == new Vector2Int(ledX, ledY))
// //                 return true;
// //         }

// //         return false;
// //     }

// //     // Check if the player's path exactly matches the mapped path
// //     bool IsPathCorrect()
// //     {
// //         if (playerPath.Count != mappedPath.Count) 
// //         {
// //             Debug.Log($"Path length mismatch: player={playerPath.Count} expected={mappedPath.Count}");
// //             return false;
// //         }

// //         for (int i = 0; i < mappedPath.Count; i++)
// //         {
// //             if (playerPath[i] != mappedPath[i])
// //             {
// //                 Debug.Log($"Mismatch at step {i}: player={playerPath[i]} expected={mappedPath[i]}");
// //                 return false;
// //             }
// //         }

// //         return true;
// //     }

// //     // ─────────────────────────────────────────
// //     // WRONG END — show X then reset to start
// //     // ─────────────────────────────────────────

// //     IEnumerator WrongEndSequence()
// //     {
// //         isResetting = true;

// //         ShowPixelCross();
// //         SetStatusText("✗ Wrong path!\nResetting...");
// //         Debug.Log($"Wrong path at end position {cursorPos}! Resetting...");

// //         if (puzzleBoardManager != null)
// //             puzzleBoardManager.OnLEDMatrixFailed();

// //         yield return new WaitForSeconds(wrongResetDelay);

// //         ClearInnerGrid();

// //         playerPath.Clear();
// //         playerPath.Add(startPos);

// //         cursorPos = startPos;

// //         if (InBounds(cursorPos))
// //             leds[cursorPos.x, cursorPos.y].color = cursorColor;

// //         SetStatusText("Move joystick to trace path");
// //         Debug.Log($"Reset to start: {startPos}");

// //         isResetting = false;
// //     }
 
// //     // ─────────────────────────────────────────
// //     // IDLE RESET
// //     // ─────────────────────────────────────────
 
// //     IEnumerator IdleResetSequence()
// //     {
// //         isResetting = true;
 
// //         SetStatusText("Idle...\nResetting soon!");
 
// //         for (int x = innerMin; x <= innerMax; x++)
// //             for (int y = innerMin; y <= innerMax; y++)
// //                 if (leds[x, y] != null)
// //                     leds[x, y].color = idleWarningColor;
 
// //         yield return new WaitForSeconds(1.5f);
 
// //         ClearInnerGrid();
// //         playerPath.Clear();
// //         playerPath.Add(startPos);
// //         cursorPos = startPos;
 
// //         if (InBounds(cursorPos))
// //             leds[cursorPos.x, cursorPos.y].color = cursorColor;
 
// //         SetStatusText("Move joystick to trace path");
// //         lastMoveTime = Time.time;
// //         Debug.Log("Idle reset complete");
 
// //         isResetting = false;
// //     }
 
// //     // ─────────────────────────────────────────
// //     // PIXEL TICK
// //     // ─────────────────────────────────────────
 
// //     void ShowPixelTick()
// //     {
// //         ClearInnerGrid();
// //         Vector2Int[] pixels = new Vector2Int[]
// //         {
// //             new Vector2Int(1, 3),
// //             new Vector2Int(2, 2),
// //             new Vector2Int(3, 3),
// //             new Vector2Int(4, 4),
// //             new Vector2Int(5, 5),
// //             new Vector2Int(6, 6),
// //         };
// //         foreach (Vector2Int p in pixels)
// //             if (InBounds(p)) leds[p.x, p.y].color = tickColor;
// //     }
 
// //     // ─────────────────────────────────────────
// //     // PIXEL CROSS
// //     // ─────────────────────────────────────────
 
// //     void ShowPixelCross()
// //     {
// //         ClearInnerGrid();
// //         Vector2Int[] pixels = new Vector2Int[]
// //         {
// //             new Vector2Int(1, 6),
// //             new Vector2Int(2, 5),
// //             new Vector2Int(3, 4),
// //             new Vector2Int(4, 3),
// //             new Vector2Int(5, 2),
// //             new Vector2Int(6, 1),
// //             new Vector2Int(1, 1),
// //             new Vector2Int(2, 2),
// //             new Vector2Int(3, 3),
// //             new Vector2Int(4, 4),
// //             new Vector2Int(5, 5),
// //             new Vector2Int(6, 6),
// //         };
// //         foreach (Vector2Int p in pixels)
// //             if (InBounds(p)) leds[p.x, p.y].color = crossColor;
// //     }
 
// //     void ClearInnerGrid()
// //     {
// //         for (int x = innerMin; x <= innerMax; x++)
// //             for (int y = innerMin; y <= innerMax; y++)
// //                 leds[x, y].color = offColor;
// //     }
 
// //     bool InBounds(Vector2Int p)
// //     {
// //         return p.x >= innerMin && p.x <= innerMax &&
// //                p.y >= innerMin && p.y <= innerMax;
// //     }
 
// //     void SetStatusText(string message)
// //     {
// //         if (statusText != null)
// //             statusText.text = message;
// //     }
 
// //     // ─────────────────────────────────────────
// //     // RESET
// //     // ─────────────────────────────────────────
 
// //     public void ResetMatrix()
// //     {
// //         puzzleSolved = false;
// //         isResetting = false;
 
// //         if (blinkCoroutine != null)
// //             StopCoroutine(blinkCoroutine);
 
// //         if (leds == null) return;
 
// //         for (int x = 0; x < gridSize; x++)
// //         {
// //             for (int y = 0; y < gridSize; y++)
// //             {
// //                 if (leds[x, y] == null) continue;
// //                 bool isBorder = x == 0 || x == gridSize - 1 ||
// //                                 y == 0 || y == gridSize - 1;
// //                 leds[x, y].color = isBorder ? borderColor : offColor;
// //             }
// //         }
 
// //         SetStatusText("Move joystick to trace path");
// //         BuildMappedPath();
 
// //         if (mappedPath != null && mappedPath.Count > 0)
// //             startPos = mappedPath[0];
// //         else
// //             startPos = new Vector2Int(innerMin, innerMax);
 
// //         cursorPos = startPos;
 
// //         if (InBounds(cursorPos))
// //             leds[cursorPos.x, cursorPos.y].color = cursorColor;
 
// //         lastMoveTime = Time.time;
 
// //         Debug.Log($"Reset - cursor at {cursorPos}");
 
// //         playerPath.Clear();
// //         playerPath.Add(cursorPos);
// //     }
 
// //     // ─────────────────────────────────────────
// //     // ROS LED MIRROR
// //     // ─────────────────────────────────────────
 
// //     public void SetLEDStatesFromROS(byte[] ledData)
// //     {
// //         if (leds == null || ledData == null) return;
// //         if (ledData.Length < gridSize * gridSize) return;
 
// //         for (int row = 0; row < gridSize; row++)
// //         {
// //             for (int col = 0; col < gridSize; col++)
// //             {
// //                 int index = row * gridSize + col;
// //                 int x = col;
// //                 int y = row;
 
// //                 if (leds[x, y] == null) continue;
 
// //                 bool isBorder = x == 0 || x == gridSize - 1 ||
// //                                 y == 0 || y == gridSize - 1;
 
// //                 if (isBorder)
// //                 {
// //                     leds[x, y].color = borderColor;
// //                     continue;
// //                 }
 
// //                 leds[x, y].color = ledData[index] == 1 ? onColor : offColor;
// //             }
// //         }
// //     }
// // }

// using UnityEngine;
// using UnityEngine.UI;
// using TMPro;
// using System.Collections;
// using System.Collections.Generic;

// public class LEDMatrixPanel : MonoBehaviour
// {
//     [Header("Grid Settings")]
//     public int gridSize = 8;

//     [Header("LED Colors")]
//     public Color offColor = new Color(0.1f, 0.1f, 0.1f);
//     public Color onColor = new Color(0f, 1f, 0f);
//     public Color cursorColor = new Color(1f, 1f, 0f);
//     public Color wrongColor = new Color(1f, 0f, 0f);
//     public Color borderColor = new Color(0.02f, 0.02f, 0.02f);
//     public Color tickColor = new Color(0f, 1f, 0f);
//     public Color crossColor = new Color(1f, 0f, 0f);
//     public Color idleWarningColor = new Color(1f, 0.5f, 0f);

//     [Header("UI References")]
//     public GameObject ledPrefab;
//     public Transform gridParent;
//     public TextMeshProUGUI statusText;

//     [Header("Lily Pad Grid Reference")]
//     public LilyPadGrid lilyPadGrid;

//     [Header("Puzzle Board Manager")]
//     public PuzzleBoardManager puzzleBoardManager;

//     [Header("Reset Settings")]
//     public float wrongResetDelay = 1.5f;

//     [Header("Idle Settings")]
//     public float idleResetTime = 5f;

//     private Image[,] leds;
//     private Vector2Int cursorPos;
//     private Vector2Int startPos;
//     private List<Vector2Int> playerPath = new List<Vector2Int>();
//     private List<Vector2Int> mappedPath = new List<Vector2Int>();
//     private bool puzzleSolved = false;
//     private bool isResetting = false;
//     private Coroutine blinkCoroutine;
//     private float lastMoveTime;
//     private int innerMin = 1;
//     private int innerMax;

//     void Start()
//     {
//         innerMin = 1;
//         innerMax = gridSize - 2;
//         GenerateLEDGrid();
//         StartCoroutine(DelayedReset());
//     }

//     void Update()
//     {
//         if (!puzzleSolved && !isResetting && playerPath.Count > 1)
//         {
//             if (Time.time - lastMoveTime > idleResetTime)
//             {
//                 Debug.Log("Idle timeout - resetting!");
//                 StartCoroutine(IdleResetSequence());
//             }
//         }
//     }

//     IEnumerator DelayedReset()
//     {
//         yield return new WaitForSeconds(0.5f);
//         while (lilyPadGrid == null || lilyPadGrid.currentPath == null
//                || lilyPadGrid.currentPath.Count == 0)
//         {
//             Debug.Log("Waiting for lily pad path...");
//             yield return new WaitForSeconds(0.1f);
//         }
//         Debug.Log($"Path ready: {lilyPadGrid.currentPath.Count} points");
//         ResetMatrix();
//     }

//     // ─────────────────────────────────────────
//     // GENERATE 8x8 GRID
//     // Border = row/col 0 and 7
//     // Inner = rows/cols 1-6
//     // Spawns top to bottom so Y=7 is top, Y=0 is bottom
//     // ─────────────────────────────────────────

//     void GenerateLEDGrid()
//     {
//         leds = new Image[gridSize, gridSize];

//         for (int row = 0; row < gridSize; row++)
//         {
//             int y = gridSize - 1 - row;
//             for (int x = 0; x < gridSize; x++)
//             {
//                 GameObject led = Instantiate(ledPrefab, gridParent);
//                 led.name = $"LED_{x}_{y}";
//                 Image img = led.GetComponent<Image>();
//                 leds[x, y] = img;

//                 bool isBorder = x == 0 || x == gridSize - 1 ||
//                                 y == 0 || y == gridSize - 1;
//                 img.color = isBorder ? borderColor : offColor;
//             }
//         }

//         Debug.Log($"Generated {gridSize}x{gridSize} LED grid");
//         Debug.Log($"Inner area: col {innerMin}-{innerMax}, row {innerMin}-{innerMax}");
//     }

//     // ─────────────────────────────────────────
//     // MAP LILY PAD PATH TO LED GRID
//     //
//     // Lily pad grid: 6x6 points (0-5 both axes)
//     // LED inner grid: 6x6 playable (1-6 both axes)
//     //
//     // Lily (0,0) = top-left → LED (1, innerMax) = (1,6) top-left inner
//     // ledX = lilyX + 1
//     // ledY = innerMax - lilyY  (flip Y: lily Y increases down, LED Y increases up)
//     // ─────────────────────────────────────────

//     void BuildMappedPath()
//     {
//         mappedPath.Clear();

//         if (lilyPadGrid == null || lilyPadGrid.currentPath == null
//             || lilyPadGrid.currentPath.Count == 0)
//         {
//             Debug.LogWarning("No lily pad path!");
//             return;
//         }

//         foreach (Vector2Int p in lilyPadGrid.currentPath)
//         {
//             //int ledX = Mathf.Clamp(p.x + 1, innerMin, innerMax);
//             //int ledY = Mathf.Clamp(innerMax - p.y, innerMin, innerMax);
//             int ledX = Mathf.Clamp(p.y + 1, innerMin, innerMax);
//             int ledY = Mathf.Clamp(innerMax - p.x, innerMin, innerMax);
//             mappedPath.Add(new Vector2Int(ledX, ledY));
//         }

//         string lily = "Lily: ";
//         string led  = "LED:  ";
//         for (int i = 0; i < lilyPadGrid.currentPath.Count; i++)
//         {
//             lily += $"({lilyPadGrid.currentPath[i].x},{lilyPadGrid.currentPath[i].y}) ";
//             led  += $"({mappedPath[i].x},{mappedPath[i].y}) ";
//         }
//         Debug.Log(lily);
//         Debug.Log(led);
//         Debug.Log($"Start: Lily(0,0) → LED({mappedPath[0].x},{mappedPath[0].y})");
//         Debug.Log($"End:   LED({mappedPath[mappedPath.Count - 1].x},{mappedPath[mappedPath.Count - 1].y})");
//     }

//     // ─────────────────────────────────────────
//     // MOVEMENT
//     // ─────────────────────────────────────────

//     public void MoveUp()    => TryMove(Vector2Int.up);
//     public void MoveDown()  => TryMove(Vector2Int.down);
//     public void MoveLeft()  => TryMove(Vector2Int.left);
//     public void MoveRight() => TryMove(Vector2Int.right);

//     void TryMove(Vector2Int direction)
//     {
//         if (puzzleSolved || isResetting) return;

//         Vector2Int newPos = cursorPos + direction;

//         newPos.x = Mathf.Clamp(newPos.x, innerMin, innerMax);
//         newPos.y = Mathf.Clamp(newPos.y, innerMin, innerMax);

//         if (newPos == cursorPos) return;

//         lastMoveTime = Time.time;

//         leds[cursorPos.x, cursorPos.y].color = onColor;
//         cursorPos = newPos;
//         playerPath.Add(cursorPos);
//         leds[cursorPos.x, cursorPos.y].color = cursorColor;

//         Debug.Log($"Cursor → {cursorPos}");
//         CheckPath();
//     }

//     // ─────────────────────────────────────────
//     // PATH VALIDATION
//     // No penalty mid-path — only judge when the
//     // player reaches one of the 4 possible end cells.
//     // ─────────────────────────────────────────

//     void CheckPath()
//     {
//         if (mappedPath == null || mappedPath.Count == 0)
//         {
//             Debug.LogWarning("No mapped path!");
//             BuildMappedPath();
//             return;
//         }

//         SetStatusText($"Step {playerPath.Count}/{mappedPath.Count}");

//         // Hard limit — exceeded total waypoints, no need to reach an end
//         if (playerPath.Count > mappedPath.Count)
//         {
//             Debug.Log("Exceeded waypoint count — resetting!");
//             StartCoroutine(WrongEndSequence());
//             return;
//         }

//         // Not at an end cell — let them keep moving freely
//         if (!IsEndPosition(cursorPos)) return;

//         // At an end cell — only judge if step count matches exactly
//         if (playerPath.Count == mappedPath.Count)
//         {
//             Debug.Log($"Reached end at correct step count. Checking path...");

//             if (IsPathCorrect())
//             {
//                 puzzleSolved = true;
//                 ShowPixelTick();
//                 SetStatusText("✓ Correct!\nPuzzle Solved!");
//                 Debug.Log("LED Matrix SOLVED!");

//                 if (puzzleBoardManager != null)
//                     puzzleBoardManager.OnLEDMatrixSolved();
//             }
//             else
//             {
//                 // Right number of steps, reached an end, but wrong path
//                 Debug.Log("Wrong path at end position!");
//                 StartCoroutine(WrongEndSequence());
//             }
//         }
//         else
//         {
//             // Reached an end cell but haven't used all steps yet — pass through freely
//             Debug.Log($"Passing through end cell {cursorPos} at step {playerPath.Count}/{mappedPath.Count} — continuing...");
//         }
//     }

//     // ─────────────────────────────────────────
//     // END POSITION CHECK
//     // Triggers on ANY of the 4 possible end cells
//     // (same mapping as BuildMappedPath)
//     // ─────────────────────────────────────────

//     bool IsEndPosition(Vector2Int pos)
//     {
//         // Always trigger on the correct end of the current path
//         if (mappedPath.Count > 0 && pos == mappedPath[mappedPath.Count - 1])
//             return true;

//         // Also trigger on the other 3 possible ends so the player
//         // gets the X feedback if they reach a wrong exit point
//         Vector2Int[] possibleEnds = new Vector2Int[]
//         {
//             new Vector2Int(1, 5),
//             new Vector2Int(3, 5),
//             new Vector2Int(5, 1),
//             new Vector2Int(5, 3)
//         };

//         foreach (Vector2Int p in possibleEnds)
//         {
//             int ledX = Mathf.Clamp(p.y + 1, innerMin, innerMax);
//             int ledY = Mathf.Clamp(innerMax - p.x, innerMin, innerMax);
//             //int ledX = Mathf.Clamp(p.x + 1, innerMin, innerMax);
//             //int ledY = Mathf.Clamp(innerMax - p.y, innerMin, innerMax);
//             if (pos == new Vector2Int(ledX, ledY))
//                 return true;
//         }

//         return false;
//     }

//     // ─────────────────────────────────────────
//     // PATH CORRECTNESS CHECK
//     // Both length AND every step must match
//     // ─────────────────────────────────────────

//     bool IsPathCorrect()
//     {
//         if (playerPath.Count != mappedPath.Count)
//         {
//             Debug.Log($"Path length mismatch: player={playerPath.Count} expected={mappedPath.Count}");
//             return false;
//         }

//         for (int i = 0; i < mappedPath.Count; i++)
//         {
//             if (playerPath[i] != mappedPath[i])
//             {
//                 Debug.Log($"Mismatch at step {i}: player={playerPath[i]} expected={mappedPath[i]}");
//                 return false;
//             }
//         }

//         return true;
//     }

//     // ─────────────────────────────────────────
//     // WRONG END — show pixel X then reset
//     // ─────────────────────────────────────────

//     IEnumerator WrongEndSequence()
//     {
//         isResetting = true;

//         ShowPixelCross();
//         SetStatusText("✗ Wrong path!\nResetting...");
//         Debug.Log($"Wrong path at end position {cursorPos}! Resetting...");

//         if (puzzleBoardManager != null)
//             puzzleBoardManager.OnLEDMatrixFailed();

//         yield return new WaitForSeconds(wrongResetDelay);

//         ClearInnerGrid();
//         playerPath.Clear();
//         playerPath.Add(startPos);
//         cursorPos = startPos;

//         if (InBounds(cursorPos))
//             leds[cursorPos.x, cursorPos.y].color = cursorColor;

//         SetStatusText("Move joystick to trace path");
//         lastMoveTime = Time.time;
//         Debug.Log($"Reset to start: {startPos}");

//         isResetting = false;
//     }

//     // ─────────────────────────────────────────
//     // IDLE RESET
//     // ─────────────────────────────────────────

//     IEnumerator IdleResetSequence()
//     {
//         isResetting = true;

//         SetStatusText("Idle...\nResetting soon!");

//         for (int x = innerMin; x <= innerMax; x++)
//             for (int y = innerMin; y <= innerMax; y++)
//                 if (leds[x, y] != null)
//                     leds[x, y].color = idleWarningColor;

//         yield return new WaitForSeconds(1.5f);

//         ClearInnerGrid();
//         playerPath.Clear();
//         playerPath.Add(startPos);
//         cursorPos = startPos;

//         if (InBounds(cursorPos))
//             leds[cursorPos.x, cursorPos.y].color = cursorColor;

//         SetStatusText("Move joystick to trace path");
//         lastMoveTime = Time.time;
//         Debug.Log("Idle reset complete");

//         isResetting = false;
//     }

//     // ─────────────────────────────────────────
//     // PIXEL TICK — drawn on inner 6x6 (1-6)
//     // ─────────────────────────────────────────

//     void ShowPixelTick()
//     {
//         ClearInnerGrid();
//         Vector2Int[] pixels = new Vector2Int[]
//         {
//             new Vector2Int(1, 3),
//             new Vector2Int(2, 2),
//             new Vector2Int(3, 3),
//             new Vector2Int(4, 4),
//             new Vector2Int(5, 5),
//             new Vector2Int(6, 6),
//         };
//         foreach (Vector2Int p in pixels)
//             if (InBounds(p)) leds[p.x, p.y].color = tickColor;
//     }

//     // ─────────────────────────────────────────
//     // PIXEL CROSS — drawn on inner 6x6 (1-6)
//     // ─────────────────────────────────────────

//     void ShowPixelCross()
//     {
//         ClearInnerGrid();
//         Vector2Int[] pixels = new Vector2Int[]
//         {
//             new Vector2Int(1, 6),
//             new Vector2Int(2, 5),
//             new Vector2Int(3, 4),
//             new Vector2Int(4, 3),
//             new Vector2Int(5, 2),
//             new Vector2Int(6, 1),
//             new Vector2Int(1, 1),
//             new Vector2Int(2, 2),
//             new Vector2Int(3, 3),
//             new Vector2Int(4, 4),
//             new Vector2Int(5, 5),
//             new Vector2Int(6, 6),
//         };
//         foreach (Vector2Int p in pixels)
//             if (InBounds(p)) leds[p.x, p.y].color = crossColor;
//     }

//     void ClearInnerGrid()
//     {
//         for (int x = innerMin; x <= innerMax; x++)
//             for (int y = innerMin; y <= innerMax; y++)
//                 leds[x, y].color = offColor;
//     }

//     bool InBounds(Vector2Int p)
//     {
//         return p.x >= innerMin && p.x <= innerMax &&
//                p.y >= innerMin && p.y <= innerMax;
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
//         isResetting = false;

//         if (blinkCoroutine != null)
//             StopCoroutine(blinkCoroutine);

//         if (leds == null) return;

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

//         SetStatusText("Move joystick to trace path");
//         BuildMappedPath();

//         if (mappedPath != null && mappedPath.Count > 0)
//             startPos = mappedPath[0];
//         else
//             startPos = new Vector2Int(innerMin, innerMax);

//         cursorPos = startPos;

//         if (InBounds(cursorPos))
//             leds[cursorPos.x, cursorPos.y].color = cursorColor;

//         lastMoveTime = Time.time;

//         Debug.Log($"Reset - cursor at {cursorPos}");
//         Debug.Log($"Inner grid: ({innerMin},{innerMin}) to ({innerMax},{innerMax})");

//         playerPath.Clear();
//         playerPath.Add(cursorPos);
//     }

//     // ─────────────────────────────────────────
//     // ROS LED MIRROR
//     // ─────────────────────────────────────────

//     public void SetLEDStatesFromROS(byte[] ledData)
//     {
//         if (leds == null || ledData == null) return;
//         if (ledData.Length < gridSize * gridSize) return;

//         for (int row = 0; row < gridSize; row++)
//         {
//             for (int col = 0; col < gridSize; col++)
//             {
//                 int index = row * gridSize + col;
//                 int x = col;
//                 int y = row;

//                 if (leds[x, y] == null) continue;

//                 bool isBorder = x == 0 || x == gridSize - 1 ||
//                                 y == 0 || y == gridSize - 1;

//                 if (isBorder)
//                 {
//                     leds[x, y].color = borderColor;
//                     continue;
//                 }

//                 leds[x, y].color = ledData[index] == 1 ? onColor : offColor;
//             }
//         }
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
    public Color idleWarningColor = new Color(1f, 0.5f, 0f);

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

    [Header("Idle Settings")]
    public float idleResetTime = 5f;

    private Image[,] leds;
    private Vector2Int cursorPos;
    private Vector2Int startPos;
    private List<Vector2Int> playerPath = new List<Vector2Int>();
    private List<Vector2Int> mappedPath = new List<Vector2Int>();
    private bool puzzleSolved = false;
    private bool isResetting = false;
    private Coroutine blinkCoroutine;
    private float lastMoveTime;
    private int innerMin = 1;
    private int innerMax;

    // ADDED: when true, Arduino is source of truth for validation.
    // Unity only moves the cursor visually. Results come from ROS state topic.
    // Set to false if you want local Unity validation (keyboard testing only).
    private bool rosValidationMode = true;

    void Start()
    {
        innerMin = 1;
        innerMax = gridSize - 2;
        GenerateLEDGrid();
        StartCoroutine(DelayedReset());
    }

    void Update()
    {
        if (!puzzleSolved && !isResetting && playerPath.Count > 1)
        {
            if (Time.time - lastMoveTime > idleResetTime)
            {
                Debug.Log("Idle timeout - resetting!");
                StartCoroutine(IdleResetSequence());
            }
        }
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
    // Unchanged
    // ─────────────────────────────────────────

    void GenerateLEDGrid()
    {
        leds = new Image[gridSize, gridSize];

        for (int row = 0; row < gridSize; row++)
        {
            int y = gridSize - 1 - row;
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
    }

    // ─────────────────────────────────────────
    // MAP LILY PAD PATH TO LED GRID
    // Unchanged — coordinate mapping kept as-is
    // ─────────────────────────────────────────

    void BuildMappedPath()
    {
        mappedPath.Clear();

        if (lilyPadGrid == null || lilyPadGrid.currentPath == null
            || lilyPadGrid.currentPath.Count == 0)
        {
            Debug.LogWarning("No lily pad path to map!");
            return;
        }

        foreach (Vector2Int p in lilyPadGrid.currentPath)
        {
            int ledX = Mathf.Clamp(p.y + 1, innerMin, innerMax);
            int ledY = Mathf.Clamp(innerMax - p.x, innerMin, innerMax);
            mappedPath.Add(new Vector2Int(ledX, ledY));
        }

        string lily = "Lily path: ";
        string led  = "LED path:  ";
        for (int i = 0; i < lilyPadGrid.currentPath.Count; i++)
        {
            lily += $"({lilyPadGrid.currentPath[i].x},{lilyPadGrid.currentPath[i].y}) ";
            led  += $"({mappedPath[i].x},{mappedPath[i].y}) ";
        }
        Debug.Log(lily);
        Debug.Log(led);
    }

    // ─────────────────────────────────────────
    // MOVEMENT
    // Unchanged — joystick moves cursor visually
    // ─────────────────────────────────────────

    public void MoveUp()    => TryMove(Vector2Int.up);
    public void MoveDown()  => TryMove(Vector2Int.down);
    public void MoveLeft()  => TryMove(Vector2Int.left);
    public void MoveRight() => TryMove(Vector2Int.right);

    void TryMove(Vector2Int direction)
    {
        if (puzzleSolved || isResetting) return;

        Vector2Int newPos = cursorPos + direction;
        newPos.x = Mathf.Clamp(newPos.x, innerMin, innerMax);
        newPos.y = Mathf.Clamp(newPos.y, innerMin, innerMax);

        if (newPos == cursorPos) return;

        lastMoveTime = Time.time;

        leds[cursorPos.x, cursorPos.y].color = onColor;
        cursorPos = newPos;
        playerPath.Add(cursorPos);
        leds[cursorPos.x, cursorPos.y].color = cursorColor;

        Debug.Log($"Cursor → {cursorPos}");

        // CHANGED: only run local path check when NOT in ROS validation mode.
        // When rosValidationMode = true, Arduino sends the result via state topic.
        if (!rosValidationMode)
            CheckPath();
        else
            SetStatusText($"Step {playerPath.Count}");
    }

    // ─────────────────────────────────────────
    // PATH VALIDATION (local mode only)
    // CHANGED: only runs when rosValidationMode = false.
    // When Arduino is the source of truth this is bypassed entirely.
    // ─────────────────────────────────────────

    void CheckPath()
    {
        if (mappedPath == null || mappedPath.Count == 0)
        {
            Debug.LogWarning("No mapped path!");
            BuildMappedPath();
            return;
        }

        SetStatusText($"Step {playerPath.Count}/{mappedPath.Count}");

        if (playerPath.Count > mappedPath.Count)
        {
            Debug.Log("Exceeded waypoint count — resetting!");
            StartCoroutine(WrongEndSequence());
            return;
        }

        if (!IsEndPosition(cursorPos)) return;

        if (playerPath.Count == mappedPath.Count)
        {
            if (IsPathCorrect())
            {
                puzzleSolved = true;
                ShowPixelTick();
                SetStatusText("Correct!\nPuzzle Solved!");
                Debug.Log("LED Matrix SOLVED (local)!");

                if (puzzleBoardManager != null)
                    puzzleBoardManager.OnLEDMatrixSolved();
            }
            else
            {
                Debug.Log("Wrong path at end position!");
                StartCoroutine(WrongEndSequence());
            }
        }
    }

    bool IsEndPosition(Vector2Int pos)
    {
        if (mappedPath.Count > 0 && pos == mappedPath[mappedPath.Count - 1])
            return true;

        Vector2Int[] possibleEnds = new Vector2Int[]
        {
            new Vector2Int(1, 5),
            new Vector2Int(3, 5),
            new Vector2Int(5, 1),
            new Vector2Int(5, 3)
        };

        foreach (Vector2Int p in possibleEnds)
        {
            int ledX = Mathf.Clamp(p.y + 1, innerMin, innerMax);
            int ledY = Mathf.Clamp(innerMax - p.x, innerMin, innerMax);
            if (pos == new Vector2Int(ledX, ledY))
                return true;
        }

        return false;
    }

    bool IsPathCorrect()
    {
        if (playerPath.Count != mappedPath.Count) return false;

        for (int i = 0; i < mappedPath.Count; i++)
            if (playerPath[i] != mappedPath[i]) return false;

        return true;
    }

    // ─────────────────────────────────────────
    // SHOW SOLVED FROM ROS
    // ADDED: called by ROSPuzzleBridge when Arduino confirms correct path.
    // Displays tick and notifies PuzzleBoardManager.
    // ─────────────────────────────────────────

    public void ShowSolvedFromROS()
    {
        puzzleSolved = true;
        ShowPixelTick();
        SetStatusText("Correct!\nPuzzle Solved!");
        Debug.Log("LED Matrix SOLVED (from ROS)");

        if (puzzleBoardManager != null)
            puzzleBoardManager.OnLEDMatrixSolved();
    }

    // ─────────────────────────────────────────
    // SHOW FAILED FROM ROS
    // ADDED: called by ROSPuzzleBridge when Arduino confirms wrong path.
    // Displays cross and resets cursor to start.
    // ─────────────────────────────────────────

    public void ShowFailedFromROS()
    {
        Debug.Log("LED Matrix FAILED (from ROS)");
        StartCoroutine(WrongEndSequence());
    }

    // ─────────────────────────────────────────
    // REBUILD AFTER PATH UPDATE
    // ADDED: called by ROSPuzzleBridge after SetPathFromMaze runs.
    // Waits for lily pad path to be ready then calls ResetMatrix
    // so the LED mapped path reflects the new maze data.
    // ─────────────────────────────────────────

    public void RebuildAfterPathUpdate()
    {
        StartCoroutine(RebuildPathCoroutine());
    }

    IEnumerator RebuildPathCoroutine()
    {
        // Wait one frame for SetPathFromMaze to finish
        yield return new WaitForEndOfFrame();

        // Wait until lily pad has a valid path (up to 5 seconds)
        int waited = 0;
        while ((lilyPadGrid == null || lilyPadGrid.currentPath == null
                || lilyPadGrid.currentPath.Count == 0) && waited < 50)
        {
            yield return new WaitForSeconds(0.1f);
            waited++;
        }

        if (lilyPadGrid == null || lilyPadGrid.currentPath == null || lilyPadGrid.currentPath.Count == 0)
        {
            Debug.LogError("RebuildPathCoroutine: lily pad path still empty after waiting!");
            yield break;
        }

        Debug.Log($"Rebuilding LED mapped path from {lilyPadGrid.currentPath.Count} lily pad points");
        ResetMatrix();
    }

    // ─────────────────────────────────────────
    // WRONG END SEQUENCE
    // Unchanged — shows pixel cross then resets
    // ─────────────────────────────────────────

    IEnumerator WrongEndSequence()
    {
        isResetting = true;

        ShowPixelCross();
        SetStatusText("Wrong path!\nResetting...");
        Debug.Log($"Wrong path! Resetting...");

        if (puzzleBoardManager != null)
            puzzleBoardManager.OnLEDMatrixFailed();

        yield return new WaitForSeconds(wrongResetDelay);

        ClearInnerGrid();
        playerPath.Clear();
        playerPath.Add(startPos);
        cursorPos = startPos;

        if (InBounds(cursorPos))
            leds[cursorPos.x, cursorPos.y].color = cursorColor;

        SetStatusText("Move joystick to trace path");
        lastMoveTime = Time.time;
        Debug.Log($"Reset to start: {startPos}");

        isResetting = false;
    }

    // ─────────────────────────────────────────
    // IDLE RESET
    // Unchanged
    // ─────────────────────────────────────────

    IEnumerator IdleResetSequence()
    {
        isResetting = true;

        SetStatusText("Idle...\nResetting soon!");

        for (int x = innerMin; x <= innerMax; x++)
            for (int y = innerMin; y <= innerMax; y++)
                if (leds[x, y] != null)
                    leds[x, y].color = idleWarningColor;

        yield return new WaitForSeconds(1.5f);

        ClearInnerGrid();
        playerPath.Clear();
        playerPath.Add(startPos);
        cursorPos = startPos;

        if (InBounds(cursorPos))
            leds[cursorPos.x, cursorPos.y].color = cursorColor;

        SetStatusText("Move joystick to trace path");
        lastMoveTime = Time.time;
        Debug.Log("Idle reset complete");

        isResetting = false;
    }

    // ─────────────────────────────────────────
    // PIXEL TICK / CROSS
    // Unchanged
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

    void ShowPixelCross()
    {
        ClearInnerGrid();
        Vector2Int[] pixels = new Vector2Int[]
        {
            new Vector2Int(1, 6), new Vector2Int(2, 5),
            new Vector2Int(3, 4), new Vector2Int(4, 3),
            new Vector2Int(5, 2), new Vector2Int(6, 1),
            new Vector2Int(1, 1), new Vector2Int(2, 2),
            new Vector2Int(3, 3), new Vector2Int(4, 4),
            new Vector2Int(5, 5), new Vector2Int(6, 6),
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
    // RESET MATRIX
    // Unchanged — clears grid, rebuilds mapped path, resets cursor
    // ─────────────────────────────────────────

    public void ResetMatrix()
    {
        puzzleSolved = false;
        isResetting = false;

        if (blinkCoroutine != null)
            StopCoroutine(blinkCoroutine);

        if (leds == null) return;

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
        BuildMappedPath();

        if (mappedPath != null && mappedPath.Count > 0)
            startPos = mappedPath[0];
        else
            startPos = new Vector2Int(innerMin, innerMax);

        cursorPos = startPos;

        if (InBounds(cursorPos))
            leds[cursorPos.x, cursorPos.y].color = cursorColor;

        lastMoveTime = Time.time;

        Debug.Log($"ResetMatrix — cursor at {cursorPos}, mapped path has {mappedPath?.Count} steps");

        playerPath.Clear();
        playerPath.Add(cursorPos);
    }

    // ─────────────────────────────────────────
    // ROS LED MIRROR
    // Unchanged — mirrors raw Arduino LED states
    // ─────────────────────────────────────────

    public void SetLEDStatesFromROS(byte[] ledData)
    {
        if (leds == null || ledData == null) return;
        if (ledData.Length < gridSize * gridSize) return;

        for (int row = 0; row < gridSize; row++)
        {
            for (int col = 0; col < gridSize; col++)
            {
                int index = row * gridSize + col;
                int x = col;
                int y = row;

                if (leds[x, y] == null) continue;

                bool isBorder = x == 0 || x == gridSize - 1 ||
                                y == 0 || y == gridSize - 1;

                if (isBorder)
                {
                    leds[x, y].color = borderColor;
                    continue;
                }

                leds[x, y].color = ledData[index] == 1 ? onColor : offColor;
            }
        }
    }
}