// // using UnityEngine;
// // using Unity.Robotics.ROSTCPConnector;
// // using RosMessageTypes.Std;
// // using RosMessageTypes.Geometry;
// // using System.Collections.Generic;
// // using System.Text;
// // using PimDeWitte.UnityMainThreadDispatcher;

// // public class ROSPuzzleBridge : MonoBehaviour
// // {
// //     [Header("Puzzle Board References")]
// //     public ButtonMatrixPanel buttonMatrixPanel;
// //     public LEDMatrixPanel ledMatrixPanel;
// //     public LilyPadGrid lilyPadGrid;
// //     public FrogCroakScript[] frogSigns;
// //     public PuzzleBoardManager puzzleBoardManager;

// //     [Header("ROS Topics")]
// //     public string buttonsTopic = "/puzzle_board/buttons";
// //     public string displayTopic = "/puzzle_board/display";
// //     public string joystickTopic = "/puzzle_board/joystick";
// //     public string puzzleGenerationTopic = "/puzzle/generation";
// //     public string ledsTopic = "/puzzle_board/leds";
// //     public string puzzleStateTopic = "/puzzle_board/state";

// //     private ROSConnection ros;
// //     private bool ledMirrorMode = false; // true = mirror Arduino LEDs directly

// //     void Start()
// //     {
// //         ros = ROSConnection.GetOrCreateInstance();
// //         Debug.Log($"ROS connecting to: {ros.RosIPAddress}:{ros.RosPort}");

// //         // Subscribe to all topics
// //         ros.Subscribe<RosMessageTypes.Std.Int32MultiArrayMsg>(buttonsTopic, OnButtonsReceived);
// //         ros.Subscribe<RosMessageTypes.Std.StringMsg>(displayTopic, OnDisplayReceived);
// //         ros.Subscribe<Vector3Msg>(joystickTopic, OnJoystickReceived);
// //         ros.Subscribe<RosMessageTypes.Std.StringMsg>(puzzleGenerationTopic, OnPuzzleGenerationReceived);
// //         ros.Subscribe<RosMessageTypes.Std.UInt8MultiArrayMsg>(ledsTopic, OnLEDsReceived);
// //         ros.Subscribe<RosMessageTypes.Std.StringMsg>(puzzleStateTopic, OnPuzzleStateReceived);

// //         Debug.Log("ROSPuzzleBridge: All topics subscribed");
// //     }

// //     // ─────────────────────────────────────────
// //     // BUTTON MATRIX
// //     // Arduino sends button states → Unity registers press
// //     // ─────────────────────────────────────────

// //     void OnButtonsReceived(RosMessageTypes.Std.Int32MultiArrayMsg msg)
// //     {
// //         Debug.Log($"ROS: Button has been pressed");
// //         for (int i = 0; i < msg.data.Length && i < 9; i++)
// //         {
// //             if (msg.data[i] == 1)
// //             {
// //                 int buttonNumber = i + 1;
// //                 Debug.Log($"ROS: Button {buttonNumber} pressed");

// //                 UnityMainThreadDispatcher.Instance().Enqueue(() =>
// //                 {
// //                     if (buttonMatrixPanel != null)
// //                         buttonMatrixPanel.PressButton(buttonNumber);
// //                 });
// //             }
// //         }
// //     }

// //     // ─────────────────────────────────────────
// //     // DISPLAY TEXT
// //     // Sync Arduino SSD display with Unity SSD
// //     // ─────────────────────────────────────────

// //     void OnDisplayReceived(RosMessageTypes.Std.StringMsg msg)
// //     {
// //         string displayText = msg.data;
// //         Debug.Log($"ROS: Display please work: {displayText}");

// //         UnityMainThreadDispatcher.Instance().Enqueue(() =>
// //         {
// //             if (buttonMatrixPanel != null)
// //                 buttonMatrixPanel.SetDisplayFromROS(displayText);
// //         });
// //     }

// //     // ─────────────────────────────────────────
// //     // JOYSTICK
// //     // Arduino joystick → Unity LED cursor movement
// //     // Matches Arduino dead zone and direction logic
// //     // ─────────────────────────────────────────

// //     void OnJoystickReceived(Vector3Msg msg)
// //     {
// //         float x = (float)msg.x;
// //         float y = (float)msg.y;

// //         // Match Arduino DEADZONE (normalised)
// //         float deadZone = 0.3f;

// //         UnityMainThreadDispatcher.Instance().Enqueue(() =>
// //         {
// //             if (ledMatrixPanel == null) return;
// //             if (ledMirrorMode) return; // skip if mirroring LED states directly

// //             // Match Arduino movement logic:
// //             // rawX < 512-DEADZONE → dx = -1 (left)
// //             // rawX > 512+DEADZONE → dx = 1 (right)
// //             // rawY < 512-DEADZONE → dy = 1 (down in Arduino = up in Unity)
// //             // rawY > 512+DEADZONE → dy = -1

// //             if (x < -deadZone && Mathf.Abs(x) > Mathf.Abs(y))
// //                 ledMatrixPanel.MoveLeft();
// //             else if (x > deadZone && Mathf.Abs(x) > Mathf.Abs(y))
// //                 ledMatrixPanel.MoveRight();
// //             else if (y > deadZone && Mathf.Abs(y) > Mathf.Abs(x))
// //                 ledMatrixPanel.MoveUp();
// //             else if (y < -deadZone && Mathf.Abs(y) > Mathf.Abs(x))
// //                 ledMatrixPanel.MoveDown();
// //         });
// //     }

// //     // ─────────────────────────────────────────
// //     // LED MATRIX STATE
// //     // Mirror Arduino LED states directly to Unity
// //     // This overrides coordinate mapping entirely
// //     // ─────────────────────────────────────────

// //     // void OnLEDsReceived(RosMessageTypes.Std.UInt8MultiArrayMsg msg)
// //     // {
// //     //     byte[] ledData = new byte[msg.data.Length];
// //     //     for (int i = 0; i < msg.data.Length; i++)
// //     //         ledData[i] = (byte)msg.data[i];

// //     //     UnityMainThreadDispatcher.Instance().Enqueue(() =>
// //     //     {
// //     //         if (ledMatrixPanel != null)
// //     //         {
// //     //             ledMirrorMode = true;
// //     //             ledMatrixPanel.SetLEDStatesFromROS(ledData);
// //     //         }
// //     //     });
// //     // }

// //     void OnLEDsReceived(RosMessageTypes.Std.UInt8MultiArrayMsg msg)
// //     {
// //         byte[] ledData = new byte[msg.data.Length];
// //         for (int i = 0; i < msg.data.Length; i++)
// //             ledData[i] = (byte)msg.data[i];
        
// //         UnityMainThreadDispatcher.Instance().Enqueue(() =>
// //         {
// //             if (ledMatrixPanel != null)
// //             {
// //                 // Flip X axis by reversing each row
// //                 byte[] flippedData = new byte[ledData.Length];
// //                 int rowSize = 8; // 8x8 LED matrix
                
// //                 for (int row = 0; row < 8; row++)
// //                 {
// //                     for (int col = 0; col < 8; col++)
// //                     {
// //                         // Map (row, col) to (row, 7-col) to flip X axis
// //                         // flippedData[row * rowSize + (7 - col)] = ledData[row * rowSize + col];
// //                         // Flip Y axis by reversing row order
// //                         flippedData[(7 - row) * rowSize + col] = ledData[row * rowSize + col];
// //                     }
// //                 }
                
// //                 ledMirrorMode = true;
// //                 ledMatrixPanel.SetLEDStatesFromROS(flippedData);
// //             }
// //         });
// //     }

// //     // ─────────────────────────────────────────
// //     // PUZZLE GENERATION
// //     // Syncs generated code with frog croaks
// //     // and maze with lily pad grid
// //     // ─────────────────────────────────────────

// //     void OnPuzzleGenerationReceived(RosMessageTypes.Std.StringMsg msg)
// //     {
// //         Debug.Log($"ROS: Puzzle generation: {msg.data}");

// //         try
// //         {
// //             PuzzleGenerationData data = JsonUtility.FromJson<PuzzleGenerationData>(msg.data);
// //             if (data == null) return;

// //             UnityMainThreadDispatcher.Instance().Enqueue(() =>
// //             {
// //                 // Sync frog croak counts with real world code
// //                 if (!string.IsNullOrEmpty(data.code))
// //                     SyncFrogCroaksWithCode(data.code);

// //                 // Sync lily pad maze end point
// //                 if (lilyPadGrid != null && data.maze_end_x >= 0)
// //                     SyncLilyPadEndPoint(data.maze_end_x, data.maze_end_y);
// //             });
// //         }
// //         catch (System.Exception e)
// //         {
// //             Debug.LogError($"Puzzle generation parse error: {e.Message}");
// //         }
// //     }

// //     // ─────────────────────────────────────────
// //     // PUZZLE STATE
// //     // Arduino validates → Unity shows result
// //     // ─────────────────────────────────────────

// //     void OnPuzzleStateReceived(RosMessageTypes.Std.StringMsg msg)
// //     {
// //         try
// //         {
// //             PuzzleStateData state = JsonUtility.FromJson<PuzzleStateData>(msg.data);
// //             if (state == null) return;

// //             UnityMainThreadDispatcher.Instance().Enqueue(() =>
// //             {
// //                 if (puzzleBoardManager == null) return;

// //                 if (state.puzzle_solved)
// //                 {
// //                     // Both puzzles solved on Arduino
// //                     puzzleBoardManager.OnLEDMatrixSolved();
// //                     puzzleBoardManager.OnButtonMatrixSolved();
// //                     Debug.Log("ROS: All puzzles solved!");
// //                 }
// //             });
// //         }
// //         catch (System.Exception e)
// //         {
// //             Debug.LogError($"Puzzle state parse error: {e.Message}");
// //         }
// //     }

// //     // ─────────────────────────────────────────
// //     // SYNC FROG CROAKS WITH REAL WORLD CODE
// //     // Arduino generatedCode → frog croak counts
// //     // ─────────────────────────────────────────

// //     void SyncFrogCroaksWithCode(string code)
// //     {
// //         if (frogSigns == null) return;

// //         for (int i = 0; i < code.Length && i < frogSigns.Length; i++)
// //         {
// //             if (frogSigns[i] == null) continue;

// //             if (int.TryParse(code[i].ToString(), out int croakCount))
// //             {
// //                 frogSigns[i].SetCroakCount(croakCount);
// //                 Debug.Log($"Sign {i+1} synced to {croakCount} croaks");
// //             }
// //         }

// //         // Refresh button matrix correct code
// //         if (buttonMatrixPanel != null)
// //             buttonMatrixPanel.RefreshCorrectCode();

// //         Debug.Log($"Frog croaks synced with code: {code}");
// //     }

// //     // ─────────────────────────────────────────
// //     // SYNC LILY PAD MAZE END POINT
// //     // Arduino maze end → lily pad possible ends
// //     // ─────────────────────────────────────────

// //     // void SyncLilyPadEndPoint(int endX, int endY)
// //     // {
// //     //     if (lilyPadGrid == null) return;

// //     //     // Convert Arduino 8x8 coords to lily pad 6x6 coords
// //     //     // Arduino inner grid 1-6 → lily pad 0-5
// //     //     int lilyEndX = Mathf.Clamp(endX - 1, 0, 5);
// //     //     int lilyEndY = Mathf.Clamp(endY - 1, 0, 5);

// //     //     Debug.Log($"Lily pad end synced to ({lilyEndX},{lilyEndY})");
// //     // }
// //     void SyncLilyPadEndPoint(int endX, int endY)
// //     {
// //         if (lilyPadGrid == null) return;

// //         // Convert Arduino 8x8 coords to lily pad 6x6 coords
// //         int lilyEndX = Mathf.Clamp(endX - 1, 0, 5);
// //         int lilyEndY = Mathf.Clamp(endY - 1, 0, 5);

// //         // ← ADD THIS - actually call the method!
// //         lilyPadGrid.SetEndPoint(lilyEndX, lilyEndY);

// //         Debug.Log($"Lily pad end synced to ({lilyEndX},{lilyEndY})");
// //     }
// // }

// // // ─────────────────────────────────────────
// // // DATA CLASSES
// // // ─────────────────────────────────────────

// // [System.Serializable]
// // public class PuzzleGenerationData
// // {
// //     public string code;
// //     //public int[][] maze;
// //     public string maze;
// //     public int maze_end_x;
// //     public int maze_end_y;
// //     public string timestamp;
// // }

// // [System.Serializable]
// // public class PuzzleStateData
// // {
// //     public bool puzzle_solved;
// //     public string display_text;
// //     public float joystick_x;
// //     public float joystick_y;
// // }

// using UnityEngine;
// using Unity.Robotics.ROSTCPConnector;
// using RosMessageTypes.Std;
// using RosMessageTypes.Geometry;
// using System.Collections.Generic;
// using System.Text;
// using PimDeWitte.UnityMainThreadDispatcher;

// public class ROSPuzzleBridge : MonoBehaviour
// {
//     [Header("Puzzle Board References")]
//     public ButtonMatrixPanel buttonMatrixPanel;
//     public LEDMatrixPanel ledMatrixPanel;           // ADDED: was missing, must assign in Inspector
//     public LilyPadGrid lilyPadGrid;
//     public FrogCroakScript[] frogSigns;
//     public PuzzleBoardManager puzzleBoardManager;

//     [Header("ROS Topics")]
//     public string buttonsTopic = "/puzzle_board/buttons";
//     public string displayTopic = "/puzzle_board/display";
//     public string joystickTopic = "/puzzle_board/joystick";
//     public string puzzleGenerationTopic = "/puzzle/generation";
//     public string ledsTopic = "/puzzle_board/leds";
//     public string puzzleStateTopic = "/puzzle_board/state";

//     private ROSConnection ros;
//     private bool ledMirrorMode = false;

//     void Start()
//     {
//         ros = ROSConnection.GetOrCreateInstance();
//         Debug.Log($"ROS connecting to: {ros.RosIPAddress}:{ros.RosPort}");

//         ros.Subscribe<RosMessageTypes.Std.Int32MultiArrayMsg>(buttonsTopic, OnButtonsReceived);
//         ros.Subscribe<RosMessageTypes.Std.StringMsg>(displayTopic, OnDisplayReceived);
//         ros.Subscribe<Vector3Msg>(joystickTopic, OnJoystickReceived);
//         ros.Subscribe<RosMessageTypes.Std.StringMsg>(puzzleGenerationTopic, OnPuzzleGenerationReceived);
//         ros.Subscribe<RosMessageTypes.Std.UInt8MultiArrayMsg>(ledsTopic, OnLEDsReceived);
//         ros.Subscribe<RosMessageTypes.Std.StringMsg>(puzzleStateTopic, OnPuzzleStateReceived);

//         Debug.Log("ROSPuzzleBridge: All topics subscribed");
//     }

//     // ─────────────────────────────────────────
//     // BUTTON MATRIX
//     // CHANGED: now calls RegisterButtonFromROS instead of PressButton
//     // This highlights the VR button blue and updates the display only.
//     // Arduino owns validation — result comes from /puzzle_board/state.
//     // ─────────────────────────────────────────

//     void OnButtonsReceived(RosMessageTypes.Std.Int32MultiArrayMsg msg)
//     {
//         Debug.Log("ROS: Button message received");
//         for (int i = 0; i < msg.data.Length && i < 9; i++)
//         {
//             if (msg.data[i] == 1)
//             {
//                 int buttonNumber = i + 1;
//                 Debug.Log($"ROS: Button {buttonNumber} pressed");

//                 UnityMainThreadDispatcher.Instance().Enqueue(() =>
//                 {
//                     if (buttonMatrixPanel != null)
//                         buttonMatrixPanel.RegisterButtonFromROS(buttonNumber); // CHANGED
//                 });
//             }
//         }
//     }

//     // ─────────────────────────────────────────
//     // DISPLAY TEXT
//     // Unchanged — mirrors Arduino SSD display to Unity SSD
//     // ─────────────────────────────────────────

//     void OnDisplayReceived(RosMessageTypes.Std.StringMsg msg)
//     {
//         string displayText = msg.data;
//         Debug.Log($"ROS: Display: {displayText}");

//         UnityMainThreadDispatcher.Instance().Enqueue(() =>
//         {
//             if (buttonMatrixPanel != null)
//                 buttonMatrixPanel.SetDisplayFromROS(displayText);
//         });
//     }

//     // ─────────────────────────────────────────
//     // JOYSTICK
//     // Unchanged — moves LED cursor visually only.
//     // Arduino validates the path, not Unity.
//     // ─────────────────────────────────────────

//     void OnJoystickReceived(Vector3Msg msg)
//     {
//         float x = (float)msg.x;
//         float y = (float)msg.y;
//         float deadZone = 0.3f;

//         UnityMainThreadDispatcher.Instance().Enqueue(() =>
//         {
//             if (ledMatrixPanel == null) return;
//             if (ledMirrorMode) return;

//             if (x < -deadZone && Mathf.Abs(x) > Mathf.Abs(y))
//                 ledMatrixPanel.MoveLeft();
//             else if (x > deadZone && Mathf.Abs(x) > Mathf.Abs(y))
//                 ledMatrixPanel.MoveRight();
//             else if (y > deadZone && Mathf.Abs(y) > Mathf.Abs(x))
//                 ledMatrixPanel.MoveUp();
//             else if (y < -deadZone && Mathf.Abs(y) > Mathf.Abs(x))
//                 ledMatrixPanel.MoveDown();
//         });
//     }

//     // ─────────────────────────────────────────
//     // LED MATRIX STATE
//     // Unchanged — mirrors Arduino LED states with Y-axis flip
//     // ─────────────────────────────────────────

//     void OnLEDsReceived(RosMessageTypes.Std.UInt8MultiArrayMsg msg)
//     {
//         byte[] ledData = new byte[msg.data.Length];
//         for (int i = 0; i < msg.data.Length; i++)
//             ledData[i] = (byte)msg.data[i];

//         UnityMainThreadDispatcher.Instance().Enqueue(() =>
//         {
//             if (ledMatrixPanel != null)
//             {
//                 byte[] flippedData = new byte[ledData.Length];
//                 int rowSize = 8;

//                 for (int row = 0; row < 8; row++)
//                     for (int col = 0; col < 8; col++)
//                         flippedData[(7 - row) * rowSize + col] = ledData[row * rowSize + col];

//                 ledMirrorMode = true;
//                 ledMatrixPanel.SetLEDStatesFromROS(flippedData);
//             }
//         });
//     }

//     // ─────────────────────────────────────────
//     // PUZZLE GENERATION
//     // CHANGED: now parses maze string into bool[8,8] and calls
//     // SetPathFromMaze on LilyPadGrid so the VR lily pad path
//     // exactly matches the physical Arduino maze.
//     // Then triggers LEDMatrixPanel to rebuild its mapped path.
//     // Frog croak sync is unchanged.
//     // ─────────────────────────────────────────

//     void OnPuzzleGenerationReceived(RosMessageTypes.Std.StringMsg msg)
//     {
//         Debug.Log($"ROS: Puzzle generation: {msg.data}");

//         try
//         {
//             PuzzleGenerationData data = JsonUtility.FromJson<PuzzleGenerationData>(msg.data);
//             if (data == null) return;

//             UnityMainThreadDispatcher.Instance().Enqueue(() =>
//             {
//                 // 1. Sync frog croaks with Arduino-generated code → drives button matrix
//                 if (!string.IsNullOrEmpty(data.code))
//                     SyncFrogCroaksWithCode(data.code);

//                 // 2. CHANGED: parse maze string and drive lily pad + LED matrix exactly
//                 if (!string.IsNullOrEmpty(data.maze))
//                 {
//                     bool[,] mazeGrid = ParseMazeString(data.maze);
//                     if (mazeGrid != null)
//                     {
//                         LogMazeGrid(mazeGrid); // debug — remove once confirmed working

//                         if (lilyPadGrid != null)
//                             lilyPadGrid.SetPathFromMaze(mazeGrid);

//                         if (ledMatrixPanel != null)
//                             ledMatrixPanel.RebuildAfterPathUpdate();
//                     }
//                 }
//                 // Fallback: no maze string, use end coords only
//                 else if (data.maze_end_x >= 0 && lilyPadGrid != null)
//                 {
//                     int lilyEndX = Mathf.Clamp(data.maze_end_x - 1, 0, 5);
//                     int lilyEndY = Mathf.Clamp(data.maze_end_y - 1, 0, 5);
//                     lilyPadGrid.SetEndPoint(lilyEndX, lilyEndY);

//                     if (ledMatrixPanel != null)
//                         ledMatrixPanel.RebuildAfterPathUpdate();
//                 }
//             });
//         }
//         catch (System.Exception e)
//         {
//             Debug.LogError($"Puzzle generation parse error: {e.Message}");
//         }
//     }

//     // ─────────────────────────────────────────
//     // PUZZLE STATE
//     // CHANGED: now drives ShowSolvedFromROS / ShowFailedFromROS
//     // on LEDMatrixPanel, and HandleROSResult on ButtonMatrixPanel.
//     // Arduino is source of truth for both puzzle results.
//     // ─────────────────────────────────────────

//     void OnPuzzleStateReceived(RosMessageTypes.Std.StringMsg msg)
//     {
//         try
//         {
//             PuzzleStateData state = JsonUtility.FromJson<PuzzleStateData>(msg.data);
//             if (state == null) return;

//             UnityMainThreadDispatcher.Instance().Enqueue(() =>
//             {
//                 // Mirror display text if present
//                 if (!string.IsNullOrEmpty(state.display_text) && buttonMatrixPanel != null)
//                     buttonMatrixPanel.SetDisplayFromROS(state.display_text);

//                 // LED matrix result
//                 if (state.led_solved)
//                     ledMatrixPanel?.ShowSolvedFromROS();
//                 else if (state.led_failed)
//                     ledMatrixPanel?.ShowFailedFromROS();

//                 // Button matrix result
//                 if (state.button_solved)
//                     buttonMatrixPanel?.HandleROSResult(true);
//                 else if (state.button_failed)
//                     buttonMatrixPanel?.HandleROSResult(false);

//                 // Both puzzles solved
//                 if (state.puzzle_solved && puzzleBoardManager != null)
//                 {
//                     puzzleBoardManager.OnLEDMatrixSolved();
//                     puzzleBoardManager.OnButtonMatrixSolved();
//                     Debug.Log("ROS: All puzzles solved!");
//                 }
//             });
//         }
//         catch (System.Exception e)
//         {
//             Debug.LogError($"Puzzle state parse error: {e.Message}");
//         }
//     }

//     // ─────────────────────────────────────────
//     // SYNC FROG CROAKS
//     // Unchanged — Arduino code string → frog croak counts
//     // ─────────────────────────────────────────

//     void SyncFrogCroaksWithCode(string code)
//     {
//         if (frogSigns == null) return;

//         for (int i = 0; i < code.Length && i < frogSigns.Length; i++)
//         {
//             if (frogSigns[i] == null) continue;
//             if (int.TryParse(code[i].ToString(), out int croakCount))
//             {
//                 frogSigns[i].SetCroakCount(croakCount);
//                 Debug.Log($"Sign {i + 1} synced to {croakCount} croaks");
//             }
//         }

//         if (buttonMatrixPanel != null)
//             buttonMatrixPanel.RefreshCorrectCode();

//         Debug.Log($"Frog croaks synced with code: {code}");
//     }

//     // ─────────────────────────────────────────
//     // PARSE MAZE STRING
//     // ADDED: converts Arduino maze string to bool[8,8]
//     // Supports comma-separated rows ("01010101,00110011,...")
//     // and flat 64-char strings ("0101010100110011...")
//     // Matches getLedMatrixState() format from Arduino
//     // ─────────────────────────────────────────

//     bool[,] ParseMazeString(string maze)
//     {
//         bool[,] grid = new bool[8, 8];
//         maze = maze.Trim();

//         // Try comma-separated rows first (getLedMatrixState format)
//         string[] rows = maze.Split(',');
//         if (rows.Length == 8)
//         {
//             for (int row = 0; row < 8; row++)
//             {
//                 string rowStr = rows[row].Trim();
//                 for (int col = 0; col < Mathf.Min(rowStr.Length, 8); col++)
//                     grid[row, col] = rowStr[col] == '1';
//             }
//             Debug.Log("Maze parsed: comma-separated rows");
//             return grid;
//         }

//         // Fallback: flat 64-char string
//         string flat = maze.Replace(",", "").Replace("\n", "").Replace(" ", "");
//         if (flat.Length >= 64)
//         {
//             for (int i = 0; i < 64; i++)
//                 grid[i / 8, i % 8] = flat[i] == '1';
//             Debug.Log("Maze parsed: flat 64-char string");
//             return grid;
//         }

//         Debug.LogError($"Could not parse maze string (length {maze.Length}): {maze}");
//         return null;
//     }

//     // ─────────────────────────────────────────
//     // DEBUG MAZE LOGGER
//     // ADDED: prints the parsed maze as a visual grid in console
//     // Remove the LogMazeGrid() call in OnPuzzleGenerationReceived
//     // once you've confirmed the maze is parsing correctly
//     // ─────────────────────────────────────────

//     void LogMazeGrid(bool[,] grid)
//     {
//         string log = "Parsed maze grid (· = off, █ = path):\n";
//         for (int row = 0; row < 8; row++)
//         {
//             for (int col = 0; col < 8; col++)
//                 log += grid[row, col] ? "█" : "·";
//             log += "\n";
//         }
//         Debug.Log(log);
//     }

//     // ─────────────────────────────────────────
//     // SYNC LILY PAD END POINT (kept as fallback)
//     // Only used when no maze string is present
//     // ─────────────────────────────────────────

//     void SyncLilyPadEndPoint(int endX, int endY)
//     {
//         if (lilyPadGrid == null) return;
//         int lilyEndX = Mathf.Clamp(endX - 1, 0, 5);
//         int lilyEndY = Mathf.Clamp(endY - 1, 0, 5);
//         lilyPadGrid.SetEndPoint(lilyEndX, lilyEndY);
//         Debug.Log($"Lily pad end synced to ({lilyEndX},{lilyEndY})");
//     }
// }

// // ─────────────────────────────────────────
// // DATA CLASSES
// // CHANGED: PuzzleStateData now has per-puzzle solved/failed flags
// // Update your Arduino ROS publisher to send these fields,
// // or keep puzzle_solved for the combined "both done" signal
// // ─────────────────────────────────────────

// [System.Serializable]
// public class PuzzleGenerationData
// {
//     public string code;
//     public string maze;
//     public int maze_end_x;
//     public int maze_end_y;
//     public string timestamp;
// }

// [System.Serializable]
// public class PuzzleStateData
// {
//     public bool puzzle_solved;    // true when BOTH puzzles complete
//     public bool led_solved;       // ADDED: LED matrix path correct
//     public bool led_failed;       // ADDED: LED matrix path wrong
//     public bool button_solved;    // ADDED: button code correct
//     public bool button_failed;    // ADDED: button code wrong
//     public string display_text;
//     public float joystick_x;
//     public float joystick_y;
// }

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using System.Collections.Generic;
using System.Text;
using PimDeWitte.UnityMainThreadDispatcher;

public class ROSPuzzleBridge : MonoBehaviour
{
    [Header("Puzzle Board References")]
    public ButtonMatrixPanel buttonMatrixPanel;
    public LEDMatrixPanel ledMatrixPanel;           // ADDED: was missing, must assign in Inspector
    public LilyPadGrid lilyPadGrid;
    public FrogCroakScript[] frogSigns;
    public PuzzleBoardManager puzzleBoardManager;

    [Header("ROS Topics")]
    public string buttonsTopic = "/puzzle_board/buttons";
    public string displayTopic = "/puzzle_board/display";
    public string joystickTopic = "/puzzle_board/joystick";
    public string puzzleGenerationTopic = "/puzzle/generation";
    public string ledsTopic = "/puzzle_board/leds";
    public string puzzleStateTopic = "/puzzle_board/state";

    private ROSConnection ros;
    private bool ledMirrorMode = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        Debug.Log($"ROS connecting to: {ros.RosIPAddress}:{ros.RosPort}");

        ros.Subscribe<RosMessageTypes.Std.Int32MultiArrayMsg>(buttonsTopic, OnButtonsReceived);
        ros.Subscribe<RosMessageTypes.Std.StringMsg>(displayTopic, OnDisplayReceived);
        ros.Subscribe<Vector3Msg>(joystickTopic, OnJoystickReceived);
        ros.Subscribe<RosMessageTypes.Std.StringMsg>(puzzleGenerationTopic, OnPuzzleGenerationReceived);
        ros.Subscribe<RosMessageTypes.Std.UInt8MultiArrayMsg>(ledsTopic, OnLEDsReceived);
        ros.Subscribe<RosMessageTypes.Std.StringMsg>(puzzleStateTopic, OnPuzzleStateReceived);

        Debug.Log("ROSPuzzleBridge: All topics subscribed");
    }

    // ─────────────────────────────────────────
    // BUTTON MATRIX
    // Arduino keypad layout (getButtonStates order):
    //   index 0-8  → digits 1-9
    //   index 9    → * (Reset)
    //   index 10   → 0
    //   index 11   → # (Enter/Submit)
    // Passes all 12 through to ButtonMatrixPanel which handles
    // each type (digit, reset, enter) separately.
    // ─────────────────────────────────────────

    void OnButtonsReceived(RosMessageTypes.Std.Int32MultiArrayMsg msg)
    {
        Debug.Log("ROS: Button message received");

        // Arduino sends 12 buttons: 1-9, *, 0, #
        for (int i = 0; i < msg.data.Length && i < 12; i++)
        {
            if (msg.data[i] == 1)
            {
                int capturedIndex = i;
                Debug.Log($"ROS: Keypad index {capturedIndex} pressed");

                UnityMainThreadDispatcher.Instance().Enqueue(() =>
                {
                    if (buttonMatrixPanel == null) return;

                    if (capturedIndex <= 8)
                    {
                        // Digits 1-9 (index 0-8)
                        buttonMatrixPanel.RegisterButtonFromROS(capturedIndex + 1);
                    }
                    else if (capturedIndex == 9)
                    {
                        // * = Reset
                        buttonMatrixPanel.RegisterResetFromROS();
                    }
                    else if (capturedIndex == 10)
                    {
                        // 0
                        buttonMatrixPanel.RegisterButtonFromROS(0);
                    }
                    else if (capturedIndex == 11)
                    {
                        // # = Enter/Submit
                        buttonMatrixPanel.RegisterEnterFromROS();
                    }
                });
            }
        }
    }

    // ─────────────────────────────────────────
    // DISPLAY TEXT
    // Unchanged — mirrors Arduino SSD display to Unity SSD
    // ─────────────────────────────────────────

    void OnDisplayReceived(RosMessageTypes.Std.StringMsg msg)
    {
        string displayText = msg.data;
        Debug.Log($"ROS: Display: {displayText}");

        UnityMainThreadDispatcher.Instance().Enqueue(() =>
        {
            if (buttonMatrixPanel != null)
                buttonMatrixPanel.SetDisplayFromROS(displayText);
        });
    }

    // ─────────────────────────────────────────
    // JOYSTICK
    // Unchanged — moves LED cursor visually only.
    // Arduino validates the path, not Unity.
    // ─────────────────────────────────────────

    void OnJoystickReceived(Vector3Msg msg)
    {
        float x = (float)msg.x;
        float y = (float)msg.y;
        float deadZone = 0.3f;

        UnityMainThreadDispatcher.Instance().Enqueue(() =>
        {
            if (ledMatrixPanel == null) return;
            if (ledMirrorMode) return;

            if (x < -deadZone && Mathf.Abs(x) > Mathf.Abs(y))
                ledMatrixPanel.MoveLeft();
            else if (x > deadZone && Mathf.Abs(x) > Mathf.Abs(y))
                ledMatrixPanel.MoveRight();
            else if (y > deadZone && Mathf.Abs(y) > Mathf.Abs(x))
                ledMatrixPanel.MoveUp();
            else if (y < -deadZone && Mathf.Abs(y) > Mathf.Abs(x))
                ledMatrixPanel.MoveDown();
        });
    }

    // ─────────────────────────────────────────
    // LED MATRIX STATE
    // Unchanged — mirrors Arduino LED states with Y-axis flip
    // ─────────────────────────────────────────

    void OnLEDsReceived(RosMessageTypes.Std.UInt8MultiArrayMsg msg)
    {
        byte[] ledData = new byte[msg.data.Length];
        for (int i = 0; i < msg.data.Length; i++)
            ledData[i] = (byte)msg.data[i];

        UnityMainThreadDispatcher.Instance().Enqueue(() =>
        {
            if (ledMatrixPanel != null)
            {
                byte[] flippedData = new byte[ledData.Length];
                int rowSize = 8;

                for (int row = 0; row < 8; row++)
                    for (int col = 0; col < 8; col++)
                        flippedData[(7 - row) * rowSize + col] = ledData[row * rowSize + col];

                ledMirrorMode = true;
                ledMatrixPanel.SetLEDStatesFromROS(flippedData);
            }
        });
    }

    // ─────────────────────────────────────────
    // PUZZLE GENERATION
    // CHANGED: now parses maze string into bool[8,8] and calls
    // SetPathFromMaze on LilyPadGrid so the VR lily pad path
    // exactly matches the physical Arduino maze.
    // Then triggers LEDMatrixPanel to rebuild its mapped path.
    // Frog croak sync is unchanged.
    // ─────────────────────────────────────────

    void OnPuzzleGenerationReceived(RosMessageTypes.Std.StringMsg msg)
    {
        Debug.Log($"ROS: Puzzle generation: {msg.data}");

        try
        {
            PuzzleGenerationData data = JsonUtility.FromJson<PuzzleGenerationData>(msg.data);
            if (data == null) return;

            UnityMainThreadDispatcher.Instance().Enqueue(() =>
            {
                // 1. Sync frog croaks with Arduino-generated code → drives button matrix
                if (!string.IsNullOrEmpty(data.code))
                    SyncFrogCroaksWithCode(data.code);

                // 2. CHANGED: parse maze string and drive lily pad + LED matrix exactly
                if (!string.IsNullOrEmpty(data.maze))
                {
                    bool[,] mazeGrid = ParseMazeString(data.maze);
                    if (mazeGrid != null)
                    {
                        LogMazeGrid(mazeGrid); // debug — remove once confirmed working

                        if (lilyPadGrid != null)
                            lilyPadGrid.SetPathFromMaze(mazeGrid);

                        if (ledMatrixPanel != null)
                            ledMatrixPanel.RebuildAfterPathUpdate();
                    }
                }
                // Fallback: no maze string, use end coords only
                else if (data.maze_end_x >= 0 && lilyPadGrid != null)
                {
                    int lilyEndX = Mathf.Clamp(data.maze_end_x - 1, 0, 5);
                    int lilyEndY = Mathf.Clamp(data.maze_end_y - 1, 0, 5);
                    lilyPadGrid.SetEndPoint(lilyEndX, lilyEndY);

                    if (ledMatrixPanel != null)
                        ledMatrixPanel.RebuildAfterPathUpdate();
                }
            });
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Puzzle generation parse error: {e.Message}");
        }
    }

    // ─────────────────────────────────────────
    // PUZZLE STATE
    // CHANGED: now drives ShowSolvedFromROS / ShowFailedFromROS
    // on LEDMatrixPanel, and HandleROSResult on ButtonMatrixPanel.
    // Arduino is source of truth for both puzzle results.
    // ─────────────────────────────────────────

    void OnPuzzleStateReceived(RosMessageTypes.Std.StringMsg msg)
    {
        try
        {
            PuzzleStateData state = JsonUtility.FromJson<PuzzleStateData>(msg.data);
            if (state == null) return;

            UnityMainThreadDispatcher.Instance().Enqueue(() =>
            {
                // Mirror display text if present
                if (!string.IsNullOrEmpty(state.display_text) && buttonMatrixPanel != null)
                    buttonMatrixPanel.SetDisplayFromROS(state.display_text);

                // LED matrix result
                if (state.led_solved)
                    ledMatrixPanel?.ShowSolvedFromROS();
                else if (state.led_failed)
                    ledMatrixPanel?.ShowFailedFromROS();

                // Button matrix result
                if (state.button_solved)
                    buttonMatrixPanel?.HandleROSResult(true);
                else if (state.button_failed)
                    buttonMatrixPanel?.HandleROSResult(false);

                // Both puzzles solved
                if (state.puzzle_solved && puzzleBoardManager != null)
                {
                    puzzleBoardManager.OnLEDMatrixSolved();
                    puzzleBoardManager.OnButtonMatrixSolved();
                    Debug.Log("ROS: All puzzles solved!");
                }
            });
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Puzzle state parse error: {e.Message}");
        }
    }

    // ─────────────────────────────────────────
    // SYNC FROG CROAKS
    // Unchanged — Arduino code string → frog croak counts
    // ─────────────────────────────────────────

    void SyncFrogCroaksWithCode(string code)
    {
        if (frogSigns == null) return;

        for (int i = 0; i < code.Length && i < frogSigns.Length; i++)
        {
            if (frogSigns[i] == null) continue;
            if (int.TryParse(code[i].ToString(), out int croakCount))
            {
                frogSigns[i].SetCroakCount(croakCount);
                Debug.Log($"Sign {i + 1} synced to {croakCount} croaks");
            }
        }

        if (buttonMatrixPanel != null)
            buttonMatrixPanel.RefreshCorrectCode();

        Debug.Log($"Frog croaks synced with code: {code}");
    }

    // ─────────────────────────────────────────
    // PARSE MAZE STRING
    // ADDED: converts Arduino maze string to bool[8,8]
    // Supports comma-separated rows ("01010101,00110011,...")
    // and flat 64-char strings ("0101010100110011...")
    // Matches getLedMatrixState() format from Arduino
    // ─────────────────────────────────────────

    bool[,] ParseMazeString(string maze)
    {
        bool[,] grid = new bool[8, 8];
        maze = maze.Trim();

        // Try comma-separated rows first (getLedMatrixState format)
        string[] rows = maze.Split(',');
        if (rows.Length == 8)
        {
            for (int row = 0; row < 8; row++)
            {
                string rowStr = rows[row].Trim();
                for (int col = 0; col < Mathf.Min(rowStr.Length, 8); col++)
                    grid[row, col] = rowStr[col] == '1';
            }
            Debug.Log("Maze parsed: comma-separated rows");
            return grid;
        }

        // Fallback: flat 64-char string
        string flat = maze.Replace(",", "").Replace("\n", "").Replace(" ", "");
        if (flat.Length >= 64)
        {
            for (int i = 0; i < 64; i++)
                grid[i / 8, i % 8] = flat[i] == '1';
            Debug.Log("Maze parsed: flat 64-char string");
            return grid;
        }

        Debug.LogError($"Could not parse maze string (length {maze.Length}): {maze}");
        return null;
    }

    // ─────────────────────────────────────────
    // DEBUG MAZE LOGGER
    // ADDED: prints the parsed maze as a visual grid in console
    // Remove the LogMazeGrid() call in OnPuzzleGenerationReceived
    // once you've confirmed the maze is parsing correctly
    // ─────────────────────────────────────────

    void LogMazeGrid(bool[,] grid)
    {
        string log = "Parsed maze grid (· = off, █ = path):\n";
        for (int row = 0; row < 8; row++)
        {
            for (int col = 0; col < 8; col++)
                log += grid[row, col] ? "█" : "·";
            log += "\n";
        }
        Debug.Log(log);
    }

    // ─────────────────────────────────────────
    // SYNC LILY PAD END POINT (kept as fallback)
    // Only used when no maze string is present
    // ─────────────────────────────────────────

    void SyncLilyPadEndPoint(int endX, int endY)
    {
        if (lilyPadGrid == null) return;
        int lilyEndX = Mathf.Clamp(endX - 1, 0, 5);
        int lilyEndY = Mathf.Clamp(endY - 1, 0, 5);
        lilyPadGrid.SetEndPoint(lilyEndX, lilyEndY);
        Debug.Log($"Lily pad end synced to ({lilyEndX},{lilyEndY})");
    }
}

// ─────────────────────────────────────────
// DATA CLASSES
// CHANGED: PuzzleStateData now has per-puzzle solved/failed flags
// Update your Arduino ROS publisher to send these fields,
// or keep puzzle_solved for the combined "both done" signal
// ─────────────────────────────────────────

[System.Serializable]
public class PuzzleGenerationData
{
    public string code;
    public string maze;
    public int maze_end_x;
    public int maze_end_y;
    public string timestamp;
}

[System.Serializable]
public class PuzzleStateData
{
    public bool puzzle_solved;    // true when BOTH puzzles complete
    public bool led_solved;       // ADDED: LED matrix path correct
    public bool led_failed;       // ADDED: LED matrix path wrong
    public bool button_solved;    // ADDED: button code correct
    public bool button_failed;    // ADDED: button code wrong
    public string display_text;
    public float joystick_x;
    public float joystick_y;
}