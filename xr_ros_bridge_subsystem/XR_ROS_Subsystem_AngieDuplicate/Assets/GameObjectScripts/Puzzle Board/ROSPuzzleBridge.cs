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
    public LEDMatrixPanel ledMatrixPanel;
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
    private bool ledMirrorMode = false; // true = mirror Arduino LEDs directly

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to all topics
        ros.Subscribe<RosMessageTypes.Std.Int32MultiArrayMsg>(
            buttonsTopic, OnButtonsReceived);
        ros.Subscribe<StringMsg>(
            displayTopic, OnDisplayReceived);
        ros.Subscribe<Vector3Msg>(
            joystickTopic, OnJoystickReceived);
        ros.Subscribe<StringMsg>(
            puzzleGenerationTopic, OnPuzzleGenerationReceived);
        ros.Subscribe<RosMessageTypes.Std.UInt8MultiArrayMsg>(
            ledsTopic, OnLEDsReceived);
        ros.Subscribe<StringMsg>(
            puzzleStateTopic, OnPuzzleStateReceived);

        Debug.Log("ROSPuzzleBridge: All topics subscribed");
    }

    // ─────────────────────────────────────────
    // BUTTON MATRIX
    // Arduino sends button states → Unity registers press
    // ─────────────────────────────────────────

    void OnButtonsReceived(RosMessageTypes.Std.Int32MultiArrayMsg msg)
    {
        for (int i = 0; i < msg.data.Length && i < 9; i++)
        {
            if (msg.data[i] == 1)
            {
                int buttonNumber = i + 1;
                Debug.Log($"ROS: Button {buttonNumber} pressed");

                UnityMainThreadDispatcher.Instance().Enqueue(() =>
                {
                    if (buttonMatrixPanel != null)
                        buttonMatrixPanel.PressButton(buttonNumber);
                });
            }
        }
    }

    // ─────────────────────────────────────────
    // DISPLAY TEXT
    // Sync Arduino SSD display with Unity SSD
    // ─────────────────────────────────────────

    void OnDisplayReceived(StringMsg msg)
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
    // Arduino joystick → Unity LED cursor movement
    // Matches Arduino dead zone and direction logic
    // ─────────────────────────────────────────

    void OnJoystickReceived(Vector3Msg msg)
    {
        float x = (float)msg.x;
        float y = (float)msg.y;

        // Match Arduino DEADZONE (normalised)
        float deadZone = 0.3f;

        UnityMainThreadDispatcher.Instance().Enqueue(() =>
        {
            if (ledMatrixPanel == null) return;
            if (ledMirrorMode) return; // skip if mirroring LED states directly

            // Match Arduino movement logic:
            // rawX < 512-DEADZONE → dx = -1 (left)
            // rawX > 512+DEADZONE → dx = 1 (right)
            // rawY < 512-DEADZONE → dy = 1 (down in Arduino = up in Unity)
            // rawY > 512+DEADZONE → dy = -1

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
    // Mirror Arduino LED states directly to Unity
    // This overrides coordinate mapping entirely
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
                ledMirrorMode = true;
                ledMatrixPanel.SetLEDStatesFromROS(ledData);
            }
        });
    }

    // ─────────────────────────────────────────
    // PUZZLE GENERATION
    // Syncs generated code with frog croaks
    // and maze with lily pad grid
    // ─────────────────────────────────────────

    void OnPuzzleGenerationReceived(StringMsg msg)
    {
        Debug.Log($"ROS: Puzzle generation: {msg.data}");

        try
        {
            PuzzleGenerationData data = JsonUtility.FromJson<PuzzleGenerationData>(msg.data);
            if (data == null) return;

            UnityMainThreadDispatcher.Instance().Enqueue(() =>
            {
                // Sync frog croak counts with real world code
                if (!string.IsNullOrEmpty(data.code))
                    SyncFrogCroaksWithCode(data.code);

                // Sync lily pad maze end point
                if (lilyPadGrid != null && data.maze_end_x >= 0)
                    SyncLilyPadEndPoint(data.maze_end_x, data.maze_end_y);
            });
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Puzzle generation parse error: {e.Message}");
        }
    }

    // ─────────────────────────────────────────
    // PUZZLE STATE
    // Arduino validates → Unity shows result
    // ─────────────────────────────────────────

    void OnPuzzleStateReceived(StringMsg msg)
    {
        try
        {
            PuzzleStateData state = JsonUtility.FromJson<PuzzleStateData>(msg.data);
            if (state == null) return;

            UnityMainThreadDispatcher.Instance().Enqueue(() =>
            {
                if (puzzleBoardManager == null) return;

                if (state.puzzle_solved)
                {
                    // Both puzzles solved on Arduino
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
    // SYNC FROG CROAKS WITH REAL WORLD CODE
    // Arduino generatedCode → frog croak counts
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
                Debug.Log($"Sign {i+1} synced to {croakCount} croaks");
            }
        }

        // Refresh button matrix correct code
        if (buttonMatrixPanel != null)
            buttonMatrixPanel.RefreshCorrectCode();

        Debug.Log($"Frog croaks synced with code: {code}");
    }

    // ─────────────────────────────────────────
    // SYNC LILY PAD MAZE END POINT
    // Arduino maze end → lily pad possible ends
    // ─────────────────────────────────────────

    // void SyncLilyPadEndPoint(int endX, int endY)
    // {
    //     if (lilyPadGrid == null) return;

    //     // Convert Arduino 8x8 coords to lily pad 6x6 coords
    //     // Arduino inner grid 1-6 → lily pad 0-5
    //     int lilyEndX = Mathf.Clamp(endX - 1, 0, 5);
    //     int lilyEndY = Mathf.Clamp(endY - 1, 0, 5);

    //     Debug.Log($"Lily pad end synced to ({lilyEndX},{lilyEndY})");
    // }
    void SyncLilyPadEndPoint(int endX, int endY)
    {
        if (lilyPadGrid == null) return;

        // Convert Arduino 8x8 coords to lily pad 6x6 coords
        int lilyEndX = Mathf.Clamp(endX - 1, 0, 5);
        int lilyEndY = Mathf.Clamp(endY - 1, 0, 5);

        // ← ADD THIS - actually call the method!
        lilyPadGrid.SetEndPoint(lilyEndX, lilyEndY);

        Debug.Log($"Lily pad end synced to ({lilyEndX},{lilyEndY})");
    }
}

// ─────────────────────────────────────────
// DATA CLASSES
// ─────────────────────────────────────────

[System.Serializable]
public class PuzzleGenerationData
{
    public string code;
    //public int[][] maze;
    public string maze;
    public int maze_end_x;
    public int maze_end_y;
    public string timestamp;
}

[System.Serializable]
public class PuzzleStateData
{
    public bool puzzle_solved;
    public string display_text;
    public float joystick_x;
    public float joystick_y;
}