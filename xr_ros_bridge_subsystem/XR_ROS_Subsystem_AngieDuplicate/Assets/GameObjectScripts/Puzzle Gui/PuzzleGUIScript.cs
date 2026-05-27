using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
<<<<<<< HEAD

=======
<<<<<<< Updated upstream
 
=======

>>>>>>> Stashed changes
>>>>>>> 60a1c52683894d3f760f90a40aea1f9935ab1347
public class PuzzleGUIScript : MonoBehaviour
{
    [Header("Input")]
    public InputActionReference openGUIAction;

    [Header("GUI")]
    public GameObject puzzleGUI;
    public Camera robotArmCamera;

    [Header("Buttons - drag from Canvas")]
    public Button closeButton;
    public Button resetRobotButton;
    public Button puzzle1Button;      // Button Matrix (LH)
    public Button puzzle2Button;      // Slider (RH)
    public Button puzzle3Button;      // Egg Sorting
    public Button resetPuzzleButton;

    [Header("Singularity Warning")]
    public GameObject singularityWarningPanel;
    public TextMeshProUGUI singularityWarningText;
    public Button dismissWarningButton;

    [Header("Status Text")]
    public TextMeshProUGUI statusText;

    [Header("Teammate Robot Scripts (leave empty for now)")]
    public MonoBehaviour robotMovementScript;
    public MonoBehaviour sliderPuzzleScript;
    public MonoBehaviour eggSortingScript;

    private bool isGUIOpen = false;
    private int currentPuzzle = 0;

    // ROS
    private ROSConnection ros;
<<<<<<< HEAD
    private const string commandTopic = "/robot_gui/command";

=======
    private string robotCommandTopic = "/robot_gui/command";
 
    private bool isGUIOpen = false;
<<<<<<< Updated upstream
 
=======
    private int currentPuzzle = 0;

    // ROS
    private ROSConnection ros;
    private const string commandTopic = "/robot_gui/command";

>>>>>>> Stashed changes
>>>>>>> 60a1c52683894d3f760f90a40aea1f9935ab1347
    void Start()
    {
        puzzleGUI.SetActive(false);
        robotArmCamera.gameObject.SetActive(false);

        if (singularityWarningPanel != null)
            singularityWarningPanel.SetActive(false);
<<<<<<< HEAD
=======
<<<<<<< Updated upstream
 
=======
>>>>>>> 60a1c52683894d3f760f90a40aea1f9935ab1347

        // Set up ROS publisher
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(commandTopic);

        // Hook up all buttons
<<<<<<< HEAD
=======
>>>>>>> Stashed changes
>>>>>>> 60a1c52683894d3f760f90a40aea1f9935ab1347
        if (closeButton != null)
            closeButton.onClick.AddListener(CloseGUI);
        if (resetRobotButton != null)
            resetRobotButton.onClick.AddListener(OnResetRobotPressed);
        if (puzzle1Button != null)
            puzzle1Button.onClick.AddListener(OnPuzzle1Pressed);
        if (puzzle2Button != null)
            puzzle2Button.onClick.AddListener(OnPuzzle2Pressed);
        if (puzzle3Button != null)
            puzzle3Button.onClick.AddListener(OnPuzzle3Pressed);
        if (resetPuzzleButton != null)
            resetPuzzleButton.onClick.AddListener(OnResetPuzzlePressed);
        if (dismissWarningButton != null)
            dismissWarningButton.onClick.AddListener(OnDismissWarning);

        UpdateStatus("Robot ready.");
    }

    void OnEnable()
    {
        if (openGUIAction != null)
            openGUIAction.action.performed += OnGUIButtonPressed;
    }

    void OnDisable()
    {
        if (openGUIAction != null)
            openGUIAction.action.performed -= OnGUIButtonPressed;
    }

    private void OnGUIButtonPressed(InputAction.CallbackContext context)
    {
        ToggleGUI();
    }

    public void ToggleGUI()
    {
        isGUIOpen = !isGUIOpen;
        puzzleGUI.SetActive(isGUIOpen);
        robotArmCamera.gameObject.SetActive(isGUIOpen);
    }

    public void CloseGUI()
    {
        isGUIOpen = false;
        puzzleGUI.SetActive(false);
        robotArmCamera.gameObject.SetActive(false);
        UpdateStatus("Robot ready.");
    }

    // ─────────────────────────────────────────
<<<<<<< HEAD
    // ROS HELPER
=======
<<<<<<< Updated upstream
    // ROS COMMAND
=======
    // ROS HELPER
    // ─────────────────────────────────────────

    private void SendCommand(string command)
    {
        var msg = new StringMsg(command);
        ros.Publish(commandTopic, msg);
        Debug.Log($"GUI: Sent ROS command → {command}");
    }

    // ─────────────────────────────────────────
    // BUTTON FUNCTIONS
>>>>>>> Stashed changes
>>>>>>> 60a1c52683894d3f760f90a40aea1f9935ab1347
    // ─────────────────────────────────────────

    private void SendCommand(string command)
    {
<<<<<<< HEAD
        var msg = new StringMsg(command);
        ros.Publish(commandTopic, msg);
        Debug.Log($"GUI: Sent ROS command → {command}");
=======
<<<<<<< Updated upstream
        Debug.Log($"Robot command: {command}");
        StringMsg msg = new StringMsg();
        msg.data = command;
        ros.Publish(robotCommandTopic, msg);
=======
        UpdateStatus("Returning robot to hover position...");
        SendCommand("hover");
>>>>>>> Stashed changes
>>>>>>> 60a1c52683894d3f760f90a40aea1f9935ab1347
    }

    // ─────────────────────────────────────────
    // BUTTON FUNCTIONS
    // ─────────────────────────────────────────

    public void OnResetRobotPressed()
    {
<<<<<<< HEAD
        UpdateStatus("Returning robot to hover position...");
        SendCommand("hover");
=======
<<<<<<< Updated upstream
        UpdateStatus("Returning to hover position...");
        Debug.Log("GUI: Reset → hover");
        SendRobotCommand("hover");
=======
        currentPuzzle = 1;
        UpdateStatus("Puzzle 1: Moving to board...");
        SendCommand("face_board");
>>>>>>> Stashed changes
>>>>>>> 60a1c52683894d3f760f90a40aea1f9935ab1347
    }

    public void OnPuzzle1Pressed()
    {
<<<<<<< HEAD
        currentPuzzle = 1;
        UpdateStatus("Puzzle 1: Moving to board...");
        SendCommand("face_board");
=======
<<<<<<< Updated upstream
        UpdateStatus("Moving to face Puzzle Board...");
        Debug.Log("GUI: Face Puzzle Board");
        SendRobotCommand("face_board");
=======
        currentPuzzle = 2;
        UpdateStatus("Puzzle 2: Lily Pad Maze");
        SendCommand("face_board");
>>>>>>> Stashed changes
>>>>>>> 60a1c52683894d3f760f90a40aea1f9935ab1347
    }

    public void OnPuzzle2Pressed()
    {
<<<<<<< HEAD
        currentPuzzle = 2;
        UpdateStatus("Puzzle 2: Lily Pad Maze");
        SendCommand("face_board");
=======
<<<<<<< Updated upstream
        UpdateStatus("Moving to face Egg Puzzle...");
        Debug.Log("GUI: Face Eggs");
        SendRobotCommand("face_eggs");
=======
        currentPuzzle = 3;
        UpdateStatus("Puzzle 3: Moving to eggs...");
        SendCommand("face_eggs");
>>>>>>> Stashed changes
>>>>>>> 60a1c52683894d3f760f90a40aea1f9935ab1347
    }

    public void OnPuzzle3Pressed()
    {
        currentPuzzle = 3;
        UpdateStatus("Puzzle 3: Moving to eggs...");
        SendCommand("face_eggs");
    }

    public void OnResetPuzzlePressed()
    {
<<<<<<< HEAD
=======
<<<<<<< Updated upstream
        UpdateStatus("Resetting - returning to hover...");
        Debug.Log("GUI: Reset Puzzle → hover");
        SendRobotCommand("hover");
=======
>>>>>>> 60a1c52683894d3f760f90a40aea1f9935ab1347
        UpdateStatus($"Resetting Puzzle {currentPuzzle}...");

        switch (currentPuzzle)
        {
            case 1:
                SendCommand("hover");
                break;
            case 2:
                SendCommand("hover");
                break;
            case 3:
                SendCommand("hover");
                break;
            default:
                UpdateStatus("No puzzle selected to reset.");
                break;
        }
<<<<<<< HEAD
=======
>>>>>>> Stashed changes
>>>>>>> 60a1c52683894d3f760f90a40aea1f9935ab1347
    }

    // ─────────────────────────────────────────
    // SINGULARITY WARNING
    // ─────────────────────────────────────────

    public void TriggerSingularityWarning()
    {
        if (singularityWarningPanel != null)
        {
            singularityWarningPanel.SetActive(true);
            if (singularityWarningText != null)
                singularityWarningText.text = "⚠ WARNING: Singularity Detected!\nReturning to safe position...";
        }
        UpdateStatus("⚠ Singularity Warning!");
<<<<<<< HEAD
        SendCommand("hover");
        Debug.Log("GUI: Singularity warning triggered");
=======
<<<<<<< Updated upstream
        SendRobotCommand("hover");
=======
        SendCommand("hover");
        Debug.Log("GUI: Singularity warning triggered");
>>>>>>> Stashed changes
>>>>>>> 60a1c52683894d3f760f90a40aea1f9935ab1347
    }

    public void OnDismissWarning()
    {
        if (singularityWarningPanel != null)
            singularityWarningPanel.SetActive(false);
        UpdateStatus("Robot ready.");
    }

    void UpdateStatus(string message)
    {
        if (statusText != null)
            statusText.text = message;
    }
}