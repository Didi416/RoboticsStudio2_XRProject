using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

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
    private const string commandTopic = "/robot_gui/command";

    void Start()
    {
        puzzleGUI.SetActive(false);
        robotArmCamera.gameObject.SetActive(false);

        if (singularityWarningPanel != null)
            singularityWarningPanel.SetActive(false);

        // Set up ROS publisher
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(commandTopic);

        // Hook up all buttons
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
    // ─────────────────────────────────────────

    public void OnResetRobotPressed()
    {
        UpdateStatus("Returning robot to hover position...");
        SendCommand("hover");
    }

    public void OnPuzzle1Pressed()
    {
        currentPuzzle = 1;
        UpdateStatus("Puzzle 1: Moving to board...");
        SendCommand("face_board");
    }

    public void OnPuzzle2Pressed()
    {
        currentPuzzle = 2;
        UpdateStatus("Puzzle 2: Lily Pad Maze");
        SendCommand("face_board");
    }

    public void OnPuzzle3Pressed()
    {
        currentPuzzle = 3;
        UpdateStatus("Puzzle 3: Moving to eggs...");
        SendCommand("face_eggs");
    }

    public void OnResetPuzzlePressed()
    {
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
        SendCommand("hover");
        Debug.Log("GUI: Singularity warning triggered");
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