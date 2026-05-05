// using UnityEngine;
// using UnityEngine.InputSystem;
// using UnityEngine.UI;

// public class PuzzleGUIScript : MonoBehaviour
// {
//     [Header("Input")]
//     public InputActionReference openGUIAction;

//     [Header("GUI")]
//     public GameObject puzzleGUI;
//     public Camera robotArmCamera;

//     private bool isGUIOpen = false;

//     void Start()
//     {
//         // Make sure GUI is hidden at start
//         puzzleGUI.SetActive(false);
//         robotArmCamera.gameObject.SetActive(false);
//     }

//     void OnEnable()
//     {
//         if (openGUIAction != null)
//             openGUIAction.action.performed += OnGUIButtonPressed;
//     }

//     void OnDisable()
//     {
//         if (openGUIAction != null)
//             openGUIAction.action.performed -= OnGUIButtonPressed;
//     }

//     private void OnGUIButtonPressed(InputAction.CallbackContext context)
//     {
//         ToggleGUI();
//     }

//     public void ToggleGUI()
//     {
//         isGUIOpen = !isGUIOpen;
//         puzzleGUI.SetActive(isGUIOpen);
//         robotArmCamera.gameObject.SetActive(isGUIOpen);
//     }

//     public void CloseGUI()
//     {
//         isGUIOpen = false;
//         puzzleGUI.SetActive(false);
//         robotArmCamera.gameObject.SetActive(false);
//     }
// }
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using TMPro;

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

    void Start()
    {
        puzzleGUI.SetActive(false);
        robotArmCamera.gameObject.SetActive(false);

        if (singularityWarningPanel != null)
            singularityWarningPanel.SetActive(false);

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
    // BUTTON FUNCTIONS
    // ─────────────────────────────────────────

    public void OnResetRobotPressed()
    {
        UpdateStatus("Returning robot to reset position...");
        Debug.Log("GUI: Reset Robot pressed");

        // TEAMMATE: call your reset function here
        // e.g. robotMovementScript.ResetPosition();
    }

    public void OnPuzzle1Pressed()
    {
        currentPuzzle = 1;
        UpdateStatus("Puzzle 1: Button Matrix (LH)");
        Debug.Log("GUI: Puzzle 1 selected");

        // TEAMMATE: call move to puzzle 1 position here
        // e.g. robotMovementScript.MoveToPuzzle1();
    }

    public void OnPuzzle2Pressed()
    {
        currentPuzzle = 2;
        UpdateStatus("Puzzle 2: Slider (RH)");
        Debug.Log("GUI: Puzzle 2 selected");

        // TEAMMATE: call move to puzzle 2 position here
        // e.g. robotMovementScript.MoveToPuzzle2();
    }

    public void OnPuzzle3Pressed()
    {
        currentPuzzle = 3;
        UpdateStatus("Puzzle 3: Egg Sorting");
        Debug.Log("GUI: Puzzle 3 selected");

        // TEAMMATE: call move to puzzle 3 position here
        // e.g. robotMovementScript.MoveToPuzzle3();
    }

    public void OnResetPuzzlePressed()
    {
        UpdateStatus($"Resetting Puzzle {currentPuzzle}...");

        switch (currentPuzzle)
        {
            case 1:
                Debug.Log("GUI: Reset Puzzle 1 - Button Matrix");
                // TEAMMATE: e.g. buttonMatrixScript.ResetPuzzle();
                break;
            case 2:
                Debug.Log("GUI: Reset Puzzle 2 - Slider to start");
                // TEAMMATE: e.g. sliderPuzzleScript.ResetToStart();
                break;
            case 3:
                Debug.Log("GUI: Reset Puzzle 3 - Egg Sorting");
                // TEAMMATE: e.g. eggSortingScript.ResetEggs();
                break;
            default:
                UpdateStatus("No puzzle selected to reset.");
                break;
        }
    }

    // ─────────────────────────────────────────
    // SINGULARITY WARNING
    // Teammate calls this from their robot script:
    // GetComponent<PuzzleGUIScript>().TriggerSingularityWarning();
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
        Debug.Log("GUI: Singularity warning triggered");

        // TEAMMATE: auto recovery here
        // e.g. robotMovementScript.RecoverFromSingularity();
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