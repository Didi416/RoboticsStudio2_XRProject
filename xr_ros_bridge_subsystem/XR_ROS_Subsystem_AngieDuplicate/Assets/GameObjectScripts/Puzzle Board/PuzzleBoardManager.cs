
// using UnityEngine;
// using UnityEngine.UI;
// using TMPro;

// public class PuzzleBoardManager : MonoBehaviour
// {
//     [Header("Panel References")]
//     public LEDMatrixPanel ledMatrix;
//     public ButtonMatrixPanel buttonMatrix;

//     [Header("RGB Status Light")]
//     public RGBStatusLight rgbStatusLight;

//     [Header("Robot GUI Verification Display")]
//     public TextMeshProUGUI ledMatrixStatusText;
//     public TextMeshProUGUI buttonMatrixStatusText;
//     public TextMeshProUGUI cubePuzzleStatusText;
//     public Image ledMatrixIndicator;
//     public Image buttonMatrixIndicator;
//     public Image cubePuzzleIndicator;

//     [Header("Reset Buttons")]
//     public Button resetLEDButton;
//     public Button resetButtonMatrixButton;

//     void Start()
//     {
//         if (resetLEDButton != null)
//             resetLEDButton.onClick.AddListener(ResetLEDMatrix);
//         if (resetButtonMatrixButton != null)
//             resetButtonMatrixButton.onClick.AddListener(ResetButtonMatrix);
//     }

//     // ─────────────────────────────────────────
//     // LED MATRIX
//     // ─────────────────────────────────────────

//     public void OnLEDMatrixSolved()
//     {
//         Debug.Log("PuzzleBoardManager: LED Matrix Solved!");
//         if (ledMatrixStatusText != null)
//             ledMatrixStatusText.text = "Slider: ✓ SOLVED";
//         if (ledMatrixIndicator != null)
//             ledMatrixIndicator.color = Color.green;
//         if (rgbStatusLight != null)
//             rgbStatusLight.OnLEDMatrixSolved();
//     }

//     public void OnLEDMatrixFailed()
//     {
//         Debug.Log("PuzzleBoardManager: LED Matrix Failed!");
//         if (ledMatrixStatusText != null)
//             ledMatrixStatusText.text = "Slider: ✗ Wrong Path";
//         if (ledMatrixIndicator != null)
//             ledMatrixIndicator.color = Color.red;
//         if (rgbStatusLight != null)
//             rgbStatusLight.OnLEDMatrixFailed();
//     }

//     // ─────────────────────────────────────────
//     // BUTTON MATRIX
//     // ─────────────────────────────────────────

//     public void OnButtonMatrixSolved()
//     {
//         Debug.Log("PuzzleBoardManager: Button Matrix Solved!");
//         if (buttonMatrixStatusText != null)
//             buttonMatrixStatusText.text = "Button Matrix: ✓ SOLVED";
//         if (buttonMatrixIndicator != null)
//             buttonMatrixIndicator.color = Color.green;
//         if (rgbStatusLight != null)
//             rgbStatusLight.OnButtonMatrixSolved();
//     }

//     public void OnButtonMatrixFailed()
//     {
//         Debug.Log("PuzzleBoardManager: Button Matrix Failed!");
//         if (buttonMatrixStatusText != null)
//             buttonMatrixStatusText.text = "Button Matrix: ✗ Wrong Code";
//         if (buttonMatrixIndicator != null)
//             buttonMatrixIndicator.color = Color.red;
//         if (rgbStatusLight != null)
//             rgbStatusLight.OnButtonMatrixFailed();
//     }

//     // ─────────────────────────────────────────
//     // CUBE PUZZLE
//     // ─────────────────────────────────────────

//     public void OnCubePuzzleSolved()
//     {
//         Debug.Log("PuzzleBoardManager: Cube Puzzle Solved!");
//         if (cubePuzzleStatusText != null)
//             cubePuzzleStatusText.text = "Cube Puzzle: ✓ SOLVED";
//         if (cubePuzzleIndicator != null)
//             cubePuzzleIndicator.color = Color.green;
//         if (rgbStatusLight != null)
//             rgbStatusLight.OnCubePuzzleSolved();
//     }

//     public void OnCubePuzzleFailed()
//     {
//         Debug.Log("PuzzleBoardManager: Cube Puzzle Failed!");
//         if (cubePuzzleStatusText != null)
//             cubePuzzleStatusText.text = "Cube Puzzle: ✗ Wrong Order";
//         if (cubePuzzleIndicator != null)
//             cubePuzzleIndicator.color = Color.red;
//         if (rgbStatusLight != null)
//             rgbStatusLight.OnCubePuzzleFailed();
//     }

//     // ─────────────────────────────────────────
//     // RESET
//     // ─────────────────────────────────────────

//     public void ResetLEDMatrix()
//     {
//         if (ledMatrix != null)
//             ledMatrix.ResetMatrix();
//         if (ledMatrixStatusText != null)
//             ledMatrixStatusText.text = "Slider: In Progress";
//         if (ledMatrixIndicator != null)
//             ledMatrixIndicator.color = Color.grey;
//     }

//     public void ResetButtonMatrix()
//     {
//         if (buttonMatrix != null)
//             buttonMatrix.ResetCode();
//         if (buttonMatrixStatusText != null)
//             buttonMatrixStatusText.text = "Button Matrix: In Progress";
//         if (buttonMatrixIndicator != null)
//             buttonMatrixIndicator.color = Color.grey;
//     }
// }

using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class PuzzleBoardManager : MonoBehaviour
{
    [Header("Panel References")]
    public LEDMatrixPanel ledMatrix;
    public ButtonMatrixPanel buttonMatrix;

    [Header("RGB Status Light")]
    public RGBStatusLight rgbStatusLight;

    [Header("Robot GUI Verification Display")]
    public TextMeshProUGUI ledMatrixStatusText;
    public TextMeshProUGUI buttonMatrixStatusText;
    public TextMeshProUGUI cubePuzzleStatusText;
    public Image ledMatrixIndicator;
    public Image buttonMatrixIndicator;
    public Image cubePuzzleIndicator;

    [Header("Reset Buttons")]
    public Button resetLEDButton;
    public Button resetButtonMatrixButton;

    // Track solved states
    private bool ledMatrixSolved = false;
    private bool buttonMatrixSolved = false;
    private bool cubePuzzleSolved = false;

    // Public getters for RFID check
    public bool IsLEDMatrixSolved() => ledMatrixSolved;
    public bool IsButtonMatrixSolved() => buttonMatrixSolved;
    public bool IsCubePuzzleSolved() => cubePuzzleSolved;
    public bool IsAllSolved() => ledMatrixSolved && buttonMatrixSolved && cubePuzzleSolved;

    void Start()
    {
        if (resetLEDButton != null)
            resetLEDButton.onClick.AddListener(ResetLEDMatrix);
        if (resetButtonMatrixButton != null)
            resetButtonMatrixButton.onClick.AddListener(ResetButtonMatrix);
    }

    // ─────────────────────────────────────────
    // LED MATRIX
    // ─────────────────────────────────────────

    public void OnLEDMatrixSolved()
    {
        ledMatrixSolved = true;
        Debug.Log("PuzzleBoardManager: LED Matrix Solved!");
        if (ledMatrixStatusText != null)
            ledMatrixStatusText.text = "Slider: ✓ SOLVED";
        if (ledMatrixIndicator != null)
            ledMatrixIndicator.color = Color.green;
        if (rgbStatusLight != null)
            rgbStatusLight.OnLEDMatrixSolved();
    }

    public void OnLEDMatrixFailed()
    {
        ledMatrixSolved = false;
        Debug.Log("PuzzleBoardManager: LED Matrix Failed!");
        if (ledMatrixStatusText != null)
            ledMatrixStatusText.text = "Slider: ✗ Wrong Path";
        if (ledMatrixIndicator != null)
            ledMatrixIndicator.color = Color.red;
        if (rgbStatusLight != null)
            rgbStatusLight.OnLEDMatrixFailed();
    }

    // ─────────────────────────────────────────
    // BUTTON MATRIX
    // ─────────────────────────────────────────

    public void OnButtonMatrixSolved()
    {
        buttonMatrixSolved = true;
        Debug.Log("PuzzleBoardManager: Button Matrix Solved!");
        if (buttonMatrixStatusText != null)
            buttonMatrixStatusText.text = "Button Matrix: ✓ SOLVED";
        if (buttonMatrixIndicator != null)
            buttonMatrixIndicator.color = Color.green;
        if (rgbStatusLight != null)
            rgbStatusLight.OnButtonMatrixSolved();
    }

    public void OnButtonMatrixFailed()
    {
        buttonMatrixSolved = false;
        Debug.Log("PuzzleBoardManager: Button Matrix Failed!");
        if (buttonMatrixStatusText != null)
            buttonMatrixStatusText.text = "Button Matrix: ✗ Wrong Code";
        if (buttonMatrixIndicator != null)
            buttonMatrixIndicator.color = Color.red;
        if (rgbStatusLight != null)
            rgbStatusLight.OnButtonMatrixFailed();
    }

    // ─────────────────────────────────────────
    // CUBE PUZZLE
    // ─────────────────────────────────────────

    public void OnCubePuzzleSolved()
    {
        cubePuzzleSolved = true;
        Debug.Log("PuzzleBoardManager: Cube Puzzle Solved!");
        if (cubePuzzleStatusText != null)
            cubePuzzleStatusText.text = "Cube Puzzle: ✓ SOLVED";
        if (cubePuzzleIndicator != null)
            cubePuzzleIndicator.color = Color.green;
        if (rgbStatusLight != null)
            rgbStatusLight.OnCubePuzzleSolved();
    }

    public void OnCubePuzzleFailed()
    {
        cubePuzzleSolved = false;
        Debug.Log("PuzzleBoardManager: Cube Puzzle Failed!");
        if (cubePuzzleStatusText != null)
            cubePuzzleStatusText.text = "Cube Puzzle: ✗ Wrong Order";
        if (cubePuzzleIndicator != null)
            cubePuzzleIndicator.color = Color.red;
        if (rgbStatusLight != null)
            rgbStatusLight.OnCubePuzzleFailed();
    }

    // ─────────────────────────────────────────
    // RESET
    // ─────────────────────────────────────────

    public void ResetLEDMatrix()
    {
        ledMatrixSolved = false;
        if (ledMatrix != null) ledMatrix.ResetMatrix();
        if (ledMatrixStatusText != null)
            ledMatrixStatusText.text = "Slider: In Progress";
        if (ledMatrixIndicator != null)
            ledMatrixIndicator.color = Color.grey;
    }

    public void ResetButtonMatrix()
    {
        buttonMatrixSolved = false;
        if (buttonMatrix != null) buttonMatrix.ResetCode();
        if (buttonMatrixStatusText != null)
            buttonMatrixStatusText.text = "Button Matrix: In Progress";
        if (buttonMatrixIndicator != null)
            buttonMatrixIndicator.color = Color.grey;
    }
}