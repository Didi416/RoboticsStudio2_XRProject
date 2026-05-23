using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class PuzzleBoardManager : MonoBehaviour
{
    [Header("Panel References")]
    public LEDMatrixPanel ledMatrix;
    public ButtonMatrixPanel buttonMatrix;

    [Header("Robot GUI Verification Display")]
    public TextMeshProUGUI ledMatrixStatusText;
    public TextMeshProUGUI buttonMatrixStatusText;
    public Image ledMatrixIndicator;
    public Image buttonMatrixIndicator;

    [Header("Reset Buttons")]
    public Button resetLEDButton;
    public Button resetButtonMatrixButton;

    void Start()
    {
        if (resetLEDButton != null)
            resetLEDButton.onClick.AddListener(ResetLEDMatrix);
        if (resetButtonMatrixButton != null)
            resetButtonMatrixButton.onClick.AddListener(ResetButtonMatrix);

        UpdateGUIStatus("LED Matrix: In Progress", "Button Matrix: In Progress");
    }

    // ─────────────────────────────────────────
    // CALLED BY LED MATRIX WHEN SOLVED
    // ─────────────────────────────────────────

    public void OnLEDMatrixSolved()
    {
        if (ledMatrixStatusText != null)
            ledMatrixStatusText.text = "LED Matrix: ✓ SOLVED";
        if (ledMatrixIndicator != null)
            ledMatrixIndicator.color = Color.green;
    }

    public void OnLEDMatrixFailed()
    {
        if (ledMatrixStatusText != null)
            ledMatrixStatusText.text = "LED Matrix: ✗ Wrong Path";
        if (ledMatrixIndicator != null)
            ledMatrixIndicator.color = Color.red;
    }

    // ─────────────────────────────────────────
    // CALLED BY BUTTON MATRIX WHEN SOLVED
    // ─────────────────────────────────────────

    public void OnButtonMatrixSolved()
    {
        if (buttonMatrixStatusText != null)
            buttonMatrixStatusText.text = "Button Matrix: ✓ SOLVED";
        if (buttonMatrixIndicator != null)
            buttonMatrixIndicator.color = Color.green;
    }

    public void OnButtonMatrixFailed()
    {
        if (buttonMatrixStatusText != null)
            buttonMatrixStatusText.text = "Button Matrix: ✗ Wrong Code";
        if (buttonMatrixIndicator != null)
            buttonMatrixIndicator.color = Color.red;
    }

    void UpdateGUIStatus(string led, string button)
    {
        if (ledMatrixStatusText != null)
            ledMatrixStatusText.text = led;
        if (buttonMatrixStatusText != null)
            buttonMatrixStatusText.text = button;
    }

    public void ResetLEDMatrix()
    {
        if (ledMatrix != null)
            ledMatrix.ResetMatrix();
        UpdateGUIStatus("LED Matrix: In Progress",
            buttonMatrixStatusText != null ? buttonMatrixStatusText.text : "");
    }

    public void ResetButtonMatrix()
    {
        if (buttonMatrix != null)
            buttonMatrix.ResetCode();
    }
}