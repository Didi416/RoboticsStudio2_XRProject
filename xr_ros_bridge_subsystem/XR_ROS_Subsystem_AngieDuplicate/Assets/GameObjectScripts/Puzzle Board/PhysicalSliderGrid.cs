using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;

public class PhysicalSliderGrid : MonoBehaviour
{
    [Header("References")]
    public LEDMatrixPanel ledMatrixPanel;

    [Header("Mouse Testing")]
    public bool enableMouseTesting = true;

    void Update()
    {
        if (!enableMouseTesting) return;

        // Arrow keys for testing in editor
        if (Input.GetKeyDown(KeyCode.UpArrow))
            ledMatrixPanel.MoveUp();
        if (Input.GetKeyDown(KeyCode.DownArrow))
            ledMatrixPanel.MoveDown();
        if (Input.GetKeyDown(KeyCode.LeftArrow))
            ledMatrixPanel.MoveLeft();
        if (Input.GetKeyDown(KeyCode.RightArrow))
            ledMatrixPanel.MoveRight();
    }

    // TEAMMATE hooks these up to joystick
    public void OnJoystickUp()    => ledMatrixPanel.MoveUp();
    public void OnJoystickDown()  => ledMatrixPanel.MoveDown();
    public void OnJoystickLeft()  => ledMatrixPanel.MoveLeft();
    public void OnJoystickRight() => ledMatrixPanel.MoveRight();
}