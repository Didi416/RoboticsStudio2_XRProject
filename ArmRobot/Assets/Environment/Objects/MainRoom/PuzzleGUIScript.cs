using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

public class PuzzleGUIScript : MonoBehaviour
{
    [Header("Input")]
    public InputActionReference openGUIAction;

    [Header("GUI")]
    public GameObject puzzleGUI;
    public Camera robotArmCamera;

    private bool isGUIOpen = false;

    void Start()
    {
        // Make sure GUI is hidden at start
        puzzleGUI.SetActive(false);
        robotArmCamera.gameObject.SetActive(false);
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
    }
}
