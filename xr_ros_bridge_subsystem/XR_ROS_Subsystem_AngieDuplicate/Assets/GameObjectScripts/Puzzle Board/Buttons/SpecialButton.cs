using UnityEngine;
using UnityEngine.InputSystem;

// ─────────────────────────────────────────
// SPECIAL BUTTON
// Attach to the * (Reset) and # (Enter) 3D objects in the scene.
// Works identically to PhysicalButton but calls Reset or Enter
// on ButtonMatrixPanel instead of a digit number.
//
// Setup in Inspector:
//   buttonType = Reset  → attach to your * button object
//   buttonType = Enter  → attach to your # button object
//   buttonMatrixPanel   → drag in the ButtonMatrixPanel
// ─────────────────────────────────────────

public class SpecialButton : MonoBehaviour
{
    public enum ButtonType { Reset, Enter }

    [Header("Button Settings")]
    public ButtonType buttonType;
    public Color defaultColor = Color.white;
    public Color pressedColor = new Color(1f, 0.4f, 0f);   // orange for reset
    public Color enterColor   = new Color(0f, 0.9f, 0.3f); // green for enter

    [Header("References")]
    public ButtonMatrixPanel buttonMatrixPanel;

    [Header("Testing")]
    public bool enableMouseClick = true;

    private Renderer buttonRenderer;
    private bool isPressed = false;

    void Start()
    {
        buttonRenderer = GetComponent<Renderer>();
        if (buttonRenderer != null)
            buttonRenderer.material.color = defaultColor;
    }

    void Update()
    {
        if (enableMouseClick && Mouse.current.leftButton.wasPressedThisFrame)
        {
            Ray ray = Camera.main.ScreenPointToRay(Mouse.current.position.ReadValue());
            RaycastHit hit;
            if (Physics.Raycast(ray, out hit) && hit.collider.gameObject == gameObject)
                PressButton();
        }
    }

    // ─────────────────────────────────────────
    // PHYSICAL TRIGGER — any collider entering
    // ─────────────────────────────────────────

    void OnTriggerEnter(Collider other)
    {
        if (other.GetComponent<PhysicalButton>() != null) return;
        if (other.GetComponent<SpecialButton>() != null) return;
        Debug.Log($"SpecialButton {buttonType} triggered by {other.gameObject.name}");
        PressButton();
    }

    // ─────────────────────────────────────────
    // PRESS
    // ─────────────────────────────────────────

    public void PressButton()
    {
        if (isPressed) return;
        StartCoroutine(FlashAndTrigger());
    }

    System.Collections.IEnumerator FlashAndTrigger()
    {
        isPressed = true;

        Color flash = (buttonType == ButtonType.Enter) ? enterColor : pressedColor;

        if (buttonRenderer != null)
            buttonRenderer.material.color = flash;

        // Call the right panel method
        if (buttonMatrixPanel != null)
        {
            if (buttonType == ButtonType.Reset)
                buttonMatrixPanel.PressReset();
            else
                buttonMatrixPanel.PressEnter();
        }
        else
        {
            Debug.LogError("SpecialButton: ButtonMatrixPanel not assigned!");
        }

        yield return new WaitForSeconds(0.5f);

        if (buttonRenderer != null)
            buttonRenderer.material.color = defaultColor;

        isPressed = false;
    }
}