// using UnityEngine;
// using UnityEngine.XR.Interaction.Toolkit;
// using UnityEngine.XR.Interaction.Toolkit.Interactables;

// public class PhysicalButton : MonoBehaviour
// {
//     [Header("Button Settings")]
//     public int buttonNumber; // set 1-9 in Inspector per button
//     public Color defaultColor = Color.white;
//     public Color pressedColor = new Color(0f, 0.8f, 1f);

//     [Header("References")]
//     public ButtonMatrixPanel buttonMatrixPanel;

//     private Renderer buttonRenderer;
//     private XRSimpleInteractable interactable;

//     void Start()
//     {
//         buttonRenderer = GetComponent<Renderer>();
//         if (buttonRenderer != null)
//             buttonRenderer.material.color = defaultColor;

//         interactable = GetComponent<XRSimpleInteractable>();
//         if (interactable != null)
//             interactable.selectEntered.AddListener(OnPressed);
//     }

//     // Called by robot script (teammate)
//     public void PressButton()
//     {
//         StartCoroutine(FlashAndRegister());
//     }

//     // Called by XR ray or mouse click
//     void OnPressed(SelectEnterEventArgs args)
//     {
//         PressButton();
//     }

//     // Mouse click testing
//     void OnMouseDown()
//     {
//         PressButton();
//     }

//     System.Collections.IEnumerator FlashAndRegister()
//     {
//         // Flash button color
//         if (buttonRenderer != null)
//             buttonRenderer.material.color = pressedColor;

//         // Register with panel
//         if (buttonMatrixPanel != null)
//             buttonMatrixPanel.PressButton(buttonNumber);

//         yield return new WaitForSeconds(0.3f);

//         if (buttonRenderer != null)
//             buttonRenderer.material.color = defaultColor;
//     }
// }

using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;

public class PhysicalButton : MonoBehaviour
{
    [Header("Button Settings")]
    public int buttonNumber;
    public Color defaultColor = Color.white;
    public Color pressedColor = new Color(0f, 0.8f, 1f);

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
        // Mouse click testing
        if (enableMouseClick && Mouse.current.leftButton.wasPressedThisFrame)
        {
            Ray ray = Camera.main.ScreenPointToRay(
                Mouse.current.position.ReadValue());
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit))
            {
                if (hit.collider.gameObject == gameObject)
                {
                    Debug.Log($"Mouse clicked Button {buttonNumber}");
                    PressButton();
                }
            }
        }
    }

    // ─────────────────────────────────────────
    // ANY OBJECT TOUCHING TRIGGERS THE BUTTON
    // ─────────────────────────────────────────
    void OnTriggerEnter(Collider other)
    {
        // Ignore other buttons touching each other
        if (other.GetComponent<PhysicalButton>() != null) return;

        Debug.Log($"Button {buttonNumber} triggered by {other.gameObject.name}");
        PressButton();
    }

    // ─────────────────────────────────────────
    // PRESS THE BUTTON
    // ─────────────────────────────────────────
    public void PressButton()
    {
        if (isPressed) return;
        StartCoroutine(FlashAndRegister());
    }

    System.Collections.IEnumerator FlashAndRegister()
    {
        isPressed = true;

        if (buttonRenderer != null)
            buttonRenderer.material.color = pressedColor;

        if (buttonMatrixPanel != null)
            buttonMatrixPanel.PressButton(buttonNumber);

        yield return new WaitForSeconds(0.5f);

        if (buttonRenderer != null)
            buttonRenderer.material.color = defaultColor;

        isPressed = false;
    }
}