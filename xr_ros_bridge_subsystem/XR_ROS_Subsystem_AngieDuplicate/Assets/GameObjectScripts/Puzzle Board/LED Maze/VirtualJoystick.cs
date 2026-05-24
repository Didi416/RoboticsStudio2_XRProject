using UnityEngine;
using UnityEngine.InputSystem;

public class VirtualJoystick : MonoBehaviour
{
    [Header("References")]
    public LEDMatrixPanel ledMatrixPanel;

    [Header("Joystick Settings")]
    public float returnSpeed = 5f;
    public float maxTilt = 0.3f;
    public float moveCooldown = 0.3f;

    private Vector3 centerPosition;
    private float lastMoveTime;
    private bool isHeld = false;
    private Vector3 lastContactPosition;

    void Start()
    {
        centerPosition = transform.localPosition;
    }

    void Update()
    {
        HandleKeyboardInput();
        ReturnToCenter();
    }

    // ─────────────────────────────────────────
    // KEYBOARD - WASD + ARROW KEYS
    // Both work identically
    // ─────────────────────────────────────────

    void HandleKeyboardInput()
    {
        if (Time.time - lastMoveTime < moveCooldown) return;

        // UP
        if (Keyboard.current[Key.UpArrow].wasPressedThisFrame ||
            Keyboard.current[Key.W].wasPressedThisFrame)
        {
            Debug.Log("Input: UP");
            TriggerDirection(Vector2Int.up);
            AnimateJoystick(Vector3.forward);
        }
        // DOWN
        else if (Keyboard.current[Key.DownArrow].wasPressedThisFrame ||
                 Keyboard.current[Key.S].wasPressedThisFrame)
        {
            Debug.Log("Input: DOWN");
            TriggerDirection(Vector2Int.down);
            AnimateJoystick(Vector3.back);
        }
        // LEFT
        else if (Keyboard.current[Key.LeftArrow].wasPressedThisFrame ||
                 Keyboard.current[Key.A].wasPressedThisFrame)
        {
            Debug.Log("Input: LEFT");
            TriggerDirection(Vector2Int.left);
            AnimateJoystick(Vector3.left);
        }
        // RIGHT
        else if (Keyboard.current[Key.RightArrow].wasPressedThisFrame ||
                 Keyboard.current[Key.D].wasPressedThisFrame)
        {
            Debug.Log("Input: RIGHT");
            TriggerDirection(Vector2Int.right);
            AnimateJoystick(Vector3.right);
        }
    }

    // ─────────────────────────────────────────
    // PHYSICAL CONTACT - any object touching
    // Detects direction of push automatically
    // ─────────────────────────────────────────

    void OnTriggerEnter(Collider other)
    {
        if (other.transform == transform) return;
        isHeld = true;
        lastContactPosition = other.transform.position;
        Debug.Log($"Joystick touched by: {other.gameObject.name}");
    }

    void OnTriggerStay(Collider other)
    {
        if (!isHeld) return;
        if (Time.time - lastMoveTime < moveCooldown) return;

        Vector3 delta = other.transform.position - lastContactPosition;

        if (delta.magnitude < 0.01f) return;

        // Find dominant axis and direction
        if (Mathf.Abs(delta.x) > Mathf.Abs(delta.z))
        {
            if (delta.x > 0)
            {
                Debug.Log("Physical: RIGHT");
                TriggerDirection(Vector2Int.right);
                AnimateJoystick(Vector3.right);
            }
            else
            {
                Debug.Log("Physical: LEFT");
                TriggerDirection(Vector2Int.left);
                AnimateJoystick(Vector3.left);
            }
        }
        else
        {
            if (delta.z > 0)
            {
                Debug.Log("Physical: UP");
                TriggerDirection(Vector2Int.up);
                AnimateJoystick(Vector3.forward);
            }
            else
            {
                Debug.Log("Physical: DOWN");
                TriggerDirection(Vector2Int.down);
                AnimateJoystick(Vector3.back);
            }
        }

        lastContactPosition = other.transform.position;
    }

    void OnTriggerExit(Collider other)
    {
        isHeld = false;
        Debug.Log($"Joystick released by: {other.gameObject.name}");
    }

    // ─────────────────────────────────────────
    // HELPERS
    // ─────────────────────────────────────────

    void AnimateJoystick(Vector3 direction)
    {
        transform.localPosition = centerPosition + direction * maxTilt;
    }

    void ReturnToCenter()
    {
        transform.localPosition = Vector3.Lerp(
            transform.localPosition,
            centerPosition,
            Time.deltaTime * returnSpeed);
    }

    void TriggerDirection(Vector2Int direction)
    {
        lastMoveTime = Time.time;

        if (ledMatrixPanel == null)
        {
            Debug.LogError("LEDMatrixPanel not assigned!");
            return;
        }

        if (direction == Vector2Int.up)    ledMatrixPanel.MoveUp();
        if (direction == Vector2Int.down)  ledMatrixPanel.MoveDown();
        if (direction == Vector2Int.left)  ledMatrixPanel.MoveLeft();
        if (direction == Vector2Int.right) ledMatrixPanel.MoveRight();
    }
}