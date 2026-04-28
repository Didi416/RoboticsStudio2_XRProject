using UnityEngine;
using System.Collections.Generic;
using UnityEngine.InputSystem;

public class RobotPuzzleController : MonoBehaviour
{
    [Header("Reference to Lily Pad Grid")]
    public LilyPadGrid lilyPadGrid;

    [Header("Input Actions")]
    public InputActionReference moveUpAction;
    public InputActionReference moveDownAction;
    public InputActionReference moveLeftAction;
    public InputActionReference moveRightAction;
    public InputActionReference confirmAction;    // trigger to confirm move
    public InputActionReference resetAction;      // reset attempt

    [Header("Robot Cursor Visual")]
    public GameObject robotCursorPrefab;  // visual marker showing current position
    public float cellSize = 1f;           // should match LilyPadGrid cellSize

    [Header("Feedback")]
    public AudioSource audioSource;
    public AudioClip correctStepSound;
    public AudioClip wrongStepSound;
    public AudioClip solvedSound;

    private Vector2Int robotPosition;
    private List<Vector2Int> playerPath = new List<Vector2Int>();
    private GameObject robotCursor;
    private bool puzzleSolved = false;

    void OnEnable()
    {
        moveUpAction.action.performed += _ => TryMove(Vector2Int.up);
        moveDownAction.action.performed += _ => TryMove(Vector2Int.down);
        moveLeftAction.action.performed += _ => TryMove(Vector2Int.left);
        moveRightAction.action.performed += _ => TryMove(Vector2Int.right);
        resetAction.action.performed += _ => ResetAttempt();
    }

    void OnDisable()
    {
        moveUpAction.action.performed -= _ => TryMove(Vector2Int.up);
        moveDownAction.action.performed -= _ => TryMove(Vector2Int.down);
        moveLeftAction.action.performed -= _ => TryMove(Vector2Int.left);
        moveRightAction.action.performed -= _ => TryMove(Vector2Int.right);
        resetAction.action.performed -= _ => ResetAttempt();
    }

    void Start()
    {
        // Start robot at same position as path start
        if (lilyPadGrid != null && lilyPadGrid.currentPath.Count > 0)
        {
            robotPosition = lilyPadGrid.currentPath[0];
            playerPath.Add(robotPosition);
        }

        // Spawn cursor
        if (robotCursorPrefab != null)
        {
            Vector3 startPos = new Vector3(robotPosition.x * cellSize, 0.1f, robotPosition.y * cellSize);
            robotCursor = Instantiate(robotCursorPrefab, startPos, Quaternion.identity);
        }
    }

    void TryMove(Vector2Int direction)
    {
        if (puzzleSolved) return;

        Vector2Int newPos = robotPosition + direction;

        // Check within grid bounds
        if (newPos.x < 0 || newPos.x >= 5 || newPos.y < 0 || newPos.y >= 5)
        {
            Debug.Log("Out of bounds!");
            return;
        }

        robotPosition = newPos;
        playerPath.Add(robotPosition);

        // Move cursor visual
        if (robotCursor != null)
        {
            robotCursor.transform.position = new Vector3(
                robotPosition.x * cellSize, 0.1f, robotPosition.y * cellSize);
        }

        // Check if this step matches the correct path
        int stepIndex = playerPath.Count - 1;
        List<Vector2Int> correctPath = lilyPadGrid.currentPath;

        if (stepIndex < correctPath.Count && robotPosition == correctPath[stepIndex])
        {
            PlaySound(correctStepSound);
            Debug.Log($"Correct step! {stepIndex + 1}/{correctPath.Count}");

            // Check if fully solved
            if (playerPath.Count == correctPath.Count)
            {
                puzzleSolved = true;
                PlaySound(solvedSound);
                Debug.Log("PUZZLE SOLVED!");
            }
        }
        else
        {
            PlaySound(wrongStepSound);
            Debug.Log("Wrong step! Resetting...");
            ResetAttempt();
        }
    }

    void ResetAttempt()
    {
        playerPath.Clear();
        if (lilyPadGrid != null && lilyPadGrid.currentPath.Count > 0)
        {
            robotPosition = lilyPadGrid.currentPath[0];
            playerPath.Add(robotPosition);
        }

        if (robotCursor != null)
        {
            robotCursor.transform.position = new Vector3(
                robotPosition.x * cellSize, 0.1f, robotPosition.y * cellSize);
        }

        Debug.Log("Attempt reset - try again!");
    }

    void PlaySound(AudioClip clip)
    {
        if (audioSource != null && clip != null)
            audioSource.PlayOneShot(clip);
    }
}