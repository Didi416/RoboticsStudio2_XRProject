using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class Puzzle3StateManager : MonoBehaviour
{
    // ── Inspector fields — drag in your Unity objects ──────────
    [Header("Puzzle 3 — Egg Sorting")]
    public GameObject solvedEffect;      // particle effect or UI panel to show on solve
    public AudioSource solvedSound;      // optional sound on solve
    public TMPro.TextMeshProUGUI statusText; // optional debug text in VR

    private ROSConnection _ros;
    private bool _lastSolvedState = false;

    void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to the Bool topic
        _ros.Subscribe<BoolMsg>(
            "/puzzle3/puzzle_solved",
            OnPuzzleSolvedUpdate
        );

        // Optional — subscribe to status string for debug display in VR
        _ros.Subscribe<StringMsg>(
            "/puzzle3/status",
            OnStatusUpdate
        );
    }

    void OnPuzzleSolvedUpdate(BoolMsg msg)
    {
        bool isSolved = msg.data;

        // Only trigger effects on the transition from unsolved → solved
        if (isSolved && !_lastSolvedState)
        {
            OnPuzzleJustSolved();
        }

        // Reset effects when a new run starts (solved → unsolved)
        if (!isSolved && _lastSolvedState)
        {
            OnPuzzleReset();
        }

        _lastSolvedState = isSolved;
    }

    void OnPuzzleJustSolved()
    {
        Debug.Log("Puzzle 3 SOLVED — triggering VR effects");

        if (solvedEffect != null)
            solvedEffect.SetActive(true);

        if (solvedSound != null)
            solvedSound.Play();

        // Add whatever else you want here:
        // - unlock next puzzle panel
        // - play animation
        // - update puzzle progress tracker
    }

    void OnPuzzleReset()
    {
        Debug.Log("Puzzle 3 reset — new run started");

        if (solvedEffect != null)
            solvedEffect.SetActive(false);
    }

    void OnStatusUpdate(StringMsg msg)
    {
        // Optional — show raw status in a VR debug panel
        if (statusText != null)
            statusText.text = msg.data;
    }
}