// using UnityEngine;
// using UnityEngine.UI;
// using TMPro;
// using System.Collections.Generic;

// public class CubePuzzleManager : MonoBehaviour
// {
//     [Header("Slots - left to right")]
//     public CubeSlot[] slots;

//     [Header("Egg Stand Randomiser Reference")]
//     public EggStandRandomiser eggStandRandomiser;

//     [Header("UI")]
//     public TextMeshProUGUI resultText;
//     public Image resultLight;

//     [Header("References")]
//     public PuzzleBoardManager puzzleBoardManager;

//     private string[] correctOrder = new string[4];
//     private string[] placedOrder = new string[4];
//     private bool puzzleSolved = false;

//     void Start()
//     {
//         Invoke("BuildCorrectOrder", 0.5f);

//         for (int i = 0; i < placedOrder.Length; i++)
//             placedOrder[i] = "";
//     }

//     // ─────────────────────────────────────────
//     // BUILD CORRECT ORDER FROM STAND POSITIONS
//     // ─────────────────────────────────────────

//     void BuildCorrectOrder()
//     {
//         if (eggStandRandomiser == null)
//         {
//             Debug.LogError("EggStandRandomiser not assigned!");
//             return;
//         }

//         List<GameObject> sortedStands = new List<GameObject>(
//             eggStandRandomiser.eggStands);

//         sortedStands.Sort((a, b) =>
//             a.transform.position.x.CompareTo(b.transform.position.x));

//         for (int i = 0; i < sortedStands.Count && i < 4; i++)
//         {
//             correctOrder[i] = GetEggTypeFromStand(sortedStands[i]);
//             Debug.Log($"Slot {i} correct type: {correctOrder[i]}");
//         }

//         Debug.Log($"Correct cube order: {string.Join(", ", correctOrder)}");
//     }

//     string GetEggTypeFromStand(GameObject stand)
//     {
//         string name = stand.name.ToLower();
//         if (name.Contains("water")) return "water";
//         if (name.Contains("earth")) return "earth";
//         if (name.Contains("space")) return "space";
//         if (name.Contains("cloud")) return "cloud";
//         return "unknown";
//     }

//     // ─────────────────────────────────────────
//     // CALLED WHEN CUBE IS PLACED IN A SLOT
//     // ─────────────────────────────────────────

//     public void OnCubePlaced(int slotIndex, string eggType)
//     {
//         if (puzzleSolved) return;

//         placedOrder[slotIndex] = eggType;

//         Debug.Log($"Placed {eggType} in slot {slotIndex}, correct is {correctOrder[slotIndex]}");

//         if (correctOrder[slotIndex] == eggType)
//         {
//             slots[slotIndex].SetCorrect();
//             Debug.Log($"Slot {slotIndex} CORRECT!");
//         }
//         else
//         {
//             slots[slotIndex].SetWrong();
//             Debug.Log($"Slot {slotIndex} WRONG! Expected {correctOrder[slotIndex]}");
//         }

//         CheckAllSlots();
//     }

//     // ─────────────────────────────────────────
//     // CHECK ALL SLOTS
//     // ─────────────────────────────────────────

//     void CheckAllSlots()
//     {
//         // Check all slots have something placed
//         foreach (string placed in placedOrder)
//             if (placed == "") return;

//         bool allCorrect = true;
//         for (int i = 0; i < 4; i++)
//         {
//             if (placedOrder[i] != correctOrder[i])
//             {
//                 allCorrect = false;
//                 break;
//             }
//         }

//         if (allCorrect)
//         {
//             puzzleSolved = true;

//             if (resultText != null)
//                 resultText.text = "✓ Correct Order!\nPuzzle Solved!";
//             if (resultLight != null)
//                 resultLight.color = Color.green;

//             // ← Notify board manager here inside the method
//             if (puzzleBoardManager != null)
//                 puzzleBoardManager.OnCubePuzzleSolved();

//             Debug.Log("CUBE PUZZLE SOLVED!");
//         }
//         else
//         {
//             if (resultText != null)
//                 resultText.text = "✗ Wrong Order!\nTry Again";
//             if (resultLight != null)
//                 resultLight.color = Color.red;

//             // ← Notify board manager here inside the method
//             if (puzzleBoardManager != null)
//                 puzzleBoardManager.OnCubePuzzleFailed();

//             Debug.Log("Wrong order - try again");
//             Invoke("ResetPuzzle", 2f);
//         }
//     }

//     // ─────────────────────────────────────────
//     // RESET
//     // ─────────────────────────────────────────

//     public void ResetPuzzle()
//     {
//         puzzleSolved = false;

//         for (int i = 0; i < 4; i++)
//             placedOrder[i] = "";

//         foreach (CubeSlot slot in slots)
//             slot.ClearSlot();

//         if (resultText != null)
//             resultText.text = "";
//         if (resultLight != null)
//             resultLight.color = Color.grey;

//         Debug.Log("Cube puzzle reset");
//     }
// }



using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// CubePuzzleManager — Puzzle 3 (Frog Egg Sorting)
///
/// Verification architecture
/// ─────────────────────────
/// This manager uses a DUAL verification approach:
///
///   1. Unity-side check (existing logic, unchanged)
///      The user places physical cubes into slots in VR.
///      CubeSlot/CubeSnapZone report placements here via OnCubePlaced().
///      CheckAllSlots() compares placedOrder[] against correctOrder[].
///
///   2. ROS vision confirmation (new, from egg_sorter_verifier_simple.py)
///      The RealSense camera independently verifies the physical egg order
///      using ArUco markers and publishes /puzzle3/puzzle_solved (Bool).
///      OnRosSolvedUpdate() receives this and sets _rosSolvedConfirmed.
///
/// The puzzle is only considered FULLY solved when BOTH agree:
///   puzzleSolved == true  (Unity VR placement correct)
///   _rosSolvedConfirmed == true  (ROS camera confirmed)
///
/// This prevents false positives where the VR placement looks correct
/// but the physical robot hasn't actually placed the egg in the right spot.
///
/// If you want Unity-only verification (no ROS camera), set
/// requireRosConfirmation = false in the Inspector.
/// </summary>
public class CubePuzzleManager : MonoBehaviour
{
    [Header("Slots - left to right")]
    public CubeSlot[] slots;

    [Header("Egg Stand Randomiser Reference")]
    public EggStandRandomiser eggStandRandomiser;

    [Header("UI")]
    public TextMeshProUGUI resultText;
    public Image resultLight;

    [Header("References")]
    public PuzzleBoardManager puzzleBoardManager;

    // ── ROS Integration ───────────────────────────────────────────────────────
    [Header("ROS Integration")]
    [Tooltip("Topic published by egg_sorter_verifier_simple.py — Bool, true when camera confirms correct order")]
    public string rosSolvedTopic = "/puzzle3/puzzle_solved";

    [Tooltip("Topic published by egg_sorter_verifier_simple.py — String, human-readable status for debug")]
    public string rosStatusTopic = "/puzzle3/status";

    [Tooltip("If true, puzzle only fully solves when BOTH Unity placement AND ROS camera agree.\n" +
             "If false, Unity placement alone is enough (useful when camera isn't running).")]
    public bool requireRosConfirmation = true;

    [Tooltip("Optional — shows raw ROS status string in scene for debugging")]
    public TextMeshProUGUI rosDebugText;

    [Tooltip("Optional — shown when waiting for ROS camera to confirm after VR placement is correct")]
    public GameObject waitingForCameraPanel;

    // ── Private state ─────────────────────────────────────────────────────────
    private string[] correctOrder  = new string[4];
    private string[] placedOrder   = new string[4];
    private bool puzzleSolved      = false;      // Unity VR placement confirmed
    private bool _rosSolvedConfirmed = false;    // ROS camera confirmed
    private bool _unityPlacementCorrect = false; // VR placement is correct, waiting for ROS

    // ─────────────────────────────────────────
    // UNITY LIFECYCLE
    // ─────────────────────────────────────────

    void Start()
    {
        // Initialise placed order
        for (int i = 0; i < placedOrder.Length; i++)
            placedOrder[i] = "";

        // Build correct order after EggStandRandomiser has run (0.5s delay)
        Invoke("BuildCorrectOrder", 0.5f);

        // Hide waiting panel at start
        if (waitingForCameraPanel != null)
            waitingForCameraPanel.SetActive(false);

        // Subscribe to ROS topics
        SubscribeToRosTopics();
    }

    // ─────────────────────────────────────────
    // ROS SUBSCRIPTION SETUP
    // ─────────────────────────────────────────

    void SubscribeToRosTopics()
    {
        ROSConnection ros = ROSConnection.GetOrCreateInstance();

        // /puzzle3/puzzle_solved — Bool
        ros.Subscribe<BoolMsg>(rosSolvedTopic, OnRosSolvedUpdate);

        // /puzzle3/status — String (optional debug display)
        ros.Subscribe<StringMsg>(rosStatusTopic, OnRosStatusUpdate);

        Debug.Log($"[CubePuzzleManager] Subscribed to {rosSolvedTopic} and {rosStatusTopic}");
    }

    // ─────────────────────────────────────────
    // ROS CALLBACKS
    // ─────────────────────────────────────────

    /// <summary>
    /// Called every time /puzzle3/puzzle_solved is published (5 Hz).
    /// Runs on the main Unity thread via ROSConnection.
    /// </summary>
    void OnRosSolvedUpdate(BoolMsg msg)
    {
        bool wasSolved = _rosSolvedConfirmed;
        _rosSolvedConfirmed = msg.data;

        // ROS just confirmed solved — check if Unity side is also ready
        if (_rosSolvedConfirmed && !wasSolved)
        {
            Debug.Log("[CubePuzzleManager] ROS camera confirmed puzzle solved.");

            if (_unityPlacementCorrect)
            {
                // Both agree — trigger full solve
                TriggerFullSolve();
            }
            else
            {
                // Camera says solved but VR placement not done yet
                // This can happen if robot placed eggs physically but
                // user hasn't completed VR interaction
                Debug.Log("[CubePuzzleManager] ROS confirmed but VR placement not complete yet.");
            }
        }

        // ROS reset (new run started from Arduino)
        if (!_rosSolvedConfirmed && wasSolved && !requireRosConfirmation)
        {
            Debug.Log("[CubePuzzleManager] ROS reset detected.");
        }

        // Hide/show waiting panel
        if (waitingForCameraPanel != null)
            waitingForCameraPanel.SetActive(_unityPlacementCorrect && !_rosSolvedConfirmed && requireRosConfirmation);
    }

    /// <summary>
    /// Called every time /puzzle3/status is published (5 Hz).
    /// Updates the optional debug text panel in VR.
    /// </summary>
    void OnRosStatusUpdate(StringMsg msg)
    {
        if (rosDebugText != null)
            rosDebugText.text = msg.data;
    }

    // ─────────────────────────────────────────
    // BUILD CORRECT ORDER FROM STAND POSITIONS
    // ─────────────────────────────────────────

    void BuildCorrectOrder()
    {
        if (eggStandRandomiser == null)
        {
            Debug.LogError("[CubePuzzleManager] EggStandRandomiser not assigned!");
            return;
        }

        List<GameObject> sortedStands = new List<GameObject>(
            eggStandRandomiser.eggStands);

        sortedStands.Sort((a, b) =>
            a.transform.position.x.CompareTo(b.transform.position.x));

        for (int i = 0; i < sortedStands.Count && i < 4; i++)
        {
            correctOrder[i] = GetEggTypeFromStand(sortedStands[i]);
            Debug.Log($"[CubePuzzleManager] Slot {i} correct type: {correctOrder[i]}");
        }

        Debug.Log($"[CubePuzzleManager] Correct order: {string.Join(", ", correctOrder)}");
    }

    string GetEggTypeFromStand(GameObject stand)
    {
        string name = stand.name.ToLower();
        if (name.Contains("water")) return "water";
        if (name.Contains("earth")) return "earth";
        if (name.Contains("space")) return "space";
        if (name.Contains("cloud")) return "cloud";
        return "unknown";
    }

    // ─────────────────────────────────────────
    // CALLED WHEN CUBE IS PLACED IN A SLOT
    // ─────────────────────────────────────────

    public void OnCubePlaced(int slotIndex, string eggType)
    {
        if (puzzleSolved) return;

        placedOrder[slotIndex] = eggType;

        Debug.Log($"[CubePuzzleManager] Placed {eggType} in slot {slotIndex}, correct is {correctOrder[slotIndex]}");

        if (correctOrder[slotIndex] == eggType)
        {
            slots[slotIndex].SetCorrect();
            Debug.Log($"[CubePuzzleManager] Slot {slotIndex} CORRECT!");
        }
        else
        {
            slots[slotIndex].SetWrong();
            Debug.Log($"[CubePuzzleManager] Slot {slotIndex} WRONG! Expected {correctOrder[slotIndex]}");
        }

        CheckAllSlots();
    }

    // ─────────────────────────────────────────
    // CHECK ALL SLOTS
    // ─────────────────────────────────────────

    void CheckAllSlots()
    {
        // All slots must have something placed
        foreach (string placed in placedOrder)
            if (placed == "") return;

        bool allCorrect = true;
        for (int i = 0; i < 4; i++)
        {
            if (placedOrder[i] != correctOrder[i])
            {
                allCorrect = false;
                break;
            }
        }

        if (allCorrect)
        {
            _unityPlacementCorrect = true;

            if (!requireRosConfirmation)
            {
                // ROS confirmation not required — solve immediately
                TriggerFullSolve();
            }
            else if (_rosSolvedConfirmed)
            {
                // ROS already confirmed — solve immediately
                TriggerFullSolve();
            }
            else
            {
                // VR placement correct but waiting for ROS camera to confirm
                Debug.Log("[CubePuzzleManager] VR placement correct — waiting for ROS camera confirmation.");

                if (resultText != null)
                    resultText.text = "✓ Placement looks correct!\nWaiting for camera...";
                if (resultLight != null)
                    resultLight.color = Color.yellow;
                if (waitingForCameraPanel != null)
                    waitingForCameraPanel.SetActive(true);
            }
        }
        else
        {
            _unityPlacementCorrect = false;

            if (resultText != null)
                resultText.text = "✗ Wrong Order!\nTry Again";
            if (resultLight != null)
                resultLight.color = Color.red;

            if (puzzleBoardManager != null)
                puzzleBoardManager.OnCubePuzzleFailed();

            Debug.Log("[CubePuzzleManager] Wrong order — try again");
            Invoke("ResetPuzzle", 2f);
        }
    }

    // ─────────────────────────────────────────
    // FULL SOLVE — both Unity + ROS agree
    // ─────────────────────────────────────────

    void TriggerFullSolve()
    {
        if (puzzleSolved) return;  // guard against double-trigger

        puzzleSolved = true;
        _unityPlacementCorrect = true;

        if (waitingForCameraPanel != null)
            waitingForCameraPanel.SetActive(false);

        if (resultText != null)
            resultText.text = "✓ Correct Order!\nPuzzle Solved!";
        if (resultLight != null)
            resultLight.color = Color.green;

        if (puzzleBoardManager != null)
            puzzleBoardManager.OnCubePuzzleSolved();

        Debug.Log("[CubePuzzleManager] PUZZLE 3 FULLY SOLVED (Unity + ROS both confirmed)!");
    }

    // ─────────────────────────────────────────
    // RESET
    // ─────────────────────────────────────────

    public void ResetPuzzle()
    {
        puzzleSolved            = false;
        _unityPlacementCorrect  = false;
        // Note: _rosSolvedConfirmed is NOT reset here —
        // it resets automatically when the Arduino resets and
        // the ROS verifier publishes false again.

        for (int i = 0; i < 4; i++)
            placedOrder[i] = "";

        foreach (CubeSlot slot in slots)
            slot.ClearSlot();

        if (resultText != null)
            resultText.text = "";
        if (resultLight != null)
            resultLight.color = Color.grey;
        if (waitingForCameraPanel != null)
            waitingForCameraPanel.SetActive(false);

        Debug.Log("[CubePuzzleManager] Cube puzzle reset.");
    }
}