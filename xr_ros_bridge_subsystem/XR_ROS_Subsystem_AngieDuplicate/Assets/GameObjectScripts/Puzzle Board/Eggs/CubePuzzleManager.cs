using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections.Generic;

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

    private string[] correctOrder = new string[4];
    private string[] placedOrder = new string[4];
    private bool puzzleSolved = false;

    void Start()
    {
        Invoke("BuildCorrectOrder", 0.5f);

        for (int i = 0; i < placedOrder.Length; i++)
            placedOrder[i] = "";
    }

    // ─────────────────────────────────────────
    // BUILD CORRECT ORDER FROM STAND POSITIONS
    // ─────────────────────────────────────────

    void BuildCorrectOrder()
    {
        if (eggStandRandomiser == null)
        {
            Debug.LogError("EggStandRandomiser not assigned!");
            return;
        }

        List<GameObject> sortedStands = new List<GameObject>(
            eggStandRandomiser.eggStands);

        sortedStands.Sort((a, b) =>
            a.transform.position.x.CompareTo(b.transform.position.x));

        for (int i = 0; i < sortedStands.Count && i < 4; i++)
        {
            correctOrder[i] = GetEggTypeFromStand(sortedStands[i]);
            Debug.Log($"Slot {i} correct type: {correctOrder[i]}");
        }

        Debug.Log($"Correct cube order: {string.Join(", ", correctOrder)}");
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

        Debug.Log($"Placed {eggType} in slot {slotIndex}, correct is {correctOrder[slotIndex]}");

        if (correctOrder[slotIndex] == eggType)
        {
            slots[slotIndex].SetCorrect();
            Debug.Log($"Slot {slotIndex} CORRECT!");
        }
        else
        {
            slots[slotIndex].SetWrong();
            Debug.Log($"Slot {slotIndex} WRONG! Expected {correctOrder[slotIndex]}");
        }

        CheckAllSlots();
    }

    // ─────────────────────────────────────────
    // CHECK ALL SLOTS
    // ─────────────────────────────────────────

    void CheckAllSlots()
    {
        // Check all slots have something placed
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
            puzzleSolved = true;

            if (resultText != null)
                resultText.text = "✓ Correct Order!\nPuzzle Solved!";
            if (resultLight != null)
                resultLight.color = Color.green;

            // ← Notify board manager here inside the method
            if (puzzleBoardManager != null)
                puzzleBoardManager.OnCubePuzzleSolved();

            Debug.Log("CUBE PUZZLE SOLVED!");
        }
        else
        {
            if (resultText != null)
                resultText.text = "✗ Wrong Order!\nTry Again";
            if (resultLight != null)
                resultLight.color = Color.red;

            // ← Notify board manager here inside the method
            if (puzzleBoardManager != null)
                puzzleBoardManager.OnCubePuzzleFailed();

            Debug.Log("Wrong order - try again");
            Invoke("ResetPuzzle", 2f);
        }
    }

    // ─────────────────────────────────────────
    // RESET
    // ─────────────────────────────────────────

    public void ResetPuzzle()
    {
        puzzleSolved = false;

        for (int i = 0; i < 4; i++)
            placedOrder[i] = "";

        foreach (CubeSlot slot in slots)
            slot.ClearSlot();

        if (resultText != null)
            resultText.text = "";
        if (resultLight != null)
            resultLight.color = Color.grey;

        Debug.Log("Cube puzzle reset");
    }
}