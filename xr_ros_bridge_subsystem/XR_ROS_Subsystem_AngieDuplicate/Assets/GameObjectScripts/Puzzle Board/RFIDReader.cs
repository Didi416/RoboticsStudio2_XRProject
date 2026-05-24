using UnityEngine;
using System.Collections;

public class RFIDReader : MonoBehaviour
{
    [Header("References")]
    public PuzzleBoardManager puzzleBoardManager;
    public RGBStatusLight rgbStatusLight;
    public FrogGuideManager frogGuideManager;

    [Header("RFID Visual")]
    public Renderer rfidRenderer;
    public Light rfidLight;
    public Color idleColor = Color.blue;
    public Color scanColor = Color.white;

    [Header("Blink Settings")]
    public float blinkDuration = 0.3f;
    public int blinkCount = 3;

    private bool isScanning = false;

    void Start()
    {
        SetRFIDColor(idleColor);
    }

    // ─────────────────────────────────────────
    // ANY OBJECT TOUCHES RFID
    // ─────────────────────────────────────────

    void OnTriggerEnter(Collider other)
    {
        // Check if it's the card
        CardObject card = other.GetComponent<CardObject>();
        if (card == null) return;
        if (isScanning) return;

        Debug.Log($"Card tapped on RFID by {other.gameObject.name}");
        StartCoroutine(ProcessCardTap());
    }

    // ─────────────────────────────────────────
    // PROCESS CARD TAP
    // ─────────────────────────────────────────

    IEnumerator ProcessCardTap()
    {
        isScanning = true;

        // Flash white to show scan detected
        SetRFIDColor(scanColor);
        yield return new WaitForSeconds(0.3f);
        SetRFIDColor(idleColor);

        // Check if all puzzles solved
        bool allSolved = CheckAllPuzzlesSolved();

        if (allSolved)
        {
            yield return StartCoroutine(BlinkGreen());
            TriggerVictory();
        }
        else
        {
            yield return StartCoroutine(BlinkRed());
            TriggerIncomplete();
        }

        isScanning = false;
    }

    // ─────────────────────────────────────────
    // CHECK ALL PUZZLES SOLVED
    // ─────────────────────────────────────────

    bool CheckAllPuzzlesSolved()
    {
        if (puzzleBoardManager == null)
        {
            Debug.LogError("PuzzleBoardManager not assigned!");
            return false;
        }

        bool allSolved = puzzleBoardManager.IsLEDMatrixSolved() &&
                         puzzleBoardManager.IsButtonMatrixSolved() &&
                         puzzleBoardManager.IsCubePuzzleSolved();

        Debug.Log($"All puzzles solved: {allSolved}");
        Debug.Log($"LED: {puzzleBoardManager.IsLEDMatrixSolved()} " +
                  $"Button: {puzzleBoardManager.IsButtonMatrixSolved()} " +
                  $"Cube: {puzzleBoardManager.IsCubePuzzleSolved()}");

        return allSolved;
    }

    // ─────────────────────────────────────────
    // VICTORY - all puzzles complete
    // ─────────────────────────────────────────

    void TriggerVictory()
    {
        Debug.Log("GAME COMPLETE! All puzzles solved!");

        // Show frog celebration
        if (frogGuideManager != null)
            frogGuideManager.PlayVictory();

        // RGB light full green pulse
        if (rgbStatusLight != null)
            rgbStatusLight.TriggerVictory();
    }

    // ─────────────────────────────────────────
    // INCOMPLETE - puzzles not all done
    // ─────────────────────────────────────────

    void TriggerIncomplete()
    {
        Debug.Log("Puzzles not complete!");

        // Show frog incomplete message
        if (frogGuideManager != null)
            frogGuideManager.PlayIncomplete();
    }

    // ─────────────────────────────────────────
    // BLINK GREEN 3 TIMES
    // ─────────────────────────────────────────

    IEnumerator BlinkGreen()
    {
        for (int i = 0; i < blinkCount; i++)
        {
            SetRFIDColor(Color.green);
            if (rfidLight != null) rfidLight.color = Color.green;
            yield return new WaitForSeconds(blinkDuration);

            SetRFIDColor(idleColor);
            if (rfidLight != null) rfidLight.color = idleColor;
            yield return new WaitForSeconds(blinkDuration);
        }
    }

    // ─────────────────────────────────────────
    // BLINK RED 3 TIMES
    // ─────────────────────────────────────────

    IEnumerator BlinkRed()
    {
        for (int i = 0; i < blinkCount; i++)
        {
            SetRFIDColor(Color.red);
            if (rfidLight != null) rfidLight.color = Color.red;
            yield return new WaitForSeconds(blinkDuration);

            SetRFIDColor(idleColor);
            if (rfidLight != null) rfidLight.color = idleColor;
            yield return new WaitForSeconds(blinkDuration);
        }
    }

    void SetRFIDColor(Color color)
    {
        if (rfidRenderer != null)
        {
            rfidRenderer.material.color = color;
            rfidRenderer.material.SetColor("_EmissionColor", color * 2f);
            rfidRenderer.material.EnableKeyword("_EMISSION");
        }
        if (rfidLight != null)
            rfidLight.color = color;
    }
}