// using UnityEngine;
// using System.Collections;

// public class RGBStatusLight : MonoBehaviour
// {
//     [Header("Light References")]
//     public Renderer lightRenderer;
//     public Light pointLight;

//     [Header("Colors")]
//     public Color idleColor = Color.grey;
//     public Color correctColor = Color.green;
//     public Color incorrectColor = Color.red;
//     public Color partialColor = Color.yellow;

//     [Header("Puzzle References")]
//     public PuzzleBoardManager puzzleBoardManager;

//     [Header("Glow Settings")]
//     public float glowIntensity = 2f;
//     public float pulseSpeed = 1f;
//     private Coroutine pulseCoroutine;

//     private bool ledMatrixSolved = false;
//     private bool buttonMatrixSolved = false;
//     private bool cubePuzzleSolved = false;

//     void Start()
//     {
//         SetColor(idleColor);
//     }

//     // ─────────────────────────────────────────
//     // CALLED BY PuzzleBoardManager
//     // ─────────────────────────────────────────

//     public void OnLEDMatrixSolved()
//     {
//         ledMatrixSolved = true;
//         UpdateStatus();
//     }

//     public void OnLEDMatrixFailed()
//     {
//         ledMatrixSolved = false;
//         UpdateStatus();
//     }

//     public void OnButtonMatrixSolved()
//     {
//         buttonMatrixSolved = true;
//         UpdateStatus();
//     }

//     public void OnButtonMatrixFailed()
//     {
//         buttonMatrixSolved = false;
//         UpdateStatus();
//     }

//     public void OnCubePuzzleSolved()
//     {
//         cubePuzzleSolved = true;
//         UpdateStatus();
//     }

//     public void OnCubePuzzleFailed()
//     {
//         cubePuzzleSolved = false;
//         UpdateStatus();
//     }

//     // ─────────────────────────────────────────
//     // UPDATE LIGHT COLOR
//     // ─────────────────────────────────────────

//     void UpdateStatus()
//     {
//         bool anySolved = ledMatrixSolved || buttonMatrixSolved || cubePuzzleSolved;
//         bool allSolved = ledMatrixSolved && buttonMatrixSolved && cubePuzzleSolved;

//         if (allSolved)
//         {
//             SetColor(correctColor);
//             StartPulse(correctColor);
//             Debug.Log("ALL PUZZLES SOLVED!");
//         }
//         else if (anySolved)
//         {
//             SetColor(correctColor);
//             StopPulse();
//             Debug.Log("Some puzzles solved - green light");
//         }
//         else
//         {
//             SetColor(idleColor);
//             StopPulse();
//         }
//     }

//     void SetColor(Color color)
//     {
//         if (lightRenderer != null)
//         {
//             lightRenderer.material.color = color;
//             lightRenderer.material.SetColor("_EmissionColor",
//                 color * glowIntensity);
//             lightRenderer.material.EnableKeyword("_EMISSION");
//         }

//         if (pointLight != null)
//             pointLight.color = color;
//     }

//     void StartPulse(Color color)
//     {
//         if (pulseCoroutine != null)
//             StopCoroutine(pulseCoroutine);
//         pulseCoroutine = StartCoroutine(PulseLight(color));
//     }

//     void StopPulse()
//     {
//         if (pulseCoroutine != null)
//         {
//             StopCoroutine(pulseCoroutine);
//             pulseCoroutine = null;
//         }
//     }

//     IEnumerator PulseLight(Color color)
//     {
//         while (true)
//         {
//             float t = Mathf.PingPong(Time.time * pulseSpeed, 1f);
//             Color pulseColor = Color.Lerp(color * 0.3f, color, t);
//             SetColor(pulseColor);
//             yield return null;
//         }
//     }
// }
using UnityEngine;
using System.Collections;

public class RGBStatusLight : MonoBehaviour
{
    [Header("Light References")]
    public Renderer lightRenderer;
    public Light pointLight;

    [Header("Colors")]
    public Color idleColor = Color.grey;
    public Color correctColor = Color.green;
    public Color incorrectColor = Color.red;

    [Header("Glow Settings")]
    public float glowIntensity = 2f;
    public float pulseSpeed = 1f;
    public float resetDelay = 3f; // seconds before returning to idle

    private Coroutine pulseCoroutine;
    private Coroutine resetCoroutine;

    // Track puzzle states
    private bool ledMatrixSolved = false;
    private bool buttonMatrixSolved = false;
    private bool cubePuzzleSolved = false;

    void Start()
    {
        SetColor(idleColor);
    }

    // ─────────────────────────────────────────
    // CALLED BY PuzzleBoardManager
    // ─────────────────────────────────────────

    public void OnLEDMatrixSolved()
    {
        ledMatrixSolved = true;
        ShowCorrect();
    }

    public void OnLEDMatrixFailed()
    {
        ledMatrixSolved = false;
        ShowIncorrect();
    }

    public void OnButtonMatrixSolved()
    {
        buttonMatrixSolved = true;
        ShowCorrect();
    }

    public void OnButtonMatrixFailed()
    {
        buttonMatrixSolved = false;
        ShowIncorrect();
    }

    public void OnCubePuzzleSolved()
    {
        cubePuzzleSolved = true;
        ShowCorrect();
    }

    public void OnCubePuzzleFailed()
    {
        cubePuzzleSolved = false;
        ShowIncorrect();
    }



    // ─────────────────────────────────────────
    // SHOW CORRECT - green
    // ─────────────────────────────────────────

    void ShowCorrect()
    {
        StopReset();
        StopPulse();

        bool allSolved = ledMatrixSolved && buttonMatrixSolved && cubePuzzleSolved;

        if (allSolved)
        {
            // All solved - pulse green
            StartPulse(correctColor);
            Debug.Log("ALL PUZZLES SOLVED - pulsing green!");
        }
        else
        {
            // Some solved - steady green
            SetColor(correctColor);
            Debug.Log("Puzzle correct - steady green");
        }
    }

    // ─────────────────────────────────────────
    // SHOW INCORRECT - red then reset
    // ─────────────────────────────────────────

    void ShowIncorrect()
    {
        StopPulse();
        StopReset();

        SetColor(incorrectColor);
        Debug.Log("Puzzle incorrect - red light");

        // Reset back to idle/correct state after delay
        resetCoroutine = StartCoroutine(ResetAfterDelay());
    }

    IEnumerator ResetAfterDelay()
    {
        yield return new WaitForSeconds(resetDelay);

        // After delay return to appropriate state
        bool anySolved = ledMatrixSolved || buttonMatrixSolved || cubePuzzleSolved;

        if (anySolved)
            SetColor(correctColor);
        else
            SetColor(idleColor);

        Debug.Log("RGB light reset after incorrect");
    }

    // ─────────────────────────────────────────
    // HELPERS
    // ─────────────────────────────────────────

    void SetColor(Color color)
    {
        if (lightRenderer != null)
        {
            lightRenderer.material.color = color;
            lightRenderer.material.SetColor("_EmissionColor",
                color * glowIntensity);
            lightRenderer.material.EnableKeyword("_EMISSION");
        }

        if (pointLight != null)
            pointLight.color = color;
    }

    void StartPulse(Color color)
    {
        if (pulseCoroutine != null)
            StopCoroutine(pulseCoroutine);
        pulseCoroutine = StartCoroutine(PulseLight(color));
    }

    void StopPulse()
    {
        if (pulseCoroutine != null)
        {
            StopCoroutine(pulseCoroutine);
            pulseCoroutine = null;
        }
    }

    void StopReset()
    {
        if (resetCoroutine != null)
        {
            StopCoroutine(resetCoroutine);
            resetCoroutine = null;
        }
    }

    IEnumerator PulseLight(Color color)
    {
        while (true)
        {
            float t = Mathf.PingPong(Time.time * pulseSpeed, 1f);
            Color pulseColor = Color.Lerp(color * 0.3f, color, t);
            SetColor(pulseColor);
            yield return null;
        }
    }

    public void TriggerVictory()
    {
        StopPulse();
        StopReset();
        // Fast rainbow-like pulse on victory
        StartCoroutine(VictoryPulse());
    }

    IEnumerator VictoryPulse()
    {
        float elapsed = 0f;
        while (elapsed < 5f) // pulse for 5 seconds
        {
            float t = Mathf.PingPong(elapsed * 3f, 1f);
            Color victoryColor = Color.Lerp(Color.green, Color.yellow, t);
            SetColor(victoryColor);
            elapsed += Time.deltaTime;
            yield return null;
        }
        SetColor(correctColor); // settle on green
    }
}