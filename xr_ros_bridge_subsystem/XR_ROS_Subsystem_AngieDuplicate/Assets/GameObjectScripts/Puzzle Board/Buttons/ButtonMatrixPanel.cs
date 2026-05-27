// // using UnityEngine;
// // using UnityEngine.UI;
// // using TMPro;
// // using System.Collections;
// // using System.Collections.Generic;
// // using UnityEngine.InputSystem;
 
// // public class ButtonMatrixPanel : MonoBehaviour
// // {
// //     [Header("Button Matrix (3x3)")]
// //     public PhysicalButton[] physicalButtons;
// //     public Color buttonDefaultColor = new Color(0.2f, 0.2f, 0.2f);
// //     public Color buttonPressedColor = new Color(0f, 0.8f, 1f);
 
// //     [Header("SSD Display")]
// //     public TextMeshProUGUI ssdDisplay;
// //     public TextMeshProUGUI resultText;
 
// //     [Header("Frog Croak Reference")]
// //     public FrogCroakScript[] frogSigns;
 
// //     [Header("Verification")]
// //     public PuzzleBoardManager puzzleBoardManager;
 
// //     [Header("Mouse Testing")]
// //     public bool enableMouseTesting = true;
 
// //     private List<int> enteredCode = new List<int>();
// //     private int[] correctCode = new int[4];
// //     private bool codeConfirmed = false;
 
// //     void Start()
// //     {
// //         UpdateSSDDisplay();
// //         GetCorrectCodeFromFrogs();
 
// //         // Make sure result text starts empty
// //         if (resultText != null)
// //             resultText.text = "";
// //     }
 
// //     void GetCorrectCodeFromFrogs()
// //     {
// //         for (int i = 0; i < frogSigns.Length && i < 4; i++)
// //         {
// //             if (frogSigns[i] != null)
// //                 correctCode[i] = frogSigns[i].GetCroakCount();
// //         }
// //         Debug.Log($"Correct code: {correctCode[0]}{correctCode[1]}{correctCode[2]}{correctCode[3]}");
// //     }
 
// //     // ─────────────────────────────────────────
// //     // PRESS BUTTON - called by robot, ROS or physical
// //     // ─────────────────────────────────────────
 
// //     public void PressButton(int number)
// //     {
// //         Debug.Log($"PressButton({number}) codeConfirmed={codeConfirmed} count={enteredCode.Count}");
 
// //         if (codeConfirmed) return;
// //         if (enteredCode.Count >= 4) return;
 
// //         enteredCode.Add(number);
// //         UpdateSSDDisplay();
 
// //         // Flash the corresponding physical button
// //         FlashButton(number);
 
// //         if (enteredCode.Count == 4)
// //             VerifyCode();
// //     }
 
// //     // ─────────────────────────────────────────
// //     // FLASH BUTTON - highlight in VR
// //     // ─────────────────────────────────────────
 
// //     void FlashButton(int number)
// //     {
// //         if (physicalButtons == null) return;
// //         int index = number - 1;
// //         if (index >= 0 && index < physicalButtons.Length && physicalButtons[index] != null)
// //         {
// //             StartCoroutine(FlashButtonCoroutine(physicalButtons[index]));
// //         }
// //     }
 
// //     IEnumerator FlashButtonCoroutine(PhysicalButton button)
// //     {
// //         Renderer rend = button.GetComponent<Renderer>();
// //         if (rend == null) yield break;
 
// //         Color original = rend.material.color;
// //         rend.material.color = buttonPressedColor;
// //         yield return new WaitForSeconds(0.2f);
// //         rend.material.color = original;
// //     }
 
// //     // ─────────────────────────────────────────
// //     // SSD DISPLAY
// //     // ─────────────────────────────────────────
 
// //     void UpdateSSDDisplay()
// //     {
// //         string display = "";
// //         for (int i = 0; i < 4; i++)
// //         {
// //             if (i < enteredCode.Count)
// //                 display += enteredCode[i].ToString();
// //             else
// //                 display += "_";
// //         }
 
// //         Debug.Log($"SSD Display: {display}");
 
// //         if (ssdDisplay != null)
// //             ssdDisplay.text = display;
// //         else
// //             Debug.LogError("SSD Display not assigned in Inspector!");
// //     }
 
// //     // ─────────────────────────────────────────
// //     // CODE VERIFICATION
// //     // ─────────────────────────────────────────
 
// //     void VerifyCode()
// //     {
// //         bool correct = true;
// //         for (int i = 0; i < 4; i++)
// //         {
// //             if (i >= enteredCode.Count || enteredCode[i] != correctCode[i])
// //             {
// //                 correct = false;
// //                 break;
// //             }
// //         }
 
// //         // Show result text
// //         if (resultText != null)
// //         {
// //             resultText.text = correct ? "Correct\nCode!" : "Incorrect!\nTry Again";
// //             resultText.color = correct ? Color.green : Color.red;
// //         }
 
// //         // Show on SSD display
// //         if (ssdDisplay != null)
// //             ssdDisplay.text = correct ? "GOOD" : "XXXX";
 
// //         codeConfirmed = correct;
 
// //         if (puzzleBoardManager != null)
// //         {
// //             if (correct)
// //                 puzzleBoardManager.OnButtonMatrixSolved();
// //             else
// //                 puzzleBoardManager.OnButtonMatrixFailed();
// //         }
 
// //         Debug.Log(correct ? "Button Matrix SOLVED!" : "Wrong code!");
 
// //         if (!correct)
// //             Invoke("ResetCode", 2f);
// //     }
 
// //     // ─────────────────────────────────────────
// //     // CALLED BY ROS BRIDGE - sync display text
// //     // ─────────────────────────────────────────
 
// //     public void SetDisplayFromROS(string text)
// //     {
// //         Debug.Log($"SetDisplayFromROS: {text}");
// //         if (ssdDisplay != null)
// //             ssdDisplay.text = text;
// //     }
 
// //     // ─────────────────────────────────────────
// //     // CALLED BY ROS BRIDGE - handle result
// //     // ─────────────────────────────────────────
 
// //     public void HandleROSResult(bool correct)
// //     {
// //         Debug.Log($"HandleROSResult: {correct}");
 
// //         if (resultText != null)
// //         {
// //             resultText.text = correct ? "✓ Correct Code!" : "✗ Incorrect!\nTry Again";
// //             resultText.color = correct ? Color.green : Color.red;
// //         }
 
// //         if (ssdDisplay != null)
// //             ssdDisplay.text = correct ? "GOOD" : "XXXX";
 
// //         codeConfirmed = correct;
 
// //         if (puzzleBoardManager != null)
// //         {
// //             if (correct)
// //                 puzzleBoardManager.OnButtonMatrixSolved();
// //             else
// //                 puzzleBoardManager.OnButtonMatrixFailed();
// //         }
 
// //         if (!correct)
// //             Invoke("ResetCode", 2f);
// //     }
 
// //     // ─────────────────────────────────────────
// //     // CALLED AFTER FROG CROAKS SYNCED
// //     // ─────────────────────────────────────────
 
// //     public void RefreshCorrectCode()
// //     {
// //         GetCorrectCodeFromFrogs();
// //     }
 
// //     // ─────────────────────────────────────────
// //     // RESET
// //     // ─────────────────────────────────────────
 
// //     public void ResetCode()
// //     {
// //         enteredCode.Clear();
// //         codeConfirmed = false;
// //         UpdateSSDDisplay();
 
// //         if (resultText != null)
// //             resultText.text = "";
 
// //         Debug.Log("Button Matrix reset");
// //     }
 
// //     // ─────────────────────────────────────────
// //     // KEYBOARD TESTING
// //     // ─────────────────────────────────────────
 
// //     void Update()
// //     {
// //         if (Keyboard.current[Key.Digit1].wasPressedThisFrame) PressButton(1);
// //         if (Keyboard.current[Key.Digit2].wasPressedThisFrame) PressButton(2);
// //         if (Keyboard.current[Key.Digit3].wasPressedThisFrame) PressButton(3);
// //         if (Keyboard.current[Key.Digit4].wasPressedThisFrame) PressButton(4);
// //         if (Keyboard.current[Key.Digit5].wasPressedThisFrame) PressButton(5);
// //         if (Keyboard.current[Key.Digit6].wasPressedThisFrame) PressButton(6);
// //         if (Keyboard.current[Key.Digit7].wasPressedThisFrame) PressButton(7);
// //         if (Keyboard.current[Key.Digit8].wasPressedThisFrame) PressButton(8);
// //         if (Keyboard.current[Key.Digit9].wasPressedThisFrame) PressButton(9);
// //         if (Keyboard.current[Key.R].wasPressedThisFrame) ResetCode();
// //     }
// // }

// // --------------------------
// //NEED TO BE TESTED
// // -------------------------
// using UnityEngine;
// using UnityEngine.UI;
// using TMPro;
// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine.InputSystem;

// public class ButtonMatrixPanel : MonoBehaviour
// {
//     [Header("Button Matrix (3x3)")]
//     public PhysicalButton[] physicalButtons;
//     public Color buttonDefaultColor = new Color(0.2f, 0.2f, 0.2f);
//     public Color buttonPressedColor = new Color(0f, 0.8f, 1f);

//     [Header("SSD Display")]
//     public TextMeshProUGUI ssdDisplay;
//     public TextMeshProUGUI resultText;

//     [Header("Frog Croak Reference")]
//     public FrogCroakScript[] frogSigns;

//     [Header("Verification")]
//     public PuzzleBoardManager puzzleBoardManager;

//     [Header("Mouse Testing")]
//     public bool enableMouseTesting = true;

//     // Local testing state (keyboard/mouse only — not used for ROS path)
//     private List<int> enteredCode = new List<int>();
//     private int[] correctCode = new int[4];
//     private bool codeConfirmed = false;

//     // ADDED: separate digit list for ROS-driven button presses
//     // These update the display only — Arduino owns the actual verification
//     private List<int> rosEnteredDigits = new List<int>();

//     void Start()
//     {
//         UpdateSSDDisplay();
//         GetCorrectCodeFromFrogs();

//         if (resultText != null)
//             resultText.text = "";
//     }

//     void GetCorrectCodeFromFrogs()
//     {
//         for (int i = 0; i < frogSigns.Length && i < 4; i++)
//         {
//             if (frogSigns[i] != null)
//                 correctCode[i] = frogSigns[i].GetCroakCount();
//         }
//         Debug.Log($"Correct code loaded: {correctCode[0]}{correctCode[1]}{correctCode[2]}{correctCode[3]}");
//     }

//     // ─────────────────────────────────────────
//     // REGISTER BUTTON FROM ROS
//     // ADDED: called by ROSPuzzleBridge when Arduino sends a button press.
//     // Does two things only:
//     //   1. Flashes the corresponding VR button blue
//     //   2. Appends the digit to the SSD display
//     // Does NOT call VerifyCode() — Arduino owns that validation.
//     // Result comes back via HandleROSResult() from the state topic.
//     // ─────────────────────────────────────────

//     public void RegisterButtonFromROS(int number)
//     {
//         Debug.Log($"RegisterButtonFromROS({number})");

//         // Flash VR button blue
//         FlashButton(number);

//         // Append to ROS display (max 4 digits)
//         if (rosEnteredDigits.Count < 4)
//         {
//             rosEnteredDigits.Add(number);
//             UpdateROSDisplay();
//         }
//     }

//     // ─────────────────────────────────────────
//     // UPDATE ROS DISPLAY
//     // ADDED: shows ROS-entered digits on the SSD (e.g. "1___ → 12__ → 123_")
//     // ─────────────────────────────────────────

//     void UpdateROSDisplay()
//     {
//         string display = "";
//         for (int i = 0; i < 4; i++)
//             display += (i < rosEnteredDigits.Count) ? rosEnteredDigits[i].ToString() : "_";

//         Debug.Log($"ROS SSD Display: {display}");

//         if (ssdDisplay != null)
//             ssdDisplay.text = display;
//         else
//             Debug.LogError("SSD Display not assigned in Inspector!");
//     }

//     // ─────────────────────────────────────────
//     // FLASH BUTTON
//     // CHANGED: works by finding the Renderer on the PhysicalButton object
//     // and briefly setting it to buttonPressedColor (cyan/blue).
//     // This is the same visual flash used for local button presses.
//     // ─────────────────────────────────────────

//     void FlashButton(int number)
//     {
//         if (physicalButtons == null) return;
//         int index = number - 1;
//         if (index >= 0 && index < physicalButtons.Length && physicalButtons[index] != null)
//             StartCoroutine(FlashButtonCoroutine(physicalButtons[index]));
//         else
//             Debug.LogWarning($"FlashButton: no PhysicalButton at index {index} — assign physicalButtons in Inspector");
//     }

//     IEnumerator FlashButtonCoroutine(PhysicalButton button)
//     {
//         Renderer rend = button.GetComponent<Renderer>();
//         if (rend == null) yield break;

//         Color original = rend.material.color;
//         rend.material.color = buttonPressedColor;
//         yield return new WaitForSeconds(0.3f);
//         rend.material.color = original;
//     }

//     // ─────────────────────────────────────────
//     // PRESS BUTTON (local testing only — keyboard / mouse / physical trigger)
//     // Unchanged — still runs local VerifyCode for keyboard/mouse testing.
//     // In production with Arduino, buttons come via RegisterButtonFromROS instead.
//     // ─────────────────────────────────────────

//     public void PressButton(int number)
//     {
//         Debug.Log($"PressButton({number}) local — codeConfirmed={codeConfirmed} count={enteredCode.Count}");

//         if (codeConfirmed) return;
//         if (enteredCode.Count >= 4) return;

//         enteredCode.Add(number);
//         UpdateSSDDisplay();
//         FlashButton(number);

//         if (enteredCode.Count == 4)
//             VerifyCode();
//     }

//     // ─────────────────────────────────────────
//     // SSD DISPLAY (local testing)
//     // Unchanged — used by PressButton for keyboard/mouse testing
//     // ─────────────────────────────────────────

//     void UpdateSSDDisplay()
//     {
//         string display = "";
//         for (int i = 0; i < 4; i++)
//             display += (i < enteredCode.Count) ? enteredCode[i].ToString() : "_";

//         Debug.Log($"Local SSD Display: {display}");

//         if (ssdDisplay != null)
//             ssdDisplay.text = display;
//     }

//     // ─────────────────────────────────────────
//     // VERIFY CODE (local testing only)
//     // Unchanged — runs when keyboard/mouse enters 4 digits.
//     // Not called for ROS button presses.
//     // ─────────────────────────────────────────

//     void VerifyCode()
//     {
//         bool correct = true;
//         for (int i = 0; i < 4; i++)
//         {
//             if (i >= enteredCode.Count || enteredCode[i] != correctCode[i])
//             {
//                 correct = false;
//                 break;
//             }
//         }

//         if (resultText != null)
//         {
//             resultText.text = correct ? "Correct\nCode!" : "Incorrect!\nTry Again";
//             resultText.color = correct ? Color.green : Color.red;
//         }

//         if (ssdDisplay != null)
//             ssdDisplay.text = correct ? "GOOD" : "XXXX";

//         codeConfirmed = correct;

//         if (puzzleBoardManager != null)
//         {
//             if (correct) puzzleBoardManager.OnButtonMatrixSolved();
//             else puzzleBoardManager.OnButtonMatrixFailed();
//         }

//         Debug.Log(correct ? "Button Matrix SOLVED (local)!" : "Wrong code (local)!");

//         if (!correct)
//             Invoke("ResetCode", 2f);
//     }

//     // ─────────────────────────────────────────
//     // HANDLE ROS RESULT
//     // CHANGED: now also clears rosEnteredDigits on failure
//     // so the display resets cleanly for the next attempt.
//     // Called by ROSPuzzleBridge from the /puzzle_board/state topic.
//     // ─────────────────────────────────────────

//     public void HandleROSResult(bool correct)
//     {
//         Debug.Log($"HandleROSResult: {correct}");

//         if (resultText != null)
//         {
//             resultText.text = correct ? "Correct\nCode!" : "Incorrect!\nTry Again";
//             resultText.color = correct ? Color.green : Color.red;
//         }

//         if (ssdDisplay != null)
//             ssdDisplay.text = correct ? "GOOD" : "XXXX";

//         codeConfirmed = correct;

//         if (puzzleBoardManager != null)
//         {
//             if (correct) puzzleBoardManager.OnButtonMatrixSolved();
//             else puzzleBoardManager.OnButtonMatrixFailed();
//         }

//         if (!correct)
//             Invoke("ResetFromROS", 2f);
//     }

//     // ─────────────────────────────────────────
//     // RESET FROM ROS
//     // ADDED: clears ROS digit list and result text after a failed attempt.
//     // Called 2 seconds after HandleROSResult(false).
//     // ─────────────────────────────────────────

//     void ResetFromROS()
//     {
//         rosEnteredDigits.Clear();
//         UpdateROSDisplay();

//         if (resultText != null)
//             resultText.text = "";

//         codeConfirmed = false;
//         Debug.Log("Button Matrix reset (ROS)");
//     }

//     // ─────────────────────────────────────────
//     // SET DISPLAY FROM ROS
//     // Unchanged — directly mirrors Arduino display text
//     // ─────────────────────────────────────────

//     public void SetDisplayFromROS(string text)
//     {
//         Debug.Log($"SetDisplayFromROS: {text}");
//         if (ssdDisplay != null)
//             ssdDisplay.text = text;
//     }

//     // ─────────────────────────────────────────
//     // REFRESH CORRECT CODE
//     // Unchanged — re-reads frog croak counts after ROS sync
//     // ─────────────────────────────────────────

//     public void RefreshCorrectCode()
//     {
//         GetCorrectCodeFromFrogs();
//     }

//     // ─────────────────────────────────────────
//     // RESET (local testing)
//     // Unchanged — resets local enteredCode list
//     // ─────────────────────────────────────────

//     public void ResetCode()
//     {
//         enteredCode.Clear();
//         codeConfirmed = false;
//         UpdateSSDDisplay();

//         if (resultText != null)
//             resultText.text = "";

//         Debug.Log("Button Matrix reset (local)");
//     }

//     // ─────────────────────────────────────────
//     // KEYBOARD TESTING
//     // Unchanged — digits 1-9 trigger local PressButton, R resets
//     // ─────────────────────────────────────────

//     void Update()
//     {
//         if (Keyboard.current[Key.Digit1].wasPressedThisFrame) PressButton(1);
//         if (Keyboard.current[Key.Digit2].wasPressedThisFrame) PressButton(2);
//         if (Keyboard.current[Key.Digit3].wasPressedThisFrame) PressButton(3);
//         if (Keyboard.current[Key.Digit4].wasPressedThisFrame) PressButton(4);
//         if (Keyboard.current[Key.Digit5].wasPressedThisFrame) PressButton(5);
//         if (Keyboard.current[Key.Digit6].wasPressedThisFrame) PressButton(6);
//         if (Keyboard.current[Key.Digit7].wasPressedThisFrame) PressButton(7);
//         if (Keyboard.current[Key.Digit8].wasPressedThisFrame) PressButton(8);
//         if (Keyboard.current[Key.Digit9].wasPressedThisFrame) PressButton(9);
//         if (Keyboard.current[Key.R].wasPressedThisFrame) ResetCode();
//     }
// }
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.InputSystem;

public class ButtonMatrixPanel : MonoBehaviour
{
    // ─────────────────────────────────────────
    // INSPECTOR FIELDS
    // physicalButtons[0-8]  = digit buttons 1-9
    // physicalButtons[9]    = * Reset button    ← NEW
    // physicalButtons[10]   = 0 button
    // physicalButtons[11]   = # Enter button    ← NEW
    // Matches Arduino keypad layout exactly
    // ─────────────────────────────────────────

    [Header("Button Matrix (3x3 + Reset + Enter + 0)")]
    public PhysicalButton[] physicalButtons;    // assign 12 buttons in Inspector
    public Color buttonDefaultColor  = new Color(0.2f, 0.2f, 0.2f);
    public Color buttonPressedColor  = new Color(0f, 0.8f, 1f);    // cyan — digit press
    public Color buttonResetColor    = new Color(1f, 0.4f, 0f);    // orange — reset
    public Color buttonEnterColor    = new Color(0f, 0.9f, 0.3f);  // green — enter

    [Header("SSD Display")]
    public TextMeshProUGUI ssdDisplay;
    public TextMeshProUGUI resultText;

    [Header("Frog Croak Reference")]
    public FrogCroakScript[] frogSigns;

    [Header("Verification")]
    public PuzzleBoardManager puzzleBoardManager;

    [Header("Testing")]
    public bool enableKeyboardTesting = true;

    // ─────────────────────────────────────────
    // PRIVATE STATE
    // ─────────────────────────────────────────

    // Correct code loaded from frog croaks
    private int[] correctCode = new int[4];

    // ROS path — tracks what Arduino has entered, display only
    private List<int> rosEnteredDigits = new List<int>();
    private bool codeConfirmed = false;

    // Local testing path — keyboard/mouse only, independent of ROS
    private List<int> localEnteredDigits = new List<int>();
    private bool localCodeConfirmed = false;

    // ─────────────────────────────────────────
    // START
    // ─────────────────────────────────────────

    void Start()
    {
        GetCorrectCodeFromFrogs();
        ShowDashes();

        if (resultText != null)
            resultText.text = "";
    }

    void GetCorrectCodeFromFrogs()
    {
        for (int i = 0; i < frogSigns.Length && i < 4; i++)
        {
            if (frogSigns[i] != null)
                correctCode[i] = frogSigns[i].GetCroakCount();
        }
        Debug.Log($"Correct code loaded: {correctCode[0]}{correctCode[1]}{correctCode[2]}{correctCode[3]}");
    }

    // ─────────────────────────────────────────
    // ROS — REGISTER DIGIT
    // Called by ROSPuzzleBridge for digit buttons 0 and 1-9.
    // Flashes the VR button cyan and appends digit to display.
    // Does NOT verify — Arduino sends result via state topic.
    // Max 4 digits; overflow shows XXXX and resets (matching Arduino).
    // ─────────────────────────────────────────

    public void RegisterButtonFromROS(int number)
    {
        Debug.Log($"RegisterButtonFromROS({number})");

        if (codeConfirmed) return;

        // Overflow — more than 4 digits (matches Arduino dispError behaviour)
        if (rosEnteredDigits.Count >= 4)
        {
            Debug.Log("ROS: overflow — more than 4 digits");
            ShowError();
            Invoke("ResetFromROS", 1f);
            return;
        }

        // Flash the correct VR button
        // Digits 1-9 → physicalButtons index 0-8
        // Digit 0    → physicalButtons index 10
        int buttonIndex = (number == 0) ? 10 : number - 1;
        FlashButton(buttonIndex, buttonPressedColor);

        rosEnteredDigits.Add(number);
        UpdateROSDisplay();
    }

    // ─────────────────────────────────────────
    // ROS — REGISTER RESET (*)
    // Clears the entered digits and resets display to "----".
    // Flashes the * button orange.
    // ─────────────────────────────────────────

    public void RegisterResetFromROS()
    {
        Debug.Log("RegisterResetFromROS: clearing entry");

        FlashButton(9, buttonResetColor); // index 9 = * button

        rosEnteredDigits.Clear();
        codeConfirmed = false;
        ShowDashes();

        if (resultText != null)
            resultText.text = "";
    }

    // ─────────────────────────────────────────
    // ROS — REGISTER ENTER (#)
    // Only valid after exactly 4 digits are entered.
    // Flashes the # button green.
    // Does NOT verify locally — waits for HandleROSResult from state topic.
    // Shows "----" as a waiting state until result arrives.
    // ─────────────────────────────────────────

    public void RegisterEnterFromROS()
    {
        Debug.Log($"RegisterEnterFromROS: {rosEnteredDigits.Count} digits entered");

        FlashButton(11, buttonEnterColor); // index 11 = # button

        if (rosEnteredDigits.Count != 4)
        {
            Debug.LogWarning($"ROS Enter pressed with {rosEnteredDigits.Count} digits — ignoring");
            return;
        }

        // Show waiting state — result comes from /puzzle_board/state
        if (ssdDisplay != null)
            ssdDisplay.text = "----";

        if (resultText != null)
            resultText.text = "Checking...";

        Debug.Log($"ROS: submitted code, waiting for Arduino validation");
    }

    // ─────────────────────────────────────────
    // HANDLE ROS RESULT
    // Called by ROSPuzzleBridge from /puzzle_board/state topic.
    // Shows GOOD/XXXX and correct/incorrect text.
    // On failure, resets after 2 seconds.
    // ─────────────────────────────────────────

    public void HandleROSResult(bool correct)
    {
        Debug.Log($"HandleROSResult: {correct}");

        if (ssdDisplay != null)
            ssdDisplay.text = correct ? "GOOD" : "XXXX";

        if (resultText != null)
        {
            resultText.text  = correct ? "Correct\nCode!"    : "Incorrect!\nTry Again";
            resultText.color = correct ? Color.green : Color.red;
        }

        codeConfirmed = correct;

        if (puzzleBoardManager != null)
        {
            if (correct) puzzleBoardManager.OnButtonMatrixSolved();
            else         puzzleBoardManager.OnButtonMatrixFailed();
        }

        if (!correct)
            Invoke("ResetFromROS", 2f);
    }

    // ─────────────────────────────────────────
    // SET DISPLAY FROM ROS
    // Direct mirror of Arduino display text.
    // Called from /puzzle_board/display topic.
    // ─────────────────────────────────────────

    public void SetDisplayFromROS(string text)
    {
        Debug.Log($"SetDisplayFromROS: {text}");
        if (ssdDisplay != null)
            ssdDisplay.text = text;
    }

    // ─────────────────────────────────────────
    // REFRESH CORRECT CODE
    // Re-reads frog croak counts after ROS sync.
    // Called by ROSPuzzleBridge after SyncFrogCroaksWithCode.
    // ─────────────────────────────────────────

    public void RefreshCorrectCode()
    {
        GetCorrectCodeFromFrogs();
    }

    // ─────────────────────────────────────────
    // LOCAL TESTING — PRESS BUTTON (keyboard/mouse)
    // Independent of ROS path.
    // Digits accumulate; Enter submits locally; Reset clears.
    // ─────────────────────────────────────────

    public void PressButton(int number)
    {
        if (localCodeConfirmed) return;

        Debug.Log($"PressButton (local): {number}, count={localEnteredDigits.Count}");

        // Overflow
        if (localEnteredDigits.Count >= 4)
        {
            ShowError();
            Invoke("ResetLocal", 1f);
            return;
        }

        int buttonIndex = (number == 0) ? 10 : number - 1;
        FlashButton(buttonIndex, buttonPressedColor);

        localEnteredDigits.Add(number);
        UpdateLocalDisplay();
    }

    public void PressReset()
    {
        Debug.Log("PressReset (local)");
        FlashButton(9, buttonResetColor);
        localEnteredDigits.Clear();
        localCodeConfirmed = false;
        ShowDashes();
        if (resultText != null) resultText.text = "";
    }

    public void PressEnter()
    {
        Debug.Log($"PressEnter (local): {localEnteredDigits.Count} digits");

        FlashButton(11, buttonEnterColor);

        if (localEnteredDigits.Count != 4)
        {
            Debug.Log("Local: need exactly 4 digits before Enter");
            return;
        }

        VerifyLocalCode();
    }

    void VerifyLocalCode()
    {
        bool correct = true;
        for (int i = 0; i < 4; i++)
        {
            if (localEnteredDigits[i] != correctCode[i])
            {
                correct = false;
                break;
            }
        }

        if (ssdDisplay != null)
            ssdDisplay.text = correct ? "GOOD" : "XXXX";

        if (resultText != null)
        {
            resultText.text  = correct ? "Correct\nCode!"    : "Incorrect!\nTry Again";
            resultText.color = correct ? Color.green : Color.red;
        }

        localCodeConfirmed = correct;

        if (puzzleBoardManager != null)
        {
            if (correct) puzzleBoardManager.OnButtonMatrixSolved();
            else         puzzleBoardManager.OnButtonMatrixFailed();
        }

        Debug.Log(correct ? "Button Matrix SOLVED (local)!" : "Wrong code (local)!");

        if (!correct)
            Invoke("ResetLocal", 2f);
    }

    // ─────────────────────────────────────────
    // DISPLAY HELPERS
    // ─────────────────────────────────────────

    // Shows "----" — idle/reset state (matches Arduino dispDashes)
    void ShowDashes()
    {
        if (ssdDisplay != null)
            ssdDisplay.text = "----";
    }

    // Shows "XXXX" — error/overflow state (matches Arduino dispError)
    void ShowError()
    {
        if (ssdDisplay != null)
            ssdDisplay.text = "XXXX";
    }

    // Shows digits entered so far, pads with "-" (matches Arduino dispCode)
    // e.g. 2 digits → "12--"
    void UpdateROSDisplay()
    {
        string display = "";
        for (int i = 0; i < 4; i++)
            display += (i < rosEnteredDigits.Count) ? rosEnteredDigits[i].ToString() : "-";

        Debug.Log($"ROS display: {display}");
        if (ssdDisplay != null)
            ssdDisplay.text = display;
    }

    void UpdateLocalDisplay()
    {
        string display = "";
        for (int i = 0; i < 4; i++)
            display += (i < localEnteredDigits.Count) ? localEnteredDigits[i].ToString() : "-";

        Debug.Log($"Local display: {display}");
        if (ssdDisplay != null)
            ssdDisplay.text = display;
    }

    // ─────────────────────────────────────────
    // BUTTON FLASH
    // ─────────────────────────────────────────

    void FlashButton(int index, Color flashColor)
    {
        if (physicalButtons == null) return;
        if (index < 0 || index >= physicalButtons.Length) return;
        if (physicalButtons[index] == null)
        {
            Debug.LogWarning($"FlashButton: physicalButtons[{index}] not assigned in Inspector");
            return;
        }
        StartCoroutine(FlashCoroutine(physicalButtons[index], flashColor));
    }

    IEnumerator FlashCoroutine(PhysicalButton button, Color flashColor)
    {
        Renderer rend = button.GetComponent<Renderer>();
        if (rend == null) yield break;

        Color original = rend.material.color;
        rend.material.color = flashColor;
        yield return new WaitForSeconds(0.3f);
        rend.material.color = original;
    }

    // ─────────────────────────────────────────
    // RESET HELPERS
    // ─────────────────────────────────────────

    void ResetFromROS()
    {
        rosEnteredDigits.Clear();
        codeConfirmed = false;
        ShowDashes();
        if (resultText != null) resultText.text = "";
        Debug.Log("Button Matrix reset (ROS)");
    }

    void ResetLocal()
    {
        localEnteredDigits.Clear();
        localCodeConfirmed = false;
        ShowDashes();
        if (resultText != null) resultText.text = "";
        Debug.Log("Button Matrix reset (local)");
    }

    public void ResetCode()
    {
        ResetLocal();
    }

    // ─────────────────────────────────────────
    // KEYBOARD TESTING
    // 1-9       → digit
    // 0         → digit 0
    // Backspace → * Reset
    // Return    → # Enter
    // R         → Reset (alias)
    // ─────────────────────────────────────────

    void Update()
    {
        if (!enableKeyboardTesting) return;

        if (Keyboard.current[Key.Digit1].wasPressedThisFrame) PressButton(1);
        if (Keyboard.current[Key.Digit2].wasPressedThisFrame) PressButton(2);
        if (Keyboard.current[Key.Digit3].wasPressedThisFrame) PressButton(3);
        if (Keyboard.current[Key.Digit4].wasPressedThisFrame) PressButton(4);
        if (Keyboard.current[Key.Digit5].wasPressedThisFrame) PressButton(5);
        if (Keyboard.current[Key.Digit6].wasPressedThisFrame) PressButton(6);
        if (Keyboard.current[Key.Digit7].wasPressedThisFrame) PressButton(7);
        if (Keyboard.current[Key.Digit8].wasPressedThisFrame) PressButton(8);
        if (Keyboard.current[Key.Digit9].wasPressedThisFrame) PressButton(9);
        if (Keyboard.current[Key.Digit0].wasPressedThisFrame) PressButton(0);

        if (Keyboard.current[Key.Backspace].wasPressedThisFrame) PressReset();
        if (Keyboard.current[Key.R].wasPressedThisFrame)         PressReset();
        if (Keyboard.current[Key.Enter].wasPressedThisFrame)     PressEnter();
        if (Keyboard.current[Key.NumpadEnter].wasPressedThisFrame) PressEnter();
    }
}