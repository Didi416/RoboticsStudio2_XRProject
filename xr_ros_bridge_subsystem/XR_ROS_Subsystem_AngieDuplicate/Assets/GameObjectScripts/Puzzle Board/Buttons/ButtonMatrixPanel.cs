// // using UnityEngine;
// // using UnityEngine.UI;
// // using TMPro;
// // using System.Collections;
// // using System.Collections.Generic;

// // public class ButtonMatrixPanel : MonoBehaviour
// // {
// //     // [Header("Button Matrix (3x3)")]
// //     // public Button[] matrixButtons;  // 9 buttons drag in order 1-9
// //     [Header("Button Matrix (3x3)")]
// //     public PhysicalButton[] physicalButtons; // drag your 3D button objects here
// //     public Color buttonDefaultColor = new Color(0.2f, 0.2f, 0.2f);
// //     public Color buttonPressedColor = new Color(0f, 0.8f, 1f);

// //     [Header("SSD Display")]
// //     public TextMeshProUGUI ssdDisplay;   // shows 4 digit code
// //     public TextMeshProUGUI resultText;   // shows tick or cross

// //     [Header("Frog Croak Reference")]
// //     public FrogCroakScript[] frogSigns;  // drag in your 4 frog sign objects

// //     [Header("Verification")]
// //     public PuzzleBoardManager puzzleBoardManager;

// //     [Header("Mouse Testing")]
// //     public bool enableMouseTesting = true;

// //     // State
// //     private List<int> enteredCode = new List<int>();
// //     private int[] correctCode = new int[4];
// //     private bool codeConfirmed = false;

// //     void Start()
// //     {
// //         SetupButtons();
// //         UpdateSSDDisplay();
// //         GetCorrectCodeFromFrogs();
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

// //     // void SetupButtons()
// //     // {
// //     //     for (int i = 0; i < matrixButtons.Length; i++)
// //     //     {
// //     //         if (matrixButtons[i] == null) continue;
// //     //         int captureNum = i + 1;
// //     //         matrixButtons[i].onClick.AddListener(
// //     //             () => OnButtonPressed(captureNum));
// //     //         matrixButtons[i].GetComponent<Image>().color = buttonDefaultColor;
// //     //     }
// //     // }

// //     // ─────────────────────────────────────────
// //     // ROBOT OR PHYSICAL BUTTON CALLS THIS
// //     // ─────────────────────────────────────────

// //     public void PressButton(int number)
// //     {
// //         if (codeConfirmed) return;
// //         if (enteredCode.Count >= 4) return;

// //         Debug.Log($"Button {number} pressed");
// //         enteredCode.Add(number);

// //         // Flash UI button if assigned
// //         if (number >= 1 && number <= matrixButtons.Length
// //             && matrixButtons[number - 1] != null)
// //         {
// //             StartCoroutine(FlashButton(
// //                 matrixButtons[number - 1].GetComponent<Image>()));
// //         }

// //         UpdateSSDDisplay();

// //         if (enteredCode.Count == 4)
// //             VerifyCode();
// //     }

// //     // ─────────────────────────────────────────
// //     // MOUSE TESTING
// //     // ─────────────────────────────────────────

// //     void OnButtonPressed(int number)
// //     {
// //         if (!enableMouseTesting) return;
// //         PressButton(number);
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

// //         if (ssdDisplay != null)
// //             ssdDisplay.text = display;
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

// //         if (resultText != null)
// //         {
// //             resultText.text = correct ? "✓" : "✗";
// //             resultText.color = correct ? Color.green : Color.red;
// //         }

// //         codeConfirmed = correct;

// //         // Notify board manager
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

// //     IEnumerator FlashButton(Image img)
// //     {
// //         if (img == null) yield break;
// //         img.color = buttonPressedColor;
// //         yield return new WaitForSeconds(0.3f);
// //         img.color = buttonDefaultColor;
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
// // }

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

//     private List<int> enteredCode = new List<int>();
//     private int[] correctCode = new int[4];
//     private bool codeConfirmed = false;

//     void Start()
//     {
//         UpdateSSDDisplay();
//         GetCorrectCodeFromFrogs();
//     }

//     void GetCorrectCodeFromFrogs()
//     {
//         for (int i = 0; i < frogSigns.Length && i < 4; i++)
//         {
//             if (frogSigns[i] != null)
//                 correctCode[i] = frogSigns[i].GetCroakCount();
//         }
//         Debug.Log($"Correct code: {correctCode[0]}{correctCode[1]}{correctCode[2]}{correctCode[3]}");
//     }

//     // ─────────────────────────────────────────
//     // ROBOT OR PHYSICAL BUTTON CALLS THIS
//     // ─────────────────────────────────────────

//     public void PressButton(int number)
//     {
//         if (codeConfirmed) return;
//         if (enteredCode.Count >= 4) return;

//         Debug.Log($"Button {number} pressed");
//         enteredCode.Add(number);
//         UpdateSSDDisplay();

//         if (enteredCode.Count == 4)
//             VerifyCode();
//     }

//     // ─────────────────────────────────────────
//     // SSD DISPLAY
//     // ─────────────────────────────────────────

//     void UpdateSSDDisplay()
//     {
//         string display = "";
//         for (int i = 0; i < 4; i++)
//         {
//             if (i < enteredCode.Count)
//                 display += enteredCode[i].ToString();
//             else
//                 display += "_";
//         }

//         if (ssdDisplay != null)
//             ssdDisplay.text = display;
//     }

//     // ─────────────────────────────────────────
//     // CODE VERIFICATION
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
//             resultText.text = correct ? "Correct Code" : "Incorrect.\nTry Again";
//             resultText.color = correct ? Color.green : Color.red;
//         }

//         codeConfirmed = correct;

//         if (puzzleBoardManager != null)
//         {
//             if (correct)
//                 puzzleBoardManager.OnButtonMatrixSolved();
//             else
//                 puzzleBoardManager.OnButtonMatrixFailed();
//         }

//         Debug.Log(correct ? "Button Matrix SOLVED!" : "Wrong code!");

//         if (!correct)
//             Invoke("ResetCode", 2f);
//     }

//     // ─────────────────────────────────────────
//     // RESET
//     // ─────────────────────────────────────────

//     public void ResetCode()
//     {
//         enteredCode.Clear();
//         codeConfirmed = false;
//         UpdateSSDDisplay();

//         if (resultText != null)
//             resultText.text = "";

//         Debug.Log("Button Matrix reset");
//     }

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
//     }

//     // Called by ROS bridge to sync display
//     public void SetDisplayFromROS(string text)
//     {
//         if (ssdDisplay != null)
//             ssdDisplay.text = text;
//     }

//     // Called after frog croaks are synced
//     public void RefreshCorrectCode()
//     {
//         GetCorrectCodeFromFrogs();
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
    [Header("Button Matrix (3x3)")]
    public PhysicalButton[] physicalButtons;
    public Color buttonDefaultColor = new Color(0.2f, 0.2f, 0.2f);
    public Color buttonPressedColor = new Color(0f, 0.8f, 1f);
 
    [Header("SSD Display")]
    public TextMeshProUGUI ssdDisplay;
    public TextMeshProUGUI resultText;
 
    [Header("Frog Croak Reference")]
    public FrogCroakScript[] frogSigns;
 
    [Header("Verification")]
    public PuzzleBoardManager puzzleBoardManager;
 
    [Header("Mouse Testing")]
    public bool enableMouseTesting = true;
 
    private List<int> enteredCode = new List<int>();
    private int[] correctCode = new int[4];
    private bool codeConfirmed = false;
 
    void Start()
    {
        UpdateSSDDisplay();
        GetCorrectCodeFromFrogs();
 
        // Make sure result text starts empty
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
        Debug.Log($"Correct code: {correctCode[0]}{correctCode[1]}{correctCode[2]}{correctCode[3]}");
    }
 
    // ─────────────────────────────────────────
    // PRESS BUTTON - called by robot, ROS or physical
    // ─────────────────────────────────────────
 
    public void PressButton(int number)
    {
        Debug.Log($"PressButton({number}) codeConfirmed={codeConfirmed} count={enteredCode.Count}");
 
        if (codeConfirmed) return;
        if (enteredCode.Count >= 4) return;
 
        enteredCode.Add(number);
        UpdateSSDDisplay();
 
        // Flash the corresponding physical button
        FlashButton(number);
 
        if (enteredCode.Count == 4)
            VerifyCode();
    }
 
    // ─────────────────────────────────────────
    // FLASH BUTTON - highlight in VR
    // ─────────────────────────────────────────
 
    void FlashButton(int number)
    {
        if (physicalButtons == null) return;
        int index = number - 1;
        if (index >= 0 && index < physicalButtons.Length && physicalButtons[index] != null)
        {
            StartCoroutine(FlashButtonCoroutine(physicalButtons[index]));
        }
    }
 
    IEnumerator FlashButtonCoroutine(PhysicalButton button)
    {
        Renderer rend = button.GetComponent<Renderer>();
        if (rend == null) yield break;
 
        Color original = rend.material.color;
        rend.material.color = buttonPressedColor;
        yield return new WaitForSeconds(0.2f);
        rend.material.color = original;
    }
 
    // ─────────────────────────────────────────
    // SSD DISPLAY
    // ─────────────────────────────────────────
 
    void UpdateSSDDisplay()
    {
        string display = "";
        for (int i = 0; i < 4; i++)
        {
            if (i < enteredCode.Count)
                display += enteredCode[i].ToString();
            else
                display += "_";
        }
 
        Debug.Log($"SSD Display: {display}");
 
        if (ssdDisplay != null)
            ssdDisplay.text = display;
        else
            Debug.LogError("SSD Display not assigned in Inspector!");
    }
 
    // ─────────────────────────────────────────
    // CODE VERIFICATION
    // ─────────────────────────────────────────
 
    void VerifyCode()
    {
        bool correct = true;
        for (int i = 0; i < 4; i++)
        {
            if (i >= enteredCode.Count || enteredCode[i] != correctCode[i])
            {
                correct = false;
                break;
            }
        }
 
        // Show result text
        if (resultText != null)
        {
            resultText.text = correct ? "✓ Correct Code!" : "✗ Incorrect!\nTry Again";
            resultText.color = correct ? Color.green : Color.red;
        }
 
        // Show on SSD display
        if (ssdDisplay != null)
            ssdDisplay.text = correct ? "GOOD" : "XXXX";
 
        codeConfirmed = correct;
 
        if (puzzleBoardManager != null)
        {
            if (correct)
                puzzleBoardManager.OnButtonMatrixSolved();
            else
                puzzleBoardManager.OnButtonMatrixFailed();
        }
 
        Debug.Log(correct ? "Button Matrix SOLVED!" : "Wrong code!");
 
        if (!correct)
            Invoke("ResetCode", 2f);
    }
 
    // ─────────────────────────────────────────
    // CALLED BY ROS BRIDGE - sync display text
    // ─────────────────────────────────────────
 
    public void SetDisplayFromROS(string text)
    {
        Debug.Log($"SetDisplayFromROS: {text}");
        if (ssdDisplay != null)
            ssdDisplay.text = text;
    }
 
    // ─────────────────────────────────────────
    // CALLED BY ROS BRIDGE - handle result
    // ─────────────────────────────────────────
 
    public void HandleROSResult(bool correct)
    {
        Debug.Log($"HandleROSResult: {correct}");
 
        if (resultText != null)
        {
            resultText.text = correct ? "✓ Correct Code!" : "✗ Incorrect!\nTry Again";
            resultText.color = correct ? Color.green : Color.red;
        }
 
        if (ssdDisplay != null)
            ssdDisplay.text = correct ? "GOOD" : "XXXX";
 
        codeConfirmed = correct;
 
        if (puzzleBoardManager != null)
        {
            if (correct)
                puzzleBoardManager.OnButtonMatrixSolved();
            else
                puzzleBoardManager.OnButtonMatrixFailed();
        }
 
        if (!correct)
            Invoke("ResetCode", 2f);
    }
 
    // ─────────────────────────────────────────
    // CALLED AFTER FROG CROAKS SYNCED
    // ─────────────────────────────────────────
 
    public void RefreshCorrectCode()
    {
        GetCorrectCodeFromFrogs();
    }
 
    // ─────────────────────────────────────────
    // RESET
    // ─────────────────────────────────────────
 
    public void ResetCode()
    {
        enteredCode.Clear();
        codeConfirmed = false;
        UpdateSSDDisplay();
 
        if (resultText != null)
            resultText.text = "";
 
        Debug.Log("Button Matrix reset");
    }
 
    // ─────────────────────────────────────────
    // KEYBOARD TESTING
    // ─────────────────────────────────────────
 
    void Update()
    {
        if (Keyboard.current[Key.Digit1].wasPressedThisFrame) PressButton(1);
        if (Keyboard.current[Key.Digit2].wasPressedThisFrame) PressButton(2);
        if (Keyboard.current[Key.Digit3].wasPressedThisFrame) PressButton(3);
        if (Keyboard.current[Key.Digit4].wasPressedThisFrame) PressButton(4);
        if (Keyboard.current[Key.Digit5].wasPressedThisFrame) PressButton(5);
        if (Keyboard.current[Key.Digit6].wasPressedThisFrame) PressButton(6);
        if (Keyboard.current[Key.Digit7].wasPressedThisFrame) PressButton(7);
        if (Keyboard.current[Key.Digit8].wasPressedThisFrame) PressButton(8);
        if (Keyboard.current[Key.Digit9].wasPressedThisFrame) PressButton(9);
        if (Keyboard.current[Key.R].wasPressedThisFrame) ResetCode();
    }
}