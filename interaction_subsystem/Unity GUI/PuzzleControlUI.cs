// PuzzleControlUI.cs
// ─────────────────────────────────────────────────────────────────────────────
// MonoBehaviour that drives the in-headset Puzzle Control Panel UI.
//
// Supports two Unity UI approaches — choose ONE by setting USE_UI_TOOLKIT:
//   true  → Unity UI Toolkit (UXML + USS) — recommended for Quest 2/3, flat panel
//   false → Unity Legacy uGUI (Canvas + prefab buttons) — simpler, works everywhere
//
// Setup (UI Toolkit path — recommended):
//   1. Create a UI Document GameObject in your scene (GameObject → UI → UI Document)
//   2. Assign the PuzzleControlPanel.uxml asset to its Source Asset field
//   3. Assign this script to the same or any other active GameObject
//   4. Drag the UI Document into the _uiDocument inspector slot
//
// Setup (Legacy uGUI path):
//   1. Create a Canvas (World Space, placed ~0.6 m in front of XR rig)
//   2. Attach this script to the Canvas GameObject
//   3. Assign the four Button references in the inspector
//   4. Assign the StatusText TMP_Text reference
//
// The script wires up PuzzleRosBridge automatically — just make sure
// PuzzleRosBridge.cs is also present in the scene (on any GameObject).
// ─────────────────────────────────────────────────────────────────────────────

// Set to false to use legacy uGUI Canvas+Buttons instead of UI Toolkit
#define USE_UI_TOOLKIT

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

#if USE_UI_TOOLKIT
using UnityEngine.UIElements;
#else
using UnityEngine.UI;
#endif

/// <summary>
/// Data container for each puzzle zone — keeps UI and logic in sync.
/// </summary>
[Serializable]
public class PuzzleZoneData
{
    public string goalName;      // name sent to /puzzle_goal topic
    public string displayName;   // shown on button label
    public string description;   // shown as subtitle
    public string poseLabel;     // e.g. "x=0.30  y=−0.12  z=0.20"
    public Color  accentColor;   // button highlight colour
}

public class PuzzleControlUI : MonoBehaviour
{
    // ── Inspector — Puzzle Zones ──────────────────────────────────────────────
    [Header("Puzzle Zones")]
    [SerializeField]
    private PuzzleZoneData[] puzzleZones = new PuzzleZoneData[]
    {
        new PuzzleZoneData
        {
            goalName    = "puzzle1",
            displayName = "Button Matrix",
            description = "Press the correct frog-call sequence",
            poseLabel   = "x=0.30  y=−0.12  z=0.20",
            accentColor = new Color(0.0f, 0.898f, 0.627f, 1f),   // #00e5a0
        },
        new PuzzleZoneData
        {
            goalName    = "puzzle2",
            displayName = "Slider Maze",
            description = "Guide frog token through lily pad path",
            poseLabel   = "x=0.30  y= 0.00  z=0.20",
            accentColor = new Color(0.0f, 0.4f, 1.0f, 1f),       // #0066ff
        },
        new PuzzleZoneData
        {
            goalName    = "puzzle3",
            displayName = "Egg Sorting",
            description = "Pick the correct species egg",
            poseLabel   = "x=0.30  y=+0.12  z=0.20",
            accentColor = new Color(1.0f, 0.42f, 0.17f, 1f),     // #ff6b2b
        },
        new PuzzleZoneData
        {
            goalName    = "home",
            displayName = "Home Position",
            description = "Retract arm to safe position",
            poseLabel   = "x=0.18  y= 0.00  z=0.35",
            accentColor = new Color(0.33f, 0.33f, 1.0f, 1f),
        },
    };

    // ── Inspector — UI Toolkit path ───────────────────────────────────────────
#if USE_UI_TOOLKIT
    [Header("UI Toolkit")]
    [Tooltip("The UIDocument component holding PuzzleControlPanel.uxml")]
    [SerializeField] private UIDocument _uiDocument;
#endif

    // ── Inspector — Legacy uGUI path ──────────────────────────────────────────
#if !USE_UI_TOOLKIT
    [Header("Legacy uGUI — assign in Inspector")]
    [SerializeField] private Button[]   _puzzleButtons;   // 4 buttons: P1 P2 P3 Home
    [SerializeField] private TMP_Text   _statusText;
    [SerializeField] private TMP_Text   _busyIndicator;
    [SerializeField] private Image      _progressBar;
#endif

    // ── Runtime state ─────────────────────────────────────────────────────────
    private bool             _isBusy         = false;
    private string           _activeGoal     = "";
    private List<string>     _logLines       = new List<string>();
    private const int        MaxLogLines     = 40;

    // UI Toolkit element cache
#if USE_UI_TOOLKIT
    private VisualElement    _root;
    private Label            _statusLabel;
    private Label            _logLabel;
    private VisualElement    _progressFill;
    private List<Button>     _uiButtons      = new List<Button>();
#endif

    // ── Unity lifecycle ───────────────────────────────────────────────────────

    private void OnEnable()
    {
        // Subscribe to bridge events once bridge is available
        StartCoroutine(WaitForBridgeAndSubscribe());
    }

    private void OnDisable()
    {
        if (PuzzleRosBridge.Instance != null)
        {
            PuzzleRosBridge.Instance.OnStatusReceived -= HandleStatus;
            PuzzleRosBridge.Instance.OnBusyChanged    -= HandleBusy;
        }
    }

    private IEnumerator WaitForBridgeAndSubscribe()
    {
        // Wait up to 5 s for the bridge to initialise
        float waited = 0f;
        while (PuzzleRosBridge.Instance == null && waited < 5f)
        {
            waited += Time.deltaTime;
            yield return null;
        }

        if (PuzzleRosBridge.Instance != null)
        {
            PuzzleRosBridge.Instance.OnStatusReceived += HandleStatus;
            PuzzleRosBridge.Instance.OnBusyChanged    += HandleBusy;
            AppendLog("Connected to ROS bridge.");
        }
        else
        {
            AppendLog("WARNING: PuzzleRosBridge not found — running in demo mode.");
        }
    }

    private void Start()
    {
#if USE_UI_TOOLKIT
        BuildUIToolkitPanel();
#else
        BuildLegacyUGUI();
#endif
        AppendLog("Puzzle Control Panel ready.");
        UpdateStatus("ready", false);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // ── UI TOOLKIT IMPLEMENTATION ────────────────────────────────────────────
    // ─────────────────────────────────────────────────────────────────────────

#if USE_UI_TOOLKIT

    private void BuildUIToolkitPanel()
    {
        if (_uiDocument == null)
        {
            Debug.LogError("[PuzzleControlUI] UIDocument not assigned in Inspector.");
            return;
        }

        _root = _uiDocument.rootVisualElement;

        // Grab cached references by name (must match names in PuzzleControlPanel.uxml)
        _statusLabel  = _root.Q<Label>("StatusLabel");
        _logLabel     = _root.Q<Label>("LogLabel");
        _progressFill = _root.Q<VisualElement>("ProgressFill");

        // Wire puzzle buttons
        foreach (var zone in puzzleZones)
        {
            // Button name in UXML is "Btn_puzzle1", "Btn_puzzle2", etc.
            var btn = _root.Q<Button>($"Btn_{zone.goalName}");
            if (btn == null)
            {
                Debug.LogWarning($"[PuzzleControlUI] Button 'Btn_{zone.goalName}' not found in UXML.");
                continue;
            }
            _uiButtons.Add(btn);

            // Capture for lambda
            string goalCapture = zone.goalName;
            btn.clicked += () => OnPuzzleButtonClicked(goalCapture);
        }
    }

    private void SetButtonsInteractable(bool interactable)
    {
        foreach (var btn in _uiButtons)
            btn.SetEnabled(interactable);
    }

    private void SetProgressBusy(bool busy)
    {
        if (_progressFill == null) return;
        if (busy)
            _progressFill.AddToClassList("busy");
        else
            _progressFill.RemoveFromClassList("busy");
    }

    private void SetStatusLabelText(string text, bool busy, bool error)
    {
        if (_statusLabel == null) return;
        _statusLabel.text = text;
        _statusLabel.RemoveFromClassList("status-busy");
        _statusLabel.RemoveFromClassList("status-error");
        _statusLabel.RemoveFromClassList("status-ready");
        _statusLabel.AddToClassList(error ? "status-error" : busy ? "status-busy" : "status-ready");
    }

    private void SetLogText(string text)
    {
        if (_logLabel != null)
            _logLabel.text = text;
    }

#endif

    // ─────────────────────────────────────────────────────────────────────────
    // ── LEGACY uGUI IMPLEMENTATION ───────────────────────────────────────────
    // ─────────────────────────────────────────────────────────────────────────

#if !USE_UI_TOOLKIT

    private void BuildLegacyUGUI()
    {
        // Wire each button to its matching puzzle zone by index
        for (int i = 0; i < _puzzleButtons.Length && i < puzzleZones.Length; i++)
        {
            int idx = i;   // capture for lambda
            _puzzleButtons[i].onClick.AddListener(() => OnPuzzleButtonClicked(puzzleZones[idx].goalName));

            // Set the label text on the button's child TMP_Text if present
            var label = _puzzleButtons[i].GetComponentInChildren<TMP_Text>();
            if (label != null)
                label.text = puzzleZones[i].displayName;
        }
    }

    private void SetButtonsInteractable(bool interactable)
    {
        foreach (var btn in _puzzleButtons)
            if (btn != null) btn.interactable = interactable;
    }

    private void SetProgressBusy(bool busy)
    {
        if (_progressBar == null) return;
        _progressBar.fillAmount = busy ? 1f : 0f;
        _progressBar.color      = busy
            ? new Color(1f, 0.42f, 0.17f, 1f)   // orange
            : new Color(0f, 0.90f, 0.63f, 1f);  // green
    }

    private void SetStatusLabelText(string text, bool busy, bool error)
    {
        if (_statusText == null) return;
        _statusText.text  = text;
        _statusText.color = error
            ? new Color(1f, 0.24f, 0.35f, 1f)   // red
            : busy
                ? new Color(1f, 0.42f, 0.17f, 1f)  // orange
                : new Color(0f, 0.90f, 0.63f, 1f); // green
    }

    private void SetLogText(string text)
    {
        if (_busyIndicator != null)
            _busyIndicator.text = text;
    }

#endif

    // ─────────────────────────────────────────────────────────────────────────
    // ── SHARED LOGIC ─────────────────────────────────────────────────────────
    // ─────────────────────────────────────────────────────────────────────────

    private void OnPuzzleButtonClicked(string goalName)
    {
        if (_isBusy)
        {
            AppendLog("Robot busy — ignoring tap.");
            return;
        }

        AppendLog($"→ Sending goal: {goalName}");

        if (PuzzleRosBridge.Instance != null)
        {
            PuzzleRosBridge.Instance.SendGoal(goalName);
        }
        else
        {
            // Demo mode — simulate a 2-second move
            AppendLog("(demo mode — no ROS connection)");
            StartCoroutine(DemoMove(goalName));
        }
    }

    private IEnumerator DemoMove(string goalName)
    {
        HandleBusy(true);
        HandleStatus($"moving to {goalName}");
        yield return new WaitForSeconds(2.0f);
        HandleStatus($"reached {goalName}");
        HandleBusy(false);
    }

    // ── ROS event handlers (called from PuzzleRosBridge on Unity main thread) ─

    private void HandleStatus(string statusText)
    {
        AppendLog($"← {statusText}");
        bool busy  = statusText.StartsWith("moving");
        bool error = statusText.StartsWith("error") || statusText.StartsWith("failed");
        UpdateStatus(statusText, busy, error);
    }

    private void HandleBusy(bool busy)
    {
        _isBusy = busy;
        SetButtonsInteractable(!busy);
        SetProgressBusy(busy);
    }

    private void UpdateStatus(string text, bool busy = false, bool error = false)
    {
        SetStatusLabelText(text, busy, error);
    }

    // ── Log ───────────────────────────────────────────────────────────────────

    private void AppendLog(string line)
    {
        string ts = DateTime.Now.ToString("HH:mm:ss");
        _logLines.Add($"{ts}  {line}");
        if (_logLines.Count > MaxLogLines)
            _logLines.RemoveAt(0);

        SetLogText(string.Join("\n", _logLines));
        Debug.Log($"[PuzzleUI] {line}");
    }
}