// PuzzleControlPanel.cs
// ─────────────────────────────────────────────────────────────────────────────
// Master controller for the entire Puzzle Control UI panel.
// Manages: connection indicator, status label, progress bar,
//          log feed, and 2D board zone highlight dots.
//
// Scene hierarchy expected (create via the setup guide in README_UNITY.md):
//
//   [Canvas] PuzzleControlCanvas
//     └─ [Panel] PuzzleControlPanel      ← attach THIS script here
//          ├─ Header
//          │    ├─ TitleText              (TMP_Text)
//          │    └─ ConnectionIndicator    (Image — coloured dot)
//          ├─ ConnectionLabel             (TMP_Text)
//          ├─ BoardVisualiser
//          │    ├─ ZoneDot_P1            (Image — puzzle 1 zone indicator)
//          │    ├─ ZoneDot_P2            (Image — puzzle 2 zone indicator)
//          │    └─ ZoneDot_P3            (Image — puzzle 3 zone indicator)
//          ├─ PuzzleButtonsGroup
//          │    ├─ Btn_Puzzle1           (Button + PuzzleButton script, GoalName="puzzle1")
//          │    ├─ Btn_Puzzle2           (Button + PuzzleButton script, GoalName="puzzle2")
//          │    └─ Btn_Puzzle3           (Button + PuzzleButton script, GoalName="puzzle3")
//          ├─ Btn_Home                   (Button + PuzzleButton script, GoalName="home")
//          ├─ StatusBar
//          │    ├─ StatusLabel           (TMP_Text)
//          │    └─ ProgressFill          (Image, ImageType=Filled, FillMethod=Horizontal)
//          ├─ PoseReadout
//          │    ├─ PoseX_Text            (TMP_Text)
//          │    ├─ PoseY_Text            (TMP_Text)
//          │    └─ PoseZ_Text            (TMP_Text)
//          └─ LogScrollView
//               └─ LogContent            (RectTransform, vertical layout group)
//                    └─ [LogEntryPrefab]  (TMP_Text — instantiated at runtime)
//
// Place this file at:
//   Assets/Scripts/UI/PuzzleControlPanel.cs
// ─────────────────────────────────────────────────────────────────────────────

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using EscapeXRtist.ROS;

namespace EscapeXRtist.UI
{
    public class PuzzleControlPanel : MonoBehaviour
    {
        // ─────────────────────────────────────────────────────────────────────
        // Inspector references
        // ─────────────────────────────────────────────────────────────────────

        [Header("Connection")]
        [Tooltip("Small coloured dot in the header")]
        public Image      ConnectionDot;
        public TMP_Text   ConnectionLabel;
        public Color      ConnectedColor    = new Color(0.00f, 0.90f, 0.63f, 1f);
        public Color      DisconnectedColor = new Color(0.35f, 0.40f, 0.50f, 1f);
        public Color      BusyColor         = new Color(1.00f, 0.42f, 0.17f, 1f);

        [Header("Status Bar")]
        public TMP_Text   StatusLabel;
        public Image      ProgressFill;    // ImageType=Filled, FillMethod=Horizontal
        public Color      StatusReadyColor  = new Color(0.00f, 0.90f, 0.63f, 1f);
        public Color      StatusBusyColor   = new Color(1.00f, 0.42f, 0.17f, 1f);
        public Color      StatusErrorColor  = new Color(1.00f, 0.24f, 0.35f, 1f);

        [Header("Board Zone Dots (optional)")]
        [Tooltip("Small Image dots on the 2D board schematic, one per puzzle zone")]
        public Image ZoneDot_P1;
        public Image ZoneDot_P2;
        public Image ZoneDot_P3;
        public Image ZoneDot_Home;

        [Header("Pose Readout")]
        public TMP_Text PoseX_Text;
        public TMP_Text PoseY_Text;
        public TMP_Text PoseZ_Text;

        [Header("Log")]
        [Tooltip("Parent RectTransform that grows as log entries are added (VerticalLayoutGroup)")]
        public RectTransform LogContent;
        [Tooltip("ScrollRect wrapping LogContent — auto-scrolls to bottom")]
        public ScrollRect    LogScrollRect;
        [Tooltip("Prefab: a single TMP_Text entry (see README_UNITY.md for setup)")]
        public TMP_Text      LogEntryPrefab;
        public int           MaxLogEntries = 60;

        // ─────────────────────────────────────────────────────────────────────
        // Pose data matching puzzle_pose_server.py
        // ─────────────────────────────────────────────────────────────────────

        private static readonly Dictionary<string, Vector3> PuzzlePoses = new()
        {
            { "puzzle1", new Vector3(0.30f, -0.12f, 0.20f) },
            { "puzzle2", new Vector3(0.30f,  0.00f, 0.20f) },
            { "puzzle3", new Vector3(0.30f,  0.12f, 0.20f) },
            { "home",    new Vector3(0.18f,  0.00f, 0.35f) },
        };

        // ─────────────────────────────────────────────────────────────────────
        // Private state
        // ─────────────────────────────────────────────────────────────────────

        private readonly Queue<TMP_Text> _logEntries = new();
        private Coroutine _progressCoroutine;

        // ─────────────────────────────────────────────────────────────────────
        // Unity lifecycle
        // ─────────────────────────────────────────────────────────────────────

        private void Start()
        {
            SetConnectionState(false);
            SetStatus("waiting for connection…", false);
            AddLog("Panel initialised.", LogColor.Info);

            // Reset zone dots
            SetAllZoneDotsInactive();
        }

        private void OnEnable()
        {
            PuzzleEvents.OnStatusChanged     += HandleStatus;
            PuzzleEvents.OnBusyChanged       += HandleBusy;
            PuzzleEvents.OnConnectionChanged += HandleConnection;
        }

        private void OnDisable()
        {
            PuzzleEvents.OnStatusChanged     -= HandleStatus;
            PuzzleEvents.OnBusyChanged       -= HandleBusy;
            PuzzleEvents.OnConnectionChanged -= HandleConnection;
        }

        // ─────────────────────────────────────────────────────────────────────
        // Event handlers
        // ─────────────────────────────────────────────────────────────────────

        private void HandleStatus(string status)
        {
            bool isError = status.StartsWith("error") || status.StartsWith("failed");
            bool isBusy  = status.StartsWith("moving");

            SetStatus(status, isError);
            AddLog($"← {status}", isError ? LogColor.Error : LogColor.Receive);

            // Update zone dots based on which puzzle was reached
            if (status.StartsWith("moving to") || status.StartsWith("reached"))
            {
                foreach (var kvp in PuzzlePoses)
                {
                    if (status.Contains(kvp.Key))
                    {
                        HighlightZone(kvp.Key);
                        UpdatePoseReadout(kvp.Key);
                        break;
                    }
                }
            }
        }

        private void HandleBusy(bool busy)
        {
            if (busy)
            {
                if (ProgressFill != null)
                {
                    if (_progressCoroutine != null) StopCoroutine(_progressCoroutine);
                    _progressCoroutine = StartCoroutine(AnimateProgressBar());
                }
                if (ConnectionDot != null) ConnectionDot.color = BusyColor;
            }
            else
            {
                if (_progressCoroutine != null)
                {
                    StopCoroutine(_progressCoroutine);
                    _progressCoroutine = null;
                }
                if (ProgressFill != null) ProgressFill.fillAmount = 0f;
                bool connected = PuzzleROSManager.Instance != null &&
                                 PuzzleROSManager.Instance.IsConnected;
                if (ConnectionDot != null)
                    ConnectionDot.color = connected ? ConnectedColor : DisconnectedColor;
            }
        }

        private void HandleConnection(bool connected)
        {
            SetConnectionState(connected);
            AddLog(connected ? "Connected to ROS." : "Disconnected from ROS.",
                   connected ? LogColor.Receive : LogColor.Warning);
        }

        // ─────────────────────────────────────────────────────────────────────
        // Public API (call from buttons or other scripts)
        // ─────────────────────────────────────────────────────────────────────

        /// <summary>
        /// Called by PuzzleButton components (or any other script) to log a
        /// sent goal and trigger the UI highlight immediately.
        /// </summary>
        public void OnGoalSent(string goalName)
        {
            AddLog($"→ goal sent: {goalName}", LogColor.Send);
            HighlightZone(goalName);
            UpdatePoseReadout(goalName);
        }

        // ─────────────────────────────────────────────────────────────────────
        // Internal UI helpers
        // ─────────────────────────────────────────────────────────────────────

        private void SetConnectionState(bool connected)
        {
            if (ConnectionDot   != null) ConnectionDot.color = connected ? ConnectedColor : DisconnectedColor;
            if (ConnectionLabel != null) ConnectionLabel.text = connected ? "Connected" : "Disconnected";
        }

        private void SetStatus(string text, bool isError)
        {
            if (StatusLabel == null) return;
            StatusLabel.text  = text;
            StatusLabel.color = isError ? StatusErrorColor
                              : text.StartsWith("moving") ? StatusBusyColor
                              : StatusReadyColor;
        }

        private IEnumerator AnimateProgressBar()
        {
            if (ProgressFill == null) yield break;
            ProgressFill.color = StatusBusyColor;
            float t = 0f;
            while (true)
            {
                // Pulsing fill: sine wave between 0.2 and 1.0
                t += Time.deltaTime * 1.4f;
                ProgressFill.fillAmount = Mathf.Lerp(0.2f, 1f, (Mathf.Sin(t) + 1f) * 0.5f);
                yield return null;
            }
        }

        // ── Zone dots ─────────────────────────────────────────────────────────

        private void SetAllZoneDotsInactive()
        {
            Color dim = new Color(0.12f, 0.14f, 0.20f, 1f);
            SetDotColor(ZoneDot_P1,   dim);
            SetDotColor(ZoneDot_P2,   dim);
            SetDotColor(ZoneDot_P3,   dim);
            SetDotColor(ZoneDot_Home, dim);
        }

        private void HighlightZone(string goalName)
        {
            SetAllZoneDotsInactive();
            Color on = ConnectedColor;
            switch (goalName)
            {
                case "puzzle1": SetDotColor(ZoneDot_P1,   on); break;
                case "puzzle2": SetDotColor(ZoneDot_P2,   on); break;
                case "puzzle3": SetDotColor(ZoneDot_P3,   on); break;
                case "home":    SetDotColor(ZoneDot_Home, on); break;
            }
        }

        private static void SetDotColor(Image dot, Color c)
        {
            if (dot != null) dot.color = c;
        }

        // ── Pose readout ──────────────────────────────────────────────────────

        private void UpdatePoseReadout(string goalName)
        {
            if (!PuzzlePoses.TryGetValue(goalName, out var pose)) return;
            if (PoseX_Text != null) PoseX_Text.text = pose.x.ToString("F3");
            if (PoseY_Text != null) PoseY_Text.text = pose.y.ToString("F3");
            if (PoseZ_Text != null) PoseZ_Text.text = pose.z.ToString("F3");
        }

        // ── Log ───────────────────────────────────────────────────────────────

        private enum LogColor { Info, Send, Receive, Warning, Error }

        private static readonly Dictionary<LogColor, Color> LogColors = new()
        {
            { LogColor.Info,    new Color(0.60f, 0.67f, 0.80f, 1f) },
            { LogColor.Send,    new Color(0.40f, 0.65f, 1.00f, 1f) },
            { LogColor.Receive, new Color(0.00f, 0.90f, 0.63f, 1f) },
            { LogColor.Warning, new Color(1.00f, 0.70f, 0.30f, 1f) },
            { LogColor.Error,   new Color(1.00f, 0.24f, 0.35f, 1f) },
        };

        private void AddLog(string message, LogColor color = LogColor.Info)
        {
            if (LogEntryPrefab == null || LogContent == null) return;

            // Timestamp
            string ts   = System.DateTime.Now.ToString("HH:mm:ss");
            var    entry = Instantiate(LogEntryPrefab, LogContent);
            entry.text  = $"<color=#{ColorUtility.ToHtmlStringRGB(LogColors[LogColor.Info])}>{ts}</color>  " +
                          $"<color=#{ColorUtility.ToHtmlStringRGB(LogColors[color])}>{message}</color>";
            entry.gameObject.SetActive(true);

            _logEntries.Enqueue(entry);
            while (_logEntries.Count > MaxLogEntries)
            {
                var old = _logEntries.Dequeue();
                Destroy(old.gameObject);
            }

            // Auto-scroll to bottom next frame
            StartCoroutine(ScrollToBottom());
        }

        private IEnumerator ScrollToBottom()
        {
            yield return new WaitForEndOfFrame();
            if (LogScrollRect != null)
                LogScrollRect.normalizedPosition = new Vector2(0f, 0f);
        }
    }
}