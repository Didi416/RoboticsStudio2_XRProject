// PuzzleButton.cs
// ─────────────────────────────────────────────────────────────────────────────
// Attach this to each puzzle Button GameObject in your Unity UI panel.
// When clicked it calls PuzzleROSManager.SendPuzzleGoal(goalName).
//
// Inspector fields:
//   Goal Name        – "puzzle1" | "puzzle2" | "puzzle3" | "home"
//   Normal Color     – button tint when idle
//   Active Color     – button tint when this puzzle is the current target
//   Busy Color       – button tint while the robot is moving
//   Button Label     – optional Text component to update on state change
//   Icon Image       – optional Image component for puzzle icon
//
// Place this file at:
//   Assets/Scripts/UI/PuzzleButton.cs
// ─────────────────────────────────────────────────────────────────────────────

using UnityEngine;
using UnityEngine.UI;
using TMPro;
using EscapeXRtist.ROS;

namespace EscapeXRtist.UI
{
    [RequireComponent(typeof(Button))]
    public class PuzzleButton : MonoBehaviour
    {
        // ── Inspector ─────────────────────────────────────────────────────────
        [Header("ROS Goal")]
        [Tooltip("Must match a key in PUZZLE_POSES in puzzle_pose_server.py")]
        public string GoalName = "puzzle1";

        [Header("Visuals")]
        public Color NormalColor  = new Color(0.07f, 0.08f, 0.12f, 1f);
        public Color ActiveColor  = new Color(0.00f, 0.90f, 0.63f, 0.25f);
        public Color BusyColor    = new Color(1.00f, 0.42f, 0.17f, 0.25f);
        public Color DisabledColor = new Color(0.15f, 0.17f, 0.22f, 0.5f);

        [Header("Optional References")]
        [Tooltip("Text label on the button — will receive GoalName if left empty")]
        public TMP_Text ButtonLabel;

        [Tooltip("Coloured accent bar at the top of the button (Image component)")]
        public Image AccentBar;
        public Color AccentColor = new Color(0.00f, 0.90f, 0.63f, 1f);

        // ── Private state ─────────────────────────────────────────────────────
        private Button          _button;
        private Image           _bgImage;
        private bool            _isActive;

        // ── Unity lifecycle ───────────────────────────────────────────────────

        private void Awake()
        {
            _button  = GetComponent<Button>();
            _bgImage = GetComponent<Image>();

            _button.onClick.AddListener(OnClick);

            if (ButtonLabel != null && string.IsNullOrEmpty(ButtonLabel.text))
                ButtonLabel.text = GoalName;

            if (AccentBar != null)
            {
                AccentBar.color = new Color(AccentColor.r, AccentColor.g, AccentColor.b, 0f);
            }

            SetNormalState();
        }

        private void OnEnable()
        {
            PuzzleEvents.OnStatusChanged    += OnStatusChanged;
            PuzzleEvents.OnBusyChanged      += OnBusyChanged;
            PuzzleEvents.OnConnectionChanged += OnConnectionChanged;
        }

        private void OnDisable()
        {
            PuzzleEvents.OnStatusChanged    -= OnStatusChanged;
            PuzzleEvents.OnBusyChanged      -= OnBusyChanged;
            PuzzleEvents.OnConnectionChanged -= OnConnectionChanged;
        }

        // ── Click handler ─────────────────────────────────────────────────────

        private void OnClick()
        {
            var manager = PuzzleROSManager.Instance;
            if (manager == null)
            {
                Debug.LogError("[PuzzleButton] PuzzleROSManager not found in scene.");
                return;
            }
            manager.SendPuzzleGoal(GoalName);
            SetActiveState();
        }

        // ── Event handlers ────────────────────────────────────────────────────

        private void OnStatusChanged(string status)
        {
            // If another puzzle was reached, deactivate this button
            if (status.StartsWith("reached") && !status.Contains(GoalName))
                SetNormalState();
            else if (status.Contains("reached " + GoalName) ||
                     status.Contains("moving to " + GoalName))
                SetActiveState();
        }

        private void OnBusyChanged(bool busy)
        {
            _button.interactable = !busy;

            if (busy && !_isActive)
                SetBusyBackground();
            else if (!busy && !_isActive)
                SetNormalState();
        }

        private void OnConnectionChanged(bool connected)
        {
            _button.interactable = connected;
            if (!connected) SetNormalState();
        }

        // ── Visual states ─────────────────────────────────────────────────────

        private void SetNormalState()
        {
            _isActive = false;
            if (_bgImage != null) _bgImage.color = NormalColor;
            if (AccentBar != null)
                AccentBar.color = new Color(AccentColor.r, AccentColor.g, AccentColor.b, 0f);
        }

        private void SetActiveState()
        {
            _isActive = true;
            if (_bgImage != null) _bgImage.color = ActiveColor;
            if (AccentBar != null)
                AccentBar.color = AccentColor;
        }

        private void SetBusyBackground()
        {
            if (_bgImage != null) _bgImage.color = BusyColor;
        }
    }
}