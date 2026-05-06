// PuzzleButtonXR.cs
// ─────────────────────────────────────────────────────────────────────────────
// XR-aware version of PuzzleButton.
// Works with Unity XR Interaction Toolkit's XRRayInteractor (controller ray)
// AND with the World-Space canvas (renderMode = WorldSpace) needed for VR.
//
// Attach to the same Button GameObject as PuzzleButton.cs.
// This script adds:
//   • Hover highlight using IPointerEnterHandler / IPointerExitHandler
//     (works with XR UI Input Module from XR Interaction Toolkit)
//   • Scale pulse animation on click
//   • Optional audio feedback
//   • Haptic pulse sent to the XR controller that clicked the button
//
// Requirements:
//   • com.unity.xr.interaction.toolkit  (XR Interaction Toolkit)
//   • com.unity.inputsystem             (Input System)
//
// Place this file at:
//   Assets/Scripts/UI/PuzzleButtonXR.cs
// ─────────────────────────────────────────────────────────────────────────────

using System.Collections;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit;
using EscapeXRtist.ROS;

#if XR_INTERACTION_TOOLKIT
using UnityEngine.XR.Interaction.Toolkit.UI;
#endif

namespace EscapeXRtist.UI
{
    [RequireComponent(typeof(Button))]
    [RequireComponent(typeof(PuzzleButton))]
    public class PuzzleButtonXR : MonoBehaviour,
        IPointerEnterHandler, IPointerExitHandler, IPointerClickHandler
    {
        // ── Inspector ─────────────────────────────────────────────────────────
        [Header("Hover / Click Animation")]
        public float HoverScaleMultiplier = 1.04f;
        public float ClickScaleMultiplier = 0.96f;
        public float AnimationSpeed       = 12f;

        [Header("Haptics")]
        [Tooltip("Amplitude of haptic pulse on click (0–1)")]
        public float HapticAmplitude  = 0.3f;
        [Tooltip("Duration of haptic pulse in seconds")]
        public float HapticDuration   = 0.08f;

        [Header("Audio")]
        public AudioSource AudioSource;
        public AudioClip   HoverClip;
        public AudioClip   ClickClip;

        // ── Private ───────────────────────────────────────────────────────────
        private Vector3   _baseScale;
        private Vector3   _targetScale;
        private bool      _isHovered;
        private Button    _button;

        // ── Unity lifecycle ───────────────────────────────────────────────────

        private void Awake()
        {
            _button    = GetComponent<Button>();
            _baseScale = transform.localScale;
            _targetScale = _baseScale;
        }

        private void Update()
        {
            // Smooth scale toward target
            transform.localScale = Vector3.Lerp(
                transform.localScale, _targetScale,
                Time.unscaledDeltaTime * AnimationSpeed);
        }

        // ── Pointer events (works with XR UI Input Module) ────────────────────

        public void OnPointerEnter(PointerEventData eventData)
        {
            if (!_button.interactable) return;
            _isHovered   = true;
            _targetScale = _baseScale * HoverScaleMultiplier;
            PlayAudio(HoverClip, 0.5f);
        }

        public void OnPointerExit(PointerEventData eventData)
        {
            _isHovered   = false;
            _targetScale = _baseScale;
        }

        public void OnPointerClick(PointerEventData eventData)
        {
            if (!_button.interactable) return;

            // Scale punch
            StartCoroutine(ClickPunch());

            // Audio
            PlayAudio(ClickClip, 1.0f);

            // Haptics — find the XR controller that triggered this click
            TrySendHaptics(eventData);
        }

        // ── Helpers ───────────────────────────────────────────────────────────

        private IEnumerator ClickPunch()
        {
            _targetScale = _baseScale * ClickScaleMultiplier;
            yield return new WaitForSecondsRealtime(0.08f);
            _targetScale = _isHovered ? _baseScale * HoverScaleMultiplier : _baseScale;
        }

        private void PlayAudio(AudioClip clip, float volume)
        {
            if (AudioSource != null && clip != null)
                AudioSource.PlayOneShot(clip, volume);
        }

        private void TrySendHaptics(PointerEventData eventData)
        {
#if XR_INTERACTION_TOOLKIT
            // XR Interaction Toolkit 2.x approach
            if (eventData is TrackedDeviceGraphicRaycaster.RaycastEventData xrData)
            {
                var interactor = xrData.interactor as XRBaseControllerInteractor;
                interactor?.SendHapticImpulse(HapticAmplitude, HapticDuration);
                return;
            }
#endif
            // Fallback: send haptics to any active controller interactor
            var controllers = FindObjectsOfType<ActionBasedController>();
            foreach (var c in controllers)
            {
                if (c.TryGetComponent<UnityEngine.XR.Interaction.Toolkit.Interactors.XRBaseInputInteractor>(out var interactor))
                    interactor.SendHapticImpulse(HapticAmplitude, HapticDuration);
            }
        }
    }
}