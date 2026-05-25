using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using UnityEngine.InputSystem;
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit;
using InputDevice = UnityEngine.XR.InputDevice;
using CommonUsages = UnityEngine.XR.CommonUsages;

/// <summary>
/// Controls the OnRobot RG2 gripper by publishing a Float64 target width (metres)
/// to /finger_width_controller/command.
///
/// Hold openKey  -> gradually opens the gripper (increases finger separation)
/// Hold closeKey -> gradually closes the gripper (decreases finger separation)
///
/// Public methods OpenGripper(), CloseGripper(), SetWidth() allow other scripts
/// (e.g. XR controller input) to drive the gripper directly.
/// </summary>
public class FingerController_TEST : MonoBehaviour
{
    [Header("ROS")]
    public string commandTopic = "/finger_width_controller/commands";

    [Header("Gripper Limits (metres, from URDF)")]
    public float closedWidth = 0.000f;
    public float openWidth   = 0.110f;

    [Header("Speed")]
    [Tooltip("How fast the gripper opens/closes in metres per second.")]
    public float gripperSpeed = 0.05f;

    [Tooltip("Minimum change in width (metres) before a new message is sent. " +
             "Avoids spamming identical values every frame.")]
    public float sendThreshold = 0.001f;

    [Header("Debug — read only")]
    [SerializeField] private float _targetWidth;
    [SerializeField] private float _lastSentWidth;

    private ROSConnection _ros;

    // --- ADDED: cached right XR controller device ---
    private InputDevice _rightController;

    void Start()
    {
        _targetWidth   = openWidth;
        _lastSentWidth = openWidth;

        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<Float64MultiArrayMsg>(commandTopic);
        Debug.Log("Registered publisher for finger width controller commands");

        // Send initial position so the controller knows where we are
        SendWidth(openWidth);

        // --- ADDED: attempt to cache the right controller at startup ---
        // If the headset isn't ready yet TryGetRightController will return
        // an invalid device — XRGrip() re-attempts each frame until valid.
        TryGetRightController();
    }

    void Update()
    {
        KeyboardGrip();
        XRGrip();       // ADDED
    }

    void KeyboardGrip()
    {
        bool opening = Keyboard.current.gKey.isPressed;
        bool closing = Keyboard.current.bKey.isPressed;

        // Both pressed or neither — do nothing
        if (opening == closing) return;

        _targetWidth = Mathf.Clamp(
            _targetWidth + (opening ? 1f : -1f) * gripperSpeed * Time.deltaTime,
            closedWidth,
            openWidth
        );

        if (Mathf.Abs(_targetWidth - _lastSentWidth) < sendThreshold) return;

        SendWidth(_targetWidth);
        _lastSentWidth = _targetWidth;
    }

    // --- ADDED: XR gripper input ---
    // Right grip button (GripButton)  -> close (width decreases)
    // Right B button   (SecondaryButton) -> open (width increases)
    void XRGrip()
    {
        // Re-attempt device lookup if not yet valid (headset may connect after Start)
        if (!_rightController.isValid)
            TryGetRightController();

        if (!_rightController.isValid) return;

        // Read grip button (primary grip = GripButton)
        _rightController.TryGetFeatureValue(CommonUsages.triggerButton, out bool closing);

        // Read B button (secondary button on right controller)
        _rightController.TryGetFeatureValue(CommonUsages.secondaryButton, out bool opening);

        // Both pressed or neither — do nothing
        if (opening == closing) return;

        _targetWidth = Mathf.Clamp(
            _targetWidth + (opening ? 1f : -1f) * gripperSpeed * Time.deltaTime,
            closedWidth,
            openWidth
        );

        if (Mathf.Abs(_targetWidth - _lastSentWidth) < sendThreshold) return;

        SendWidth(_targetWidth);
        _lastSentWidth = _targetWidth;
    }

    void TryGetRightController()
    {
        var devices = new System.Collections.Generic.List<InputDevice>();
        InputDevices.GetDevicesWithCharacteristics(
            InputDeviceCharacteristics.Right | InputDeviceCharacteristics.Controller,
            devices
        );
        if (devices.Count > 0)
            _rightController = devices[0];
    }
    // --- END ADDED ---

    void SendWidth(float widthMetres)
    {
        _ros.Publish(commandTopic, new Float64MultiArrayMsg { data = new double[] { widthMetres } });
    }

    // -------------------------------------------------------------------------
    // Public API — wire these to XR controller input if needed
    // -------------------------------------------------------------------------
    public void OpenGripper()  => SendWidth(openWidth);
    public void CloseGripper() => SendWidth(closedWidth);

    public void SetWidth(float metres)
    {
        _targetWidth   = Mathf.Clamp(metres, closedWidth, openWidth);
        _lastSentWidth = _targetWidth;
        SendWidth(_targetWidth);
    }
}