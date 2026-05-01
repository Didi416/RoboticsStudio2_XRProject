// XRTwistPublisher.cs
// Attach to XR Origin. Computes frame-to-frame controller delta → TwistStamped
// → /servo_node/delta_twist_cmds, which MoveIt Servo consumes directly.

using UnityEngine;
using XR = UnityEngine.XR;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Trajectory;
using CommonUsages = UnityEngine.XR.CommonUsages;

public class XRTwistPublisher : MonoBehaviour
{
    [Header("ROS")]
    public string twistTopic = "/servo_node/delta_twist_cmds";
    public string frameId    = "base_link";

    // [Header("Input mode")]
    public enum InputMode { SimulatedController, XRHeadset }
    public InputMode inputMode = InputMode.SimulatedController;

    [Header("Activation (XR mode)")]
    [Tooltip("Grip threshold to enable streaming")]
    [Range(0f, 1f)]
    public float gripThreshold = 0.7f;

    [Header("Scaling")]
    [Tooltip("Multiplies linear velocity sent to Servo (tune to taste)")]
    public float linearScale  = 0.5f;
    [Tooltip("Multiplies angular velocity sent to Servo")]
    public float angularScale = 2.5f;

    [Header("Deadband — suppresses noise / hand tremor")]
    public float linearDeadband  = 0.0005f;   // metres/frame below which we zero out
    public float angularDeadband = 0.005f;     // radians/frame

    [Header("Simulated speed")]
    public float simLinearSpeed  = 0.3f;
    public float simAngularSpeed = 45f;        // degrees/s

    [Header("Gripper")]
    public string gripperOpenTopic   = "/finger_width_controller/commands";
    public float  gripperOpenWidth   = 0.100f;   // metres — fully open (from your SRDF)
    public float  gripperClosedWidth = 0.0f;     // metres — fully closed
    public float  gripperMoveTime    = 1.0f;     // seconds to open/close
    private bool  gripperOpen_       = true;     // track current state for toggle
    private bool  xButtonPrev_       = false;    // edge detection for XR button

    // ── Private ──────────────────────────────────────────────────────────────
    private ROSConnection ros_;
    private XR.InputDevice   rightDevice_;

    // Previous frame values for delta computation
    private Vector3    prevPos_ = Vector3.zero;
    private Quaternion prevRot_ = Quaternion.identity;
    private bool       firstFrame_ = true;
    private Vector3 pendingLinVel_ = Vector3.zero;
    private Vector3 pendingAngVel_ = Vector3.zero;

    void Start()
    {
        ros_ = ROSConnection.GetOrCreateInstance();
        ros_.RegisterPublisher<TwistStampedMsg>(twistTopic);

        // Cache right hand device
        var devices = new System.Collections.Generic.List<XR.InputDevice>();
        XR.InputDevices.GetDevicesWithCharacteristics(
            XR.InputDeviceCharacteristics.Right | XR.InputDeviceCharacteristics.Controller,
            devices);
        if (devices.Count > 0) rightDevice_ = devices[0];

        XR.InputDevices.deviceConnected    += OnDeviceConnected;
        XR.InputDevices.deviceDisconnected += OnDeviceDisconnected;
    }

    void OnDestroy()
    {
        XR.InputDevices.deviceConnected    -= OnDeviceConnected;
        XR.InputDevices.deviceDisconnected -= OnDeviceDisconnected;
    }

    void Update()
    {
        if (inputMode == InputMode.SimulatedController){
            SampleKeyboard();
            Debug.Log("Sampling simulated twist");
        }
        if (inputMode == InputMode.XRHeadset){
            CheckGripperButton();
        }
    }

    void FixedUpdate()
    {
        if (inputMode == InputMode.SimulatedController){
            PublishTwist(pendingLinVel_, pendingAngVel_);
        }
        else{
            PublishXR();
            Debug.Log("Publishing XR twist");
        }
    }

    // ── Simulated (keyboard) mode ─────────────────────────────────────────────

    void SampleKeyboard()
    {
        // Linear: arrow keys + comma/period
        Vector3 linVel = Vector3.zero;
        if (!Keyboard.current.leftShiftKey.isPressed && Keyboard.current.leftArrowKey.isPressed)     linVel += Vector3.forward;
        if (!Keyboard.current.leftShiftKey.isPressed && Keyboard.current.rightArrowKey.isPressed)      linVel += Vector3.back;
        if (!Keyboard.current.leftShiftKey.isPressed && Keyboard.current.commaKey.isPressed)  linVel += Vector3.left;
        if (!Keyboard.current.leftShiftKey.isPressed && Keyboard.current.periodKey.isPressed) linVel += Vector3.right;
        if (!Keyboard.current.leftShiftKey.isPressed && Keyboard.current.upArrowKey.isPressed)    linVel += Vector3.up;
        if (!Keyboard.current.leftShiftKey.isPressed && Keyboard.current.downArrowKey.isPressed)  linVel += Vector3.down;
        linVel *= simLinearSpeed;

        // Angular: rpy
        Vector3 angVel = Vector3.zero;
        if (Keyboard.current.leftShiftKey.isPressed && Keyboard.current.upArrowKey.isPressed) angVel += Vector3.right  *  simAngularSpeed;
        if (Keyboard.current.leftShiftKey.isPressed && Keyboard.current.downArrowKey.isPressed) angVel += Vector3.right  * -simAngularSpeed;
        if (Keyboard.current.leftShiftKey.isPressed && Keyboard.current.leftArrowKey.isPressed) angVel += Vector3.up     * -simAngularSpeed;
        if (Keyboard.current.leftShiftKey.isPressed && Keyboard.current.rightArrowKey.isPressed) angVel += Vector3.up     *  simAngularSpeed;
        if (Keyboard.current.leftShiftKey.isPressed && Keyboard.current.periodKey.isPressed) angVel += Vector3.forward*  simAngularSpeed;
        if (Keyboard.current.leftShiftKey.isPressed && Keyboard.current.commaKey.isPressed) angVel += Vector3.forward* -simAngularSpeed;
        angVel *= Mathf.Deg2Rad; // Servo expects rad/s

        // if (!Keyboard.current.leftShiftKey.isPressed && Keyboard.current.gKey.isPressed) angVel += Vector3.right  *  simAngularSpeed;
        // if (!Keyboard.current.leftShiftKey.isPressed && Keyboard.current.rKey.isPressed) angVel += Vector3.right  * -simAngularSpeed;

        if (Keyboard.current.gKey.wasPressedThisFrame){
            ToggleGripper();
        }

        pendingLinVel_ = linVel;
        pendingAngVel_ = angVel;
    }

    // ── XR (Quest 2) mode ────────────────────────────────────────────────────

void CheckGripperButton()
{
    // Using left controller primary button (X on Quest 2 left controller)
    // If you only have right controller, use secondaryButton (B) instead
    // and make sure leftDevice_ is cached similarly to rightDevice_
    rightDevice_.TryGetFeatureValue(CommonUsages.secondaryButton, out bool bButton);

    // Edge detection — only fire once per press not every frame held
    bool justPressed = bButton && !xButtonPrev_;
    xButtonPrev_ = bButton;

    if (justPressed)
        ToggleGripper();
}

void ToggleGripper()
{
    gripperOpen_ = !gripperOpen_;
    float targetWidth = gripperOpen_ ? gripperOpenWidth : gripperClosedWidth;
    SendGripperTrajectory(targetWidth);
    Debug.Log($"[Gripper] {(gripperOpen_ ? "Opening" : "Closing")} to {targetWidth}m");
}

void SendGripperTrajectory(float width)
{
    // finger_width is a single joint that controls the OnRobot RG2 gap width
    // Values from your SRDF: open=0.100, closed=0.0
    var point = new JointTrajectoryPointMsg
    {
        positions      = new double[] { width },
        velocities     = new double[] { 0.0 },
        accelerations  = new double[] { 0.0 },
        time_from_start = new RosMessageTypes.BuiltinInterfaces.DurationMsg
        {
            sec     = (int)gripperMoveTime,
            nanosec = 0
        }
    };

    var traj = new JointTrajectoryMsg
    {
        header      = new HeaderMsg { frame_id = "" },
        joint_names = new string[] { "finger_width" },
        points      = new JointTrajectoryPointMsg[] { point }
    };

    ros_.Publish(gripperOpenTopic, traj);
}

    void PublishXR()
    {
        if (!rightDevice_.isValid) return;

        // Gate on grip button
        rightDevice_.TryGetFeatureValue(CommonUsages.grip, out float grip);
        if (grip < gripThreshold)
        {
            // Send zero twist so Servo knows to stop
            PublishTwist(Vector3.zero, Vector3.zero);
            firstFrame_ = true;   // reset delta on next grab
            // Debug.Log("Grip lower than threshold, not publishing");
            return;
        }

        rightDevice_.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 pos);
        rightDevice_.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rot);

        if (firstFrame_)
        {
            prevPos_    = pos;
            prevRot_    = rot;
            firstFrame_ = false;
            return; // skip first frame — delta would be meaningless
        }

        float dt = Time.fixedDeltaTime;

        // ── Linear velocity from position delta ───────────────────────────
        Vector3 deltaPos = pos - prevPos_;

        // Apply deadband per axis
        deltaPos.x = Mathf.Abs(deltaPos.x) < linearDeadband ? 0f : deltaPos.x;
        deltaPos.y = Mathf.Abs(deltaPos.y) < linearDeadband ? 0f : deltaPos.y;
        deltaPos.z = Mathf.Abs(deltaPos.z) < linearDeadband ? 0f : deltaPos.z;

        Vector3 linVel = (deltaPos / dt) * linearScale;

        // ── Angular velocity from rotation delta ──────────────────────────
        Quaternion deltaRot = rot * Quaternion.Inverse(prevRot_);
        deltaRot.ToAngleAxis(out float angleDeg, out Vector3 axis);

        // Wrap to [-180, 180] to avoid the long-way-round spin
        if (angleDeg > 180f) angleDeg -= 360f;

        float angleRad = angleDeg * Mathf.Deg2Rad;
        Vector3 angVel = (Mathf.Abs(angleRad) < angularDeadband)
            ? Vector3.zero
            : (axis * angleRad / dt) * angularScale;

        prevPos_ = pos;
        prevRot_ = rot;
        // Debug.Log("Publishing XR Twist)");

        PublishTwist(linVel, angVel);
    }

    // ── Coordinate conversion + publish ──────────────────────────────────────

    RosMessageTypes.BuiltinInterfaces.TimeMsg GetUnixTime()
    {
        double unixTime = (System.DateTime.UtcNow - new System.DateTime(1970, 1, 1, 0, 0, 0, System.DateTimeKind.Utc)).TotalSeconds;

        return new RosMessageTypes.BuiltinInterfaces.TimeMsg
        {
            sec     = (int)unixTime,
            nanosec = (uint)((unixTime % 1.0f) * 1e9f)
        };
    }
    void PublishTwist(Vector3 linUnity, Vector3 angUnity)
    {
        // Unity (left-handed, Y-up) → ROS (right-handed, Z-up)
        // Matches your existing pos conversion: (z, -x, y)
        Vector3 linROS = new Vector3(linUnity.z, -linUnity.x,  linUnity.y);
        Vector3 angROS = new Vector3(angUnity.z, -angUnity.x, angUnity.y);
        if (inputMode == InputMode.XRHeadset){
            angROS = new Vector3(-angUnity.x, angUnity.z, -angUnity.y);
            linROS = new Vector3(-linUnity.x, -linUnity.z,  linUnity.y);
        }
        
        var msg = new TwistStampedMsg
        {
            header = new HeaderMsg
            {
                frame_id = frameId,
                stamp    = GetUnixTime()
            },
            twist = new TwistMsg
            {
                linear  = new Vector3Msg(linROS.x, linROS.y, linROS.z),
                angular = new Vector3Msg(angROS.x, angROS.y, angROS.z)
            }
        };

        ros_.Publish(twistTopic, msg);
    }

    // ── Device lifecycle ──────────────────────────────────────────────────────

    void OnDeviceConnected(XR.InputDevice d)
    {
        if (d.characteristics.HasFlag(
            XR.InputDeviceCharacteristics.Right | XR.InputDeviceCharacteristics.Controller))
            rightDevice_ = d;
    }

    void OnDeviceDisconnected(XR.InputDevice d)
    {
        if (rightDevice_ == d) rightDevice_ = default;
    }
}

