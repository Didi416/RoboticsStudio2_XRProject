// XRTwistPublisher.cs
// Attach to XR Origin. Computes frame-to-frame controller delta → TwistStamped
// → /servo_node/delta_twist_cmds, which MoveIt Servo consumes directly.

using UnityEngine;
using XR = UnityEngine.XR;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
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
    public float linearScale  = 3.0f;
    [Tooltip("Multiplies angular velocity sent to Servo")]
    public float angularScale = 2.5f;

    [Header("Deadband — suppresses noise / hand tremor")]
    public float linearDeadband  = 0.0005f;   // metres/frame below which we zero out
    public float angularDeadband = 0.005f;     // radians/frame

    [Header("Simulated speed")]
    public float simLinearSpeed  = 0.3f;
    public float simAngularSpeed = 45f;        // degrees/s

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
            Debug.Log("Publishing simulated twist");
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
        if (Keyboard.current.periodKey.isPressed)     linVel += Vector3.forward;
        if (Keyboard.current.commaKey.isPressed)      linVel += Vector3.back;
        if (Keyboard.current.leftArrowKey.isPressed)  linVel += Vector3.left;
        if (Keyboard.current.rightArrowKey.isPressed) linVel += Vector3.right;
        if (Keyboard.current.upArrowKey.isPressed)    linVel += Vector3.up;
        if (Keyboard.current.downArrowKey.isPressed)  linVel += Vector3.down;
        linVel *= simLinearSpeed;

        // Angular: WASD = pitch/yaw, QE = roll
        Vector3 angVel = Vector3.zero;
        if (Keyboard.current.wKey.isPressed) angVel += Vector3.right  *  simAngularSpeed;
        if (Keyboard.current.sKey.isPressed) angVel += Vector3.right  * -simAngularSpeed;
        if (Keyboard.current.aKey.isPressed) angVel += Vector3.up     * -simAngularSpeed;
        if (Keyboard.current.dKey.isPressed) angVel += Vector3.up     *  simAngularSpeed;
        if (Keyboard.current.qKey.isPressed) angVel += Vector3.forward*  simAngularSpeed;
        if (Keyboard.current.eKey.isPressed) angVel += Vector3.forward* -simAngularSpeed;
        angVel *= Mathf.Deg2Rad; // Servo expects rad/s

        pendingLinVel_ = linVel;
        pendingAngVel_ = angVel;
    }

    // ── XR (Quest 2) mode ────────────────────────────────────────────────────

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

        PublishTwist(linVel, angVel);
    }

    // ── Coordinate conversion + publish ──────────────────────────────────────

    void PublishTwist(Vector3 linUnity, Vector3 angUnity)
    {
        // Unity (left-handed, Y-up) → ROS (right-handed, Z-up)
        // Matches your existing pos conversion: (z, -x, y)
        Vector3 linROS = new Vector3( linUnity.z, -linUnity.x,  linUnity.y);
        Vector3 angROS = new Vector3( angUnity.z, -angUnity.x,  angUnity.y);

        var msg = new TwistStampedMsg
        {
            header = new HeaderMsg
            {
                frame_id = frameId,
                stamp    = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec     = (int)(Time.realtimeSinceStartup),
                    nanosec = (uint)((Time.realtimeSinceStartup % 1f) * 1e9f)
                }
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