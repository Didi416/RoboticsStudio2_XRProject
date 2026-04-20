// XRServoTeleop.cs
// ─────────────────────────────────────────────────────────────────────────────
// Attaches to the Meta Quest 2 XR Rig (or any XRController GameObject).
//
// What it does:
//   1. Reads the right-hand XR controller pose (position + rotation) each frame.
//   2. Converts a "grip" button press into an active teleoperation signal.
//   3. Publishes geometry_msgs/PoseStamped to /xr_target_pose via
//      Unity Robotics Hub ROS-TCP-Connector.
//   4. Subscribes to /unity_joint_states (sensor_msgs/JointState) and drives
//      the Articulation Body chain of the UR3e digital twin.
//
// Dependencies (install via Package Manager or git UPM):
//   • Unity Robotics Hub  – com.unity.robotics.ros-tcp-connector  ≥ 0.7
//   • XR Interaction Toolkit – com.unity.xr.interaction.toolkit ≥ 2.5
//   • Oculus XR Plugin   – com.unity.xr.oculus ≥ 4.x
//
// Setup:
//   1. Add this script to the XR Origin GameObject (or a child).
//   2. Assign robotRoot to the root ArticulationBody of the UR3e model.
//   3. Assign rightHandAnchor to the Right Controller / Right Hand Anchor.
//   4. In ROSConnection settings, set ROS IP = Linux machine IP, Port = 10000.
//   5. In Build Settings → XR Plug-in Management, enable Oculus.
// ─────────────────────────────────────────────────────────────────────────────

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

// [RequireComponent(typeof(ROSConnection))]
public class XRServoTeleop : MonoBehaviour
{
    // ──────────────────────────────────────────────────────────────────────────
    // Inspector fields
    // ──────────────────────────────────────────────────────────────────────────

    [Header("ROS Topics")]
    [Tooltip("Must match servo_command_topic source in xr_servo_node")]
    public string targetPoseTopic    = "/xr_target_pose";
    public string jointStateTopic    = "/joint_states";

    [Header("XR Rig")]
    [Tooltip("Right Controller anchor (XRController or TrackedPoseDriver host)")]
    public Transform rightHandAnchor;

    [Header("Robot Digital Twin")]
    [Tooltip("Root ArticulationBody of the UR3e prefab (base_link)")]
    public ArticulationBody robotRoot;

    [Header("Teleoperation Settings")]
    [Tooltip("Scale controller workspace → robot workspace (default 1:1)")]
    public float workspaceScale = 1.0f;

    [Tooltip("Minimum grip value to begin streaming (0-1)")]
    [Range(0f, 1f)]
    public float gripThreshold = 0.7f;

    [Tooltip("Position smoothing (0 = none, higher = smoother but laggier)")]
    [Range(0f, 0.95f)]
    public float positionSmoothing = 0.5f;

    [Tooltip("Publish rate in Hz")]
    [Range(5, 90)]
    public int publishHz = 30;

    [Header("Coordinate Frame Offset")]
    [Tooltip("Optional: world-space offset so controller origin aligns with robot base")]
    public Vector3    robotBaseOffset    = Vector3.zero;
    public Quaternion robotBaseRotOffset = Quaternion.identity;

    [Header("Visual Feedback")]
    [Tooltip("Optional indicator toggled when servo is active")]
    public GameObject activeIndicator;

    // ──────────────────────────────────────────────────────────────────────────
    // Private state
    // ──────────────────────────────────────────────────────────────────────────

    private ROSConnection      ros_;
    private InputDevice        rightController_;
    private bool               rosReady_          = false;
    private float              publishInterval_;
    private float              publishTimer_       = 0f;
    private bool               servoActive_        = false;

    // Smoothed pose
    private Vector3    smoothPos_ = Vector3.zero;
    private Quaternion smoothRot_ = Quaternion.identity;
    private bool       firstFrame_ = true;

    // UR3e joint names – must match URDF / JointState message order
    private static readonly string[] kJointNames = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

    // ArticulationBody chain, indexed to match kJointNames
    private ArticulationBody[] articulationChain_;

    // ──────────────────────────────────────────────────────────────────────────
    // Unity lifecycle
    // ──────────────────────────────────────────────────────────────────────────

    private void Awake()
    {
        ros_             = ROSConnection.GetOrCreateInstance();
        publishInterval_ = 1f / Mathf.Max(1, publishHz);

        // Register publisher
        ros_.RegisterPublisher<PoseStampedMsg>(targetPoseTopic);

        // Register subscriber for joint states → drive digital twin
        ros_.Subscribe<JointStateMsg>(jointStateTopic, OnJointState);

        rosReady_ = true;
    }

    private void Start()
    {
        // Find XR right controller input device
        var devices = new List<InputDevice>();
        InputDevices.GetDevicesWithCharacteristics(
            InputDeviceCharacteristics.Right | InputDeviceCharacteristics.Controller,
            devices);
        if (devices.Count > 0)
            rightController_ = devices[0];

        InputDevices.deviceConnected    += OnDeviceConnected;
        InputDevices.deviceDisconnected += OnDeviceDisconnected;

        // Build articulation chain
        articulationChain_ = BuildArticulationChain(robotRoot);

        if (activeIndicator != null)
            activeIndicator.SetActive(false);
    }

    private void Update()
    {
        if (!rosReady_ || rightHandAnchor == null) return;

        // ── Read grip button ───────────────────────────────────────────────
        float grip = 0f;
        bool  primaryBtn = false;
        rightController_.TryGetFeatureValue(CommonUsages.grip,          out grip);
        rightController_.TryGetFeatureValue(CommonUsages.primaryButton, out primaryBtn);

        servoActive_ = grip >= gripThreshold;

        if (activeIndicator != null)
            activeIndicator.SetActive(servoActive_);

        // ── Accumulate publish timer ───────────────────────────────────────
        publishTimer_ += Time.deltaTime;
        if (publishTimer_ < publishInterval_) return;
        publishTimer_ = 0f;

        if (!servoActive_) return;

        // ── Compute smoothed controller pose in robot-base frame ──────────
        Vector3    rawPos = rightHandAnchor.position;
        Quaternion rawRot = rightHandAnchor.rotation;

        // Apply workspace-to-robot offset
        rawPos = robotBaseRotOffset * (rawPos - robotBaseOffset);
        rawRot = robotBaseRotOffset * rawRot;

        // Scale workspace
        rawPos *= workspaceScale;

        if (firstFrame_)
        {
            smoothPos_  = rawPos;
            smoothRot_  = rawRot;
            firstFrame_ = false;
        }
        else
        {
            smoothPos_ = Vector3.Lerp(rawPos, smoothPos_, positionSmoothing);
            smoothRot_ = Quaternion.Slerp(rawRot, smoothRot_, positionSmoothing);
        }

        // ── Build ROS PoseStamped message ─────────────────────────────────
        // Unity uses left-handed Y-up; ROS uses right-handed Z-up.
        // The ROSTCPConnector GeometryExtensions handle this via .To<RUF>().
        // We use the manual conversion for clarity.

        var msg = new PoseStampedMsg
        {
            header = new HeaderMsg
            {
                stamp    = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec      = (int)   DateTimeOffset.UtcNow.ToUnixTimeSeconds(),
                    nanosec  = (uint) ((DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() % 1000) * 1_000_000)
                },
                frame_id = "base_link"
            },
            pose = new PoseMsg
            {
                position    = smoothPos_.To<FLU>(),   // Unity → ROS (FLU = Forward-Left-Up)
                orientation = smoothRot_.To<FLU>()
            }
        };

        ros_.Publish(targetPoseTopic, msg);
    }

    private void OnDestroy()
    {
        InputDevices.deviceConnected    -= OnDeviceConnected;
        InputDevices.deviceDisconnected -= OnDeviceDisconnected;
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Joint state callback → drive Articulation Bodies (digital twin)
    // ──────────────────────────────────────────────────────────────────────────

    private void OnJointState(JointStateMsg msg)
    {
        if (articulationChain_ == null) return;

        // Build a name → position lookup from the incoming message
        var posMap = new Dictionary<string, float>(msg.name.Length);
        for (int i = 0; i < msg.name.Length && i < msg.position.Length; i++)
            posMap[msg.name[i]] = (float)msg.position[i];

        // Apply each joint on the main thread via coroutine-free approach.
        // (UnityEngine APIs must run on main thread; this callback IS on main thread
        //  in ROS-TCP-Connector ≥ 0.7 when using ROSConnection.Subscribe.)
        for (int j = 0; j < kJointNames.Length; j++)
        {
            if (!posMap.TryGetValue(kJointNames[j], out float radians)) continue;
            if (j >= articulationChain_.Length || articulationChain_[j] == null) continue;

            var body = articulationChain_[j];
            if (body.jointType == ArticulationJointType.RevoluteJoint)
            {
                var drive = body.xDrive;
                drive.target = radians * Mathf.Rad2Deg;
                body.xDrive  = drive;
            }
        }
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Helpers
    // ──────────────────────────────────────────────────────────────────────────

    /// <summary>
    /// Walk the ArticulationBody hierarchy starting from root and collect
    /// bodies in the order that matches kJointNames.
    /// Assumes the UR3e prefab hierarchy: base → shoulder_pan → shoulder_lift
    ///   → elbow → wrist_1 → wrist_2 → wrist_3
    /// </summary>
    private ArticulationBody[] BuildArticulationChain(ArticulationBody root)
    {
        if (root == null) return Array.Empty<ArticulationBody>();

        var allBodies = root.GetComponentsInChildren<ArticulationBody>();
        var result    = new ArticulationBody[kJointNames.Length];

        foreach (var body in allBodies)
        {
            string n = body.gameObject.name.ToLower();
            for (int i = 0; i < kJointNames.Length; i++)
            {
                if (n.Contains(kJointNames[i].ToLower()))
                {
                    result[i] = body;
                    break;
                }
            }
        }
        return result;
    }

    private void OnDeviceConnected(InputDevice device)
    {
        if (device.characteristics.HasFlag(
                InputDeviceCharacteristics.Right |
                InputDeviceCharacteristics.Controller))
            rightController_ = device;
    }

    private void OnDeviceDisconnected(InputDevice device)
    {
        if (rightController_ == device)
            rightController_ = default;
    }

    public bool IsServoActive => servoActive_;

    /// <summary>Call from UI button to start/stop MoveIt Servo via ROS service.</summary>
    public void StartServo()  => ros_.SendServiceMessage<TriggerResponse>(
            "/servo_node/start_servo",
            new TriggerRequest(),
            resp => Debug.Log("Servo started: " + resp.message));

    public void StopServo() => ros_.SendServiceMessage<TriggerResponse>(
            "/servo_node/stop_servo",
            new TriggerRequest(),
            resp => Debug.Log("Servo stopped: " + resp.message));
}