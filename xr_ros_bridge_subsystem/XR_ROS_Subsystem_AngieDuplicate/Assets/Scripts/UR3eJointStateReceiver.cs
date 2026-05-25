// using System;
// using UnityEngine;
// using Unity.Robotics.ROSTCPConnector;
// using RosMessageTypes.Geometry;
// using RosMessageTypes.Sensor;
// using UnityEngine.XR;
// using UnityEngine.InputSystem;
// using CommonUsages = UnityEngine.XR.CommonUsages; // Alias for distinguishing between CommonUsages in XR and InputSystem

// public class UR3eJointStateReceiver : MonoBehaviour
// {
//     // public string poseTopic = "/ik_target";
//     public string jointStateTopic = "/joint_states";
    
//     [Tooltip("ArticulationBodies in order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3")]
//     public ArticulationBody[] joints = new ArticulationBody[6];

//     [Header("Drive settings")]
//     [Tooltip("Spring stiffness — must be > 0 for xDrive.target to have any effect")]
//     public float stiffness = 10000f;

//     [Tooltip("Damping — prevents oscillation")]
//     public float damping = 200f;

//     [Header("Debug — read only at runtime")]
//     public float[] receivedDeg = new float[6];
//     public bool    receiving   = false;

//     // Incoming angles land here from the ROS callback thread.
//     // Applied to drives on the main thread in FixedUpdate.
//     readonly float[] _pending    = new float[6];
//     bool             _hasData    = false;
//     readonly object  _lock       = new object();

//     // move offset by one joint
//     readonly float[] _offset = {0,90,0,90,0,0};
//     readonly float[] _dirCorrection = {-1,-1,1,1,-1,-1};
//     ROSConnection ros;

//     public enum InputMode { SimulatedController, XRHeadset }
//     public InputMode inputMode = InputMode.SimulatedController;

//     public float simulatedSpeed = 0.3f;

//     void Start()
//     {
//         string[] jointNames = {
//             "Base",
//             "Shoulder",
//             "Elbow",
//             "Wrist2",
//             "Wrist3",
//             "HandE"
//         };

//         for (int i = 0; i < joints.Length; i++)
//         {
//             joints[i] = GameObject.Find(jointNames[i]).GetComponent<ArticulationBody>();
//             if (joints[i] == null) continue;
//             ArticulationDrive d = joints[i].xDrive;
//             d.driveType  = ArticulationDriveType.Target;
//             d.stiffness  = stiffness;
//             d.damping    = damping;
//             d.forceLimit = float.MaxValue;
//             joints[i].xDrive = d;
//         };
        
//         ros = ROSConnection.GetOrCreateInstance();
//         ros.Subscribe<JointStateMsg>(jointStateTopic, OnJointStateReceived);
//     }

//     void FixedUpdate()
//     {
//         float[] angles = null;
//         lock (_lock)
//         {
//             if (_hasData)
//             {
//                 angles   = (float[])_pending.Clone();
//                 _hasData = false;
//             }
//         }

//         receiving = angles != null;
//         if (angles == null) return;

//         for (int i = 0; i < joints.Length; i++)
//         {
//             if (joints[i] == null) continue;

//             // ArticulationDrive is a struct — copy, edit, reassign
//             ArticulationDrive d = joints[i].xDrive;
//             d.target = angles[i];
//             joints[i].xDrive = d;

//             receivedDeg[i] = angles[i];
//         }
//     }

//     // void UpdateIKTarget()
//     // {
//     //     if (ikTarget == null) return;
//     //     switch (inputMode)
//     //     {
//     //         case InputMode.SimulatedController:
//     //             UpdateSimulated();
//     //             break;
//     //         case InputMode.XRHeadset:
//     //             UpdateXR();
//     //             break;
//     //     }
//     // }

//     // void UpdateSimulated()
//     // {
//     //     // Arrow Keys for Up/DOwn/Left/Right, Comma/Period for Back/Forward, mouse for orientation
//     //     Vector3 delta = Vector3.zero;
//     //     if (Keyboard.current.periodKey.isPressed) delta += Vector3.forward;
//     //     if (Keyboard.current.commaKey.isPressed) delta += Vector3.back;
//     //     if (Keyboard.current.leftArrowKey.isPressed) delta += Vector3.left;
//     //     if (Keyboard.current.rightArrowKey.isPressed) delta += Vector3.right;
//     //     if (Keyboard.current.upArrowKey.isPressed) delta += Vector3.up;
//     //     if (Keyboard.current.downArrowKey.isPressed) delta += Vector3.down;

//     //     _simPosition += delta * simulatedSpeed * Time.deltaTime;

//     //     // Optional: hold right mouse to orbit orientation
//     //     if (Mouse.current.rightButton.isPressed)
//     //     {
//     //         var md = Mouse.current.delta.ReadValue();
//     //         _simRotation *= Quaternion.Euler(-md.y * 0.5f, md.x * 0.5f, 0f);
//     //     }

//     //     ikTarget.position = _simPosition;
//     //     ikTarget.rotation = _simRotation;
//     // }

//     // void UpdateXR()
//     // {
//     //     var inputDevice = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
//     //     if (!inputDevice.isValid) return;

//     //     if (inputDevice.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 pos))
//     //         ikTarget.position = transform.TransformPoint(pos);
//     //     if (inputDevice.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rot))
//     //         ikTarget.rotation = transform.rotation * rot;
//     // }

//     // void PublishIKTarget()
//     // {
//     //     UpdateIKTarget();
//     //     // Vector3 localPos = robotBase.InverseTransformPoint(ikTarget.position);
//     //     // Quaternion localRot = Quaternion.Inverse(robotBase.rotation) * ikTarget.rotation;
//     //     Vector3 localPos = ikTarget.position;
//     //     Quaternion localRot = ikTarget.rotation;

//     //     // Unity → ROS coordinate conversion (CRITICAL due to different coordinate systems)
//     //     Vector3 rosPos = new Vector3(
//     //         localPos.z,
//     //         -localPos.x,
//     //         localPos.y
//     //     );

//     //     Quaternion rosRot = new Quaternion(
//     //         localRot.z,
//     //         -localRot.x,
//     //         localRot.y,
//     //         -localRot.w
//     //     );

//     //     PoseMsg msg = new PoseMsg(
//     //         new PointMsg(rosPos.x, rosPos.y, rosPos.z),
//     //         new QuaternionMsg(rosRot.x, rosRot.y, rosRot.z, rosRot.w)
//     //     );

//     //     ros.Publish(poseTopic, msg);
//     // }

//     // void OnJointStateReceived(JointStateMsg msg)
//     // {
//     //     if (msg.position.Length != joints.Length)
//     //         return;

//     //     for (int i = 0; i < joints.Length; i++)
//     //     {
//     //         var drive = joints[i].xDrive;

//     //         // radians → degrees
//     //         float targetDeg = (float)(msg.position[i] * Mathf.Rad2Deg);

//     //         drive.target = targetDeg;
//     //         joints[i].xDrive = drive;
//     //     }
//     // }

//     void OnJointStateReceived(JointStateMsg msg)
//     {
//         if (msg.position == null || msg.position.Length < joints.Length)
//             return;
        
//         lock (_lock)
//         {
//             for (int i = 0; i < joints.Length; i++)
//             {
//                 int rosIndex = (i + 1) % 6;
//                 // ROS joint angles are in radians — convert to degrees for xDrive.target
//                 _pending[rosIndex] = ((float)(msg.position[i] * Mathf.Rad2Deg) + _offset[rosIndex])*_dirCorrection[rosIndex];
//             }
//             _hasData = true;

//         }
//     }
// }

using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class UR3eJointStateReceiver : MonoBehaviour
{
    public string jointStateTopic = "/joint_states";

    [Tooltip("ArticulationBodies in order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3")]
    public ArticulationBody[] joints = new ArticulationBody[6];

    [Header("Drive settings")]
    [Tooltip("Spring stiffness — must be > 0 for xDrive.target to have any effect")]
    public float stiffness = 10000f;

    [Tooltip("Damping — prevents oscillation")]
    public float damping = 200f;

    [Header("Debug — read only at runtime")]
    public float[] receivedDeg = new float[6];
    public bool    receiving   = false;

    // Incoming angles land here from the ROS callback thread.
    // Applied to drives on the main thread in FixedUpdate.
    readonly float[] _pending = new float[6];
    bool             _hasData = false;
    readonly object  _lock    = new object();

    // readonly float[] _offset          = { 0, 90, 0, 90, 0, 0 };
    readonly float[] _dirCorrection   = { -1, -1, 1, 1, -1, -1 };

    ROSConnection ros;

    // -------------------------------------------------------------------------
    // GRIPPER — added from JointStateSubscriber
    // -------------------------------------------------------------------------

    // Unity name → Transform, cached at Start for direct transform-based control.
    // We use transform rotation (not ArticulationBody drives) for the gripper
    // because mimic joints aren't natively supported by Unity's physics — the
    // same approach the JointStateSubscriber used, which was confirmed working.
    private Dictionary<string, Transform>   gripperJointMap      = new Dictionary<string, Transform>();
    private Dictionary<string, Quaternion>  gripperInitialRots   = new Dictionary<string, Quaternion>();
    private Dictionary<string, Vector3>     gripperJointAxes     = new Dictionary<string, Vector3>();

    // Gripper joint names in the Unity hierarchy
    private static readonly string[] gripperJointNames = {
        "left_outer_knuckle",   // finger_joint (primary driver)
        "left_inner_knuckle",   // mimic × -1
        "left_inner_finger",    // mimic ×  1
        "right_outer_knuckle",  // mimic × -1
        "right_inner_knuckle",  // mimic × -1
        "right_inner_finger",   // mimic ×  1
    };

    // Mimic multipliers matching the URDF mimic tags
    private static readonly float[] gripperMimicMultipliers = { 1f, -1f, 1f, -1f, -1f, 1f };

    // finger_joint angle range (radians): open = -0.558505, closed = 0.785398
    private const float FINGER_JOINT_OPEN   = -0.558505f;
    private const float FINGER_JOINT_CLOSED =  0.785398f;
    private const float RG2_MAX_WIDTH       =  0.110f;   // metres, URDF upper limit

    // Latest finger_width value received from /joint_states (metres).
    // Written from ROS callback, read on main thread in Update — a single
    // float write is atomic on all Unity-supported platforms so no lock needed.
    private float _currentFingerWidth = RG2_MAX_WIDTH;

    // -------------------------------------------------------------------------

    void Start()
    {
        string[] jointNames = {
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link"
        };

        for (int i = 0; i < joints.Length; i++)
        {
            joints[i] = GameObject.Find(jointNames[i]).GetComponent<ArticulationBody>();
            if (joints[i] == null) continue;

            ArticulationDrive d = joints[i].xDrive;
            d.driveType  = ArticulationDriveType.Target;
            d.stiffness  = stiffness;
            d.damping    = damping;
            d.forceLimit = float.MaxValue;
            joints[i].xDrive = d;

            Debug.Log("Joint " + jointNames[i] + " found and configured.");
        }

        // --- GRIPPER SETUP ---
        // Walk every ArticulationBody in the hierarchy and cache the ones
        // that belong to the gripper. We mirror exactly what JointStateSubscriber
        // did: store the initial local rotation and compute the rotation axis
        // from the anchor, then drive via transform.localRotation in Update.
        foreach (var body in GetComponentsInChildren<ArticulationBody>(true))
        {
            string n = body.gameObject.name;
            bool isGripperJoint = System.Array.IndexOf(gripperJointNames, n) >= 0;

            if (isGripperJoint && body.jointType != ArticulationJointType.FixedJoint)
            {
                gripperJointMap[n]    = body.transform;
                gripperInitialRots[n] = body.transform.localRotation;
                // anchorRotation * Vector3.right gives the revolute axis in local space,
                // matching how JointStateSubscriber extracted axes.
                gripperJointAxes[n]   = body.anchorRotation * Vector3.right;

                // Disable the ArticulationBody drive so physics doesn't fight
                // the transform writes we do in Update.
                body.enabled = false;

                Debug.Log($"[Gripper] Registered: {n} axis: {gripperJointAxes[n]}");
            }
        }
        // --- END GRIPPER SETUP ---

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>(jointStateTopic, OnJointStateReceived);
    }

    void FixedUpdate()
    {
        float[] angles = null;
        lock (_lock)
        {
            if (_hasData)
            {
                angles   = (float[])_pending.Clone();
                _hasData = false;
            }
        }

        receiving = angles != null;
        if (angles == null) return;

        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i] == null) continue;

            ArticulationDrive d = joints[i].xDrive;
            d.target     = angles[i];
            joints[i].xDrive = d;
            receivedDeg[i]   = angles[i];
        }
    }

    // --- GRIPPER UPDATE ---
    // Runs on the main thread every frame. Converts the latest finger_width
    // (metres) to a finger_joint angle (radians) and applies it plus each
    // mimic multiplier to the gripper transforms directly.
    // Using Update (not FixedUpdate) here matches JointStateSubscriber and
    // keeps gripper visuals smooth regardless of physics timestep.
    void Update()
    {
        float closedFraction = 1f - Mathf.Clamp01(_currentFingerWidth / RG2_MAX_WIDTH);
        float fingerAngle    = Mathf.Lerp(FINGER_JOINT_OPEN, FINGER_JOINT_CLOSED, closedFraction);

        for (int i = 0; i < gripperJointNames.Length; i++)
        {
            string n = gripperJointNames[i];
            if (!gripperJointMap.TryGetValue(n, out Transform joint)) continue;

            float angleDeg = fingerAngle * gripperMimicMultipliers[i] * Mathf.Rad2Deg;
            joint.localRotation = gripperInitialRots[n] * Quaternion.AngleAxis(angleDeg, gripperJointAxes[n]);
        }
    }
    // --- END GRIPPER UPDATE ---

    // ROS joint name -> index into the joints[] array
    private static readonly Dictionary<string, int> rosNameToJointIndex = new Dictionary<string, int>
    {
        { "shoulder_pan_joint",  0 },
        { "shoulder_lift_joint", 1 },
        { "elbow_joint",         2 },
        { "wrist_1_joint",       3 },
        { "wrist_2_joint",       4 },
        { "wrist_3_joint",       5 },
    };

    void OnJointStateReceived(JointStateMsg msg)
    {
        if (msg.position == null || msg.name == null) return;

        lock (_lock)
        {
            bool anyArm = false;

            for (int i = 0; i < msg.name.Length; i++)
            {
                // Handle finger_width separately — no lock needed for the float
                // write but we're already inside the lock here so it's fine
                if (msg.name[i] == "finger_width")
                {
                    _currentFingerWidth = (float)msg.position[i];
                    continue;
                }

                // Map arm joints by name, not position
                if (rosNameToJointIndex.TryGetValue(msg.name[i], out int jointIdx))
                {
                    _pending[jointIdx] = (float)(msg.position[i] * Mathf.Rad2Deg);
                    anyArm = true;
                }
            }

            _hasData = anyArm;
        }
    }
}

