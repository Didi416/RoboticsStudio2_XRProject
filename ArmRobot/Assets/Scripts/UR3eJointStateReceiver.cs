using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using UnityEngine.XR;
using UnityEngine.InputSystem;
using CommonUsages = UnityEngine.XR.CommonUsages; // Alias for distinguishing between CommonUsages in XR and InputSystem

public class UR3eJointStateReceiver : MonoBehaviour
{
    // public string poseTopic = "/ik_target";
    public string jointStateTopic = "/joint_states";

    public Transform ikTarget;                  // the VR target
    
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
    readonly float[] _pending    = new float[6];
    bool             _hasData    = false;
    readonly object  _lock       = new object();

    // move offset by one joint
    readonly float[] _offset = {0,90,0,90,0,0};
    readonly float[] _dirCorrection = {-1,-1,1,1,-1,-1};
    ROSConnection ros;

    public enum InputMode { SimulatedController, XRHeadset }
    public InputMode inputMode = InputMode.SimulatedController;

    public float simulatedSpeed = 0.3f;
    // Simulated controller state
    // Vector3 _simPosition;
    // Quaternion _simRotation;

    void Start()
    {
        string[] jointNames = {
            "Base",
            "Shoulder",
            "Elbow",
            "Wrist2",
            "Wrist3",
            "HandE"
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
        };

        // Register the ROS subscriber
        // ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(jointStateTopic, OnJointStateReceived);
        
        ros = ROSConnection.GetOrCreateInstance();
        // ros.RegisterPublisher<PoseMsg>(poseTopic);
        ros.Subscribe<JointStateMsg>(jointStateTopic, OnJointStateReceived);

        // ikTarget = GameObject.Find("HandE").transform;

        // _simPosition = ikTarget.position;
        // _simRotation = ikTarget.rotation;
    }

    void FixedUpdate()
    {
        // PublishIKTarget();
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

            // ArticulationDrive is a struct — copy, edit, reassign
            ArticulationDrive d = joints[i].xDrive;
            d.target = angles[i];
            joints[i].xDrive = d;

            receivedDeg[i] = angles[i];
        }
    }

    // void UpdateIKTarget()
    // {
    //     if (ikTarget == null) return;
    //     switch (inputMode)
    //     {
    //         case InputMode.SimulatedController:
    //             UpdateSimulated();
    //             break;
    //         case InputMode.XRHeadset:
    //             UpdateXR();
    //             break;
    //     }
    // }

    // void UpdateSimulated()
    // {
    //     // Arrow Keys for Up/DOwn/Left/Right, Comma/Period for Back/Forward, mouse for orientation
    //     Vector3 delta = Vector3.zero;
    //     if (Keyboard.current.periodKey.isPressed) delta += Vector3.forward;
    //     if (Keyboard.current.commaKey.isPressed) delta += Vector3.back;
    //     if (Keyboard.current.leftArrowKey.isPressed) delta += Vector3.left;
    //     if (Keyboard.current.rightArrowKey.isPressed) delta += Vector3.right;
    //     if (Keyboard.current.upArrowKey.isPressed) delta += Vector3.up;
    //     if (Keyboard.current.downArrowKey.isPressed) delta += Vector3.down;

    //     _simPosition += delta * simulatedSpeed * Time.deltaTime;

    //     // Optional: hold right mouse to orbit orientation
    //     if (Mouse.current.rightButton.isPressed)
    //     {
    //         var md = Mouse.current.delta.ReadValue();
    //         _simRotation *= Quaternion.Euler(-md.y * 0.5f, md.x * 0.5f, 0f);
    //     }

    //     ikTarget.position = _simPosition;
    //     ikTarget.rotation = _simRotation;
    // }

    // void UpdateXR()
    // {
    //     var inputDevice = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
    //     if (!inputDevice.isValid) return;

    //     if (inputDevice.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 pos))
    //         ikTarget.position = transform.TransformPoint(pos);
    //     if (inputDevice.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rot))
    //         ikTarget.rotation = transform.rotation * rot;
    // }

    // void PublishIKTarget()
    // {
    //     UpdateIKTarget();
    //     // Vector3 localPos = robotBase.InverseTransformPoint(ikTarget.position);
    //     // Quaternion localRot = Quaternion.Inverse(robotBase.rotation) * ikTarget.rotation;
    //     Vector3 localPos = ikTarget.position;
    //     Quaternion localRot = ikTarget.rotation;

    //     // Unity → ROS coordinate conversion (CRITICAL due to different coordinate systems)
    //     Vector3 rosPos = new Vector3(
    //         localPos.z,
    //         -localPos.x,
    //         localPos.y
    //     );

    //     Quaternion rosRot = new Quaternion(
    //         localRot.z,
    //         -localRot.x,
    //         localRot.y,
    //         -localRot.w
    //     );

    //     PoseMsg msg = new PoseMsg(
    //         new PointMsg(rosPos.x, rosPos.y, rosPos.z),
    //         new QuaternionMsg(rosRot.x, rosRot.y, rosRot.z, rosRot.w)
    //     );

    //     ros.Publish(poseTopic, msg);
    // }

    // void OnJointStateReceived(JointStateMsg msg)
    // {
    //     if (msg.position.Length != joints.Length)
    //         return;

    //     for (int i = 0; i < joints.Length; i++)
    //     {
    //         var drive = joints[i].xDrive;

    //         // radians → degrees
    //         float targetDeg = (float)(msg.position[i] * Mathf.Rad2Deg);

    //         drive.target = targetDeg;
    //         joints[i].xDrive = drive;
    //     }
    // }

    void OnJointStateReceived(JointStateMsg msg)
    {
        if (msg.position == null || msg.position.Length < joints.Length)
            return;
        
        lock (_lock)
        {
            for (int i = 0; i < joints.Length; i++)
            {
                int rosIndex = (i + 1) % 6;
                // ROS joint angles are in radians — convert to degrees for xDrive.target
                _pending[rosIndex] = ((float)(msg.position[i] * Mathf.Rad2Deg) + _offset[rosIndex])*_dirCorrection[rosIndex];
            }
            _hasData = true;

        }
    }
}
