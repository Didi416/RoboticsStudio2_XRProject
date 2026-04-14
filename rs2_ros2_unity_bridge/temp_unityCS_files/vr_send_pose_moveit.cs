using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using UnityEngine.XR;
using UnityEngine.InputSystem;
using CommonUsages = UnityEngine.XR.CommonUsages; // Alias for distinguishing between CommonUsages in XR and InputSystem

public class UR3eIKPublisher : MonoBehaviour
{
    public string poseTopic = "/ik_target";
    public string jointStateTopic = "/ik_solution";

    public Transform ikTarget;                  // the VR target
    public ArticulationBody[] joints;           // UR3e joints (base → wrist3)

    ROSConnection ros;

    public enum InputMode { SimulatedController, XRHeadset }
    public InputMode inputMode = InputMode.SimulatedController;

    public float simulatedSpeed = 0.3f;
    // Simulated controller state
    Vector3 _simPosition;
    Quaternion _simRotation;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterPublisher<PoseMsg>(poseTopic);
        ros.Subscribe<JointStateMsg>(jointStateTopic, OnJointStateReceived);
    }

    void Update()
    {
        PublishIKTarget();
    }

    void UpdateIKTarget()
    {
        if (ikTarget == null) return;

        switch (inputMode)
        {
            case InputMode.SimulatedController:
                UpdateSimulated();
                break;
            case InputMode.XRHeadset:
                UpdateXR();
                break;
        }
    }

    void UpdateSimulated()
    {
        // Arrow Keys for Up/DOwn/Left/Right, Comma/Period for Back/Forward, mouse for orientation
        Vector3 delta = Vector3.zero;
        if (Keyboard.current.commaKey.isPressed) delta += Vector3.forward;
        if (Keyboard.current.periodKey.isPressed) delta += Vector3.back;
        if (Keyboard.current.leftArrowKey.isPressed) delta += Vector3.left;
        if (Keyboard.current.rightArrowKey.isPressed) delta += Vector3.right;
        if (Keyboard.current.upArrowKey.isPressed) delta += Vector3.up;
        if (Keyboard.current.downArrowKey.isPressed) delta += Vector3.down;

        _simPosition += delta * simulatedSpeed * Time.deltaTime;

        // Optional: hold right mouse to orbit orientation
        if (Mouse.current.rightButton.isPressed)
        {
            var md = Mouse.current.delta.ReadValue();
            _simRotation *= Quaternion.Euler(-md.y * 0.5f, md.x * 0.5f, 0f);
        }

        ikTarget.position = _simPosition;
        ikTarget.rotation = _simRotation;
    }

    void UpdateXR()
    {
        var inputDevice = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        if (!inputDevice.isValid) return;

        if (inputDevice.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 pos))
            ikTarget.position = transform.TransformPoint(pos);
        if (inputDevice.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rot))
            ikTarget.rotation = transform.rotation * rot;
    }

    void PublishIKTarget()
    {
        UpdateIKTarget();
        Vector3 pos = ikTarget.position;
        Quaternion rot = ikTarget.rotation;

        // Unity → ROS coordinate conversion (CRITICAL due to different coordinate systems)
        Vector3 rosPos = new Vector3(
            pos.z,
            -pos.x,
            pos.y
        );

        Quaternion rosRot = new Quaternion(
            rot.z,
            -rot.x,
            rot.y,
            -rot.w
        );

        PoseMsg msg = new PoseMsg(
            new PointMsg(rosPos.x, rosPos.y, rosPos.z),
            new QuaternionMsg(rosRot.x, rosRot.y, rosRot.z, rosRot.w)
        );

        ros.Publish(poseTopic, msg);
    }

    void OnJointStateReceived(JointStateMsg msg)
    {
        if (msg.position.Length != joints.Length)
            return;

        for (int i = 0; i < joints.Length; i++)
        {
            var drive = joints[i].xDrive;

            // radians → degrees
            float targetDeg = (float)(msg.position[i] * Mathf.Rad2Deg);

            drive.target = targetDeg;
            joints[i].xDrive = drive;
        }
    }
}