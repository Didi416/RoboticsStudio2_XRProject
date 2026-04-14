using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.InputSystem;
using CommonUsages = UnityEngine.XR.CommonUsages; // Alias for distinguishing between CommonUsages in XR and InputSystem
/// <summary>
/// Attach to the UR3e root GameObject in the Unity Articulation demo.
/// Reads end-effector target from VR controller or simulated mouse input,
/// solves IK via the ArticulationBody chain, and streams joint angles over UDP.
/// </summary>
public class UR3eVRPublisher : MonoBehaviour
{
    // [Header("Robot Chain")]
    // [Tooltip("ArticulationBody joints in order: shoulder_pan, shoulder_lift, elbow, wrist1, wrist2, wrist3")]
    public ArticulationBody[] joints = new ArticulationBody[6];

    // [Tooltip("The end effector Transform (tool0 or similar)")]
    public Transform endEffector;

    // [Tooltip("IK target. Move this to drive the robot.")]
    public Transform ikTarget;

    // [Header("Mode")]
    public enum InputMode { SimulatedController, XRHeadset }
    public InputMode inputMode = InputMode.SimulatedController;

    // [Header("Simulated Controller (no headset)")]
    // [Tooltip("Speed in m/s for keyboard/mouse movement")]
    public float simulatedSpeed = 0.3f;

    // [Header("XR Headset")]
    // [Tooltip("Which hand to track")]
    public XRNode xrHand = XRNode.RightHand;

    // [Header("UDP")]
    public string targetIP = "127.0.0.1";
    public int targetPort = 50001;
    // [Tooltip("Sends per second")]
    public float sendRate = 125f;

    // [Header("Safety")]
    // [Tooltip("Max joint velocity (rad/s) sent to real robot")]
    public float maxJointSpeed = 1.05f; // UR3e default max ~1.05 rad/s

    // --- private ---
    UdpClient _udp;
    IPEndPoint _endpoint;
    float _sendTimer;
    float[] _lastAngles = new float[6];

    // Simulated controller state
    Vector3 _simPosition;
    Quaternion _simRotation;

    void Start()
    {
        _udp = new UdpClient();
        _endpoint = new IPEndPoint(IPAddress.Parse(targetIP), targetPort);

        if (ikTarget != null)
            _simPosition = ikTarget.position;
        _simRotation = Quaternion.identity;

        Debug.Log($"[UR3eVRPublisher] Mode: {inputMode} | Target: {targetIP}:{targetPort}");
    }

    void Update()
    {
        UpdateIKTarget();

        _sendTimer += Time.deltaTime;
        if (_sendTimer >= 1f / sendRate)
        {
            _sendTimer = 0f;
            SendJointAngles();
        }
    }

    // ---------------------------------------------------------------
    //  IK Target Update
    // ---------------------------------------------------------------
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
        var inputDevice = InputDevices.GetDeviceAtXRNode(xrHand);
        if (!inputDevice.isValid) return;

        if (inputDevice.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 pos))
            ikTarget.position = transform.TransformPoint(pos);
        if (inputDevice.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rot))
            ikTarget.rotation = transform.rotation * rot;
    }

    // ---------------------------------------------------------------
    //  Read current joint angles from ArticulationBodies
    //  (Unity Articulation demo uses xDrive on single-DOF joints)
    // ---------------------------------------------------------------
    float[] GetJointAngles()
    {
        float[] angles = new float[6];
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i] == null) continue;
            angles[i] = joints[i].jointPosition[0]; // radians
        }
        return angles;
    }

    // ---------------------------------------------------------------
    //  Speed-limit: clamp delta to maxJointSpeed
    // ---------------------------------------------------------------
    float[] SafetyClamp(float[] angles)
    {
        float[] safe = new float[6];
        float dt = 1f / sendRate;
        for (int i = 0; i < 6; i++)
        {
            float delta = angles[i] - _lastAngles[i];
            float maxDelta = maxJointSpeed * dt;
            safe[i] = _lastAngles[i] + Mathf.Clamp(delta, -maxDelta, maxDelta);
        }
        return safe;
    }

    // ---------------------------------------------------------------
    //  UDP send joint angles
    // ---------------------------------------------------------------
    void SendJointAngles()
    {
        float[] rawAngles = GetJointAngles();
        float[] safe = SafetyClamp(rawAngles);

        // Build JSON payload
        string json = $"{{\"joints\":[{safe[0]:F5},{safe[1]:F5},{safe[2]:F5},{safe[3]:F5},{safe[4]:F5},{safe[5]:F5}]," +
                      $"\"ee_pos\":[{endEffector.position.x:F4},{endEffector.position.y:F4},{endEffector.position.z:F4}]," +
                      $"\"timestamp\":{Time.realtimeSinceStartup:F4}}}";

        byte[] data = Encoding.UTF8.GetBytes(json);
        try { _udp.Send(data, data.Length, _endpoint); }
        catch (Exception e) { Debug.LogWarning($"[UR3eVRPublisher] UDP send failed: {e.Message}"); }

        Array.Copy(safe, _lastAngles, 6);
    }

    void OnDestroy()
    {
        _udp?.Close();
    }

    // ---------------------------------------------------------------
    //  Editor helper: visualise IK target in Scene view
    // ---------------------------------------------------------------
    void OnDrawGizmos()
    {
        if (ikTarget == null) return;
        Gizmos.color = inputMode == InputMode.XRHeadset ? Color.cyan : Color.yellow;
        Gizmos.DrawWireSphere(ikTarget.position, 0.025f);
        Gizmos.DrawLine(ikTarget.position, ikTarget.position + ikTarget.forward * 0.05f);
    }
}