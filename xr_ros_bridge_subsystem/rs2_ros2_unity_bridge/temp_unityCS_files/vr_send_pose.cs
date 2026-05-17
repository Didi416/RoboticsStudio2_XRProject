using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.InputSystem;
using CommonUsages = UnityEngine.XR.CommonUsages;

// =============================================================================
//  UR3eVRPublisher  —  Hybrid pose-out / joints-in architecture
//
//  OUTBOUND  (Unity → bridge):  end-effector Cartesian pose at ~125 Hz
//  INBOUND   (bridge → Unity):  actual joint angles from RTDE at ~125 Hz
//
//  The real robot / URSim runs its own IK via servol().
//  Unity receives the resulting joint angles back over UDP and drives the
//  VR arm ArticulationBodies to match — no local IK required.
//
//  Round-trip latency on wired LAN: ~8–15 ms.
//
//  INSPECTOR SETUP
//    joints[0..5]   ArticulationBodies: shoulder_pan … wrist_3
//    endEffector    tool0 / TCP Transform
//    ikTarget       Empty GameObject — move this in VR/keyboard
// =============================================================================
public class UR3eVRPublisher : MonoBehaviour
{
    // ── Inspector ─────────────────────────────────────────────────────────────
    // [Tooltip("ArticulationBodies in order: shoulder_pan, shoulder_lift, elbow, wrist1, wrist2, wrist3")]
    public ArticulationBody[] joints = new ArticulationBody[6];

    // [Tooltip("tool0 / TCP Transform")]
    public Transform endEffector;

    // [Tooltip("Empty GameObject you move — the real robot chases this")]
    public Transform ikTarget;

    // [Header("Drive")]
    // [Tooltip("Spring stiffness on each joint drive")]
    public float driveStiffness = 10000f;
    // [Tooltip("Damping on each joint drive")]
    public float driveDamping   = 200f;
    // [Tooltip("Max slew rate applying incoming joint angles (deg/s). 0 = instant.")]
    public float driveSpeed     = 400f;

    // [Header("Input")]
    public enum InputMode { Keyboard, XRController }
    public InputMode inputMode  = InputMode.Keyboard;
    public float     moveSpeed  = 0.4f;
    public XRNode    xrHand     = XRNode.RightHand;

    // [Header("UDP — Outbound pose")]
    public string bridgeIP      = "127.0.0.1";
    public int    poseOutPort   = 50001;   // Unity sends pose to bridge here
    public float  sendHz        = 125f;

    // [Header("UDP — Inbound joints")]
    public int    jointsInPort  = 50002;   // Bridge sends joint angles to Unity here

    // [Header("Debug — read only")]
    public float[] liveJointsDeg  = new float[6];
    public float   trackingError;
    public bool    receivingData;

    // ── Private ───────────────────────────────────────────────────────────────
    // Outbound
    UdpClient  _sendUdp;
    IPEndPoint _sendEp;
    float      _sendTimer;

    // Inbound — runs on a background thread
    UdpClient       _recvUdp;
    Thread          _recvThread;
    volatile bool   _recvRunning;

    // Shared between recv thread and main thread
    readonly object  _lock         = new object();
    readonly float[] _latestAngles = new float[6];  // radians, from RTDE
    bool             _hasNewAngles = false;

    // Current xDrive targets (degrees) — lerped toward _latestAngles each FixedUpdate
    readonly float[] _driveDeg = new float[6];

    Vector3    _simPos;
    Quaternion _simRot = Quaternion.identity;

    // =========================================================================
    //  Start
    // =========================================================================
    void Start()
    {
        // Force ArticulationBody drives into Target mode with real stiffness
        for (int i = 0; i < 6; i++)
        {
            if (joints[i] == null) continue;
            ArticulationDrive d = joints[i].xDrive;
            d.driveType  = ArticulationDriveType.Target;
            d.stiffness  = driveStiffness;
            d.damping    = driveDamping;
            d.forceLimit = float.MaxValue;
            d.target     = joints[i].jointPosition[0] * Mathf.Rad2Deg;
            joints[i].xDrive  = d;
            _driveDeg[i] = d.target;
        }

        _simPos = endEffector != null ? endEffector.position : Vector3.zero;

        // Outbound UDP socket (send-only)
        _sendUdp = new UdpClient();
        _sendEp  = new IPEndPoint(IPAddress.Parse(bridgeIP), poseOutPort);

        // Inbound UDP — background thread so recv never blocks the main thread
        _recvUdp     = new UdpClient(jointsInPort);
        _recvRunning = true;
        _recvThread  = new Thread(RecvLoop) { IsBackground = true };
        _recvThread.Start();

        Debug.Log($"[UR3e] Pose out → {bridgeIP}:{poseOutPort} | Joints in ← :{jointsInPort}");
    }

    // =========================================================================
    //  Update — input only (render framerate)
    // =========================================================================
    void Update()
    {
        if (ikTarget == null) return;
        if (inputMode == InputMode.Keyboard) UpdateKeyboard();
        else                                 UpdateXR();
    }

    // =========================================================================
    //  FixedUpdate — apply incoming joint angles + send pose
    //
    //  Both happen in FixedUpdate so they're synchronised with physics.
    // =========================================================================
    void FixedUpdate()
    {
        // Apply the latest joint angles received from the robot
        ApplyIncomingAngles();

        // Send the current ikTarget pose to the bridge
        _sendTimer += Time.fixedDeltaTime;
        if (_sendTimer >= 1f / sendHz)
        {
            _sendTimer = 0f;
            SendPose();
        }
    }

    void OnDestroy()
    {
        _recvRunning = false;
        _recvThread?.Join(200);
        _sendUdp?.Close();
        _recvUdp?.Close();
    }

    // =========================================================================
    //  Apply incoming joint angles to ArticulationBody drives
    //
    //  The background thread writes _latestAngles.
    //  We read it here under a lock and slew _driveDeg toward it.
    // =========================================================================
    void ApplyIncomingAngles()
    {
        float[] angles = null;
        lock (_lock)
        {
            if (_hasNewAngles)
            {
                angles       = (float[])_latestAngles.Clone();
                _hasNewAngles = false;
            }
        }

        receivingData = angles != null;
        if (angles == null) return;

        float dt = Time.fixedDeltaTime;
        for (int i = 0; i < 6; i++)
        {
            float targetDeg = angles[i] * Mathf.Rad2Deg;
            _driveDeg[i] = driveSpeed <= 0f
                ? targetDeg
                : Mathf.MoveTowards(_driveDeg[i], targetDeg, driveSpeed * dt);
            liveJointsDeg[i] = _driveDeg[i];
        }

        for (int i = 0; i < 6; i++)
        {
            if (joints[i] == null) continue;
            ArticulationDrive d = joints[i].xDrive;
            d.target = _driveDeg[i];
            joints[i].xDrive = d;
        }

        if (endEffector != null && ikTarget != null)
            trackingError = Vector3.Distance(endEffector.position, ikTarget.position);
    }

    // =========================================================================
    //  Send end-effector pose to the bridge
    //
    //  Pose is expressed in the robot's base frame (local to this GameObject).
    //  The bridge forwards it to the robot as a servol() target.
    //
    //  Format:
    //    { "pose": [x,y,z, rx,ry,rz], "t": <timestamp> }
    //    Position in metres, orientation as rotation vector (axis * angle, radians)
    //    — this is exactly what UR's servol() expects.
    // =========================================================================
    void SendPose()
    {
        if (ikTarget == null) return;

        // Convert to robot base frame
        Vector3    lp = transform.InverseTransformPoint(ikTarget.position);
        Quaternion lr = Quaternion.Inverse(transform.rotation) * ikTarget.rotation;

        // Unity Y-up left-hand → UR Z-up right-hand
        float urX =  lp.x;
        float urY =  lp.z;   // Unity Z → UR Y
        float urZ =  lp.y;   // Unity Y → UR Z

        // Convert quaternion to UR rotation vector (axis-angle, rodrigues)
        // UR expects [rx, ry, rz] where magnitude = rotation angle in radians
        Vector3 rv = QuatToRotVec(lr);

        string json =
            $"{{\"pose\":[{urX:F5},{urY:F5},{urZ:F5}," +
            $"{rv.x:F5},{rv.y:F5},{rv.z:F5}]," +
            $"\"t\":{Time.realtimeSinceStartup:F4}}}";

        byte[] data = Encoding.UTF8.GetBytes(json);
        try   { _sendUdp.Send(data, data.Length, _sendEp); }
        catch (Exception e) { Debug.LogWarning($"[UR3e] Send: {e.Message}"); }
    }

    // Convert Unity Quaternion to a UR-style rotation vector.
    // The rotation vector is: axis * angle (radians).
    // Axis remapped from Unity (Y-up) to UR (Z-up).
    static Vector3 QuatToRotVec(Quaternion q)
    {
        // Ensure shortest path
        if (q.w < 0) { q.x=-q.x; q.y=-q.y; q.z=-q.z; q.w=-q.w; }
        float angle = 2f * Mathf.Acos(Mathf.Clamp(q.w, -1f, 1f));
        float s     = Mathf.Sqrt(1f - q.w*q.w);
        if (s < 1e-6f) return Vector3.zero;
        // Remap: Unity X→UR X, Unity Y→UR Z, Unity Z→UR Y
        float ax = q.x / s;
        float ay = q.z / s;   // Unity Z → UR Y
        float az = q.y / s;   // Unity Y → UR Z
        return new Vector3(ax * angle, ay * angle, az * angle);
    }

    // =========================================================================
    //  Background UDP receive loop
    //
    //  Runs on a dedicated thread so it never stalls the main thread.
    //  Parses incoming joint-angle packets from the bridge and writes them
    //  to _latestAngles under a lock.
    //
    //  Expected packet format (same as existing bridge output):
    //    { "joints": [j0,j1,j2,j3,j4,j5], ... }   (radians)
    // =========================================================================
    void RecvLoop()
    {
        IPEndPoint any = new IPEndPoint(IPAddress.Any, 0);
        _recvUdp.Client.ReceiveTimeout = 200;   // ms — allows clean shutdown check

        while (_recvRunning)
        {
            try
            {
                byte[] data = _recvUdp.Receive(ref any);
                string json = Encoding.UTF8.GetString(data);
                ParseJoints(json);
            }
            catch (SocketException) { /* timeout — loop back and check _recvRunning */ }
            catch (Exception e)     { Debug.LogWarning($"[UR3e] Recv: {e.Message}"); }
        }
    }

    // Fast inline JSON parse — no allocations beyond the string itself
    void ParseJoints(string json)
    {
        int bracket = json.IndexOf('[');
        if (bracket < 0) return;

        float[] vals  = new float[6];
        int     count = 0;
        int     i     = bracket + 1;
        int     len   = json.Length;

        while (count < 6 && i < len)
        {
            // Skip whitespace and commas
            while (i < len && (json[i]==' '||json[i]==','||json[i]=='\t')) i++;
            if (i >= len || json[i] == ']') break;

            // Find end of this number
            int start = i;
            if (json[i]=='-') i++;
            while (i < len && (char.IsDigit(json[i])||json[i]=='.')) i++;

            if (float.TryParse(json.Substring(start, i-start),
                System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out float v))
            {
                vals[count++] = v;
            }
        }

        if (count != 6) return;

        lock (_lock)
        {
            Array.Copy(vals, _latestAngles, 6);
            _hasNewAngles = true;
        }
    }

    // =========================================================================
    //  Input
    // =========================================================================
    void UpdateKeyboard()
    {
        Vector3 d = Vector3.zero;
        if (Keyboard.current.commaKey.isPressed)      d += Vector3.forward;
        if (Keyboard.current.periodKey.isPressed)     d += Vector3.back;
        if (Keyboard.current.leftArrowKey.isPressed)  d += Vector3.left;
        if (Keyboard.current.rightArrowKey.isPressed) d += Vector3.right;
        if (Keyboard.current.upArrowKey.isPressed)    d += Vector3.up;
        if (Keyboard.current.downArrowKey.isPressed)  d += Vector3.down;
        _simPos += d * moveSpeed * Time.deltaTime;
        if (Mouse.current.rightButton.isPressed)
        {
            Vector2 md = Mouse.current.delta.ReadValue();
            _simRot *= Quaternion.Euler(-md.y*0.5f, md.x*0.5f, 0f);
        }
        ikTarget.position = _simPos;
        ikTarget.rotation = _simRot;
    }

    void UpdateXR()
    {
        var dev = InputDevices.GetDeviceAtXRNode(xrHand);
        if (!dev.isValid) return;
        if (dev.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 p))
            ikTarget.position = transform.TransformPoint(p);
        if (dev.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion r))
            ikTarget.rotation = transform.rotation * r;
    }

    // =========================================================================
    //  Gizmos
    // =========================================================================
    void OnDrawGizmos()
    {
        if (ikTarget == null) return;
        Gizmos.color = receivingData ? Color.green : Color.red;
        Gizmos.DrawWireSphere(ikTarget.position, 0.025f);
        Gizmos.DrawRay(ikTarget.position, ikTarget.forward * 0.06f);
        if (endEffector != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(endEffector.position, ikTarget.position);
        }
    }
}