using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.InputSystem;
using CommonUsages = UnityEngine.XR.CommonUsages;

// =============================================================================
//  UR3eVRPublisher
//
//  Pipeline (mirrors the 5-step UR16e approach):
//    Step 1 — Define UR3e kinematics via DH parameters
//    Step 2 — Compute target end-effector pose from VR / keyboard input
//    Step 3 — Solve first 3 joints for position  (shoulder + elbow)
//    Step 4 — Solve last 3 joints for orientation (spherical wrist)
//    Step 5 — Apply joint targets to ArticulationBody drives + stream UDP
//
//  Attach to the UR3e root GameObject.
//  Assign joints[0..5] in Inspector:
//    0=shoulder_pan, 1=shoulder_lift, 2=elbow,
//    3=wrist_1,      4=wrist_2,       5=wrist_3
// =============================================================================
public class UR3eVRPublisher : MonoBehaviour
{
    // ── Inspector fields ──────────────────────────────────────────────────────

    // [Tooltip("ArticulationBodies in DH order (shoulder_pan … wrist_3)")]
    public ArticulationBody[] joints = new ArticulationBody[6];

    // [Tooltip("tool0 / TCP Transform at the arm tip")]
    public Transform endEffector;

    // [Tooltip("Empty GameObject — the arm tracks this")]
    public Transform ikTarget;

    // [Header("Input")]
    public enum InputMode { Keyboard, XRController }
    public InputMode inputMode      = InputMode.Keyboard;
    public float     moveSpeed      = 0.4f;   // m/s for keyboard
    public XRNode    xrHand         = XRNode.RightHand;

    // [Header("Drive")]
    // [Tooltip("Max slew rate for xDrive.target (deg/s). 0 = instant.")]
    public float driveSpeed = 250f;

    // [Header("UDP")]
    public string remoteIP   = "127.0.0.1";
    public int    remotePort = 50001;
    public float  sendHz     = 125f;

    // [Header("Safety")]
    // [Tooltip("Max joint speed forwarded to real robot (rad/s)")]
    public float maxJointVel = 1.05f;

    // [Header("Debug (read-only at runtime)")]
    public bool    hasSolution;
    public float   trackingError;        // metres
    public float[] liveJointsDeg = new float[6];

    // ── Private state ─────────────────────────────────────────────────────────

    // Current xDrive.target values (degrees) — updated each frame
    readonly float[] _driveDeg = new float[6];

    // Last angles streamed over UDP (radians) — for velocity clamping
    readonly float[] _lastSentRad = new float[6];

    // Keyboard sim state
    Vector3    _simPos;
    Quaternion _simRot = Quaternion.identity;

    // UDP
    UdpClient  _udp;
    IPEndPoint _ep;
    float      _sendTimer;


    // ==========================================================================
    //  STEP 1 — UR3e DH Parameters
    //
    //  Modified DH convention, sourced from the Universal Robots UR3e datasheet.
    //  Each row: { d, a, alpha }  (theta is the joint variable, not listed here)
    //
    //  d     offset along previous z-axis        (metres)
    //  a     offset along new x-axis (link len)  (metres)
    //  alpha twist around new x-axis             (radians)
    // ==========================================================================
    static readonly double[,] DH = new double[6, 3]
    {
        //       d          a        alpha
        {  0.15185,    0.0000,   Math.PI/2.0 },   // joint 0  shoulder pan
        {  0.00000,  -0.24355,   0.0         },   // joint 1  shoulder lift
        {  0.00000,  -0.21320,   0.0         },   // joint 2  elbow
        {  0.13105,   0.00000,   Math.PI/2.0 },   // joint 3  wrist 1
        {  0.08535,   0.00000,  -Math.PI/2.0 },   // joint 4  wrist 2
        {  0.09210,   0.00000,   0.0         },   // joint 5  wrist 3
    };

    // Convenience accessors into the DH table
    static double Dd(int i) => DH[i, 0];   // d_i
    static double Aa(int i) => DH[i, 1];   // a_i
    static double Al(int i) => DH[i, 2];   // alpha_i

    // Soft joint limits (radians)
    static readonly double[] Jmin = { -6.28, -3.14, -3.14, -3.14, -6.28, -6.28 };
    static readonly double[] Jmax = {  6.28,  0.00,  3.14,  3.14,  6.28,  6.28 };


    // ==========================================================================
    //  Unity lifecycle
    // ==========================================================================

    void Start()
    {
        _udp = new UdpClient();
        _ep  = new IPEndPoint(IPAddress.Parse(remoteIP), remotePort);

        // Seed keyboard position at current end-effector location
        _simPos = endEffector != null ? endEffector.position : Vector3.zero;

        // Seed drive targets from wherever the arm is right now
        for (int i = 0; i < 6; i++)
            if (joints[i] != null) _driveDeg[i] = joints[i].xDrive.target;

        Debug.Log($"[UR3e] IK publisher ready — mode={inputMode} UDP→{remoteIP}:{remotePort}");
    }

    void Update()
    {
        // Step 2: update target pose
        ComputeTargetPose();

        // Steps 3+4: solve IK and write drives
        SolveAndApply();
    }

    void LateUpdate()
    {
        // Step 5 (UDP): sample after physics has settled this frame
        _sendTimer += Time.deltaTime;
        if (_sendTimer < 1f / sendHz) return;
        _sendTimer = 0f;
        StreamUDP();
    }

    void OnDestroy() => _udp?.Close();


    // ==========================================================================
    //  STEP 2 — Compute target end-effector pose
    //
    //  Moves ikTarget based on user input (keyboard or XR controller).
    //  The result is a world-space position + orientation stored in ikTarget.
    // ==========================================================================
    void ComputeTargetPose()
    {
        if (ikTarget == null) return;

        if (inputMode == InputMode.Keyboard)
        {
            // Arrow keys + comma/period for 3-axis translation
            // Right-mouse drag for orientation
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
                _simRot *= Quaternion.Euler(-md.y * 0.5f, md.x * 0.5f, 0f);
            }

            ikTarget.position = _simPos;
            ikTarget.rotation = _simRot;
        }
        else // XRController
        {
            var dev = InputDevices.GetDeviceAtXRNode(xrHand);
            if (!dev.isValid) return;

            if (dev.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 pos))
                ikTarget.position = transform.TransformPoint(pos);
            if (dev.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rot))
                ikTarget.rotation = transform.rotation * rot;
        }
    }


    // ==========================================================================
    //  STEP 3 + 4 — Solve IK then apply to drives
    //
    //  Converts the ikTarget world pose into robot-base-frame coordinates,
    //  runs the analytical solver, picks the best solution, and slews the
    //  ArticulationBody xDrive targets toward it.
    // ==========================================================================
    void SolveAndApply()
    {
        if (ikTarget == null) return;

        // Express desired pose in robot base frame
        Vector3    localPos = transform.InverseTransformPoint(ikTarget.position);
        Quaternion localRot = Quaternion.Inverse(transform.rotation) * ikTarget.rotation;

        // Run the 5-step analytical solver
        double[][] solutions = Solve(localPos, localRot);

        hasSolution = solutions != null && solutions.Length > 0;
        if (!hasSolution) return;

        // Pick solution closest to current arm configuration (avoids flipping)
        double[] best = PickClosest(solutions);

        // Update debug/tracking fields
        if (endEffector != null)
            trackingError = Vector3.Distance(endEffector.position, ikTarget.position);

        // Slew xDrive targets toward the chosen solution
        for (int i = 0; i < 6; i++)
        {
            float target = (float)(best[i] * Mathf.Rad2Deg);
            _driveDeg[i] = driveSpeed <= 0f
                ? target
                : Mathf.MoveTowards(_driveDeg[i], target, driveSpeed * Time.deltaTime);

            liveJointsDeg[i] = _driveDeg[i];
        }

        // Write to ArticulationBody drives
        ApplyDrives();
    }

    // Writes _driveDeg[i] into each ArticulationBody's xDrive.target.
    // ArticulationDrive is a struct — must copy → edit → reassign.
    void ApplyDrives()
    {
        for (int i = 0; i < 6; i++)
        {
            if (joints[i] == null) continue;
            ArticulationDrive drv = joints[i].xDrive;
            drv.target = _driveDeg[i];
            joints[i].xDrive = drv;
        }
    }

    // Among all valid IK solutions, return the one whose joint angles are
    // collectively closest (L2 norm) to the arm's current configuration.
    double[] PickClosest(double[][] solutions)
    {
        double   bestCost = double.MaxValue;
        double[] bestSol  = solutions[0];

        foreach (double[] s in solutions)
        {
            double cost = 0;
            for (int i = 0; i < 6; i++)
            {
                double cur  = joints[i] != null ? joints[i].jointPosition[0] : 0.0;
                double diff = s[i] - cur;
                // Wrap to [-π, π] so a 2π ambiguity doesn't look like a large jump
                while (diff >  Math.PI) diff -= 2 * Math.PI;
                while (diff < -Math.PI) diff += 2 * Math.PI;
                cost += diff * diff;
            }
            if (cost < bestCost) { bestCost = cost; bestSol = s; }
        }
        return bestSol;
    }


    // ==========================================================================
    //  STEP 5 — Stream UDP
    //
    //  Reads actual joint positions from physics (LateUpdate), velocity-clamps
    //  them for real-robot safety, and sends a JSON packet.
    // ==========================================================================
    void StreamUDP()
    {
        float[] radNow = new float[6];
        for (int i = 0; i < 6; i++)
            radNow[i] = joints[i] != null ? joints[i].jointPosition[0] : 0f;

        // Velocity clamp: real robot must never be asked to jump instantaneously
        float dt = 1f / sendHz;
        float[] safe = new float[6];
        for (int i = 0; i < 6; i++)
        {
            float delta = radNow[i] - _lastSentRad[i];
            float cap   = maxJointVel * dt;
            safe[i] = _lastSentRad[i] + Mathf.Clamp(delta, -cap, cap);
        }
        Array.Copy(safe, _lastSentRad, 6);

        Vector3 ee = endEffector != null ? endEffector.position : Vector3.zero;
        string json =
            $"{{\"joints\":[{safe[0]:F5},{safe[1]:F5},{safe[2]:F5}," +
            $"{safe[3]:F5},{safe[4]:F5},{safe[5]:F5}]," +
            $"\"ee_pos\":[{ee.x:F4},{ee.y:F4},{ee.z:F4}]," +
            $"\"t\":{Time.realtimeSinceStartup:F4}}}";

        byte[] data = Encoding.UTF8.GetBytes(json);
        try   { _udp.Send(data, data.Length, _ep); }
        catch (Exception e) { Debug.LogWarning($"[UR3e] UDP: {e.Message}"); }
    }


    // ==========================================================================
    //  ANALYTICAL IK SOLVER  (steps 3 & 4 internals)
    //
    //  Step 3 — Position subproblem (joints 0, 1, 2)
    //    The wrist centre (WC) is a fixed offset D6 back from the tool tip.
    //    With its position known in the base frame, joint 0 (shoulder pan) is
    //    found by atan2, then joints 1 & 2 are a classic 2-link planar problem.
    //
    //  Step 4 — Orientation subproblem (joints 3, 4, 5)
    //    With joints 0-2 determined, the rotation that 0-2 produce is known.
    //    The remaining rotation needed is split across the spherical wrist:
    //    joint 4 (wrist tilt), joint 5 (wrist spin), joint 3 (wrist pan).
    //
    //  Returns up to 8 solutions (2 shoulder × 2 elbow × 2 wrist).
    // ==========================================================================
    double[][] Solve(Vector3 pos, Quaternion rot)
    {
        // Build 4×4 target matrix T in the UR (Z-up right-handed) frame.
        // Unity uses Y-up left-handed, so Y↔Z swap + sign fix is applied.
        double[,] T = BuildT(pos, rot);

        var results = new System.Collections.Generic.List<double[]>();

        // ── Step 3a : Shoulder pan θ0 ─────────────────────────────────────────
        // Wrist centre = tool position minus D6 along the approach axis (T column 2)
        double wx = T[0,3] - Dd(5)*T[0,2];
        double wy = T[1,3] - Dd(5)*T[1,2];

        double r   = Math.Sqrt(wx*wx + wy*wy);
        double psi = Math.Atan2(wy, wx);

        // The wrist sits laterally offset D4 from the shoulder axis;
        // acos accounts for that offset in the pan angle.
        double ratio = Dd(3) / r;
        if (Math.Abs(ratio) > 1.0) return null;   // target unreachable
        double phi = Math.Acos(Math.Max(-1.0, Math.Min(1.0, ratio)));

        // Two shoulder candidates (robot can pan left or right to place WC)
        double[] t0s = { psi + phi + Math.PI/2.0,
                         psi - phi + Math.PI/2.0 };

        foreach (double t0 in t0s)
        {
            double s0 = Math.Sin(t0), c0 = Math.Cos(t0);

            // ── Step 4a : Wrist tilt θ4 ───────────────────────────────────────
            // Projects tool position onto the axis that θ4 controls.
            double v4 = T[0,3]*s0 - T[1,3]*c0 - Dd(3);
            v4 = Math.Max(-1.0, Math.Min(1.0, v4 / Dd(5)));

            double[] t4s = {  Math.Acos(v4), -Math.Acos(v4) };

            foreach (double t4 in t4s)
            {
                double s4 = Math.Sin(t4);

                // ── Step 4b : Wrist spin θ5 ──────────────────────────────────
                // Undefined at wrist singularity (s4≈0); set to 0 arbitrarily.
                double t5 = Math.Abs(s4) < 1e-10
                    ? 0.0
                    : Math.Atan2(
                        (-T[0,1]*s0 + T[1,1]*c0) / s4,
                        ( T[0,0]*s0 - T[1,0]*c0) / s4);

                // ── Step 3b : Elbow θ2 (2-link planar IK) ────────────────────
                // Strip out the known wrist contribution → get the position
                // that joints 0-2 must reach (the T14 sub-matrix)
                double[,] T14 = ComputeT14(T, t0, t4, t5);
                double px = T14[0,3];
                double pz = T14[2,3];

                double a1 = Aa(1), a2 = Aa(2);   // link lengths (negative in UR DH)
                double c2 = (px*px + pz*pz - a1*a1 - a2*a2) / (2.0*a1*a2);
                c2 = Math.Max(-1.0, Math.Min(1.0, c2));

                // Elbow-up and elbow-down variants
                double[] t2s = { Math.Acos(c2), -Math.Acos(c2) };

                foreach (double t2 in t2s)
                {
                    // ── Step 3c : Shoulder lift θ1 ───────────────────────────
                    double k1 = a1 + a2*Math.Cos(t2);
                    double k2 = a2*Math.Sin(t2);
                    double t1 = Math.Atan2(pz, px) - Math.Atan2(k2, k1);

                    // ── Step 4c : Wrist pan θ3 ───────────────────────────────
                    // Makes up whatever rotation remains after joints 0-2 and 4-5
                    double t3 = Math.Atan2(T14[1,0], T14[0,0]) - t1 - t2;

                    double[] sol = { t0, t1, t2, t3, t4, t5 };
                    if (InLimits(sol)) results.Add(sol);
                }
            }
        }

        return results.Count > 0 ? results.ToArray() : null;
    }


    // ==========================================================================
    //  Maths helpers
    // ==========================================================================

    // Build T14 = inv(T01) · T · inv(T56) · inv(T45)
    // Peels off the known outer-joint rotations so joints 1-3 see only
    // the sub-problem they need to solve.
    static double[,] ComputeT14(double[,] T, double t0, double t4, double t5)
    {
        double s0=Math.Sin(t0), c0=Math.Cos(t0);
        double s4=Math.Sin(t4), c4=Math.Cos(t4);
        double s5=Math.Sin(t5), c5=Math.Cos(t5);

        double[,] iT01 = {      // inverse of T01(t0)
            { c0, s0, 0,      0 },
            {-s0, c0, 0,      0 },
            { 0,  0,  1, -Dd(0) },
            { 0,  0,  0,      1 }
        };
        double[,] iT56 = {      // inverse of T56(t5)
            { c5,-s5, 0,      0 },
            { s5, c5, 0,      0 },
            { 0,  0,  1, -Dd(5) },
            { 0,  0,  0,      1 }
        };
        double[,] iT45 = {      // inverse of T45(t4) — DH alpha=-π/2
            { c4, 0, s4,      0 },
            { 0,  1, 0,       0 },
            {-s4, 0, c4, -Dd(4) },
            { 0,  0, 0,       1 }
        };

        return Mul4(Mul4(Mul4(iT01, T), iT56), iT45);
    }

    // Convert a Unity pose (Y-up left-hand) into a 4×4 homogeneous matrix
    // in the UR robot frame (Z-up right-hand).
    // Axis remap: Unity X→UR X,  Unity Y→UR Z,  Unity Z→UR Y
    static double[,] BuildT(Vector3 p, Quaternion q)
    {
        // Remap quaternion axes: swap q.y ↔ q.z, negate new q.y
        double qx= q.x, qy= q.z, qz=-q.y, qw= q.w;
        return new double[4,4] {
            { 1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw), p.x },
            {   2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw), p.z },  // Unity Z → UR Y
            {   2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy), p.y },  // Unity Y → UR Z
            { 0, 0, 0, 1 }
        };
    }

    // Standard 4×4 matrix multiply
    static double[,] Mul4(double[,] A, double[,] B)
    {
        double[,] C = new double[4,4];
        for (int i=0;i<4;i++)
            for (int j=0;j<4;j++)
                for (int k=0;k<4;k++)
                    C[i,j] += A[i,k]*B[k,j];
        return C;
    }

    // Reject solutions that violate soft joint limits
    static bool InLimits(double[] s)
    {
        for (int i=0;i<6;i++)
            if (s[i]<Jmin[i] || s[i]>Jmax[i]) return false;
        return true;
    }


    // ==========================================================================
    //  Gizmos — Scene view only
    // ==========================================================================
    void OnDrawGizmos()
    {
        if (ikTarget == null) return;
        Gizmos.color = hasSolution ? Color.green : Color.red;
        Gizmos.DrawWireSphere(ikTarget.position, 0.025f);
        Gizmos.DrawRay(ikTarget.position, ikTarget.forward * 0.06f);
        if (endEffector != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(endEffector.position, ikTarget.position);
        }
    }
}