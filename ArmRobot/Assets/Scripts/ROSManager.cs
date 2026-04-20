// ROSSetup.cs
// ─────────────────────────────────────────────────────────────────────────────
// Attach to a persistent GameObject (e.g. "ROS Manager") in your scene.
// Registers all custom ROS message types used by the XR teleop stack and
// provides a simple editor-configurable connection to the Linux ROS machine.
//
// This is separate from XRServoTeleop.cs so ROS initialisation happens before
// any subscriber/publisher tries to register.
// ─────────────────────────────────────────────────────────────────────────────

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class ROSSetup : MonoBehaviour
{
    [Header("ROS Bridge Connection")]
    [Tooltip("IP address of the Linux machine running ros_tcp_endpoint")]
    public string rosIP   = "192.168.1.100";
    [Tooltip("Must match ros_tcp_port in the launch file (default 10000)")]
    public int    rosPort = 10000;

    [Tooltip("Show the connection status overlay in the Game view")]
    public bool showHUD = true;

    private ROSConnection ros_;

    private void Awake()
    {
        ros_         = ROSConnection.GetOrCreateInstance();
        ros_.RosIPAddress = rosIP;
        ros_.RosPort      = rosPort;
        ros_.ShowHud      = showHUD;

        // Optionally connect immediately (otherwise first Publish/Subscribe triggers it)
        ros_.ConnectOnStart = true;

        Debug.Log($"[ROSSetup] Will connect to ROS at {rosIP}:{rosPort}");
    }

    // Visual connection status in Editor play mode
    private void OnGUI()
    {
        if (!showHUD) return;

        GUIStyle style = new GUIStyle(GUI.skin.box);
        style.fontSize  = 16;
        style.alignment = TextAnchor.MiddleCenter;

        bool connected = ros_ != null && ros_.HasConnectionError == false;
        string label   = connected ? "ROS ✓ Connected" : "ROS ✗ Disconnected";
        Color  col     = connected ? Color.green : Color.red;

        var prev = GUI.color;
        GUI.color = col;
        GUI.Box(new Rect(10, 10, 200, 30), label, style);
        GUI.color = prev;
    }
}