// PuzzleRosBridge.cs
// ─────────────────────────────────────────────────────────────────────────────
// Singleton MonoBehaviour that manages all ROS topic communication for the
// EscapeXRtist puzzle control panel.
//
// Publishes  :  /puzzle_goal   (std_msgs/String)
// Subscribes :  /puzzle_status (std_msgs/String)
//               /puzzle_busy   (std_msgs/Bool)
//
// Depends on Unity Robotics Hub — ROS TCP Connector package.
// Install via Package Manager:
//   https://github.com/Unity-Technologies/ROS-TCP-Connector
//   or: Window → Package Manager → Add package from git URL →
//       https://github.com/Unity-Technologies/ROS-TCP-Connector.git
//
// The ROSConnection singleton is configured in:
//   Edit → Project Settings → Robotics → ROS Settings
//     ROS IP Address : your Linux machine running ros_tcp_endpoint
//     ROS Port       : 10000   (matches xr_teleop_launch.py)
// ─────────────────────────────────────────────────────────────────────────────

using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// Thin bridge between the Unity UI and ROS 2 via ros_tcp_endpoint.
/// Other scripts call PuzzleRosBridge.Instance.SendGoal("puzzle1") etc.
/// Subscribe to the C# events to receive status updates.
/// </summary>
public class PuzzleRosBridge : MonoBehaviour
{
    // ── Singleton ─────────────────────────────────────────────────────────────
    public static PuzzleRosBridge Instance { get; private set; }

    // ── Inspector fields ──────────────────────────────────────────────────────
    [Header("ROS Topic Names")]
    [Tooltip("Topic this script publishes goal names to.")]
    [SerializeField] private string goalTopic   = "/puzzle_goal";

    [Tooltip("Topic carrying human-readable robot status text.")]
    [SerializeField] private string statusTopic = "/puzzle_status";

    [Tooltip("Topic carrying a bool: true = robot busy, false = ready.")]
    [SerializeField] private string busyTopic   = "/puzzle_busy";

    // ── C# Events (subscribe in PuzzleControlUI or other scripts) ─────────────
    /// <summary>Fired whenever /puzzle_status publishes a new string.</summary>
    public event Action<string> OnStatusReceived;

    /// <summary>Fired whenever /puzzle_busy changes value.</summary>
    public event Action<bool>   OnBusyChanged;

    // ── Private ───────────────────────────────────────────────────────────────
    private ROSConnection _ros;
    private bool          _lastBusy = false;

    // ── Unity lifecycle ───────────────────────────────────────────────────────

    private void Awake()
    {
        // Enforce singleton
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
            return;
        }
        Instance = this;
        DontDestroyOnLoad(gameObject);
    }

    private void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();

        // Register publisher
        _ros.RegisterPublisher<StringMsg>(goalTopic);

        // Register subscribers
        _ros.Subscribe<StringMsg>(statusTopic, OnStatusMsg);
        _ros.Subscribe<BoolMsg>  (busyTopic,   OnBusyMsg);

        Debug.Log($"[PuzzleRosBridge] Connected.\n" +
                  $"  Publishing to  : {goalTopic}\n" +
                  $"  Listening on   : {statusTopic}, {busyTopic}");
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /// <summary>
    /// Send the robot to a named puzzle position.
    /// Valid names: "puzzle1", "puzzle2", "puzzle3", "home"
    /// </summary>
    public void SendGoal(string goalName)
    {
        if (string.IsNullOrEmpty(goalName)) return;

        var msg = new StringMsg(goalName);
        _ros.Publish(goalTopic, msg);
        Debug.Log($"[PuzzleRosBridge] Published goal: {goalName}");
    }

    // ── ROS callbacks (called on Unity main thread by ROSConnection) ──────────

    private void OnStatusMsg(StringMsg msg)
    {
        Debug.Log($"[PuzzleRosBridge] Status: {msg.data}");
        OnStatusReceived?.Invoke(msg.data);
    }

    private void OnBusyMsg(BoolMsg msg)
    {
        if (msg.data == _lastBusy) return;   // skip duplicate values
        _lastBusy = msg.data;
        Debug.Log($"[PuzzleRosBridge] Busy: {msg.data}");
        OnBusyChanged?.Invoke(msg.data);
    }
}