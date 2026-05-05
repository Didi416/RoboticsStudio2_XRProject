// PuzzleROSManager.cs
// ─────────────────────────────────────────────────────────────────────────────
// Singleton manager that handles ALL ROS communication for the puzzle GUI.
//
// Publishes:
//   /puzzle_goal    std_msgs/String   ("puzzle1" | "puzzle2" | "puzzle3" | "home")
//
// Subscribes:
//   /puzzle_status  std_msgs/String   ("ready" | "moving to puzzle1" | "reached puzzle1" | ...)
//   /puzzle_busy    std_msgs/Bool     (true while robot is executing a motion)
//
// Communicates via ROS-TCP-Connector (Unity-Robotics-Hub package).
// The ROS side uses ros_tcp_endpoint on port 10000 (matches your xr_teleop_launch.py).
//
// Place this file at:
//   Assets/Scripts/ROS/PuzzleROSManager.cs
//
// Setup:
//   1. Add this component to a GameObject named "ROSManager" in your scene.
//   2. In the Inspector, set RosIP and RosPort to match your machine running ROS.
//   3. Ensure ROSConnectionPrefab is assigned (from Window > Robotics > ROS Settings).
// ─────────────────────────────────────────────────────────────────────────────

using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

namespace EscapeXRtist.ROS
{
    /// <summary>
    /// Fired whenever the robot's status string changes.
    /// </summary>
    public static class PuzzleEvents
    {
        public static event Action<string> OnStatusChanged;
        public static event Action<bool>   OnBusyChanged;
        public static event Action<bool>   OnConnectionChanged;

        public static void RaiseStatus(string status)      => OnStatusChanged?.Invoke(status);
        public static void RaiseBusy(bool busy)            => OnBusyChanged?.Invoke(busy);
        public static void RaiseConnection(bool connected) => OnConnectionChanged?.Invoke(connected);
    }

    public class PuzzleROSManager : MonoBehaviour
    {
        // ── Inspector fields ──────────────────────────────────────────────────
        [Header("ROS TCP Connection")]
        [Tooltip("IP address of the machine running ros_tcp_endpoint (your ROS PC).")]
        public string RosIP   = "192.168.1.100";

        [Tooltip("Port of ros_tcp_endpoint – must match your launch file (default 10000).")]
        public int    RosPort = 10000;

        [Header("Topic Names")]
        public string GoalTopic  = "/puzzle_goal";
        public string StatusTopic = "/puzzle_status";
        public string BusyTopic   = "/puzzle_busy";

        [Header("Debug")]
        public bool VerboseLogging = true;

        // ── Singleton ─────────────────────────────────────────────────────────
        public static PuzzleROSManager Instance { get; private set; }

        // ── State ─────────────────────────────────────────────────────────────
        public bool IsConnected { get; private set; }
        public bool IsRobotBusy { get; private set; }
        public string LastStatus { get; private set; } = "—";

        private ROSConnection _ros;

        // ── Unity lifecycle ───────────────────────────────────────────────────

        private void Awake()
        {
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
            InitROSConnection();
        }

        private void OnDestroy()
        {
            if (Instance == this) Instance = null;
        }

        // ── ROS initialisation ────────────────────────────────────────────────

        private void InitROSConnection()
        {
            // Override the IP/port from inspector values before connecting.
            // ROSConnection.GetOrCreateInstance() reads from ROS Settings asset,
            // so we set it here programmatically for runtime flexibility.
            ROSConnection.GetOrCreateInstance().Connect(RosIP, RosPort);
            _ros = ROSConnection.GetOrCreateInstance();

            // Register the publisher
            _ros.RegisterPublisher<StringMsg>(GoalTopic);

            // Register subscribers
            _ros.Subscribe<StringMsg>(StatusTopic, OnStatusReceived);
            _ros.Subscribe<BoolMsg>(BusyTopic,     OnBusyReceived);

            // Poll connection state (ROSConnection doesn't expose a direct event)
            InvokeRepeating(nameof(PollConnectionState), 1f, 2f);

            if (VerboseLogging)
                Debug.Log($"[PuzzleROSManager] Connecting to {RosIP}:{RosPort}");
        }

        // ── Publish goal ──────────────────────────────────────────────────────

        /// <summary>
        /// Send a goal name to ROS.
        /// Valid values: "puzzle1", "puzzle2", "puzzle3", "home"
        /// </summary>
        public void SendPuzzleGoal(string goalName)
        {
            if (IsRobotBusy)
            {
                Debug.LogWarning("[PuzzleROSManager] Robot is busy — goal rejected.");
                return;
            }

            var msg = new StringMsg(goalName);
            _ros.Publish(GoalTopic, msg);

            if (VerboseLogging)
                Debug.Log($"[PuzzleROSManager] Published goal: {goalName}");
        }

        // ── Subscribers ───────────────────────────────────────────────────────

        private void OnStatusReceived(StringMsg msg)
        {
            LastStatus = msg.data;
            // Fire on Unity main thread (subscribers may arrive on background thread)
            UnityMainThread.Enqueue(() =>
            {
                PuzzleEvents.RaiseStatus(msg.data);
                if (VerboseLogging)
                    Debug.Log($"[PuzzleROSManager] Status: {msg.data}");
            });
        }

        private void OnBusyReceived(BoolMsg msg)
        {
            IsRobotBusy = msg.data;
            UnityMainThread.Enqueue(() =>
            {
                PuzzleEvents.RaiseBusy(msg.data);
            });
        }

        // ── Connection polling ────────────────────────────────────────────────

        private void PollConnectionState()
        {
            // ROSConnection.HasConnectionThread is the clearest available signal
            bool connected = _ros != null && _ros.HasConnectionThread;
            if (connected != IsConnected)
            {
                IsConnected = connected;
                PuzzleEvents.RaiseConnection(connected);
                Debug.Log($"[PuzzleROSManager] Connection: {(connected ? "UP" : "DOWN")}");
            }
        }
    }
}