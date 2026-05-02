// BoolMsg.cs
// ─────────────────────────────────────────────────────────────────────────────
// ROS message wrapper for std_msgs/Bool.
// Used to subscribe to /puzzle_busy from the ROS TCP Endpoint.
//
// Place this file at:
//   Assets/Scripts/ROS/Messages/BoolMsg.cs
//
// Note: Unity-Robotics-Hub already includes RosMessageTypes.Std.BoolMsg.
// Only use this file as a fallback.
// ─────────────────────────────────────────────────────────────────────────────

using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Std
{
    [Serializable]
    public class BoolMsg : Message
    {
        public const string k_RosMessageName = "std_msgs/Bool";
        public override string RosMessageName => k_RosMessageName;

        public bool data;

        public BoolMsg()
        {
            data = false;
        }

        public BoolMsg(bool data)
        {
            this.data = data;
        }
    }
}