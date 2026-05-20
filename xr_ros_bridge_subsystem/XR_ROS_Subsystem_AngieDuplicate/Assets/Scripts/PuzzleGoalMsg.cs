// PuzzleGoalMsg.cs
// ─────────────────────────────────────────────────────────────────────────────
// ROS message wrapper for std_msgs/String.
// Used to publish /puzzle_goal to the ROS TCP Endpoint.
//
// Place this file at:
//   Assets/Scripts/ROS/Messages/PuzzleGoalMsg.cs
//
// The ROS TCP Connector package already includes RosMessageTypes.Std.StringMsg,
// so you do NOT need this file if you import Unity-Robotics-Hub.
// It is provided here as a fallback if StringMsg is not available.
// ─────────────────────────────────────────────────────────────────────────────

using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Std
{
    [Serializable]
    public class StringMsg : Message
    {
        public const string k_RosMessageName = "std_msgs/String";
        public override string RosMessageName => k_RosMessageName;

        public string data;

        public StringMsg()
        {
            data = "";
        }

        public StringMsg(string data)
        {
            this.data = data;
        }
    }
}