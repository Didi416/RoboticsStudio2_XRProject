// RosString.cs
// ─────────────────────────────────────────────────────────────────────────────
// ROS message definition for std_msgs/String.
// The Unity Robotics Hub (ROS-TCP-Connector) requires a C# class per message
// type that inherits from Message and implements the serialisation interface.
//
// HOW TO USE:
//   Place this file anywhere inside your Unity project's Assets/ folder.
//   The ROS TCP Connector package will automatically discover it.
//
// If you already generated messages via the ROS Message Generator tool
// (Window → Robotics → Generate ROS Messages), you will already have
// RosMessageTypes.Std.StringMsg — delete this file and use that instead,
// replacing every reference to RosString with RosMessageTypes.Std.StringMsg.
// ─────────────────────────────────────────────────────────────────────────────

using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.BuiltinInterfaces;

namespace RosMessageTypes.Std
{
    /// <summary>
    /// std_msgs/String  –  single string data field.
    /// </summary>
    public class StringMsg : Message
    {
        // ROS type identifier used by the TCP connector for routing.
        public const string k_RosMessageName = "std_msgs/String";
        public override string RosMessageName => k_RosMessageName;

        public string data;

        public StringMsg()
        {
            data = string.Empty;
        }

        public StringMsg(string data)
        {
            this.data = data;
        }

        public static StringMsg Deserialize(MessageDeserializer deserializer)
        {
            return new StringMsg(deserializer);
        }

        private StringMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out data);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(data);
        }

        public override string ToString()
        {
            return $"std_msgs/String [data={data}]";
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// std_msgs/Bool  –  needed for the /puzzle_busy topic.
// ─────────────────────────────────────────────────────────────────────────────
namespace RosMessageTypes.Std
{
    public class BoolMsg : Message
    {
        public const string k_RosMessageName = "std_msgs/Bool";
        public override string RosMessageName => k_RosMessageName;

        public bool data;

        public BoolMsg() { data = false; }
        public BoolMsg(bool data) { this.data = data; }

        public static BoolMsg Deserialize(MessageDeserializer deserializer)
            => new BoolMsg(deserializer);

        private BoolMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out data);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(data);
        }

        public override string ToString() => $"std_msgs/Bool [data={data}]";
    }
}