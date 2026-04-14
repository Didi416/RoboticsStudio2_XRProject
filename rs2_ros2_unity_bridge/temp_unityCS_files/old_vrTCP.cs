using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class CartesianServoController : MonoBehaviour
{
    ROSConnection ros;
    string twistTopic = "/servo_node/delta_twist_cmds";
    float moveSpeed = 0.3f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistStampedMsg>(twistTopic);
    }

    void Update()
    {
        var twist = new TwistStampedMsg();
        
        // Header — must match your planning_frame in servo config
        twist.header = new HeaderMsg {
            frame_id = "base_link",
            stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg {
                sec = (uint)Time.time,
                nanosec = (uint)((Time.time % 1f) * 1e9)
            }
        };

        // Arrow keys = XZ plane movement
        // Q/E = up/down (Y)
        // Unity → ROS coordinate conversion applied here
        twist.twist.linear.z =  Input.GetKey(KeyCode.UpArrow)    ?  moveSpeed : 
                                 Input.GetKey(KeyCode.DownArrow)  ? -moveSpeed : 0;
        twist.twist.linear.x =  Input.GetKey(KeyCode.RightArrow) ?  moveSpeed : 
                                 Input.GetKey(KeyCode.LeftArrow)  ? -moveSpeed : 0;
        twist.twist.linear.y =  Input.GetKey(KeyCode.Q)          ?  moveSpeed : 
                                 Input.GetKey(KeyCode.E)          ? -moveSpeed : 0;

        // Only publish when a key is held
        bool anyKey = Input.GetKey(KeyCode.UpArrow) || Input.GetKey(KeyCode.DownArrow) ||
                      Input.GetKey(KeyCode.LeftArrow) || Input.GetKey(KeyCode.RightArrow) ||
                      Input.GetKey(KeyCode.Q) || Input.GetKey(KeyCode.E);

        if (anyKey)
            ros.Publish(twistTopic, twist);
    }

    void OnGUI() {
        GUI.Label(new Rect(10,10,500,20), "↑↓ = Forward/Back  ←→ = Left/Right  Q/E = Up/Down");
    }
}