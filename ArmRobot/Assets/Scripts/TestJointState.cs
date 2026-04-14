using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateReceiver : MonoBehaviour
{
    [Tooltip("ROS topic to subscribe to")]
    public string jointStateTopic = "/ik_solution";

    [Tooltip("ArticulationBodies in order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3")]
    public ArticulationBody[] joints = new ArticulationBody[6];

    [Header("Drive settings")]
    [Tooltip("Spring stiffness — must be > 0 for xDrive.target to have any effect")]
    public float stiffness = 10000f;

    [Tooltip("Damping — prevents oscillation")]
    public float damping = 200f;

    [Header("Debug — read only at runtime")]
    public float[] receivedDeg = new float[6];
    public bool receiving = false;

    // Incoming angles land here from the ROS callback thread.
    // Applied to drives on the main thread in FixedUpdate.
    readonly float[] _pending = new float[6];
    bool _hasData = false;
    readonly object _lock = new object();

    // move offset by one joint
    float[] _offset = {0,90,0,90,0,0};
    float[] _dirCorrection = {-1,-1,1,1,-1,-1};
    
    void Start()
    {
        string[] jointNames = {
            "Base",
            "Shoulder",
            "Elbow",
            "Wrist2",
            "Wrist3",
            "HandE"
        };

        for (int i = 0; i < joints.Length; i++) // Cycle through all joints and assign automatically each GameObject
        {
            joints[i] = GameObject.Find(jointNames[i]).GetComponent<ArticulationBody>();
            if (joints[i] == null) continue;
            ArticulationDrive d = joints[i].xDrive;
            d.driveType = ArticulationDriveType.Target;
            d.stiffness = stiffness;
            d.damping = damping;
            d.forceLimit = float.MaxValue;
            joints[i].xDrive = d;
        };

        // Register the ROS subscriber
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(jointStateTopic, OnJointStateReceived);

        Debug.Log($"[JointStateReceiver] Subscribed to {jointStateTopic}");
    }

    // FixedUpdate: apply joint angles on the main Unity thread.
    // ArticulationBody writes MUST happen on the main thread.
    void FixedUpdate()
    {
        float[] angles = null;
        lock (_lock)
        {
            if (_hasData)
            {
                angles   = (float[])_pending.Clone();
                _hasData = false;
            }
        }

        receiving = angles != null;
        if (angles == null) return;

        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i] == null) continue;

            // ArticulationDrive is a struct — copy, edit, reassign
            ArticulationDrive d = joints[i].xDrive;
            d.target = angles[i];
            joints[i].xDrive = d;

            receivedDeg[i] = angles[i];
        }
    }

    // Called on the ROS-TCP-Connector background thread — no Unity API calls here
    void OnJointStateReceived(JointStateMsg msg)
    {
        if (msg.position == null || msg.position.Length < joints.Length)
            return;
        
        lock (_lock)
        {
            for (int i = 0; i < joints.Length; i++)
            {
                int rosIndex = (i + 1) % 6;
                // ROS joint angles are in radians — convert to degrees for xDrive.target
                _pending[rosIndex] = ((float)(msg.position[i] * Mathf.Rad2Deg) + _offset[rosIndex])*_dirCorrection[rosIndex];
            }
            _hasData = true;

        }
    }
}