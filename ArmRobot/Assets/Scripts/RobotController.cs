using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class RobotController : MonoBehaviour
{
    [System.Serializable]
    public struct Joint
    {
        public string inputAxis;
        public GameObject robotPart;
    }
    public Joint[] joints;


    // CONTROL

    public void StopAllJointRotations()
    {
        for (uint i = 0; i < joints.Length; i++)
        {
            GameObject robotPart = joints[i].robotPart;
            UpdateRotationState(RotationDirection.None, robotPart);
        }
    }

    public void RotateJoint(uint jointIndex, RotationDirection direction)
    {
        StopAllJointRotations();
        Joint joint = joints[jointIndex];
        UpdateRotationState(direction, joint.robotPart);
    }

    // HELPERS

    static void UpdateRotationState(RotationDirection direction, GameObject robotPart)
    {
        ArticulationJointController jointController = robotPart.GetComponent<ArticulationJointController>();
        jointController.rotationState = direction;
    }
}

// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;
// using Unity.Robotics.ROSTCPConnector;
// using RosMessageTypes.Sensor;
// using RosMessageTypes.BuiltinInterfaces;

// public class RobotController : MonoBehaviour
// {
//     [System.Serializable]
//     public struct Joint
//     {
//         public string inputAxis;
//         public GameObject robotPart;
//     }

//     public Joint[] joints;

//     [Header("ROS Settings")]
//     public string jointStateTopic = "/joint_states";
//     public float publishRateHz = 30f;

//     // Joint names must match ROS urdf exactly
//     private static readonly string[] JointNames = {
//         "shoulder_pan_joint",
//         "shoulder_lift_joint",
//         "elbow_joint",
//         "wrist_1_joint",
//         "wrist_2_joint",
//         "wrist_3_joint"
//     };

//     private ROSConnection ros_;
//     private float publishInterval_;
//     private float timer_;
//     private bool isReady_ = false;

//     void Start()
//     {
//         ros_             = ROSConnection.GetOrCreateInstance();
//         publishInterval_ = 1f / publishRateHz;

//         ros_.RegisterPublisher<JointStateMsg>(jointStateTopic);
//         isReady_ = true;
//     }

//     void FixedUpdate()
//     {
//         if (!isReady_) return;

//         timer_ += Time.fixedDeltaTime;
//         if (timer_ < publishInterval_) return;
//         timer_ = 0f;

//         PublishJointStates();
//     }

//     void PublishJointStates()
//     {
//         uint count = Mathf.Min(joints.Length, JointNames.Length);

//         double[]   positions  = new double[count];
//         double[]   velocities = new double[count];
//         double[]   efforts    = new double[count];

//         for (uint i = 0; i < count; i++)
//         {
//             if (joints[i].robotPart == null) continue;

//             ArticulationBody ab = joints[i].robotPart.GetComponent<ArticulationBody>();
//             if (ab == null) continue;

//             // Position: convert degrees → radians
//             positions[i]  = ab.jointPosition[0];   // already in radians for ArticulationBody
//             velocities[i] = ab.jointVelocity[0];
//             efforts[i]    = ab.jointForce[0];
//         }

//         // Build ROS timestamp
//         double rosTime = Time.timeAsDouble;
//         var stamp = new TimeMsg
//         {
//             sec     = (uint)rosTime,
//             nanosec = (uint)((rosTime - (uint)rosTime) * 1e9)
//         };

//         var msg = new JointStateMsg
//         {
//             header = new RosMessageTypes.Std.HeaderMsg
//             {
//                 stamp    = stamp,
//                 frame_id = ""
//             },
//             name     = JointNames[..count],
//             position = positions,
//             velocity = velocities,
//             effort   = efforts
//         };

//         ros_.Publish(jointStateTopic, msg);
//     }

//     // CONTROL
//     public void StopAllJointRotations()
//     {
//         for (uint i = 0; i < joints.Length; i++)
//         {
//             GameObject robotPart = joints[i].robotPart;
//             UpdateRotationState(RotationDirection.None, robotPart);
//         }
//     }

//     public void RotateJoint(uint jointIndex, RotationDirection direction)
//     {
//         StopAllJointRotations();
//         Joint joint = joints[jointIndex];
//         UpdateRotationState(direction, joint.robotPart);
//     }

//     // HELPERS
//     static void UpdateRotationState(RotationDirection direction, GameObject robotPart)
//     {
//         ArticulationJointController jointController = robotPart.GetComponent<ArticulationJointController>();
//         jointController.rotationState = direction;
//     }
// }