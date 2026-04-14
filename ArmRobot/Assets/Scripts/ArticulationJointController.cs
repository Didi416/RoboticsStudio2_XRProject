using UnityEngine;

public enum RotationDirection { None = 0, Positive = 1, Negative = -1 }

public class ArticulationJointController : MonoBehaviour
{
    public RotationDirection rotationState = RotationDirection.None;
    public float speed = 300.0f;
    private ArticulationBody articulation;

    void Start()
    {
        articulation = GetComponent<ArticulationBody>();
    }

    void FixedUpdate()
    {
        // Only used if you want manual joint control as fallback
        // When ROS is driving, rotationState stays None
        if (rotationState != RotationDirection.None) {
            float rotationChange = (float)rotationState * speed * Time.fixedDeltaTime;
            float rotationGoal = CurrentPrimaryAxisRotation() + rotationChange;
            RotateTo(rotationGoal);
        }
    }

    // NEW: called by RobotController to drive joint from ROS data
    public void SetJointAngle(float angleDegrees)
    {
        rotationState = RotationDirection.None; // disable manual control
        RotateTo(angleDegrees);
    }

    public float CurrentPrimaryAxisRotation()
    {
        return Mathf.Rad2Deg * articulation.jointPosition[0];
    }

    void RotateTo(float primaryAxisRotation)
    {
        var drive = articulation.xDrive;
        drive.target = primaryAxisRotation;
        articulation.xDrive = drive;
    }
}
