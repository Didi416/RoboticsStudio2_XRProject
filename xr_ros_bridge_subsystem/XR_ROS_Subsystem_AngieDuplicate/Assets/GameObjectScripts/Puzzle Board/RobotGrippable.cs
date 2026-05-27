// using UnityEngine;
// using UnityEngine.InputSystem;

// public class RobotGrippable : MonoBehaviour
// {
//     [Header("Grip Settings")]
//     public bool isGripped = false;
//     public float followSpeed = 20f;

//     [Header("Physics")]
//     public bool usePhysics = true;

//     [Header("Keyboard Testing")]
//     public bool enableKeyboardTest = true;

//     private Transform gripperTransform;
//     private Vector3 gripOffset;
//     private Quaternion rotationOffset;
//     private Rigidbody rb;

//     void Start()
//     {
//         rb = GetComponent<Rigidbody>();
//     }

//     void Update()
//     {
//         if (!enableKeyboardTest) return;

//         if (Keyboard.current[Key.G].wasPressedThisFrame && !isGripped)
//         {
//             TryGrip(Camera.main.transform);
//         }

//         if (Keyboard.current[Key.R].wasPressedThisFrame && isGripped)
//         {
//             Release();
//         }
//     }

//     void FixedUpdate()
//     {
//         if (!isGripped || gripperTransform == null) return;

//         if (usePhysics && rb != null)
//         {
//             Vector3 targetPos = gripperTransform.position + gripOffset;
//             Vector3 moveDir = targetPos - transform.position;
//             rb.linearVelocity = moveDir * followSpeed;

//             Quaternion targetRot = gripperTransform.rotation * rotationOffset;
//             rb.MoveRotation(Quaternion.Slerp(
//                 transform.rotation, targetRot, Time.fixedDeltaTime * followSpeed));
//         }
//         else
//         {
//             transform.position = gripperTransform.position + gripOffset;
//             transform.rotation = gripperTransform.rotation * rotationOffset;
//         }
//     }

//     // ─────────────────────────────────────────
//     // CALLED WHEN GRIPPER TOUCHES THIS OBJECT
//     // ─────────────────────────────────────────

//     void OnTriggerEnter(Collider other)
//     {
//         if (!isGripped)
//             TryGrip(other.transform);
//     }

//     void OnCollisionEnter(Collision collision)
//     {
//         if (!isGripped)
//             TryGrip(collision.transform);
//     }

//     void TryGrip(Transform gripper)
//     {
//         // Don't grip other grippable objects
//         if (gripper.GetComponent<RobotGrippable>() != null) return;

//         // Don't grip ground or walls
//         if (gripper.GetComponent<MeshRenderer>() == null &&
//             gripper.GetComponent<SkinnedMeshRenderer>() == null) return;

//         isGripped = true;
//         gripperTransform = gripper;
//         gripOffset = transform.position - gripper.position;
//         rotationOffset = Quaternion.Inverse(gripper.rotation) * transform.rotation;

//         if (rb != null)
//         {
//             rb.useGravity = false;
//             rb.linearDamping = 10f;
//         }

//         Debug.Log($"{gameObject.name} gripped by {gripper.name}");
//     }

//     // ─────────────────────────────────────────
//     // RELEASE
//     // ─────────────────────────────────────────

//     void OnTriggerExit(Collider other)
//     {
//         if (isGripped && gripperTransform == other.transform)
//             Release();
//     }

//     void OnCollisionExit(Collision collision)
//     {
//         if (isGripped && gripperTransform == collision.transform)
//             Release();
//     }

//     public void Release()
//     {
//         isGripped = false;
//         gripperTransform = null;

//         if (rb != null)
//         {
//             rb.useGravity = true;
//             rb.linearDamping = 0f;
//         }

//         Debug.Log($"{gameObject.name} released");
//     }
// }

using UnityEngine;
using UnityEngine.InputSystem;

public class RobotGrippable : MonoBehaviour
{
    [Header("Grip Settings")]
    public bool isGripped = false;
    public float snapSpeed = 15f;
    public float followSpeed = 20f;

    [Header("Gripper Tag")]
    public string gripperTag = "GripperTip";
    public bool requireGripperTag = true;

    [Header("Snap Settings")]
    public bool snapToGripperCenter = true;
    public float minimumGripDistance = 0.05f;

    [Header("Physics")]
    public bool usePhysics = true;

    [Header("Keyboard Testing")]
    public bool enableKeyboardTest = true;
    public Key testGripKey = Key.G;
    public Key testReleaseKey = Key.R;

    private Transform gripperTransform;
    private Vector3 currentGripOffset;
    private Quaternion rotationOffset;
    private Rigidbody rb;
    private bool gripEnabled = false; // prevents gripping on start

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        // Wait 1 second before allowing gripping
        // prevents robot colliders triggering on spawn
        Invoke("EnableGrip", 1f);
    }

    void EnableGrip()
    {
        gripEnabled = true;
        Debug.Log($"{gameObject.name} grip enabled");
    }

    void FixedUpdate()
    {
        if (!isGripped || gripperTransform == null) return;

        if (usePhysics && rb != null)
        {
            Vector3 targetPos = gripperTransform.position +
                gripperTransform.TransformDirection(currentGripOffset);
            Vector3 moveDir = targetPos - transform.position;
            rb.linearVelocity = moveDir * snapSpeed;

            Quaternion targetRot = gripperTransform.rotation * rotationOffset;
            rb.MoveRotation(Quaternion.Slerp(
                transform.rotation, targetRot,
                Time.fixedDeltaTime * snapSpeed));
        }
        else
        {
            transform.position = Vector3.Lerp(
                transform.position,
                gripperTransform.position +
                gripperTransform.TransformDirection(currentGripOffset),
                Time.fixedDeltaTime * snapSpeed);
            transform.rotation = Quaternion.Slerp(
                transform.rotation,
                gripperTransform.rotation * rotationOffset,
                Time.fixedDeltaTime * snapSpeed);
        }
    }

    // ─────────────────────────────────────────
    // TRIGGER ENTER
    // ─────────────────────────────────────────

    void OnTriggerEnter(Collider other)
    {
        if (!gripEnabled) return; // ignore on start
        if (!isGripped)
            TryGrip(other.transform);
    }

    void OnCollisionEnter(Collision collision)
    {
        if (!gripEnabled) return; // ignore on start
        if (!isGripped)
            TryGrip(collision.transform);
    }

    void TryGrip(Transform gripper)
    {
        // Only grip if tagged as GripperTip
        if (requireGripperTag && !gripper.CompareTag(gripperTag))
        {
            Debug.Log($"Ignored grip from {gripper.name} - not GripperTip tag");
            return;
        }

        // Don't grip other grippable objects
        if (gripper.GetComponent<RobotGrippable>() != null) return;

        // Check minimum distance to prevent instant snap
        float distance = Vector3.Distance(
            transform.position, gripper.position);
        if (distance > minimumGripDistance * 10f)
        {
            Debug.Log($"Too far to grip: {distance}");
            return;
        }

        isGripped = true;
        gripperTransform = gripper;

        if (snapToGripperCenter)
            currentGripOffset = Vector3.zero;
        else
            currentGripOffset = gripperTransform.InverseTransformDirection(
                transform.position - gripper.position);

        rotationOffset = Quaternion.Inverse(gripper.rotation) *
                         transform.rotation;

        if (rb != null)
        {
            rb.useGravity = false;
            rb.linearDamping = 15f;
            rb.angularDamping = 15f;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        Debug.Log($"{gameObject.name} gripped by {gripper.name}");
    }

    // ─────────────────────────────────────────
    // RELEASE
    // ─────────────────────────────────────────

    void OnTriggerExit(Collider other)
    {
        if (isGripped && gripperTransform == other.transform)
            Release();
    }

    void OnCollisionExit(Collision collision)
    {
        if (isGripped && gripperTransform == collision.transform)
            Release();
    }

    public void Release()
    {
        isGripped = false;
        gripperTransform = null;

        if (rb != null)
        {
            rb.useGravity = true;
            rb.linearDamping = 1f;
            rb.angularDamping = 1f;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.interpolation = RigidbodyInterpolation.None;
        }

        Debug.Log($"{gameObject.name} released");
    }

    // ─────────────────────────────────────────
    // KEYBOARD TESTING
    // ─────────────────────────────────────────

    void Update()
    {
        if (!enableKeyboardTest) return;

        if (Keyboard.current[testGripKey].wasPressedThisFrame && !isGripped)
        {
            isGripped = true;
            gripperTransform = Camera.main.transform;
            currentGripOffset = Vector3.zero;
            rotationOffset = Quaternion.Inverse(
                Camera.main.transform.rotation) * transform.rotation;

            if (rb != null)
            {
                rb.useGravity = false;
                rb.linearDamping = 15f;
                rb.angularDamping = 15f;
                rb.interpolation = RigidbodyInterpolation.Interpolate;
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }

            Debug.Log($"{gameObject.name} keyboard gripped");
        }

        if (Keyboard.current[testReleaseKey].wasPressedThisFrame && isGripped)
            Release();
    }
}