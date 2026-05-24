using UnityEngine;
using UnityEngine.InputSystem;

public class RobotGrippable : MonoBehaviour
{
    [Header("Grip Settings")]
    public bool isGripped = false;
    public float followSpeed = 20f;

    [Header("Physics")]
    public bool usePhysics = true;

    [Header("Keyboard Testing")]
    public bool enableKeyboardTest = true;

    private Transform gripperTransform;
    private Vector3 gripOffset;
    private Quaternion rotationOffset;
    private Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        if (!enableKeyboardTest) return;

        if (Keyboard.current[Key.G].wasPressedThisFrame && !isGripped)
        {
            TryGrip(Camera.main.transform);
        }

        if (Keyboard.current[Key.R].wasPressedThisFrame && isGripped)
        {
            Release();
        }
    }

    void FixedUpdate()
    {
        if (!isGripped || gripperTransform == null) return;

        if (usePhysics && rb != null)
        {
            Vector3 targetPos = gripperTransform.position + gripOffset;
            Vector3 moveDir = targetPos - transform.position;
            rb.linearVelocity = moveDir * followSpeed;

            Quaternion targetRot = gripperTransform.rotation * rotationOffset;
            rb.MoveRotation(Quaternion.Slerp(
                transform.rotation, targetRot, Time.fixedDeltaTime * followSpeed));
        }
        else
        {
            transform.position = gripperTransform.position + gripOffset;
            transform.rotation = gripperTransform.rotation * rotationOffset;
        }
    }

    // ─────────────────────────────────────────
    // CALLED WHEN GRIPPER TOUCHES THIS OBJECT
    // ─────────────────────────────────────────

    void OnTriggerEnter(Collider other)
    {
        if (!isGripped)
            TryGrip(other.transform);
    }

    void OnCollisionEnter(Collision collision)
    {
        if (!isGripped)
            TryGrip(collision.transform);
    }

    void TryGrip(Transform gripper)
    {
        // Don't grip other grippable objects
        if (gripper.GetComponent<RobotGrippable>() != null) return;

        // Don't grip ground or walls
        if (gripper.GetComponent<MeshRenderer>() == null &&
            gripper.GetComponent<SkinnedMeshRenderer>() == null) return;

        isGripped = true;
        gripperTransform = gripper;
        gripOffset = transform.position - gripper.position;
        rotationOffset = Quaternion.Inverse(gripper.rotation) * transform.rotation;

        if (rb != null)
        {
            rb.useGravity = false;
            rb.linearDamping = 10f;
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
            rb.linearDamping = 0f;
        }

        Debug.Log($"{gameObject.name} released");
    }
}