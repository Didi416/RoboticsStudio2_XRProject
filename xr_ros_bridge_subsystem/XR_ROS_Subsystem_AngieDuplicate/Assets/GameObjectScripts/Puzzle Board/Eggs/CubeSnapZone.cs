using UnityEngine;
using System.Collections;

public class CubeSnapZone : MonoBehaviour
{
    [Header("Snap Settings")]
    public float snapDistance = 0.15f;   // how close before snapping
    public float snapSpeed = 10f;         // speed of snap animation
    public string acceptsTag = "";        // leave empty for any cube

    [Header("Visual Feedback")]
    public Renderer zoneRenderer;
    public Color emptyColor = new Color(0.3f, 0.3f, 0.3f, 0.5f);
    public Color hoverColor = new Color(0f, 0.8f, 1f, 0.5f);
    public Color snappedColor = new Color(0f, 1f, 0f, 0.5f);

    [Header("References")]
    public CubePuzzleManager cubePuzzleManager;
    public int slotIndex;

    private GameObject snappedCube = null;
    private bool isOccupied = false;
    private Coroutine snapCoroutine;

    void Start()
    {
        SetZoneColor(emptyColor);
    }

    void Update()
    {
        if (isOccupied) return;

        // Check for nearby cubes
        Collider[] nearby = Physics.OverlapSphere(
            transform.position, snapDistance);

        bool cubeNearby = false;
        foreach (Collider col in nearby)
        {
            EggCube cube = col.GetComponent<EggCube>();
            if (cube == null) continue;

            RobotGrippable grippable = cube.GetComponent<RobotGrippable>();

            // Show hover color when cube nearby but not gripped
            if (grippable != null && !grippable.isGripped)
            {
                cubeNearby = true;
                SetZoneColor(hoverColor);

                // Auto snap when close enough and not gripped
                SnapCube(cube.gameObject, grippable, cube.eggType);
                break;
            }
            else if (grippable != null && grippable.isGripped)
            {
                // Show hover while being carried
                cubeNearby = true;
                SetZoneColor(hoverColor);
            }
        }

        if (!cubeNearby)
            SetZoneColor(emptyColor);
    }

    void SnapCube(GameObject cube, RobotGrippable grippable, string eggType)
    {
        if (isOccupied) return;

        isOccupied = true;
        snappedCube = cube;

        // Stop cube physics
        Rigidbody rb = cube.GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.isKinematic = true;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        // Disable grippable
        if (grippable != null)
            grippable.enabled = false;

        // Smooth snap to slot position
        if (snapCoroutine != null)
            StopCoroutine(snapCoroutine);
        snapCoroutine = StartCoroutine(SmoothSnap(cube));

        SetZoneColor(snappedColor);

        Debug.Log($"Slot {slotIndex} snapped {eggType}");

        // Notify puzzle manager
        if (cubePuzzleManager != null)
            cubePuzzleManager.OnCubePlaced(slotIndex, eggType);
    }

    IEnumerator SmoothSnap(GameObject cube)
    {
        float elapsed = 0f;
        float duration = 0.3f; // 0.3 second snap animation

        Vector3 startPos = cube.transform.position;
        Quaternion startRot = cube.transform.rotation;

        while (elapsed < duration)
        {
            elapsed += Time.deltaTime;
            float t = Mathf.SmoothStep(0f, 1f, elapsed / duration);

            cube.transform.position = Vector3.Lerp(
                startPos, transform.position, t);
            cube.transform.rotation = Quaternion.Slerp(
                startRot, transform.rotation, t);

            yield return null;
        }

        // Final snap to exact position
        cube.transform.position = transform.position;
        cube.transform.rotation = transform.rotation;

        Debug.Log($"Cube snapped to slot {slotIndex}");
    }

    public void ClearSlot()
    {
        if (snappedCube != null)
        {
            // Re-enable physics and grippable
            Rigidbody rb = snappedCube.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.isKinematic = false;
                rb.useGravity = true;
            }

            RobotGrippable grippable = snappedCube.GetComponent<RobotGrippable>();
            if (grippable != null)
                grippable.enabled = true;
        }

        snappedCube = null;
        isOccupied = false;
        SetZoneColor(emptyColor);
    }

    void SetZoneColor(Color color)
    {
        if (zoneRenderer != null)
            zoneRenderer.material.color = color;
    }

    void OnDrawGizmosSelected()
    {
        // Show snap radius in Scene view
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(transform.position, snapDistance);
    }
}