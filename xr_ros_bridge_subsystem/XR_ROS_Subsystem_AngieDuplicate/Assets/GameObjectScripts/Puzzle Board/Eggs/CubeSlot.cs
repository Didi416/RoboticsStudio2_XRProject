using UnityEngine;

public class CubeSlot : MonoBehaviour
{
    [Header("Slot Settings")]
    public string acceptsEggType;  // "water" "earth" "alien" "angel"
    public int slotIndex;          // 0-3 left to right

    [Header("Visuals")]
    public Color emptyColor = new Color(0.3f, 0.3f, 0.3f, 0.5f);
    public Color correctColor = Color.green;
    public Color wrongColor = Color.red;
    public Renderer slotRenderer;

    [Header("References")]
    public CubePuzzleManager cubePuzzleManager;

    private bool isOccupied = false;
    private GameObject placedCube = null;

    void Start()
    {
        if (slotRenderer != null)
            slotRenderer.material.color = emptyColor;
    }

    // ─────────────────────────────────────────
    // WHEN CUBE IS PLACED IN SLOT
    // ─────────────────────────────────────────

    void OnTriggerEnter(Collider other)
    {
        if (isOccupied) return;

        EggCube cube = other.GetComponent<EggCube>();
        if (cube == null) return;

        PlaceCube(cube);
    }

    void PlaceCube(EggCube cube)
    {
        isOccupied = true;
        placedCube = cube.gameObject;

        // Snap cube to slot position
        cube.transform.position = transform.position;
        cube.transform.rotation = transform.rotation;

        // Disable physics so it stays
        Rigidbody rb = cube.GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.isKinematic = true;
            rb.useGravity = false;
        }

        // Disable grippable so it cant be picked up again
        RobotGrippable grippable = cube.GetComponent<RobotGrippable>();
        if (grippable != null)
            grippable.enabled = false;

        Debug.Log($"Slot {slotIndex} received {cube.eggType} cube");

        // Check with manager
        if (cubePuzzleManager != null)
            cubePuzzleManager.OnCubePlaced(slotIndex, cube.eggType);
    }

    public void ClearSlot()
    {
        isOccupied = false;
        placedCube = null;

        if (slotRenderer != null)
            slotRenderer.material.color = emptyColor;
    }

    public void SetCorrect()
    {
        if (slotRenderer != null)
            slotRenderer.material.color = correctColor;
    }

    public void SetWrong()
    {
        if (slotRenderer != null)
            slotRenderer.material.color = wrongColor;
    }
}