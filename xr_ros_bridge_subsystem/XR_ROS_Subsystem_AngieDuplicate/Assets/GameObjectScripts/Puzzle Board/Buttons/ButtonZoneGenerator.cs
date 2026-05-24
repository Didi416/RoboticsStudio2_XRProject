using UnityEngine;

public class ButtonZoneGenerator : MonoBehaviour
{
    [Header("Grid Settings")]
    public int columns = 3;
    public int rows = 3;
    public float buttonWidth = 0.1f;    // adjust to match your mesh
    public float buttonHeight = 0.1f;   // adjust to match your mesh
    public float buttonDepth = 0.05f;   // how deep the trigger zone is
    public float spacingX = 0.02f;      // gap between buttons
    public float spacingY = 0.02f;

    [Header("References")]
    public ButtonMatrixPanel buttonMatrixPanel;

    [Header("Visuals (for testing)")]
    public bool showDebugCubes = true;  // shows coloured cubes so you can see zones
    public Color debugColor = new Color(0f, 1f, 0f, 0.3f);

    void Start()
    {
        GenerateButtonZones();
    }

    void GenerateButtonZones()
    {
        // Clear existing children
        foreach (Transform child in transform)
            Destroy(child.gameObject);

        int buttonNum = 1;

        for (int row = rows - 1; row >= 0; row--)
        {
            for (int col = 0; col < columns; col++)
            {
                // Calculate position
                float x = col * (buttonWidth + spacingX);
                float y = row * (buttonHeight + spacingY);

                GameObject zone = new GameObject($"ButtonZone_{buttonNum}");
                zone.transform.parent = transform;
                zone.transform.localPosition = new Vector3(x, y, 0);

                // Add box collider as trigger
                BoxCollider col3d = zone.AddComponent<BoxCollider>();
                col3d.size = new Vector3(buttonWidth, buttonHeight, buttonDepth);
                col3d.isTrigger = true;

                // Add physical button script
                PhysicalButton btn = zone.AddComponent<PhysicalButton>();
                btn.buttonNumber = buttonNum;
                btn.buttonMatrixPanel = buttonMatrixPanel;

                // Debug visual
                if (showDebugCubes)
                {
                    GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    cube.transform.parent = zone.transform;
                    cube.transform.localPosition = Vector3.zero;
                    cube.transform.localScale = new Vector3(
                        buttonWidth, buttonHeight, buttonDepth * 0.1f);

                    Renderer rend = cube.GetComponent<Renderer>();
                    if (rend != null)
                    {
                        rend.material.color = debugColor;
                        // Make transparent
                        rend.material.SetFloat("_Mode", 3);
                        rend.material.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
                        rend.material.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
                        rend.material.EnableKeyword("_ALPHABLEND_ON");
                        rend.material.renderQueue = 3000;
                    }

                    // Remove collider from cube so it doesn't interfere
                    Destroy(cube.GetComponent<Collider>());
                }

                buttonNum++;
            }
        }

        Debug.Log($"Generated {rows * columns} button zones");
    }
}