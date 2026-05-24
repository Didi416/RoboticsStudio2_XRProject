using UnityEngine;

public class EggCube : MonoBehaviour
{
    [Header("Cube Settings")]
    public string eggType;  // "water" "earth" "alien" "angel"
    public Color cubeColor;

    private Renderer cubeRenderer;

    void Start()
    {
        cubeRenderer = GetComponent<Renderer>();
        if (cubeRenderer != null)
            cubeRenderer.material.color = cubeColor;
    }
}

