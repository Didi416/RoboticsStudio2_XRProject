using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactors;

public class RayColorChanger : MonoBehaviour
{
    public XRRayInteractor rayInteractor;
    public LineRenderer lineRenderer;

    public Color defaultColor = Color.white;
    public Color hoverColor = Color.green;
    public Color selectColor = Color.yellow;

    void Update()
    {
        if (rayInteractor == null || lineRenderer == null) return;

        if (rayInteractor.hasSelection)
        {
            // Currently grabbing something
            lineRenderer.startColor = selectColor;
            lineRenderer.endColor = selectColor;
        }
        else if (rayInteractor.hasHover)
        {
            // Hovering over something
            lineRenderer.startColor = hoverColor;
            lineRenderer.endColor = hoverColor;
        }
        else
        {
            // Nothing
            lineRenderer.startColor = defaultColor;
            lineRenderer.endColor = defaultColor;
        }
    }
}
