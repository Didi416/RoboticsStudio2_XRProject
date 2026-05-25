// Assets/Scripts/CameraSourceUI.cs
using UnityEngine;
using UnityEngine.UI;

public class CameraSourceUI : MonoBehaviour
{
    public CameraFeedManager cameraManager;
    public Text sourceLabel;          // or use TextMeshPro if preferred
    public KeyCode toggleKey = KeyCode.Tab;

    void Update()
    {
        if (sourceLabel != null)
            sourceLabel.text = cameraManager.activeSource == CameraSource.WebcamDirect
                ? "📷 Source: Laptop Webcam  [TAB to switch]"
                : "📷 Source: RealSense D435i  [TAB to switch]";
    }
}