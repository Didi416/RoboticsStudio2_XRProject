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
        if (sourceLabel != null && cameraManager != null)
            sourceLabel.text = cameraManager.activeSource == CameraSource.WebcamDirect
                ? "Source: Webcam  [F1 to switch]"        // changed from Tab
                : "Source: RealSense D435i  [F1 to switch]";
    }
}