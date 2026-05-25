// Assets/Scripts/CameraSourceConfig.cs
public enum CameraSource
{
    WebcamDirect,       // Laptop webcam via Unity WebCamTexture (no ROS)
    RealSenseROS        // Intel RealSense D435i via ROS2 topic
}