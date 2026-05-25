// Assets/Scripts/CameraFeedManager.cs
using UnityEngine;
using UnityEngine.UI;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class CameraFeedManager : MonoBehaviour
{
    // ---------------------------------------------------------------
    // Inspector Settings
    // ---------------------------------------------------------------
    [Header("Camera Source")]
    [Tooltip("WebcamDirect = laptop webcam (no ROS needed)\nRealSenseROS = D435i via ROS2")]
    public CameraSource activeSource = CameraSource.WebcamDirect;

    [Header("Display")]
    [Tooltip("Drag your RawImage UI element here")]
    public RawImage displayImage;

    [Header("Webcam Settings (WebcamDirect mode)")]
    [Tooltip("Leave blank to use the default/first webcam found")]
    public string preferredWebcamName = "";
    public int webcamWidth  = 640;
    public int webcamHeight = 480;
    public int webcamFPS    = 30;

    [Header("RealSense ROS Settings (RealSenseROS mode)")]
    [Tooltip("ROS2 topic published by the RealSense driver")]
    public string rosImageTopic = "/camera/color/image_raw";
    public int rosImageWidth  = 640;
    public int rosImageHeight = 480;

    [Header("Runtime Switch")]
    [Tooltip("Press this key to toggle between sources at runtime")]
    public KeyCode toggleKey = KeyCode.Tab;

    [Header("Performance Monitoring")]
    public bool showFPSInConsole = true;
    public float fpsLogInterval = 2f;   // log every 2 seconds

    private int   _frameCount     = 0;
    private float _fpsTimer       = 0f;
    private float _currentFPS     = 0f;
    private float _lastFrameTime  = 0f;
    private float _currentLatency = 0f;

    // ---------------------------------------------------------------
    // Private state
    // ---------------------------------------------------------------
    private WebCamTexture  _webcamTexture;
    private Texture2D      _rosTexture;
    private bool           _rosSubscribed = false;
    private CameraSource   _currentSource;

    // ---------------------------------------------------------------
    // Unity lifecycle
    // ---------------------------------------------------------------
    void Start()
    {
        _currentSource = activeSource;
        ActivateSource(_currentSource);
    }

    void Update()
    {
        // Runtime toggle with keypress
        if (Input.GetKeyDown(toggleKey))
        {
            Debug.Log("[CameraFeedManager] Tab pressed"); 
            Toggle();
        }

        // Keep webcam texture live each frame
        if (_currentSource == CameraSource.WebcamDirect &&
            _webcamTexture != null && _webcamTexture.isPlaying)
        {
            displayImage.texture = _webcamTexture;   // WebCamTexture updates automatically
        }

        // FPS logging timer
        _fpsTimer += Time.deltaTime;
        if (_fpsTimer >= fpsLogInterval)
        {
            _currentFPS = _frameCount / _fpsTimer;

            if (showFPSInConsole)
                Debug.Log($"[CameraFeedManager] ROS Stream FPS: {_currentFPS:F1} " +
                        $"| Frame interval: {_currentLatency:F1}ms " +
                        $"| Source: {_currentSource}");

            _frameCount = 0;
            _fpsTimer   = 0f;
        }
    }

    void OnDestroy()
    {
        StopWebcam();
    }

    // ---------------------------------------------------------------
    // Public API
    // ---------------------------------------------------------------

    /// <summary>Switch to a specific source.</summary>
    public void SetSource(CameraSource source)
    {
        if (_currentSource == source) return;
        DeactivateCurrentSource();
        _currentSource = source;
        ActivateSource(_currentSource);
    }

    /// <summary>Toggle between webcam and RealSense.</summary>
    public void Toggle()
    {
        SetSource(_currentSource == CameraSource.WebcamDirect
            ? CameraSource.RealSenseROS
            : CameraSource.WebcamDirect);

        Debug.Log($"[CameraFeedManager] Switched to: {_currentSource}");
    }

    // ---------------------------------------------------------------
    // Internal activation / deactivation
    // ---------------------------------------------------------------
    void ActivateSource(CameraSource source)
    {
        switch (source)
        {
            case CameraSource.WebcamDirect:
                StartWebcam();
                break;

            case CameraSource.RealSenseROS:
                StartRealSense();
                break;
        }
    }

    void DeactivateCurrentSource()
    {
        switch (_currentSource)
        {
            case CameraSource.WebcamDirect:
                StopWebcam();
                break;

            case CameraSource.RealSenseROS:
                // ROS subscription stays open — just stop updating the display
                // (no unsubscribe API in ROS-TCP-Connector; we gate in the callback)
                break;
        }
    }

    // ---------------------------------------------------------------
    // Webcam (laptop camera)
    // ---------------------------------------------------------------
    void StartWebcam()
    {
        WebCamDevice[] devices = WebCamTexture.devices;

        if (devices.Length == 0)
        {
            Debug.LogWarning("[CameraFeedManager] No webcam found on this machine.");
            return;
        }

        // Pick preferred device by name, or fall back to first available
        string deviceName = devices[0].name;
        foreach (var d in devices)
        {
            if (!string.IsNullOrEmpty(preferredWebcamName) &&
                d.name.Contains(preferredWebcamName))
            {
                deviceName = d.name;
                break;
            }
        }

        Debug.Log($"[CameraFeedManager] Starting webcam: {deviceName}");

        _webcamTexture = new WebCamTexture(deviceName, webcamWidth, webcamHeight, webcamFPS);
        _webcamTexture.Play();
        displayImage.texture = _webcamTexture;

        // Correct for webcam vertical flip (common on laptops)
        displayImage.transform.localScale = new Vector3(1, 1, 1);
    }

    void StopWebcam()
    {
        if (_webcamTexture != null && _webcamTexture.isPlaying)
        {
            _webcamTexture.Stop();
            _webcamTexture = null;
        }
        // Reset any scale flip
        if (displayImage != null)
            displayImage.transform.localScale = Vector3.one;
    }

    // ---------------------------------------------------------------
    // RealSense via ROS2
    // ---------------------------------------------------------------
    void StartRealSense()
    {
        // Create texture to receive ROS image data
        if (_rosTexture == null)
            _rosTexture = new Texture2D(rosImageWidth, rosImageHeight,
                                        TextureFormat.RGB24, false);

        displayImage.texture = _rosTexture;

        // Subscribe once (guard against duplicate subscriptions)
        if (!_rosSubscribed)
        {
            ROSConnection.GetOrCreateInstance()
                .Subscribe<ImageMsg>(rosImageTopic, OnRosImageReceived);
            _rosSubscribed = true;
            Debug.Log($"[CameraFeedManager] Subscribed to ROS topic: {rosImageTopic}");
        }
    }

    void OnRosImageReceived(ImageMsg msg)
    {
        if (_currentSource != CameraSource.RealSenseROS) return;

        // --- FPS tracking ---
        float now = Time.realtimeSinceStartup;
        if (_lastFrameTime > 0f)
            _currentLatency = (now - _lastFrameTime) * 1000f;  // ms between frames
        _lastFrameTime = now;
        _frameCount++;

        // --- Image update ---
        byte[] pixels = msg.data;
        if (msg.encoding == "bgr8")
            pixels = BGRtoRGB(msg.data);

        _rosTexture.LoadRawTextureData(pixels);
        _rosTexture.Apply();
    }

    // RealSense publishes BGR8 by default; Unity expects RGB
    byte[] BGRtoRGB(byte[] bgr)
    {
        byte[] rgb = new byte[bgr.Length];
        for (int i = 0; i < bgr.Length; i += 3)
        {
            rgb[i]     = bgr[i + 2]; // R
            rgb[i + 1] = bgr[i + 1]; // G
            rgb[i + 2] = bgr[i];     // B
        }
        return rgb;
    }
}