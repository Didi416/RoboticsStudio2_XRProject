// Assets/Scripts/CameraFeedManager.cs
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.InputSystem;          // New Input System
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class CameraFeedManager : MonoBehaviour
{
    // ---------------------------------------------------------------
    // Inspector Settings
    // ---------------------------------------------------------------
    [Header("Camera Source")]
    public CameraSource activeSource = CameraSource.WebcamDirect;

    [Header("Display")]
    public RawImage displayImage;

    [Header("Webcam Settings")]
    public string preferredWebcamName = "";
    public int webcamWidth  = 640;
    public int webcamHeight = 480;
    public int webcamFPS    = 30;

    [Header("RealSense ROS Settings")]
    public string rosImageTopic = "/camera/camera/color/image_raw";
    public int rosImageWidth  = 640;
    public int rosImageHeight = 480;

    [Header("Runtime Switch")]
    public Key toggleKey = Key.F1;     // New Input System Key enum

    [Header("Performance Monitoring")]
    public bool  showFPSInConsole = true;
    public float fpsLogInterval   = 2f;

    // ---------------------------------------------------------------
    // Private state
    // ---------------------------------------------------------------
    private WebCamTexture _webcamTexture;
    private Texture2D     _rosTexture;
    private bool          _rosSubscribed  = false;
    private CameraSource  _currentSource;

    private int   _frameCount     = 0;
    private float _fpsTimer       = 0f;
    private float _currentFPS     = 0f;
    private float _lastFrameTime  = 0f;
    private float _currentLatency = 0f;

    // ---------------------------------------------------------------
    // Unity lifecycle
    // ---------------------------------------------------------------
    void Start()
    {
        _currentSource = activeSource;
        ActivateSource(_currentSource);
        Debug.Log($"[CameraFeedManager] Started with source: {_currentSource}");
    }

    void Update()
    {
        // ── New Input System toggle ──
        if (Keyboard.current != null &&
            Keyboard.current[toggleKey].wasPressedThisFrame)
        {
            Debug.Log("[CameraFeedManager] F1 key detected");
            Toggle();
        }

        // ── Keep webcam texture live ──
        if (_currentSource == CameraSource.WebcamDirect &&
            _webcamTexture  != null &&
            _webcamTexture.isPlaying)
        {
            displayImage.texture = _webcamTexture;
        }

        // ── FPS logging timer (RealSense mode) ──
        if (_currentSource == CameraSource.RealSenseROS)
        {
            _fpsTimer += Time.deltaTime;
            if (_fpsTimer >= fpsLogInterval)
            {
                _currentFPS = _frameCount / _fpsTimer;

                if (_frameCount == 0)
                {
                    // No frames received yet — ROS probably not connected
                    Debug.LogWarning("[CameraFeedManager] RealSense: No frames received. " +
                                    "Is ROS2 running and the topic publishing?");
                }
                else
                {
                    Debug.Log($"[CameraFeedManager] ── RealSense Stream Stats ──");
                    Debug.Log($"[CameraFeedManager] Source:         Intel RealSense D435i");
                    Debug.Log($"[CameraFeedManager] Stream FPS:     {_currentFPS:F1} fps");
                    Debug.Log($"[CameraFeedManager] Frame interval: {_currentLatency:F1} ms");
                    Debug.Log($"[CameraFeedManager] Topic:          {rosImageTopic}");
                    Debug.Log($"[CameraFeedManager] ──────────────────────────");
                }

                _frameCount = 0;
                _fpsTimer   = 0f;
            }
        }
    }

    void OnDestroy()
    {
        StopWebcam();
    }

    // ---------------------------------------------------------------
    // Public API
    // ---------------------------------------------------------------
    public void SetSource(CameraSource source)
    {
        if (_currentSource == source) return;
        DeactivateCurrentSource();
        _currentSource  = source;
        activeSource    = source;       // keep Inspector in sync
        ActivateSource(_currentSource);
    }

    public void Toggle()
    {
        CameraSource next = _currentSource == CameraSource.WebcamDirect
            ? CameraSource.RealSenseROS
            : CameraSource.WebcamDirect;

        SetSource(next);
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
                Debug.Log("[CameraFeedManager] ── Activating: Webcam Direct ──");
                StartWebcam();
                break;

            case CameraSource.RealSenseROS:
                Debug.Log("[CameraFeedManager] ── Activating: Intel RealSense D435i (ROS2) ──");
                Debug.Log($"[CameraFeedManager] Subscribing to topic: {rosImageTopic}");
                Debug.Log("[CameraFeedManager] Waiting for frames... (will print FPS every " +
                        $"{fpsLogInterval}s once frames arrive)");
                StartRealSense();
                break;
        }
    }

    void DeactivateCurrentSource()
    {
        if (_currentSource == CameraSource.WebcamDirect)
            StopWebcam();
        // ROS subscription stays open — gated in callback
    }

    // ---------------------------------------------------------------
    // Webcam
    // ---------------------------------------------------------------
    void StartWebcam()
    {
        WebCamDevice[] devices = WebCamTexture.devices;

        if (devices.Length == 0)
        {
            Debug.LogWarning("[CameraFeedManager] No webcam found.");
            return;
        }

        // Log all available devices so you can check names
        foreach (var d in devices)
            Debug.Log($"[CameraFeedManager] Found webcam: {d.name}");

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
        displayImage.transform.localScale = new Vector3(1, 1, 1);

        Debug.Log($"[CameraFeedManager] Webcam started: {deviceName}");
        Debug.Log($"[CameraFeedManager] Requested FPS: {webcamFPS} fps " +
              $"| Resolution: {webcamWidth}x{webcamHeight}");

        // Log actual FPS after a short delay (webcam needs time to initialise)
        StartCoroutine(LogWebcamActualFPS());
    }

    
    private System.Collections.IEnumerator LogWebcamActualFPS()
    {
        // Wait 1 second for webcam to fully initialise
        yield return new WaitForSeconds(1f);

        if (_webcamTexture != null && _webcamTexture.isPlaying)
        {
            Debug.Log($"[CameraFeedManager] ── Webcam Actual Stats ──");
            Debug.Log($"[CameraFeedManager] Actual FPS:        {_webcamTexture.requestedFPS}");
            Debug.Log($"[CameraFeedManager] Actual Resolution: " +
                    $"{_webcamTexture.width}x{_webcamTexture.height}");
            Debug.Log($"[CameraFeedManager] Device name:       {_webcamTexture.deviceName}");
            Debug.Log($"[CameraFeedManager] ───────────────────────");
        }
    }

    void StopWebcam()
    {
        if (_webcamTexture != null && _webcamTexture.isPlaying)
        {
            _webcamTexture.Stop();
            _webcamTexture = null;
        }
        if (displayImage != null)
            displayImage.transform.localScale = Vector3.one;
    }

    // ---------------------------------------------------------------
    // RealSense via ROS2
    // ---------------------------------------------------------------
    void StartRealSense()
    {
        //Debug.Log($"[CameraFeedManager] Starting RealSense — topic: {rosImageTopic}");
        Debug.Log($"[CameraFeedManager] ── Activating: Intel RealSense D435i (ROS2) ──");
        Debug.Log($"[CameraFeedManager] Subscribing to topic: {rosImageTopic}");
        Debug.Log($"[CameraFeedManager] Waiting for frames... (will print FPS every " +
                $"{fpsLogInterval}s once frames arrive)");

        if (_rosTexture == null)
            _rosTexture = new Texture2D(rosImageWidth, rosImageHeight,
                                        TextureFormat.RGB24, false);

        // Show black frame while waiting for ROS
        displayImage.texture = _rosTexture;
        displayImage.transform.localScale = new Vector3(1, -1, 1);   // flip vertical only

        if (!_rosSubscribed)
        {
            ROSConnection.GetOrCreateInstance()
                .Subscribe<ImageMsg>(rosImageTopic, OnRosImageReceived);
            _rosSubscribed = true;
            Debug.Log("[CameraFeedManager] Subscribed to ROS topic.");
        }
        else
        {
            Debug.Log("[CameraFeedManager] Already subscribed — waiting for frames.");
        }

        // Reset FPS counters
        _frameCount    = 0;
        _fpsTimer      = 0f;
        _lastFrameTime = 0f;
    }

    void OnRosImageReceived(ImageMsg msg)
    {
        if (_currentSource != CameraSource.RealSenseROS) return;

        // FPS tracking
        float now = Time.realtimeSinceStartup;
        if (_lastFrameTime > 0f)
            _currentLatency = (now - _lastFrameTime) * 1000f;
        _lastFrameTime = now;
        _frameCount++;

        // Handle all common RealSense encodings
        byte[] pixels;
        switch (msg.encoding)
        {
            case "bgr8":
                Debug.Log("[CameraFeedManager] Encoding: bgr8 — converting to RGB");
                pixels = BGRtoRGB(msg.data);
                break;

            case "rgb8":
                // Already correct for Unity — no conversion needed
                pixels = msg.data;
                break;

            case "rgba8":
                pixels = RGBAtoRGB(msg.data);
                break;

            default:
                Debug.LogWarning($"[CameraFeedManager] Unknown encoding: {msg.encoding} " +
                                "— attempting raw load");
                pixels = msg.data;
                break;
        }

        // Resize texture if dimensions changed
        if (_rosTexture.width != (int)msg.width || 
            _rosTexture.height != (int)msg.height)
        {
            Debug.Log($"[CameraFeedManager] Resizing texture to {msg.width}x{msg.height}");
            _rosTexture.Reinitialize((int)msg.width, (int)msg.height);
        }

        _rosTexture.LoadRawTextureData(pixels);
        _rosTexture.Apply();
    }

    byte[] BGRtoRGB(byte[] bgr)
    {
        byte[] rgb = new byte[bgr.Length];
        for (int i = 0; i < bgr.Length; i += 3)
        {
            rgb[i]     = bgr[i + 2];
            rgb[i + 1] = bgr[i + 1];
            rgb[i + 2] = bgr[i];
        }
        return rgb;
    }

    byte[] RGBAtoRGB(byte[] rgba)
    {
        byte[] rgb = new byte[(rgba.Length / 4) * 3];
        int j = 0;
        for (int i = 0; i < rgba.Length; i += 4)
        {
            rgb[j++] = rgba[i];        // R
            rgb[j++] = rgba[i + 1];    // G
            rgb[j++] = rgba[i + 2];    // B
            // skip rgba[i + 3] — alpha
        }
        return rgb;
    }
    
}