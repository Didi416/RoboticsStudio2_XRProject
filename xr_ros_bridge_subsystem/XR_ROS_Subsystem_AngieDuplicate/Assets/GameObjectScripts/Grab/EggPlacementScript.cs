using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using System.Collections;

public class EggPlacementScript : MonoBehaviour
{
    [Header("Audio")]
    public AudioSource audioSource;
    public AudioSource grabAudioSource; // separate audio source for looping grab sound
    public AudioClip correctSound;
    public AudioClip wrongSound;
    public AudioClip droppedSound;
    public AudioClip grabbingSound; // looping sound while held

    [Header("Sockets")]
    public UnityEngine.XR.Interaction.Toolkit.Interactors.XRSocketInteractor correctSocket;

    [Header("Visuals")]
    public Renderer objectRenderer;
    public Color wrongColor = Color.red;
    public Color correctColor = Color.green;
    public float effectDuration = 2f;

    private UnityEngine.XR.Interaction.Toolkit.Interactables.XRGrabInteractable grabInteractable;
    private Color originalColor;

    // void Start()
    // {
    //     grabInteractable = GetComponent<UnityEngine.XR.Interaction.Toolkit.Interactables.XRGrabInteractable>();
    //     grabInteractable.selectEntered.AddListener(OnGrabbed);
    //     grabInteractable.selectExited.AddListener(OnReleased);

    //     if (objectRenderer != null)
    //         originalColor = objectRenderer.material.color;
    // }
    void Start()
    {
        grabInteractable = GetComponent<UnityEngine.XR.Interaction.Toolkit.Interactables.XRGrabInteractable>();
        
        if (grabInteractable == null)
        {
            Debug.LogError("XR Grab Interactable not found on " + gameObject.name);
            return;
        }
        
        grabInteractable.selectEntered.AddListener(OnGrabbed);
        grabInteractable.selectExited.AddListener(OnReleased);

        if (objectRenderer != null)
            originalColor = objectRenderer.material.color;
    }

    // private void OnGrabbed(SelectEnterEventArgs args)
    // {
    //     if (grabAudioSource != null && grabbingSound != null)
    //     {
    //         grabAudioSource.clip = grabbingSound;
    //         grabAudioSource.loop = true;
    //         grabAudioSource.Play();
    //     }
    // }

    // private void OnReleased(SelectExitEventArgs args)
    // {
    //     // Stop grab sound
    //     if (grabAudioSource != null)
    //     {
    //         grabAudioSource.loop = false;
    //         grabAudioSource.Stop();
    //     }

    //     StartCoroutine(CheckPlacement());
    // }
    private void OnGrabbed(SelectEnterEventArgs args)
    {
        // Only play grab sound if grabbed by a CONTROLLER not a socket
        if (args.interactorObject is UnityEngine.XR.Interaction.Toolkit.Interactors.XRSocketInteractor)
            return;

        if (grabAudioSource != null && grabbingSound != null)
        {
            grabAudioSource.clip = grabbingSound;
            grabAudioSource.loop = true;
            grabAudioSource.Play();
        }
    }

    private void OnReleased(SelectExitEventArgs args)
    {
        // Always stop grab sound on any release
        if (grabAudioSource != null)
        {
            grabAudioSource.loop = false;
            grabAudioSource.Stop();
        }

        StartCoroutine(CheckPlacement());
    }

    private IEnumerator CheckPlacement()
    {
        yield return new WaitForSeconds(0.1f);

        UnityEngine.XR.Interaction.Toolkit.Interactors.XRSocketInteractor currentSocket = GetCurrentSocket();

        if (currentSocket == null)
        {
            if (IsNearWrongStand())
            {
                PlaySound(wrongSound);
                StartCoroutine(GlowEffect(wrongColor));
            }
            else
            {
                PlaySound(droppedSound);
            }
        }
        else if (currentSocket == correctSocket)
        {
            PlaySound(correctSound);
            StartCoroutine(GlowEffect(correctColor));
        }
    }

    private UnityEngine.XR.Interaction.Toolkit.Interactors.XRSocketInteractor GetCurrentSocket()
    {
        UnityEngine.XR.Interaction.Toolkit.Interactors.XRSocketInteractor[] allSockets = FindObjectsOfType<UnityEngine.XR.Interaction.Toolkit.Interactors.XRSocketInteractor>();
        foreach (var socket in allSockets)
        {
            if (socket.hasSelection)
            {
                foreach (var interactable in socket.interactablesSelected)
                {
                    if (interactable.transform == this.transform)
                        return socket;
                }
            }
        }
        return null;
    }

    private bool IsNearWrongStand()
    {
        UnityEngine.XR.Interaction.Toolkit.Interactors.XRSocketInteractor[] allSockets = FindObjectsOfType<UnityEngine.XR.Interaction.Toolkit.Interactors.XRSocketInteractor>();
        foreach (var socket in allSockets)
        {
            if (socket != correctSocket)
            {
                float distance = Vector3.Distance(
                    this.transform.position,
                    socket.transform.position
                );
                if (distance < 0.5f)
                    return true;
            }
        }
        return false;
    }

    private IEnumerator GlowEffect(Color glowColor)
    {
        if (objectRenderer != null)
        {
            objectRenderer.material.color = glowColor;
            yield return new WaitForSeconds(effectDuration);
            objectRenderer.material.color = originalColor;
        }
    }

    private void PlaySound(AudioClip clip)
    {
        if (audioSource != null && clip != null)
        {
            audioSource.clip = clip;
            audioSource.Play();
        }
    }
}