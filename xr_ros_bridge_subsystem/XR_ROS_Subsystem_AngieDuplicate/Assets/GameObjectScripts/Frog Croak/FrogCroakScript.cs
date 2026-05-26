using UnityEngine;
using System.Collections;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.InputSystem;

public class FrogCroakScript : MonoBehaviour
{
    public AudioSource audioSource;
    public float timeBetweenCroaks = 0f;

    [Header("Croak Range")]
    public int minCroaks = 1;
    public int maxCroaks = 9;

    [Header("Debug Testing")]
    public Key testKey = Key.G;
    
    private int maxPlays;
    private bool isPlaying = false;
    private UnityEngine.XR.Interaction.Toolkit.Interactables.XRBaseInteractable interactable;

    void Awake()
    {
        maxPlays = Random.Range(minCroaks, maxCroaks + 1);
        Debug.Log($"{gameObject.name} croak count set to: {maxPlays}");
    }

    void Start()
    {
       // maxPlays = Random.Range(minCroaks, maxCroaks + 1);
        Debug.Log($"{gameObject.name} will croak {maxPlays} times (range {minCroaks}-{maxCroaks})");
        interactable = GetComponent<UnityEngine.XR.Interaction.Toolkit.Interactables.XRBaseInteractable>();
        if (interactable != null)
            interactable.selectEntered.AddListener(OnSelected);
    }

    public int GetCroakCount()
    {
        return maxPlays;
    }

    public void SetCroakCount(int count)
    {
        maxPlays = Mathf.Clamp(count, 1, 9);
        Debug.Log($"{gameObject.name} croak count synced to: {maxPlays}");
    }

    void OnDestroy()
    {
        if (interactable != null)
            interactable.selectEntered.RemoveListener(OnSelected);
    }

    void Update()
    {
        if (Keyboard.current != null && Keyboard.current[testKey].wasPressedThisFrame)
        {
            TriggerCroaks();
        }
    }

    private void OnSelected(SelectEnterEventArgs args)
    {
        TriggerCroaks();
    }

    private void TriggerCroaks()
    {
        if (!isPlaying && audioSource != null)
        {
            StartCoroutine(PlayMultipleTimes());
        }
    }

    private IEnumerator PlayMultipleTimes()
    {
        isPlaying = true;
        for (int i = 0; i < maxPlays; i++)
        {
            audioSource.Play();
            yield return new WaitForSeconds(audioSource.clip.length + timeBetweenCroaks);
        }
        isPlaying = false;
    }
}