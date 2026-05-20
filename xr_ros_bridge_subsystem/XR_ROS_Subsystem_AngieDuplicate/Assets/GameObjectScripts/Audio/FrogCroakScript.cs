//fixed number of croaks
// using UnityEngine;
// using System.Collections;

// public class FrogCroakScript : MonoBehaviour
// {
//     public AudioSource audioSource;
//     public int maxPlays = 3;
//     public float timeBetweenCroaks = 0f;
//     private bool hasPlayed = false;

//     public void PlaySound()
//     {
//         if (!hasPlayed && audioSource != null)
//         {
//             hasPlayed = true;
//             StartCoroutine(PlayMultipleTimes());
//         }
//     }

//     private IEnumerator PlayMultipleTimes()
//     {
//         for (int i = 0; i < maxPlays; i++)
//         {
//             audioSource.Play();
//             yield return new WaitForSeconds(audioSource.clip.length + timeBetweenCroaks);
//         }
//     }

//     public void ResetSound()
//     {
//         hasPlayed = false;
//         audioSource.Stop();
//     }
// }

// with ray interactor

// using UnityEngine;
// using System.Collections;

// public class FrogCroakScript : MonoBehaviour
// {
//     public AudioSource audioSource;
//     public float timeBetweenCroaks = 0f;

//     private bool hasPlayed = false;
//     private int maxPlays;

//     public void PlaySound()
//     {
//         if (!hasPlayed && audioSource != null)
//         {
//             hasPlayed = true;

//             // Random number between 1 and 9 (inclusive)
//             maxPlays = Random.Range(1, 10);

//             StartCoroutine(PlayMultipleTimes());
//         }
//     }

//     private IEnumerator PlayMultipleTimes()
//     {
//         for (int i = 0; i < maxPlays; i++)
//         {
//             audioSource.Play();
//             yield return new WaitForSeconds(audioSource.clip.length + timeBetweenCroaks);
//         }
//     }

//     public void ResetSound()
//     {
//         hasPlayed = false;
//         audioSource.Stop();
//     }
// }

//pressing g works once

// using UnityEngine;
// using System.Collections;
// using UnityEngine.InputSystem;

// public class FrogCroakScript : MonoBehaviour
// {
//     public AudioSource audioSource;
//     public float timeBetweenCroaks = 0f;
//     public InputActionReference triggerAction;

//     private bool hasPlayed = false;
//     private int maxPlays;

//     void OnEnable()
//     {
//         if (triggerAction != null)
//             triggerAction.action.performed += OnButtonPressed;
//     }

//     void OnDisable()
//     {
//         if (triggerAction != null)
//             triggerAction.action.performed -= OnButtonPressed;
//     }

//     private void OnButtonPressed(InputAction.CallbackContext context)
//     {
//         PlaySound();
//     }

//     public void PlaySound()
//     {
//         if (!hasPlayed && audioSource != null)
//         {
//             hasPlayed = true;
//             maxPlays = Random.Range(1, 10);
//             StartCoroutine(PlayMultipleTimes());
//         }
//     }

//     private IEnumerator PlayMultipleTimes()
//     {
//         for (int i = 0; i < maxPlays; i++)
//         {
//             audioSource.Play();
//             yield return new WaitForSeconds(audioSource.clip.length + timeBetweenCroaks);
//         }
//     }

//     public void ResetSound()
//     {
//         hasPlayed = false;
//         if (audioSource != null)
//             audioSource.Stop();
//     }
// }

// //replayability fixed number of croaks each game
// using UnityEngine;
// using System.Collections;
// using UnityEngine.InputSystem;

// public class FrogCroakScript : MonoBehaviour
// {
//     public AudioSource audioSource;
//     public float timeBetweenCroaks = 0f;
//     public InputActionReference triggerAction;

//     private int maxPlays;
//     private bool isPlaying = false;

//     void Start()
//     {
//         // Random number set once when game starts
//         maxPlays = Random.Range(1, 10);
//     }

//     void OnEnable()
//     {
//         if (triggerAction != null)
//             triggerAction.action.performed += OnButtonPressed;
//     }

//     void OnDisable()
//     {
//         if (triggerAction != null)
//             triggerAction.action.performed -= OnButtonPressed;
//     }

//     private void OnButtonPressed(InputAction.CallbackContext context)
//     {
//         if (!isPlaying && audioSource != null)
//         {
//             StartCoroutine(PlayMultipleTimes());
//         }
//     }

//     private IEnumerator PlayMultipleTimes()
//     {
//         isPlaying = true;
//         for (int i = 0; i < maxPlays; i++)
//         {
//             audioSource.Play();
//             yield return new WaitForSeconds(audioSource.clip.length + timeBetweenCroaks);
//         }
//         isPlaying = false;
//     }
// }

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

    void Start()
    {
        maxPlays = Random.Range(minCroaks, maxCroaks + 1);
        Debug.Log($"{gameObject.name} will croak {maxPlays} times (range {minCroaks}-{maxCroaks})");
        interactable = GetComponent<UnityEngine.XR.Interaction.Toolkit.Interactables.XRBaseInteractable>();
        if (interactable != null)
            interactable.selectEntered.AddListener(OnSelected);
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