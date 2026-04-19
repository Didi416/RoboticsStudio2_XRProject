
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

using UnityEngine;
using System.Collections;

public class FrogCroakScript : MonoBehaviour
{
    public AudioSource audioSource;
    public float timeBetweenCroaks = 0f;

    private bool hasPlayed = false;
    private int maxPlays;

    public void PlaySound()
    {
        if (!hasPlayed && audioSource != null)
        {
            hasPlayed = true;

            // Random number between 1 and 9 (inclusive)
            maxPlays = Random.Range(1, 10);

            StartCoroutine(PlayMultipleTimes());
        }
    }

    private IEnumerator PlayMultipleTimes()
    {
        for (int i = 0; i < maxPlays; i++)
        {
            audioSource.Play();
            yield return new WaitForSeconds(audioSource.clip.length + timeBetweenCroaks);
        }
    }

    public void ResetSound()
    {
        hasPlayed = false;
        audioSource.Stop();
    }
}

