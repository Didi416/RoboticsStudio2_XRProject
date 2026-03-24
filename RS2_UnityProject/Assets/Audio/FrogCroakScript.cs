using UnityEngine;

public class FrogCroakScript : MonoBehaviour
{
    public AudioSource audioSource;
    public int maxPlays = 3;

    private int playCount = 0;

    public void PlaySound()
    {
        if (playCount < maxPlays && audioSource != null)
        {
            audioSource.Play();
            playCount++;
        }
    }
}

