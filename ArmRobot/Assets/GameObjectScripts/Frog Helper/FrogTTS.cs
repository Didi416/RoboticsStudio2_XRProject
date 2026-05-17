using UnityEngine;
using System.Collections;
using System.Runtime.InteropServices;

public class FrogTTS : MonoBehaviour
{
    [Header("TTS Settings")]
    public float speechRate = 0.9f;   // slightly slower for clarity
    public float pitch = 1.2f;        // slightly higher pitch for frog voice

    // [Header("Fallback Audio")]
    // public AudioSource audioSource;
    // public AudioClip frogSoundPrefix;  // short ribbit before each line

    [Header("Audio")]
    public AudioSource audioSource;
    public AudioClip frogSoundPrefix; // short ribbit before each line

    public bool isSpeaking = false;

    public void Speak(string text, AudioClip clip = null)
    {
        StopAllCoroutines();
        StartCoroutine(SpeakCoroutine(text, clip));
    }

    public void StopSpeaking()
    {
        StopAllCoroutines();
        isSpeaking = false;
        if (audioSource != null && audioSource.isPlaying)
            audioSource.Stop();
    }

    IEnumerator SpeakCoroutine(string text, AudioClip clip = null)
    {
        isSpeaking = true;
        Debug.Log($"SpeakCoroutine started. Clip: {(clip != null ? clip.name : "null")}, AudioSource: {(audioSource != null ? "assigned" : "NULL")}");

        if (audioSource != null && clip != null)
        {
            Debug.Log($"Playing clip: {clip.name}");
            audioSource.PlayOneShot(clip);
            yield return new WaitForSeconds(clip.length);
        }
        else if (audioSource != null && frogSoundPrefix != null)
        {
            Debug.Log($"Playing frog prefix: {frogSoundPrefix.name}");
            audioSource.PlayOneShot(frogSoundPrefix);
            yield return new WaitForSeconds(frogSoundPrefix.length);
        }
        else
        {
            Debug.LogWarning($"No audio played! AudioSource null: {audioSource == null}, clip null: {clip == null}, frogSoundPrefix null: {frogSoundPrefix == null}");
        }

        isSpeaking = false;
    }   

    #if UNITY_ANDROID && !UNITY_EDITOR
    void AndroidTTS(string text)
    {
        AndroidJavaClass unityPlayer = 
            new AndroidJavaClass("com.unity3d.player.UnityPlayer");
        AndroidJavaObject activity = 
            unityPlayer.GetStatic<AndroidJavaObject>("currentActivity");
        AndroidJavaObject tts = 
            new AndroidJavaObject("android.speech.tts.TextToSpeech", 
                activity, null);
        tts.Call<int>("speak", text, 0, null, null);
    }
    #endif

    #if UNITY_IOS && !UNITY_EDITOR
    [DllImport("__Internal")]
    private static extern void _Speak(string text, float rate, float pitch);

    void IosTTS(string text)
    {
        _Speak(text, speechRate, pitch);
    }
    #endif
}

