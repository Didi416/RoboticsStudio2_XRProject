using UnityEngine;
using System.Collections;
using System.Runtime.InteropServices;

public class FrogTTS : MonoBehaviour
{
    [Header("TTS Settings")]
    public float speechRate = 0.9f;   // slightly slower for clarity
    public float pitch = 1.2f;        // slightly higher pitch for frog voice

    [Header("Fallback Audio")]
    public AudioSource audioSource;
    public AudioClip frogSoundPrefix;  // short ribbit before each line

    public bool isSpeaking = false;

    public void Speak(string text)
    {
        StopAllCoroutines();
        StartCoroutine(SpeakCoroutine(text));
    }

    public void StopSpeaking()
    {
        StopAllCoroutines();
        isSpeaking = false;

        // Stop any playing audio
        if (audioSource != null && audioSource.isPlaying)
            audioSource.Stop();
    }

    IEnumerator SpeakCoroutine(string text)
    {
        isSpeaking = true;
        Debug.Log($"TTS SpeakCoroutine started: {text}");

        if (audioSource != null && frogSoundPrefix != null)
        {
            Debug.Log("Playing frog sound prefix");
            audioSource.PlayOneShot(frogSoundPrefix);
            yield return new WaitForSeconds(frogSoundPrefix.length);
        }
        else
        {
            Debug.LogWarning($"Audio Source null: {audioSource == null}, Frog Sound null: {frogSoundPrefix == null}");
        }

        Debug.Log("TTS speak finished");
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