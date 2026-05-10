using UnityEngine;
using UnityEngine.UI;
using UnityEngine.InputSystem;
using TMPro;
using System.Collections;
using System.Collections.Generic;

public class FrogGuideManager : MonoBehaviour
{
    [Header("Input")]
    public InputActionReference helpButtonAction;  // controller help button
    public InputActionReference nextLineAction;    // trigger to advance dialogue
    public InputActionReference exitAction;        // X button to exit

    [Header("UI References")]
    public GameObject guidePanel;          // the whole bottom panel
    public Image frogImage;                // frog character image
    public GameObject textBubble;          // speech bubble background
    public TextMeshProUGUI dialogueText;   // main text
    public TextMeshProUGUI speakerName;    // "Lily the Frog"
    public Button nextButton;
    public Button exitButton;

    [Header("Hint Room Selection")]
    public GameObject roomSelectionPanel;  // shows after help pressed in puzzle
    public Button mainRoomButton;
    public Button puzzle1Button;
    public Button puzzle2Button;
    public Button puzzle3Button;

    [Header("Frog Images")]
    public Sprite frogNeutral;
    public Sprite frogHappy;
    public Sprite frogWorried;
    public Sprite frogPointing;

    [Header("Dialogue Data")]
    public FrogDialogue dialogueData;

    [Header("TTS")]
    public FrogTTS tts;

    [Header("Settings")]
    public float typewriterSpeed = 0.03f;  // seconds per character
    public bool useTypewriterEffect = true;

    [Header("Player Reference")]
    public Transform playerCamera; // drag in your Main Camera

    void LateUpdate()
    {
        if (guideOpen && guidePanel.activeSelf)
        {
            // Position panel in front of player at all times
            Vector3 targetPos = playerCamera.position 
                + playerCamera.forward * 2f  // 2 metres in front
                + Vector3.down * 0.5f;       // slightly below eye level
            
            guidePanel.transform.position = Vector3.Lerp(
                guidePanel.transform.position, 
                targetPos, 
                Time.deltaTime * 5f); // smooth follow
            
            // Always face the player
            guidePanel.transform.rotation = Quaternion.Lerp(
                guidePanel.transform.rotation,
                Quaternion.LookRotation(
                    guidePanel.transform.position - playerCamera.position),
                Time.deltaTime * 5f);
        }
    }

    // State
    private List<DialogueLine> currentSequence;
    private int currentLineIndex = 0;
    private bool isTyping = false;
    private bool guideOpen = false;
    private bool introPlayed = false;
    private Coroutine typewriterCoroutine;

    void Start()
    {
        // Hide everything at start
        guidePanel.SetActive(false);
        roomSelectionPanel.SetActive(false);

        // Hook up buttons
        if (nextButton != null)
            nextButton.onClick.AddListener(OnNextPressed);
        if (exitButton != null)
            exitButton.onClick.AddListener(CloseGuide);
        if (mainRoomButton != null)
            mainRoomButton.onClick.AddListener(() => ShowHints(dialogueData.mainRoomLines));
        if (puzzle1Button != null)
            puzzle1Button.onClick.AddListener(() => ShowHints(dialogueData.puzzle1Hints));
        if (puzzle2Button != null)
            puzzle2Button.onClick.AddListener(() => ShowHints(dialogueData.puzzle2Hints));
        if (puzzle3Button != null)
            puzzle3Button.onClick.AddListener(() => ShowHints(dialogueData.puzzle3Hints));

        // Auto play intro after short delay
        Invoke("PlayIntro", 1.5f);
    }

    void OnEnable()
    {
        if (helpButtonAction != null)
            helpButtonAction.action.performed += OnHelpPressed;
        if (nextLineAction != null)
            nextLineAction.action.performed += OnNextLine;
        if (exitAction != null)
            exitAction.action.performed += OnExitPressed;
    }

    void OnDisable()
    {
        if (helpButtonAction != null)
            helpButtonAction.action.performed -= OnHelpPressed;
        if (nextLineAction != null)
            nextLineAction.action.performed -= OnNextLine;
        if (exitAction != null)
            exitAction.action.performed -= OnExitPressed;
    }

    // ─────────────────────────────────────────
    // INPUT HANDLERS
    // ─────────────────────────────────────────

    void OnHelpPressed(InputAction.CallbackContext ctx)
    {
        if (guideOpen)
            CloseGuide();
        else
            OpenHelpMenu();
    }

    void OnNextLine(InputAction.CallbackContext ctx)
    {
        OnNextPressed();
    }

    void OnExitPressed(InputAction.CallbackContext ctx)
    {
        CloseGuide();
    }

    // ─────────────────────────────────────────
    // GUIDE FLOW
    // ─────────────────────────────────────────

    public void PlayIntro()
    {
        if (introPlayed) return;
        introPlayed = true;
        StartSequence(dialogueData.introLines, frogHappy);
    }

    void OpenHelpMenu()
    {
        guidePanel.SetActive(true);
        roomSelectionPanel.SetActive(true);
        guideOpen = true;

        // Show frog asking what help is needed
        ShowSingleLine("Ribbit! Which puzzle do you need help with?", frogPointing);
    }

    void ShowHints(List<DialogueLine> hints)
    {
        roomSelectionPanel.SetActive(false);
        StartSequence(hints, frogPointing);
    }

    void StartSequence(List<DialogueLine> sequence, Sprite frogSprite)
    {
        currentSequence = sequence;
        currentLineIndex = 0;
        guidePanel.SetActive(true);
        guideOpen = true;

        if (frogImage != null && frogSprite != null)
            frogImage.sprite = frogSprite;

        // Speak first line immediately
        if (tts != null && sequence.Count > 0)
            tts.Speak(sequence[0].text);

        ShowCurrentLine();
    }

    void ShowCurrentLine()
    {
        if (currentSequence == null || currentLineIndex >= currentSequence.Count)
        {
            OnSequenceEnd();
            return;
        }

        DialogueLine line = currentSequence[currentLineIndex];

        if (useTypewriterEffect)
        {
            if (typewriterCoroutine != null)
                StopCoroutine(typewriterCoroutine);
            typewriterCoroutine = StartCoroutine(TypewriterEffect(line.text));
        }
        else
        {
            dialogueText.text = line.text;
        }
    }

    void ShowSingleLine(string text, Sprite frogSprite)
    {
        if (frogImage != null && frogSprite != null)
            frogImage.sprite = frogSprite;

        if (useTypewriterEffect)
        {
            if (typewriterCoroutine != null)
                StopCoroutine(typewriterCoroutine);
            typewriterCoroutine = StartCoroutine(TypewriterEffect(text));
        }
        else
        {
            dialogueText.text = text;
        }
    }

    public void OnNextPressed()
    {
        Debug.Log($"Next pressed - isTyping: {isTyping}, lineIndex: {currentLineIndex}");

        if (isTyping)
        {
            StopCoroutine(typewriterCoroutine);
            isTyping = false;
            dialogueText.text = currentSequence[currentLineIndex].text;
            return;
        }

        currentLineIndex++;

        if (currentSequence != null && currentLineIndex < currentSequence.Count)
        {
            Debug.Log($"Showing line {currentLineIndex}: {currentSequence[currentLineIndex].text}");

            if (tts != null)
            {
                Debug.Log("Calling TTS speak");
                tts.Speak(currentSequence[currentLineIndex].text);
            }
            else
            {
                Debug.LogError("TTS is null! Drag FrogTTS object into TTS slot in Inspector");
            }

            ShowCurrentLine();
        }
        else
        {
            Debug.Log("Sequence ended");
            OnSequenceEnd();
        }
    }

    void OnSequenceEnd()
    {
        // After intro, close and let player explore
        StartCoroutine(CloseAfterDelay(1.5f));
    }

    IEnumerator CloseAfterDelay(float delay)
    {
        yield return new WaitForSeconds(delay);
        CloseGuide();
    }

    public void CloseGuide()
    {
        guideOpen = false;
        guidePanel.SetActive(false);
        roomSelectionPanel.SetActive(false);

        if (tts != null)
            tts.StopSpeaking();

        if (typewriterCoroutine != null)
            StopCoroutine(typewriterCoroutine);
    }

    // ─────────────────────────────────────────
    // TYPEWRITER EFFECT
    // ─────────────────────────────────────────

    IEnumerator TypewriterEffect(string text)
    {
        isTyping = true;
        dialogueText.text = "";

        foreach (char c in text)
        {
            dialogueText.text += c;
            yield return new WaitForSeconds(typewriterSpeed);
        }

        isTyping = false;
    }
}