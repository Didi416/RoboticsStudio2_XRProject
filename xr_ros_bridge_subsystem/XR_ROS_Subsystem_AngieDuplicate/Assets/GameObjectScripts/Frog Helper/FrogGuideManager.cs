using UnityEngine;
using UnityEngine.UI;
using UnityEngine.InputSystem;
using TMPro;
using System.Collections;
using System.Collections.Generic;

public class FrogGuideManager : MonoBehaviour
{
    [Header("Input")]
    public InputActionReference helpButtonAction;
    public InputActionReference nextLineAction;
    public InputActionReference exitAction;

    [Header("UI References")]
    public GameObject guidePanel;
    public Image frogImage;
    public GameObject textBubble;
    public TextMeshProUGUI dialogueText;
    public TextMeshProUGUI speakerName;
    public Button nextButton;
    public Button exitButton;

    [Header("Hint Room Selection")]
    public GameObject roomSelectionPanel;
    public Button mainRoomButton;
    public Button puzzle1Button;
    public Button puzzle2Button;
    public Button puzzle3Button;
    public Button backButton;

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
    public float typewriterSpeed = 0.03f;
    public bool useTypewriterEffect = true;

    [Header("Player Reference")]
    public Transform playerCamera;
    

    // State
    private List<DialogueLine> currentSequence;
    private int currentLineIndex = 0;
    private bool isTyping = false;
    private bool guideOpen = false;
    private bool introPlayed = false;
    private Coroutine typewriterCoroutine;

    void LateUpdate()
    {
        if (guideOpen && guidePanel.activeSelf)
        {
            Vector3 targetPos = playerCamera.position
                + playerCamera.forward * 2f;

            guidePanel.transform.position = Vector3.Lerp(
                guidePanel.transform.position,
                targetPos,
                Time.deltaTime * 5f);

            guidePanel.transform.rotation = Quaternion.Lerp(
                guidePanel.transform.rotation,
                Quaternion.LookRotation(
                    guidePanel.transform.position - playerCamera.position),
                Time.deltaTime * 5f);
        }
    }

    void Start()
    {
        guidePanel.SetActive(false);
        roomSelectionPanel.SetActive(false);

        if (nextButton != null)
            nextButton.onClick.AddListener(OnNextPressed);
        if (exitButton != null)
            exitButton.onClick.AddListener(CloseGuide);
        if (mainRoomButton != null)
            mainRoomButton.onClick.AddListener(
                () => ShowHints(dialogueData.mainRoomLines));
        if (puzzle1Button != null)
            puzzle1Button.onClick.AddListener(
                () => ShowHints(dialogueData.puzzle1Hints));
        if (puzzle2Button != null)
            puzzle2Button.onClick.AddListener(
                () => ShowHints(dialogueData.puzzle2Hints));
        if (puzzle3Button != null)
            puzzle3Button.onClick.AddListener(
                () => ShowHints(dialogueData.puzzle3Hints));
        if (backButton != null)
            backButton.onClick.AddListener(OnBackPressed);

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
        if (guideOpen) CloseGuide();
        else OpenHelpMenu();
    }

    void OnNextLine(InputAction.CallbackContext ctx) => OnNextPressed();
    void OnExitPressed(InputAction.CallbackContext ctx) => CloseGuide();

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
        currentSequence = null;
        currentLineIndex = 0;
        isTyping = false;

        if (typewriterCoroutine != null)
        {
            StopCoroutine(typewriterCoroutine);
            typewriterCoroutine = null;
        }

        if (tts != null) tts.StopSpeaking();

        ShowSingleLine("Ribbit! Which puzzle do you need help with?", frogPointing);
    }

    void ShowHints(List<DialogueLine> hints)
    {
        if (hints == null || hints.Count == 0)
        {
            Debug.LogWarning("No hints available!");
            return;
        }
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

        if (tts != null && sequence.Count > 0)
            tts.Speak(sequence[0].text, sequence[0].audioClip);

        ShowCurrentLine();
    }

    void ShowCurrentLine()
    {
        if (currentSequence == null ||
            currentLineIndex >= currentSequence.Count)
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

    public void OnBackPressed()
    {
        if (typewriterCoroutine != null)
        {
            StopCoroutine(typewriterCoroutine);
            typewriterCoroutine = null;
        }

        if (tts != null) tts.StopSpeaking();

        currentSequence = null;
        currentLineIndex = 0;
        isTyping = false;

        if (dialogueText != null) dialogueText.text = "";

        roomSelectionPanel.SetActive(true);
        ShowSingleLine("Ribbit! Which puzzle do you need help with?", frogPointing);
    }

    public void OnNextPressed()
    {
        if (currentSequence == null)
        {
            Debug.LogWarning("Next pressed but no sequence active");
            return;
        }

        if (isTyping)
        {
            StopCoroutine(typewriterCoroutine);
            isTyping = false;
            dialogueText.text = currentSequence[currentLineIndex].text;
            return;
        }

        currentLineIndex++;

        if (currentLineIndex < currentSequence.Count)
        {
            if (tts != null)
                tts.Speak(
                    currentSequence[currentLineIndex].text,
                    currentSequence[currentLineIndex].audioClip);

            ShowCurrentLine();
        }
        else
        {
            OnSequenceEnd();
        }
    }

    void OnSequenceEnd()
    {
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

        if (tts != null) tts.StopSpeaking();
        if (typewriterCoroutine != null)
            StopCoroutine(typewriterCoroutine);
    }

    // ─────────────────────────────────────────
    // VICTORY
    // ─────────────────────────────────────────

    public void PlayVictory()
    {
        StartSequence(dialogueData.victoryLines, frogHappy);
        StartCoroutine(CelebrationEffect());
    }
    

    // ─────────────────────────────────────────
    // INCOMPLETE
    // ─────────────────────────────────────────

    public void PlayIncomplete()
    {
        StartSequence(dialogueData.incompleteLines, frogWorried);
    }

    // ─────────────────────────────────────────
    // CELEBRATION EFFECT
    // ─────────────────────────────────────────

    IEnumerator CelebrationEffect()
    {
        if (frogImage == null) yield break;

        Vector3 originalScale = frogImage.transform.localScale;

        for (int i = 0; i < 3; i++)
        {
            float elapsed = 0f;
            while (elapsed < 0.2f)
            {
                float t = Mathf.PingPong(elapsed * 10f, 1f);
                frogImage.transform.localScale = originalScale *
                    Mathf.Lerp(1f, 1.3f, t);
                elapsed += Time.deltaTime;
                yield return null;
            }
        }

        frogImage.transform.localScale = originalScale;
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




