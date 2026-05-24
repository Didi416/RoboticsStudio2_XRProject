using UnityEngine;

public class CardObject : MonoBehaviour
{
    [Header("Card Settings")]
    public string cardID = "MainCard";

    void Start()
    {
        Debug.Log($"Card {cardID} ready");
    }
}