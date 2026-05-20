using UnityEngine;
using System.Collections.Generic;

public class EggRandomiser : MonoBehaviour
{
    [Header("All possible egg spawn positions")]
    public Transform[] eggSpawnPoints;

    [Header("All eggs")]
    public GameObject[] eggs;

    void Start()
    {
        RandomiseEggPositions();
    }

    void RandomiseEggPositions()
    {
        if (eggSpawnPoints.Length < eggs.Length)
        {
            Debug.LogError($"Not enough egg spawn points! Have {eggSpawnPoints.Length} but need {eggs.Length}");
            return;
        }

        List<Transform> availablePoints = new List<Transform>(eggSpawnPoints);

        for (int i = 0; i < eggs.Length; i++)
        {
            int randomIndex = Random.Range(0, availablePoints.Count);
            Transform chosenPoint = availablePoints[randomIndex];

            eggs[i].transform.position = chosenPoint.position;
            eggs[i].transform.rotation = chosenPoint.rotation;

            Debug.Log($"{eggs[i].name} → {chosenPoint.name}");

            availablePoints.RemoveAt(randomIndex);
        }
    }
}
