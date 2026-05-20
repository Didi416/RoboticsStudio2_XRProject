using UnityEngine;
using System.Collections.Generic;

public class EggStandRandomiser : MonoBehaviour
{
    [Header("All possible spawn positions")]
    public Transform[] spawnPoints;  // empty GameObjects placed around the scene

    [Header("All egg stands")]
    public GameObject[] eggStands;   // your 4 stand objects (water, fire, etc)

    void Start()
    {
        RandomiseStandPositions();
    }

    void RandomiseStandPositions()
    {
        // Make sure we have enough spawn points
        if (spawnPoints.Length < eggStands.Length)
        {
            Debug.LogError("Not enough spawn points for all stands!");
            return;
        }

        // Shuffle the spawn points list
        List<Transform> shuffledPoints = new List<Transform>(spawnPoints);
        for (int i = shuffledPoints.Count - 1; i > 0; i--)
        {
            int randomIndex = Random.Range(0, i + 1);
            Transform temp = shuffledPoints[i];
            shuffledPoints[i] = shuffledPoints[randomIndex];
            shuffledPoints[randomIndex] = temp;
        }

        // Assign each stand to a shuffled position
        for (int i = 0; i < eggStands.Length; i++)
        {
            eggStands[i].transform.position = shuffledPoints[i].position;
            eggStands[i].transform.rotation = shuffledPoints[i].rotation;
            Debug.Log($"{eggStands[i].name} placed at {shuffledPoints[i].name}");
        }
    }
}