using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using PimDeWitte.UnityMainThreadDispatcher;

public class EggMarkerSync : MonoBehaviour
{
    [SerializeField] public int eggID = 1;

    void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .Subscribe<StringMsg>("/perception/unity_sync", OnSync);
        Debug.Log("[EggMarkerSync] Egg " + eggID + " ready.");
    }

    void OnSync(StringMsg msg)
    {
        try
        {
            PuzzleWallSync.SyncPayload data =
                JsonUtility.FromJson<PuzzleWallSync.SyncPayload>(msg.data);
            if (data == null || data.eggs == null) return;

            foreach (var egg in data.eggs)
            {
                if (egg.id != eggID) continue;
                if (egg.status != "visible") return;

                UnityMainThreadDispatcher.Instance().Enqueue(() =>
                {
                    transform.position = new Vector3(
                        egg.position.x, egg.position.y, egg.position.z);
                    transform.rotation = new Quaternion(
                        egg.rotation.x, egg.rotation.y,
                        egg.rotation.z, egg.rotation.w);
                });
                return;
            }
        }
        catch (Exception e)
        {
            Debug.LogError("[EggMarkerSync] Error: " + e.Message);
        }
    }
}
