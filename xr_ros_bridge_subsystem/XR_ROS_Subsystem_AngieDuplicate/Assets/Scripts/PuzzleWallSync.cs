using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using PimDeWitte.UnityMainThreadDispatcher;

public class PuzzleWallSync : MonoBehaviour
{
    [SerializeField] string topic = "/perception/unity_sync";

    void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .Subscribe<StringMsg>(topic, OnSync);
        Debug.Log("[PuzzleWallSync] Subscribed to " + topic);
    }

    void OnSync(StringMsg msg)
    {
        try
        {
            SyncPayload data = JsonUtility.FromJson<SyncPayload>(msg.data);
            if (data == null || data.wall == null) return;
            if (data.wall.status != "OK") return;
            if (data.wall.confidence < 0.5f) return;

            UnityMainThreadDispatcher.Instance().Enqueue(() =>
            {
                transform.position = new Vector3(
                    data.wall.position.x,
                    data.wall.position.y,
                    data.wall.position.z);
                transform.rotation = new Quaternion(
                    data.wall.rotation.x,
                    data.wall.rotation.y,
                    data.wall.rotation.z,
                    data.wall.rotation.w);
            });
        }
        catch (Exception e)
        {
            Debug.LogError("[PuzzleWallSync] Error: " + e.Message);
        }
    }

    [Serializable] public class SyncPayload { public WallData wall; public EggData[] eggs; }
    [Serializable] public class WallData    { public string status; public float confidence; public Vec3 position; public Vec4 rotation; }
    [Serializable] public class EggData    { public int id; public string status; public Vec3 position; public Vec4 rotation; }
    [Serializable] public class Vec3       { public float x, y, z; }
    [Serializable] public class Vec4       { public float x, y, z, w; }
}
