// Subscribes to /perception/unity_sync and moves  puzzle wall GameObject
// Attach this to puzzle wall parent GameObject in the Unity scene

using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class PuzzleWallSync : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .Subscribe<StringMsg>("/perception/unity_sync", OnSync);
    }

    void OnSync(StringMsg msg)
    {
        SyncPayload data = JsonUtility.FromJson<SyncPayload>(msg.data);

        if (data.wall.status != "OK") return;
        if (data.wall.confidence < 0.5f) return;

        transform.position = new Vector3(
            data.wall.position.x,
            data.wall.position.y,
            data.wall.position.z);
        transform.rotation = new Quaternion(
            data.wall.rotation.x,
            data.wall.rotation.y,
            data.wall.rotation.z,
            data.wall.rotation.w);
    }

    // ── JSON schema matching unity_pose_publisher output ──
    [Serializable] class SyncPayload { public WallData wall; public EggData[] eggs; }
    [Serializable] class WallData {
        public string status;
        public float confidence;
        public Vec3 position;
        public Vec4 rotation;
    }
    [Serializable] class Vec3 { public float x, y, z; }
    [Serializable] class Vec4 { public float x, y, z, w; }
    [Serializable] class EggData { public int id; public string status; public Vec3 position; public Vec4 rotation; }
}