// Subscribes to /aruco/marker_N/pose for each egg and moves that egg's GameObject

// Attach one copy of this script to each egg GameObject in Unity (Egg_Green, Egg_Blue, 
// Egg_White, Egg_Purple), and set markerID to 1, 2, 3, 4 respectively via the Inspector.


using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class EggMarkerSync : MonoBehaviour
{
    [SerializeField] int eggID = 1;  // Set to 1, 2, 3, or 4 in Inspector

    void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .Subscribe<StringMsg>("/perception/unity_sync", OnSync);
    }

    void OnSync(StringMsg msg)
    {
        SyncPayload data = JsonUtility.FromJson<SyncPayload>(msg.data);
        if (data.eggs == null) return;

        foreach (var egg in data.eggs)
        {
            if (egg.id != eggID) continue;
            if (egg.status != "visible") return;

            transform.position = new Vector3(
                egg.position.x,
                egg.position.y,
                egg.position.z);
            transform.rotation = new Quaternion(
                egg.rotation.x,
                egg.rotation.y,
                egg.rotation.z,
                egg.rotation.w);
            return;
        }
    }

    [Serializable] class SyncPayload { public WallData wall; public EggData[] eggs; }
    [Serializable] class WallData { public string status; }
    [Serializable] class EggData {
        public int id;
        public string status;
        public Vec3 position;
        public Vec4 rotation;
    }
    [Serializable] class Vec3 { public float x, y, z; }
    [Serializable] class Vec4 { public float x, y, z, w; }
}