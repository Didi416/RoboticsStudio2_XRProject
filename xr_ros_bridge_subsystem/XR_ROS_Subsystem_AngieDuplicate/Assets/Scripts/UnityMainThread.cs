// UnityMainThread.cs
// ─────────────────────────────────────────────────────────────────────────────
// Tiny dispatcher that lets background threads (e.g. ROS TCP subscriber
// callbacks) safely execute code on Unity's main thread.
//
// Usage:
//   UnityMainThread.Enqueue(() => myText.text = "hello");
//
// Place this file at:
//   Assets/Scripts/Utils/UnityMainThread.cs
//
// Add this component to the same "ROSManager" GameObject as PuzzleROSManager.
// ─────────────────────────────────────────────────────────────────────────────

using System;
using System.Collections.Generic;
using UnityEngine;

namespace EscapeXRtist.ROS
{
    public class UnityMainThread : MonoBehaviour
    {
        private static readonly Queue<Action> _queue = new Queue<Action>();
        private static UnityMainThread        _instance;

        private void Awake()
        {
            _instance = this;
        }

        private void Update()
        {
            lock (_queue)
            {
                while (_queue.Count > 0)
                    _queue.Dequeue()?.Invoke();
            }
        }

        /// <summary>Enqueue an action to run on the Unity main thread next Update().</summary>
        public static void Enqueue(Action action)
        {
            lock (_queue)
            {
                _queue.Enqueue(action);
            }
        }
    }
}