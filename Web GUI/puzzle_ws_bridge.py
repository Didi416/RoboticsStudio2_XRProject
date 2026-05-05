"""
puzzle_ws_bridge.py
─────────────────────────────────────────────────────────────────────────────
Lightweight WebSocket bridge between the HTML GUI and ROS 2.
Uses Python's built-in asyncio + websockets (no rosbridge_suite required).

The GUI sends:  {"op":"publish","topic":"/puzzle_goal","msg":{"data":"puzzle1"}}
The bridge returns: {"topic":"/puzzle_status","msg":{"data":"moving to puzzle1"}}
              and:  {"topic":"/puzzle_busy","msg":{"data": true}}

Install once:
  pip3 install websockets --break-system-packages

Run alongside puzzle_pose_server.py:
  python3 puzzle_ws_bridge.py
"""

import asyncio
import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool

WS_PORT = 9090     # same default port rosbridge_suite uses

try:
    import websockets
    WS_AVAILABLE = True
except ImportError:
    WS_AVAILABLE = False
    print("[WARN] websockets not installed.  Run:  pip3 install websockets --break-system-packages")


class RosBridge(Node):
    """Thin ROS 2 node that forwards topics to/from connected WebSocket clients."""

    def __init__(self):
        super().__init__("puzzle_ws_bridge")

        self._clients: set = set()
        self._lock = threading.Lock()

        # Publisher: sends goal names to the pose server
        self._goal_pub = self.create_publisher(String, "/puzzle_goal", 10)

        # Subscribers: forward ROS status back to GUI
        self.create_subscription(String, "/puzzle_status", self._on_status, 10)
        self.create_subscription(Bool,   "/puzzle_busy",   self._on_busy,   10)

        self.get_logger().info(f"RosBridge node ready.  WS port: {WS_PORT}")

    # ── ROS → WebSocket ───────────────────────────────────────────────────────

    def _on_status(self, msg: String):
        self._broadcast({"topic": "/puzzle_status", "msg": {"data": msg.data}})

    def _on_busy(self, msg: Bool):
        self._broadcast({"topic": "/puzzle_busy", "msg": {"data": msg.data}})

    def _broadcast(self, payload: dict):
        text = json.dumps(payload)
        with self._lock:
            clients = set(self._clients)
        for ws in clients:
            asyncio.run_coroutine_threadsafe(ws.send(text), self._loop)

    # ── WebSocket → ROS ───────────────────────────────────────────────────────

    async def _ws_handler(self, websocket):
        with self._lock:
            self._clients.add(websocket)
        self.get_logger().info(f"Client connected: {websocket.remote_address}")
        try:
            async for raw in websocket:
                try:
                    pkt = json.loads(raw)
                    if pkt.get("op") == "publish" and pkt.get("topic") == "/puzzle_goal":
                        data = pkt.get("msg", {}).get("data", "")
                        if data:
                            self._goal_pub.publish(String(data=data))
                            self.get_logger().info(f"Forwarded goal: {data}")
                except json.JSONDecodeError:
                    pass
        except Exception:
            pass
        finally:
            with self._lock:
                self._clients.discard(websocket)
            self.get_logger().info("Client disconnected.")

    # ── Start WebSocket server ─────────────────────────────────────────────────

    def start_ws_server(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop

        async def _serve():
            async with websockets.serve(self._ws_handler, "0.0.0.0", WS_PORT):
                self.get_logger().info(f"WebSocket server listening on ws://0.0.0.0:{WS_PORT}")
                await asyncio.Future()   # run forever

        loop.run_until_complete(_serve())


def main():
    if not WS_AVAILABLE:
        print("Install websockets first:  pip3 install websockets --break-system-packages")
        return

    rclpy.init()
    node = RosBridge()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Spin ROS in a background thread
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Run asyncio WebSocket server in main thread
    loop = asyncio.new_event_loop()
    try:
        node.start_ws_server(loop)
    except KeyboardInterrupt:
        print("\nShutting down bridge.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()