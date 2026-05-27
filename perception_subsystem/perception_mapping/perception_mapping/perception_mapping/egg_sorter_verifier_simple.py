#!/usr/bin/env python3
# """
# egg_sorter_verifier_simple.py  —  Puzzle 3 simple X-ordering verification.

# NO depth camera required. NO TF tree required. NO slot calibration required.
# Works with a plain webcam and the existing multi_aruco_detector.py as-is.

# How it works
# ------------
# multi_aruco_detector already publishes:
#     /aruco/marker_<ID>/pose   (geometry_msgs/PoseStamped)
#                                frame_id = 'camera_color_optical_frame'

# In the camera optical frame, the X axis points RIGHT across the image.
# So pose.position.x directly gives the left-to-right position of each egg.

# This node:
#   1. Subscribes to /aruco/marker_<ID>/pose for each egg ID
#   2. Collects whichever eggs are currently visible
#   3. Sorts them by pose.position.x  (most negative = leftmost)
#   4. Compares the sorted ID sequence against the expected sequence
#   5. Publishes the result on /puzzle3/puzzle_solved and /puzzle3/status

# Camera frame reminder (OpenCV / RealSense optical convention)
# -------------------------------------------------------------
#     +X → right across image
#     +Y → down the image
#     +Z → into the scene (depth)

# So sorting by X ascending gives left-to-right order as seen by the camera.

# What you need to change
# -----------------------
#   EXPECTED_SEQUENCE  — the correct left-to-right order of egg ArUco IDs
#   EGG_ARUCO_IDS      — which ID maps to which colour label
#   EGG_PUZZLE_IDS     — which IDs belong to Puzzle 3 (subset of PUZZLE_MARKERS)

# What you do NOT need to change in multi_aruco_detector.py
# ---------------------------------------------------------
#   Nothing. This node subscribes to the topics it already publishes.
#   The only thing to confirm is that your egg IDs are NOT already used
#   by other puzzle markers (IDs 0,1,2,3,10 are taken — use 11,12,13,14).

# Running alongside the existing detector (webcam)
# -------------------------------------------------
#   Terminal 1 — start the webcam bridge (publishes /camera/color/image_raw):
#     ros2 run perception_mapping webcam_realsense_bridge

#   Terminal 2 — start the detector (no depth remapping needed for webcam):
#     ros2 run perception_mapping multi_aruco_detector

#   Terminal 3 — start this verifier:
#     ros2 run perception_mapping egg_sorter_verifier_simple

#   Monitor results:
#     ros2 topic echo /puzzle3/status
#     ros2 topic echo /puzzle3/puzzle_solved
# """

# import json

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Bool, String


# # ─────────────────────────────────────────────────────────────────────────────
# # CONFIGURATION — edit these to match your printed egg markers
# # ─────────────────────────────────────────────────────────────────────────────

# # ArUco IDs printed on each egg block (must not clash with IDs 0,1,2,3,10)
# EGG_ARUCO_IDS: dict[int, str] = {
#     1: "white",
#     2: "green",
#     3: "purple",
#     4: "blue",
# }

# # The correct left-to-right order of egg IDs as seen by the camera.
# # Example: green(11) must be leftmost, then blue(12), yellow(13), purple(14) rightmost.
# # Change this to match your puzzle's correct answer for the current seed/run.
# EXPECTED_SEQUENCE: list[int] = [1, 2, 3, 4]

# # How many eggs must be visible before we attempt a verdict.
# # Set to len(EXPECTED_SEQUENCE) to require all eggs present.
# MIN_EGGS_VISIBLE: int = len(EXPECTED_SEQUENCE)

# # Minimum X separation (metres in camera frame) between adjacent eggs.
# # Eggs closer than this are considered "ambiguous" and the check is skipped.
# # In camera frame units ~0.01 m (1 cm) is a safe minimum for desktop distances.
# MIN_X_SEPARATION_M: float = 0.01

# # Publish rate in Hz.
# PUBLISH_RATE_HZ: float = 5.0

# # ─────────────────────────────────────────────────────────────────────────────


# class EggSorterVerifierSimple(Node):
#     """
#     Subscribes to /aruco/marker_<ID>/pose topics published by
#     multi_aruco_detector and verifies Puzzle 3 by sorting detected
#     egg markers by their camera-frame X coordinate (left-to-right).

#     Published topics
#     ----------------
#     /puzzle3/puzzle_solved   std_msgs/Bool    True when order matches
#     /puzzle3/status          std_msgs/String  human-readable summary
#     /puzzle3/egg_state       std_msgs/String  JSON detail for SS4
#     """

#     def __init__(self) -> None:
#         super().__init__("egg_sorter_verifier_simple")

#         # ── Parameters ────────────────────────────────────────────────────────
#         self.declare_parameter("publish_rate_hz", PUBLISH_RATE_HZ)
#         self.declare_parameter("min_x_separation_m", MIN_X_SEPARATION_M)
#         rate_hz         = self.get_parameter("publish_rate_hz").value
#         self._min_sep   = self.get_parameter("min_x_separation_m").value

#         # ── Latest pose per egg ID  {id: PoseStamped} ─────────────────────────
#         self._poses: dict[int, PoseStamped] = {}

#         # ── Subscribe to each egg's pose topic ───────────────────────────────
#         # multi_aruco_detector publishes /aruco/marker_<ID>/pose for every
#         # detected marker. We subscribe to each egg ID explicitly.
#         for marker_id in EGG_ARUCO_IDS:
#             self.create_subscription(
#                 PoseStamped,
#                 f"/aruco/marker_{marker_id}/pose",
#                 lambda msg, mid=marker_id: self._pose_cb(mid, msg),
#                 10,
#             )

#         # ── Publishers ────────────────────────────────────────────────────────
#         self._pub_solved  = self.create_publisher(Bool,   "/puzzle3/puzzle_solved", 10)
#         self._pub_status  = self.create_publisher(String, "/puzzle3/status",        10)
#         self._pub_egg_state = self.create_publisher(String, "/puzzle3/egg_state",   10)

#         # ── Verification timer ────────────────────────────────────────────────
#         self._timer = self.create_timer(1.0 / rate_hz, self._verify_callback)

#         self.get_logger().info(
#             f"EggSorterVerifierSimple started.\n"
#             f"  Tracking egg IDs : {list(EGG_ARUCO_IDS.keys())}\n"
#             f"  Expected sequence: {EXPECTED_SEQUENCE} "
#             f"({[EGG_ARUCO_IDS[i] for i in EXPECTED_SEQUENCE]})\n"
#             f"  Min X separation : {self._min_sep * 100:.0f} cm\n"
#             f"  Min eggs visible : {MIN_EGGS_VISIBLE}"
#         )

#     # ── Pose subscriber callback ───────────────────────────────────────────────

#     def _pose_cb(self, marker_id: int, msg: PoseStamped) -> None:
#         """Store the latest pose for this egg marker."""
#         self._poses[marker_id] = msg

#     # ── Main verification callback ─────────────────────────────────────────────

#     def _verify_callback(self) -> None:
#         """
#         Called at PUBLISH_RATE_HZ.

#         1. Collect all currently visible eggs (have a pose).
#         2. Sort them by camera-frame X (ascending = left to right).
#         3. Compare sorted ID sequence to EXPECTED_SEQUENCE.
#         4. Publish results.
#         """

#         # ── Step 1: collect visible eggs ──────────────────────────────────────
#         visible: dict[int, float] = {}   # {marker_id: x_in_camera_frame}
#         for marker_id, pose_msg in self._poses.items():
#             visible[marker_id] = pose_msg.pose.position.x

#         n_visible = len(visible)

#         # ── Step 2: sort by X ascending (left → right in camera frame) ────────
#         sorted_eggs = sorted(visible.items(), key=lambda kv: kv[1])
#         # sorted_eggs is a list of (marker_id, x_value) tuples, left to right

#         sorted_ids = [egg_id for egg_id, _ in sorted_eggs]
#         sorted_xs  = [x     for _,      x in sorted_eggs]

#         # ── Step 3: check separation — are eggs spread enough to be distinct? ──
#         ambiguous = False
#         if len(sorted_xs) >= 2:
#             for i in range(len(sorted_xs) - 1):
#                 if abs(sorted_xs[i + 1] - sorted_xs[i]) < self._min_sep:
#                     ambiguous = True
#                     break

#         # ── Step 4: determine verdict ──────────────────────────────────────────
#         if n_visible < MIN_EGGS_VISIBLE:
#             verdict = "WAITING"
#             solved  = False
#             reason  = f"only {n_visible}/{MIN_EGGS_VISIBLE} eggs visible"
#         elif ambiguous:
#             verdict = "AMBIGUOUS"
#             solved  = False
#             reason  = "eggs too close together to determine order reliably"
#         elif sorted_ids == EXPECTED_SEQUENCE:
#             verdict = "SOLVED ✓"
#             solved  = True
#             reason  = "sequence matches"
#         else:
#             verdict = "INCORRECT ✗"
#             solved  = False
#             reason  = (
#                 f"got {[EGG_ARUCO_IDS.get(i, str(i)) for i in sorted_ids]}, "
#                 f"expected {[EGG_ARUCO_IDS.get(i, str(i)) for i in EXPECTED_SEQUENCE]}"
#             )

#         # ── Step 5: build human-readable status ───────────────────────────────
#         lines = ["=== Puzzle 3 — Frog Egg Sorting (X-order check) ==="]
#         lines.append(f"  Visible eggs: {n_visible}/{len(EGG_ARUCO_IDS)}")
#         lines.append("")

#         if sorted_eggs:
#             lines.append("  Left → Right order detected:")
#             for rank, (egg_id, x_val) in enumerate(sorted_eggs, start=1):
#                 colour   = EGG_ARUCO_IDS.get(egg_id, "unknown")
#                 expected = EXPECTED_SEQUENCE[rank - 1] if rank <= len(EXPECTED_SEQUENCE) else "?"
#                 match    = "✓" if egg_id == expected else "✗"
#                 lines.append(
#                     f"    Position {rank}: ID {egg_id} ({colour:<8}) "
#                     f"x={x_val:+.4f}m   [{match}]"
#                 )
#         else:
#             lines.append("  No eggs visible.")

#         lines.append("")
#         lines.append(f"  Expected: {[EGG_ARUCO_IDS.get(i, str(i)) for i in EXPECTED_SEQUENCE]}")
#         lines.append(f"  → {verdict}  ({reason})")

#         status_str = "\n".join(lines)

#         # ── Step 6: build JSON egg state for SS4 ──────────────────────────────
#         egg_state = {
#             "detected_order": [
#                 {
#                     "rank":   rank + 1,
#                     "id":     egg_id,
#                     "colour": EGG_ARUCO_IDS.get(egg_id, "unknown"),
#                     "x_camera_m": round(x_val, 4),
#                 }
#                 for rank, (egg_id, x_val) in enumerate(sorted_eggs)
#             ],
#             "expected_sequence": EXPECTED_SEQUENCE,
#             "solved":   solved,
#             "verdict":  verdict,
#             "reason":   reason,
#         }

#         # ── Step 7: publish ───────────────────────────────────────────────────
#         solved_msg      = Bool();   solved_msg.data  = solved
#         status_msg      = String(); status_msg.data  = status_str
#         egg_state_msg   = String(); egg_state_msg.data = json.dumps(egg_state, indent=2)

#         self._pub_solved.publish(solved_msg)
#         self._pub_status.publish(status_msg)
#         self._pub_egg_state.publish(egg_state_msg)

#         # if solved:
#         #     self.get_logger().info("PUZZLE 3 SOLVED ✓")
#         # else:
#         #     self.get_logger().debug(status_str)
#         self.get_logger().info(status_str)

# # ─────────────────────────────────────────────────────────────────────────────
# # Entry point
# # ─────────────────────────────────────────────────────────────────────────────

# def main(args=None) -> None:
#     rclpy.init(args=args)
#     node = EggSorterVerifierSimple()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()





#!/usr/bin/env python3
"""
egg_sorter_verifier_simple.py  —  Puzzle 3 X-ordering verification
                                   with Arduino serial integration.

Data flow
---------
Arduino (game_state.cpp + egg_sorter.cpp)
    │  Serial line: "CODE:...|MAZE:...|MAZE_END:...|EGG:2,4,1,3\n"
    │  sent every 5 seconds and on every reset
    ▼
THIS NODE  (serial bridge thread reads EGG sequence)
    │  updates self._expected_sequence dynamically each run
    ▼
multi_aruco_detector  →  /aruco/marker_<ID>/pose
    │  camera-frame X positions of each egg block
    ▼
THIS NODE  (verification timer at 5 Hz)
    │  sorts detected eggs by X, compares to expected sequence
    │  when solved → writes "EGG_SOLVED\n" back to Arduino serial
    ▼
Published ROS topics:
    /puzzle3/puzzle_solved   std_msgs/Bool
    /puzzle3/status          std_msgs/String
    /puzzle3/egg_state       std_msgs/String  (JSON for SS4)

Arduino serial format (sent every 5 s and on reset)
----------------------------------------------------
  "CODE:1234|MAZE:00000...|MAZE_END:5,3|EGG:2,4,1,3"
   └─ only the EGG field is parsed here; others are ignored

Arduino serial format (sent back when solved)
---------------------------------------------
  "EGG_SOLVED\n"
  The Arduino's egg_sorter_work() reads this and sets puzzleEggSolved = true.
"""

import json
import threading

import serial          # pip install pyserial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String, Int32MultiArray


# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

# ArUco IDs and colours — must match your printed egg blocks and egg_sorter.h
EGG_ARUCO_IDS: dict[int, str] = {
    1: "white",
    2: "green",
    3: "purple",
    4: "blue",
}

# Fallback expected sequence used until Arduino sends the first EGG: field.
# Matches EGG_POOL order in egg_sorter.cpp (white, green, purple, blue).
EXPECTED_SEQUENCE: list[int] = [1, 2, 3, 4]

MIN_EGGS_VISIBLE: int = len(EXPECTED_SEQUENCE)
MIN_X_SEPARATION_M: float = 0.01
PUBLISH_RATE_HZ: float = 5.0

# ── Arduino serial port ───────────────────────────────────────────────────────
# Change to match your system. Check with: ls /dev/ttyACM* or ls /dev/ttyUSB*
SERIAL_PORT: str = "/dev/ttyACM0"
SERIAL_BAUD: int = 9600

# ─────────────────────────────────────────────────────────────────────────────


class EggSorterVerifierSimple(Node):

    def __init__(self) -> None:
        super().__init__("egg_sorter_verifier_simple")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("publish_rate_hz",    PUBLISH_RATE_HZ)
        self.declare_parameter("min_x_separation_m", MIN_X_SEPARATION_M)
        self.declare_parameter("serial_port",        SERIAL_PORT)
        self.declare_parameter("serial_baud",        SERIAL_BAUD)

        rate_hz          = self.get_parameter("publish_rate_hz").value
        self._min_sep    = self.get_parameter("min_x_separation_m").value
        serial_port      = self.get_parameter("serial_port").value
        serial_baud      = self.get_parameter("serial_baud").value

        # ── Expected sequence (updated dynamically from Arduino) ──────────────
        self._expected_sequence: list[int] = list(EXPECTED_SEQUENCE)
        self._seq_lock = threading.Lock()   # guards _expected_sequence

        # ── Track whether we already sent EGG_SOLVED this run ─────────────────
        self._solved_sent = False

        # ── Pose storage ──────────────────────────────────────────────────────
        self._poses: dict[int, PoseStamped] = {}

        # ── ArUco pose subscribers ─────────────────────────────────────────────
        for marker_id in EGG_ARUCO_IDS:
            self.create_subscription(
                PoseStamped,
                f"/aruco/marker_{marker_id}/pose",
                lambda msg, mid=marker_id: self._pose_cb(mid, msg),
                10,
            )

        # ── /puzzle3/expected_sequence subscriber (SS4 override) ──────────────
        # SS4 can also publish here to override the Arduino sequence directly.
        self.create_subscription(
            Int32MultiArray,
            "/puzzle3/expected_sequence",
            self._ros_sequence_cb,
            10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_solved    = self.create_publisher(Bool,   "/puzzle3/puzzle_solved", 10)
        self._pub_status    = self.create_publisher(String, "/puzzle3/status",        10)
        self._pub_egg_state = self.create_publisher(String, "/puzzle3/egg_state",     10)

        # ── Arduino serial connection ──────────────────────────────────────────
        self._serial: serial.Serial | None = None
        self._connect_serial(serial_port, serial_baud)

        # ── Serial reader thread ───────────────────────────────────────────────
        # Runs in background so it never blocks the ROS spin thread.
        if self._serial is not None:
            self._serial_thread = threading.Thread(
                target=self._serial_reader, daemon=True)
            self._serial_thread.start()

        # ── Verification timer ────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / rate_hz, self._verify_callback)

        self.get_logger().info(
            f"EggSorterVerifierSimple started.\n"
            f"  Serial port      : {serial_port} @ {serial_baud}\n"
            f"  Tracking egg IDs : {list(EGG_ARUCO_IDS.keys())}\n"
            f"  Fallback sequence: {self._expected_sequence} "
            f"({[EGG_ARUCO_IDS[i] for i in self._expected_sequence]})\n"
            f"  Min X separation : {self._min_sep * 100:.0f} cm"
        )

    # ── Serial helpers ────────────────────────────────────────────────────────

    def _connect_serial(self, port: str, baud: int) -> None:
        try:
            self._serial = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Serial connected: {port} @ {baud}")
        except serial.SerialException as e:
            self._serial = None
            self.get_logger().warn(
                f"Could not open serial port {port}: {e}\n"
                f"Running without Arduino — using fallback sequence. "
                f"To specify a different port:\n"
                f"  ros2 run perception_mapping egg_sorter_verifier_simple "
                f"--ros-args -p serial_port:=/dev/ttyUSB0"
            )

    def _serial_reader(self) -> None:
        """
        Background thread. Reads lines from the Arduino and looks for
        the EGG: field in the puzzle generation message.

        Expected line format (sent every 5 s by main.cpp):
            CODE:1234|MAZE:000...|MAZE_END:5,3|EGG:2,4,1,3

        Also handles RESET BOARD lines to clear the solved flag.
        """
        while True:
            try:
                if self._serial is None or not self._serial.is_open:
                    break
                raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                # ── Parse EGG: field ────────────────────────────────────────
                if "|EGG:" in line or line.startswith("EGG:"):
                    self._parse_egg_line(line)

                # ── Reset: clear solved flag so a new run can be confirmed ──
                if "RESET BOARD" in line:
                    self._solved_sent = False
                    self.get_logger().info("Arduino reset detected — clearing solved flag.")

            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                break

    def _parse_egg_line(self, line: str) -> None:
        """
        Extract the EGG:x,x,x,x field and update _expected_sequence.
        Example input: "CODE:1234|MAZE:000...|MAZE_END:5,3|EGG:2,4,1,3"
        """
        try:
            # Find the EGG: segment
            egg_start = line.index("EGG:") + len("EGG:")
            # Everything after EGG: until the next | or end of string
            egg_part  = line[egg_start:].split("|")[0].strip()
            new_seq   = [int(x) for x in egg_part.split(",")]

            # Validate
            if len(new_seq) != len(EGG_ARUCO_IDS):
                self.get_logger().warn(
                    f"EGG sequence has {len(new_seq)} entries, "
                    f"expected {len(EGG_ARUCO_IDS)} — ignoring."
                )
                return

            unknown = [i for i in new_seq if i not in EGG_ARUCO_IDS]
            if unknown:
                self.get_logger().warn(
                    f"EGG sequence contains unknown IDs {unknown} — ignoring."
                )
                return

            with self._seq_lock:
                self._expected_sequence = new_seq
                self._solved_sent = False   # new sequence = new run

            self.get_logger().info(
                f"New egg sequence from Arduino: "
                f"{[EGG_ARUCO_IDS[i] for i in new_seq]} {new_seq}"
            )

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Failed to parse EGG line '{line}': {e}")

    def _send_solved(self) -> None:
        """Write EGG_SOLVED back to the Arduino once per run."""
        if self._serial is not None and self._serial.is_open:
            try:
                self._serial.write(b"EGG_SOLVED\n")
                self.get_logger().info("Sent EGG_SOLVED to Arduino.")
            except serial.SerialException as e:
                self.get_logger().warn(f"Failed to send EGG_SOLVED: {e}")

    # ── ROS sequence override (SS4) ───────────────────────────────────────────

    def _ros_sequence_cb(self, msg: Int32MultiArray) -> None:
        """SS4 can override the sequence via ROS topic (takes priority)."""
        new_seq = list(msg.data)
        unknown = [i for i in new_seq if i not in EGG_ARUCO_IDS]
        if unknown:
            self.get_logger().warn(f"ROS sequence contains unknown IDs {unknown} — ignoring.")
            return
        with self._seq_lock:
            self._expected_sequence = new_seq
            self._solved_sent = False
        self.get_logger().info(
            f"ROS override sequence: {[EGG_ARUCO_IDS[i] for i in new_seq]}"
        )

    # ── Pose callback ─────────────────────────────────────────────────────────

    def _pose_cb(self, marker_id: int, msg: PoseStamped) -> None:
        self._poses[marker_id] = msg

    # ── Main verification callback ─────────────────────────────────────────────

    def _verify_callback(self) -> None:

        with self._seq_lock:
            expected = list(self._expected_sequence)

        # ── Collect visible eggs ───────────────────────────────────────────────
        visible: dict[int, float] = {
            mid: pose.pose.position.x
            for mid, pose in self._poses.items()
        }
        n_visible = len(visible)

        # ── Sort by X ascending (left → right) ────────────────────────────────
        sorted_eggs = sorted(visible.items(), key=lambda kv: kv[1])
        sorted_ids  = [eid for eid, _ in sorted_eggs]
        sorted_xs   = [x   for _,   x in sorted_eggs]

        # ── Ambiguity check ───────────────────────────────────────────────────
        ambiguous = any(
            abs(sorted_xs[i + 1] - sorted_xs[i]) < self._min_sep
            for i in range(len(sorted_xs) - 1)
        ) if len(sorted_xs) >= 2 else False

        # ── Verdict ───────────────────────────────────────────────────────────
        if n_visible < MIN_EGGS_VISIBLE:
            verdict = "WAITING"
            solved  = False
            reason  = f"only {n_visible}/{MIN_EGGS_VISIBLE} eggs visible"
        elif ambiguous:
            verdict = "AMBIGUOUS"
            solved  = False
            reason  = "eggs too close together"
        elif sorted_ids == expected:
            verdict = "SOLVED ✓"
            solved  = True
            reason  = "sequence matches"
        else:
            verdict = "INCORRECT ✗"
            solved  = False
            reason  = (
                f"got {[EGG_ARUCO_IDS.get(i, str(i)) for i in sorted_ids]}, "
                f"expected {[EGG_ARUCO_IDS.get(i, str(i)) for i in expected]}"
            )

        # ── Send EGG_SOLVED to Arduino once when first solved ─────────────────
        if solved and not self._solved_sent:
            self._send_solved()
            self._solved_sent = True

        # ── Human-readable status ─────────────────────────────────────────────
        lines = ["=== Puzzle 3 — Frog Egg Sorting (X-order check) ==="]
        lines.append(f"  Expected sequence : {[EGG_ARUCO_IDS.get(i, str(i)) for i in expected]}")
        lines.append(f"  Visible eggs      : {n_visible}/{len(EGG_ARUCO_IDS)}")
        lines.append("")

        if sorted_eggs:
            lines.append("  Left → Right order detected:")
            for rank, (egg_id, x_val) in enumerate(sorted_eggs, start=1):
                colour   = EGG_ARUCO_IDS.get(egg_id, "unknown")
                exp_id   = expected[rank - 1] if rank <= len(expected) else "?"
                match    = "✓" if egg_id == exp_id else "✗"
                lines.append(
                    f"    Position {rank}: ID {egg_id} ({colour:<8}) "
                    f"x={x_val:+.4f}m   [{match}]"
                )
        else:
            lines.append("  No eggs visible.")

        lines.append("")
        lines.append(f"  → {verdict}  ({reason})")
        status_str = "\n".join(lines)

        # ── JSON for SS4 ──────────────────────────────────────────────────────
        egg_state = {
            "detected_order": [
                {
                    "rank":       rank + 1,
                    "id":         egg_id,
                    "colour":     EGG_ARUCO_IDS.get(egg_id, "unknown"),
                    "x_camera_m": round(x_val, 4),
                }
                for rank, (egg_id, x_val) in enumerate(sorted_eggs)
            ],
            "expected_sequence": expected,
            "solved":  solved,
            "verdict": verdict,
            "reason":  reason,
        }

        # ── Publish ───────────────────────────────────────────────────────────
        solved_msg          = Bool();   solved_msg.data          = solved
        status_msg          = String(); status_msg.data          = status_str
        egg_state_msg       = String(); egg_state_msg.data       = json.dumps(egg_state, indent=2)

        self._pub_solved.publish(solved_msg)
        self._pub_status.publish(status_msg)
        self._pub_egg_state.publish(egg_state_msg)

        self.get_logger().info(status_str)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = EggSorterVerifierSimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()