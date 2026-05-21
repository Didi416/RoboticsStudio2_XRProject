#!/usr/bin/env python3
"""
egg_sorter_verifier_simple.py  —  Puzzle 3 simple X-ordering verification.

NO depth camera required. NO TF tree required. NO slot calibration required.
Works with a plain webcam and the existing multi_aruco_detector.py as-is.

How it works
------------
multi_aruco_detector already publishes:
    /aruco/marker_<ID>/pose   (geometry_msgs/PoseStamped)
                               frame_id = 'camera_color_optical_frame'

In the camera optical frame, the X axis points RIGHT across the image.
So pose.position.x directly gives the left-to-right position of each egg.

This node:
  1. Subscribes to /aruco/marker_<ID>/pose for each egg ID
  2. Collects whichever eggs are currently visible
  3. Sorts them by pose.position.x  (most negative = leftmost)
  4. Compares the sorted ID sequence against the expected sequence
  5. Publishes the result on /puzzle3/puzzle_solved and /puzzle3/status

Camera frame reminder (OpenCV / RealSense optical convention)
-------------------------------------------------------------
    +X → right across image
    +Y → down the image
    +Z → into the scene (depth)

So sorting by X ascending gives left-to-right order as seen by the camera.

What you need to change
-----------------------
  EXPECTED_SEQUENCE  — the correct left-to-right order of egg ArUco IDs
  EGG_ARUCO_IDS      — which ID maps to which colour label
  EGG_PUZZLE_IDS     — which IDs belong to Puzzle 3 (subset of PUZZLE_MARKERS)

What you do NOT need to change in multi_aruco_detector.py
---------------------------------------------------------
  Nothing. This node subscribes to the topics it already publishes.
  The only thing to confirm is that your egg IDs are NOT already used
  by other puzzle markers (IDs 0,1,2,3,10 are taken — use 11,12,13,14).

Running alongside the existing detector (webcam)
-------------------------------------------------
  Terminal 1 — start the webcam bridge (publishes /camera/color/image_raw):
    ros2 run perception_mapping webcam_realsense_bridge

  Terminal 2 — start the detector (no depth remapping needed for webcam):
    ros2 run perception_mapping multi_aruco_detector

  Terminal 3 — start this verifier:
    ros2 run perception_mapping egg_sorter_verifier_simple

  Monitor results:
    ros2 topic echo /puzzle3/status
    ros2 topic echo /puzzle3/puzzle_solved
"""

import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String


# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION — edit these to match your printed egg markers
# ─────────────────────────────────────────────────────────────────────────────

# ArUco IDs printed on each egg block (must not clash with IDs 0,1,2,3,10)
EGG_ARUCO_IDS: dict[int, str] = {
    1: "green",
    2: "blue",
    3: "white",
    4: "purple",
}

# The correct left-to-right order of egg IDs as seen by the camera.
# Example: green(11) must be leftmost, then blue(12), yellow(13), purple(14) rightmost.
# Change this to match your puzzle's correct answer for the current seed/run.
EXPECTED_SEQUENCE: list[int] = [1, 2, 3, 4]

# How many eggs must be visible before we attempt a verdict.
# Set to len(EXPECTED_SEQUENCE) to require all eggs present.
MIN_EGGS_VISIBLE: int = len(EXPECTED_SEQUENCE)

# Minimum X separation (metres in camera frame) between adjacent eggs.
# Eggs closer than this are considered "ambiguous" and the check is skipped.
# In camera frame units ~0.01 m (1 cm) is a safe minimum for desktop distances.
MIN_X_SEPARATION_M: float = 0.01

# Publish rate in Hz.
PUBLISH_RATE_HZ: float = 5.0

# ─────────────────────────────────────────────────────────────────────────────


class EggSorterVerifierSimple(Node):
    """
    Subscribes to /aruco/marker_<ID>/pose topics published by
    multi_aruco_detector and verifies Puzzle 3 by sorting detected
    egg markers by their camera-frame X coordinate (left-to-right).

    Published topics
    ----------------
    /puzzle3/puzzle_solved   std_msgs/Bool    True when order matches
    /puzzle3/status          std_msgs/String  human-readable summary
    /puzzle3/egg_state       std_msgs/String  JSON detail for SS4
    """

    def __init__(self) -> None:
        super().__init__("egg_sorter_verifier_simple")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("publish_rate_hz", PUBLISH_RATE_HZ)
        self.declare_parameter("min_x_separation_m", MIN_X_SEPARATION_M)
        rate_hz         = self.get_parameter("publish_rate_hz").value
        self._min_sep   = self.get_parameter("min_x_separation_m").value

        # ── Latest pose per egg ID  {id: PoseStamped} ─────────────────────────
        self._poses: dict[int, PoseStamped] = {}

        # ── Subscribe to each egg's pose topic ───────────────────────────────
        # multi_aruco_detector publishes /aruco/marker_<ID>/pose for every
        # detected marker. We subscribe to each egg ID explicitly.
        for marker_id in EGG_ARUCO_IDS:
            self.create_subscription(
                PoseStamped,
                f"/aruco/marker_{marker_id}/pose",
                lambda msg, mid=marker_id: self._pose_cb(mid, msg),
                10,
            )

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_solved  = self.create_publisher(Bool,   "/puzzle3/puzzle_solved", 10)
        self._pub_status  = self.create_publisher(String, "/puzzle3/status",        10)
        self._pub_egg_state = self.create_publisher(String, "/puzzle3/egg_state",   10)

        # ── Verification timer ────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / rate_hz, self._verify_callback)

        self.get_logger().info(
            f"EggSorterVerifierSimple started.\n"
            f"  Tracking egg IDs : {list(EGG_ARUCO_IDS.keys())}\n"
            f"  Expected sequence: {EXPECTED_SEQUENCE} "
            f"({[EGG_ARUCO_IDS[i] for i in EXPECTED_SEQUENCE]})\n"
            f"  Min X separation : {self._min_sep * 100:.0f} cm\n"
            f"  Min eggs visible : {MIN_EGGS_VISIBLE}"
        )

    # ── Pose subscriber callback ───────────────────────────────────────────────

    def _pose_cb(self, marker_id: int, msg: PoseStamped) -> None:
        """Store the latest pose for this egg marker."""
        self._poses[marker_id] = msg

    # ── Main verification callback ─────────────────────────────────────────────

    def _verify_callback(self) -> None:
        """
        Called at PUBLISH_RATE_HZ.

        1. Collect all currently visible eggs (have a pose).
        2. Sort them by camera-frame X (ascending = left to right).
        3. Compare sorted ID sequence to EXPECTED_SEQUENCE.
        4. Publish results.
        """

        # ── Step 1: collect visible eggs ──────────────────────────────────────
        visible: dict[int, float] = {}   # {marker_id: x_in_camera_frame}
        for marker_id, pose_msg in self._poses.items():
            visible[marker_id] = pose_msg.pose.position.x

        n_visible = len(visible)

        # ── Step 2: sort by X ascending (left → right in camera frame) ────────
        sorted_eggs = sorted(visible.items(), key=lambda kv: kv[1])
        # sorted_eggs is a list of (marker_id, x_value) tuples, left to right

        sorted_ids = [egg_id for egg_id, _ in sorted_eggs]
        sorted_xs  = [x     for _,      x in sorted_eggs]

        # ── Step 3: check separation — are eggs spread enough to be distinct? ──
        ambiguous = False
        if len(sorted_xs) >= 2:
            for i in range(len(sorted_xs) - 1):
                if abs(sorted_xs[i + 1] - sorted_xs[i]) < self._min_sep:
                    ambiguous = True
                    break

        # ── Step 4: determine verdict ──────────────────────────────────────────
        if n_visible < MIN_EGGS_VISIBLE:
            verdict = "WAITING"
            solved  = False
            reason  = f"only {n_visible}/{MIN_EGGS_VISIBLE} eggs visible"
        elif ambiguous:
            verdict = "AMBIGUOUS"
            solved  = False
            reason  = "eggs too close together to determine order reliably"
        elif sorted_ids == EXPECTED_SEQUENCE:
            verdict = "SOLVED ✓"
            solved  = True
            reason  = "sequence matches"
        else:
            verdict = "INCORRECT ✗"
            solved  = False
            reason  = (
                f"got {[EGG_ARUCO_IDS.get(i, str(i)) for i in sorted_ids]}, "
                f"expected {[EGG_ARUCO_IDS.get(i, str(i)) for i in EXPECTED_SEQUENCE]}"
            )

        # ── Step 5: build human-readable status ───────────────────────────────
        lines = ["=== Puzzle 3 — Frog Egg Sorting (X-order check) ==="]
        lines.append(f"  Visible eggs: {n_visible}/{len(EGG_ARUCO_IDS)}")
        lines.append("")

        if sorted_eggs:
            lines.append("  Left → Right order detected:")
            for rank, (egg_id, x_val) in enumerate(sorted_eggs, start=1):
                colour   = EGG_ARUCO_IDS.get(egg_id, "unknown")
                expected = EXPECTED_SEQUENCE[rank - 1] if rank <= len(EXPECTED_SEQUENCE) else "?"
                match    = "✓" if egg_id == expected else "✗"
                lines.append(
                    f"    Position {rank}: ID {egg_id} ({colour:<8}) "
                    f"x={x_val:+.4f}m   [{match}]"
                )
        else:
            lines.append("  No eggs visible.")

        lines.append("")
        lines.append(f"  Expected: {[EGG_ARUCO_IDS.get(i, str(i)) for i in EXPECTED_SEQUENCE]}")
        lines.append(f"  → {verdict}  ({reason})")

        status_str = "\n".join(lines)

        # ── Step 6: build JSON egg state for SS4 ──────────────────────────────
        egg_state = {
            "detected_order": [
                {
                    "rank":   rank + 1,
                    "id":     egg_id,
                    "colour": EGG_ARUCO_IDS.get(egg_id, "unknown"),
                    "x_camera_m": round(x_val, 4),
                }
                for rank, (egg_id, x_val) in enumerate(sorted_eggs)
            ],
            "expected_sequence": EXPECTED_SEQUENCE,
            "solved":   solved,
            "verdict":  verdict,
            "reason":   reason,
        }

        # ── Step 7: publish ───────────────────────────────────────────────────
        solved_msg      = Bool();   solved_msg.data  = solved
        status_msg      = String(); status_msg.data  = status_str
        egg_state_msg   = String(); egg_state_msg.data = json.dumps(egg_state, indent=2)

        self._pub_solved.publish(solved_msg)
        self._pub_status.publish(status_msg)
        self._pub_egg_state.publish(egg_state_msg)

        if solved:
            self.get_logger().info("PUZZLE 3 SOLVED ✓")
        else:
            self.get_logger().debug(status_str)


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