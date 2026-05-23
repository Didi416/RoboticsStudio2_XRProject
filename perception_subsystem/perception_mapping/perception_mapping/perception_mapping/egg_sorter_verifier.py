#!/usr/bin/env python3
"""
egg_sorter_verifier.py  —  Puzzle 3 (Frog Egg Sorting) state-verification node.

Architecture position
---------------------
  Subsystem 1 (multi_aruco_detector)
      │  TF frames: aruco_marker_N  (in camera_optical_frame)
      │  which chain up via calibration to base_link
      ▼
  THIS NODE  (egg_sorter_verifier)
      │  /puzzle3/egg_state      — per-slot detail (JSON string)
      │  /puzzle3/puzzle_solved  — std_msgs/Bool
      │  /puzzle3/status         — human-readable String for debug
      ▼
  Subsystem 4 (XR/ROS bridge) → Unity VR

How slot positions are defined
-------------------------------
Each nest slot has a known position in the robot's 'base_link' frame.
You must measure / calibrate these once the physical puzzle board is fixed
on the table, then fill in EGG_SLOT_POSITIONS below (or override via params).

How to get real values
-----------------------
  1. Jog the UR3e end-effector to hover directly above each nest slot centre.
  2. Read the XYZ from the teach pendant (Tool Position readout).
  3. Subtract ~10mm from Z to get the table-surface level where the egg sits.
  4. Paste those numbers into EGG_SLOT_POSITIONS below.

ArUco ID → egg colour mapping
-------------------------------
IDs 1-4 map to the four eggs. The correct egg for the current run is set
via the ROS2 parameter 'correct_egg_aruco_id' (default 1 = green, matching
the story clue "the tree frog prefers emerald waters").

Coordinate frames used
-----------------------
  aruco_marker_N  →  (via TF chain)  →  base_link
Full chain: base_link → tool0 → camera_optical_frame → aruco_marker_N
This chain is live once the UR3e driver, hand-eye calibration publisher,
and multi_aruco_detector are all running.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Bool, String
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)


# ---------------------------------------------------------------------------
# ── CONFIGURATION ──  Edit these values to match your physical puzzle board
# ---------------------------------------------------------------------------

# Nest slot positions in the robot's base_link frame (metres).
# PLACEHOLDERS — replace with real measurements after physical setup (see docstring above).
# Four slots for four eggs arranged left-to-right on the puzzle board.
EGG_SLOT_POSITIONS: dict[str, tuple[float, float, float]] = {
    "slot_1_leftmost":  (0.250,  0.225, 0.005),   # (x, y, z) in base_link metres
    "slot_2_left":      (0.250,  0.075, 0.005),
    "slot_3_right":     (0.250, -0.075, 0.005),
    "slot_4_rightmost": (0.250, -0.225, 0.005),
}

# ArUco ID → egg colour (all four eggs in the puzzle)
EGG_ARUCO_IDS: dict[int, str] = {
    1: "green",    # tree frog egg  ← story answer
    2: "blue",
    3: "white",
    4: "purple",
}

# Default solved configuration: which ArUco ID belongs in which slot.
# The correct egg (answer) goes to slot_2_left; the others fill the remaining slots.
# SS4 can override 'correct_egg_aruco_id' at launch for seeded runs.
DEFAULT_PUZZLE_CONFIG: dict[str, int] = {
    "slot_1_leftmost":  2,   # blue
    "slot_2_left":      1,   # green  ← correct answer slot
    "slot_3_right":     3,   # white
    "slot_4_rightmost": 4,   # purple
}

# Placement tolerance in metres. An egg is "in" a slot if its ArUco marker
# is within this distance of the slot's known position. 3 cm is a good
# starting value; tighten once you've measured real pick-and-place accuracy.
PLACEMENT_TOLERANCE_M: float = 0.03

# How long (seconds) to wait for a TF lookup before giving up on that frame.
TF_TIMEOUT_S: float = 0.1

# Root of the robot TF tree.
ROBOT_BASE_FRAME: str = "base_link"

# How often to run verification and publish results.
PUBLISH_RATE_HZ: float = 5.0


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class EggSorterVerifier(Node):
    """
    Subscribes to TF frames from multi_aruco_detector.
    For each egg marker, transforms its pose into base_link, finds the
    nearest slot, and checks whether it matches the expected configuration.

    Published topics
    ----------------
    /puzzle3/egg_state      std_msgs/String (JSON)  per-slot detail
    /puzzle3/puzzle_solved  std_msgs/Bool           True when all slots correct
    /puzzle3/status         std_msgs/String         human-readable summary
    """

    def __init__(self) -> None:
        super().__init__("egg_sorter_verifier")

        # ── Parameters (can be overridden at launch or via ros2 param set) ──
        self.declare_parameter("correct_egg_aruco_id", DEFAULT_PUZZLE_CONFIG["slot_2_left"])
        self.declare_parameter("placement_tolerance_m", PLACEMENT_TOLERANCE_M)
        self.declare_parameter("robot_base_frame", ROBOT_BASE_FRAME)
        self.declare_parameter("publish_rate_hz", PUBLISH_RATE_HZ)

        self._correct_id = self.get_parameter("correct_egg_aruco_id").value
        self._tolerance  = self.get_parameter("placement_tolerance_m").value
        self._base_frame = self.get_parameter("robot_base_frame").value
        rate_hz          = self.get_parameter("publish_rate_hz").value

        # Build live puzzle config from the correct egg ID
        self._puzzle_config = self._build_puzzle_config(self._correct_id)

        # ── TF listener ──────────────────────────────────────────────────────
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Publishers ───────────────────────────────────────────────────────
        self._pub_egg_state = self.create_publisher(String, "/puzzle3/egg_state",    10)
        self._pub_solved    = self.create_publisher(Bool,   "/puzzle3/puzzle_solved", 10)
        self._pub_status    = self.create_publisher(String, "/puzzle3/status",        10)

        # ── Verification timer ───────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / rate_hz, self._verify_callback)

        self.get_logger().info(
            f"EggSorterVerifier started.\n"
            f"  Base frame     : {self._base_frame}\n"
            f"  Tolerance      : {self._tolerance * 1000:.0f} mm\n"
            f"  Correct egg ID : {self._correct_id} "
            f"({EGG_ARUCO_IDS.get(self._correct_id, 'unknown')})\n"
            f"  Puzzle config  : {self._puzzle_config}\n"
            f"  Slot positions : {EGG_SLOT_POSITIONS}"
        )

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _build_puzzle_config(self, correct_id: int) -> dict[str, int]:
        """
        Put the correct egg in slot_2_left; fill the other three slots with
        the remaining eggs in ascending ID order.
        """
        all_ids  = sorted(EGG_ARUCO_IDS.keys())
        others   = [i for i in all_ids if i != correct_id]
        # Pad if fewer than 3 other eggs are defined (shouldn't happen with 4 eggs)
        while len(others) < 3:
            others.append(-1)
        return {
            "slot_1_leftmost":  others[0],
            "slot_2_left":      correct_id,
            "slot_3_right":     others[1],
            "slot_4_rightmost": others[2],
        }

    def _lookup_egg_in_base(self, marker_id: int) -> tuple[float, float, float] | None:
        """
        Look up TF for aruco_marker_<marker_id> in base_link.
        Returns (x, y, z) in metres, or None if the marker is not visible.
        """
        source_frame = f"aruco_marker_{marker_id}"
        try:
            t = self._tf_buffer.lookup_transform(
                self._base_frame,
                source_frame,
                rclpy.time.Time(),
                Duration(seconds=TF_TIMEOUT_S),
            )
            tx = t.transform.translation
            return (tx.x, tx.y, tx.z)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    @staticmethod
    def _distance(a: tuple[float, float, float],
                  b: tuple[float, float, float]) -> float:
        return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))

    def _nearest_slot(
        self, position: tuple[float, float, float]
    ) -> tuple[str, float]:
        """Return (slot_name, distance_m) for the slot closest to position."""
        best_slot, best_dist = None, float("inf")
        for slot_name, slot_pos in EGG_SLOT_POSITIONS.items():
            d = self._distance(position, slot_pos)
            if d < best_dist:
                best_dist = d
                best_slot = slot_name
        return best_slot, best_dist

    # ── Main verification callback ────────────────────────────────────────────

    def _verify_callback(self) -> None:
        """
        Runs at PUBLISH_RATE_HZ. Looks up each egg's pose, finds its nearest
        slot, checks correctness, and publishes results on all three topics.
        """
        # Initialise result dict for all four slots
        slot_results: dict[str, dict] = {
            slot: {
                "expected_id":     self._puzzle_config[slot],
                "expected_colour": EGG_ARUCO_IDS.get(self._puzzle_config[slot], "unknown"),
                "detected_id":     None,
                "detected_colour": None,
                "distance_m":      None,
                "correct":         False,
                "egg_visible":     False,
            }
            for slot in EGG_SLOT_POSITIONS
        }

        # Check every egg
        for marker_id, colour in EGG_ARUCO_IDS.items():
            pos = self._lookup_egg_in_base(marker_id)
            if pos is None:
                continue  # not visible

            nearest_slot, dist = self._nearest_slot(pos)
            if dist > self._tolerance:
                continue  # too far from any slot — egg is in transit or off-board

            r = slot_results[nearest_slot]
            r["egg_visible"]    = True
            r["detected_id"]    = marker_id
            r["detected_colour"] = colour
            r["distance_m"]     = round(dist, 4)
            r["correct"]        = (marker_id == self._puzzle_config[nearest_slot])

        # ── Overall solved state ──────────────────────────────────────────────
        all_correct = all(r["correct"] for r in slot_results.values())
        any_visible = any(r["egg_visible"] for r in slot_results.values())

        # ── Human-readable status ────────────────────────────────────────────
        lines = ["=== Puzzle 3 — Frog Egg Sorting ==="]
        for slot, r in slot_results.items():
            icon         = "✓" if r["correct"] else ("○" if not r["egg_visible"] else "✗")
            detected_str = (
                f"ID {r['detected_id']} ({r['detected_colour']})"
                if r["egg_visible"] else "empty"
            )
            dist_str = (
                f"{r['distance_m'] * 1000:.1f}mm"
                if r["distance_m"] is not None else "N/A"
            )
            lines.append(
                f"  [{icon}] {slot:<18}  "
                f"expected={r['expected_colour']:<8}  "
                f"detected={detected_str:<24}  d={dist_str}"
            )
        lines.append(
            f"\n  → "
            + ("SOLVED ✓" if all_correct
               else ("WAITING — no eggs visible" if not any_visible
                     else "INCORRECT ✗"))
        )
        status_str = "\n".join(lines)

        # ── Publish ───────────────────────────────────────────────────────────
        egg_msg      = String(); egg_msg.data    = json.dumps(slot_results, indent=2)
        solved_msg   = Bool();   solved_msg.data = all_correct
        status_msg   = String(); status_msg.data = status_str

        self._pub_egg_state.publish(egg_msg)
        self._pub_solved.publish(solved_msg)
        self._pub_status.publish(status_msg)

        if all_correct:
            self.get_logger().info("PUZZLE 3 SOLVED ✓")
        else:
            self.get_logger().debug(status_str)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = EggSorterVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()