# EscapeXRtist — Puzzle Control GUI
## Setup, Simulation & Real Robot Guide

---

## File Overview

| File | Purpose |
|---|---|
| `puzzle_pose_server.py` | ROS 2 node — receives goal name, moves UR3e with MoveIt |
| `puzzle_ws_bridge.py`   | WebSocket bridge — connects HTML GUI to ROS 2 topics |
| `puzzle_control_gui.html` | Browser GUI — puzzle buttons, board visualiser, log |

---

## Architecture

```
Browser GUI (HTML)
      │  WebSocket  ws://localhost:9090
      ▼
puzzle_ws_bridge.py    ◄──► /puzzle_goal   (std_msgs/String)
                       ◄──► /puzzle_status (std_msgs/String)
                       ◄──► /puzzle_busy   (std_msgs/Bool)
      │  ROS 2 topics
      ▼
puzzle_pose_server.py  ◄──► MoveIt / MoveGroupInterface
      │
      ▼
UR3e robot (real or URSim)
```

---

## Prerequisites

```bash
# ROS 2 Humble (or Iron) with UR packages
sudo apt install ros-humble-ur ros-humble-ur-robot-driver ros-humble-ur-moveit-config

# Python websockets (for the bridge)
pip3 install websockets --break-system-packages

# moveit_py (comes with moveit2 source build or:)
sudo apt install ros-humble-moveit
```

---

## Part 1 — Simulation (URSim)

### Step 1 — Start URSim

```bash
# Option A: Docker (easiest)
docker run --rm -it -p 5900:5900 -p 6080:6080 \
  -p 50001-50006:50001-50006 \
  universalrobots/ursim_e-series

# Option B: if ur_client_library is installed
ros2 run ur_client_library start_ursim.sh -m ur3e
```

Wait for URSim to boot (~15 s). Open http://localhost:6080 to see the pendant.
On the pendant: go to **Program → URCaps → External Control**,
confirm the host IP is `192.168.56.1` (or your machine IP), then press **Play**.

### Step 2 — Launch UR driver + MoveIt

```bash
# Terminal 1
source /opt/ros/humble/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e \
    robot_ip:=192.168.56.101 \
    use_fake_hardware:=false \
    launch_rviz:=true

# Wait for "Robot ready to receive commands" in the terminal output
```

```bash
# Terminal 2 — MoveIt
source /opt/ros/humble/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur3e \
    robot_ip:=192.168.56.101 \
    use_fake_hardware:=false \
    launch_rviz:=false
```

### Step 3 — (Quick test) Fake hardware only

If you don't have URSim, use fake hardware — the robot won't move physically
but MoveIt planning works and the GUI/bridge works:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e \
    robot_ip:=192.168.56.101 \
    use_fake_hardware:=true \
    launch_rviz:=true

# separate terminal:
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur3e \
    use_fake_hardware:=true \
    launch_rviz:=false
```

### Step 4 — Run the puzzle pose server

```bash
# Terminal 3 — copy the file into your ROS 2 package or run directly
source /opt/ros/humble/setup.bash

# Simple way (no package needed for testing):
python3 puzzle_pose_server.py
```

You should see:
```
[puzzle_pose_server]: PuzzlePoseServer ready.  Known puzzles: puzzle1, puzzle2, puzzle3, home
```

### Step 5 — Run the WebSocket bridge

```bash
# Terminal 4
source /opt/ros/humble/setup.bash
python3 puzzle_ws_bridge.py
```

You should see:
```
[puzzle_ws_bridge]: WebSocket server listening on ws://0.0.0.0:9090
```

### Step 6 — Open the GUI

Open `puzzle_control_gui.html` in any browser:

```bash
# Linux
xdg-open puzzle_control_gui.html

# macOS
open puzzle_control_gui.html

# Or just drag the file into Chrome/Firefox
```

- The GUI will auto-connect to `ws://localhost:9090`
- Click any puzzle button — the robot moves in RViz / URSim
- Watch the board visualiser and log panel update

---

## Part 2 — Real Robot

### Different from simulation: just change the IP

```bash
# Replace 192.168.56.101 with your robot's actual IP
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e \
    robot_ip:=192.168.1.100 \      # ← your robot IP
    use_fake_hardware:=false \
    launch_rviz:=true
```

Everything else (MoveIt, pose server, bridge, GUI) stays the same.

---

## Part 3 — Test without ROS (GUI demo mode)

The GUI works standalone without any ROS connection:

1. Open `puzzle_control_gui.html` in a browser
2. If the WebSocket connection fails, the GUI falls into **demo mode**
3. Clicking puzzle buttons shows animated cursor movement on the board and simulates a 2-second motion delay
4. Useful for UI testing / presentations

---

## Adjusting Puzzle Positions

Edit the `PUZZLE_POSES` dictionary in `puzzle_pose_server.py`:

```python
PUZZLE_POSES: dict[str, Pose] = {
    "puzzle1": Pose(
        position=Point(x=0.30, y=-0.12, z=0.20),   # ← change these
        orientation=DOWN_QUAT,
    ),
    ...
}
```

Also update the matching display in `puzzle_control_gui.html`:
```javascript
const POSES = {
  puzzle1: { x: 0.30, y: -0.12, z: 0.20, ... },
  ...
};
```

### How to find the right coordinates

In RViz with MoveIt:
1. Manually drag the end-effector to the puzzle position using the interactive marker
2. Open a terminal and run:
   ```bash
   ros2 topic echo /move_group/monitored_planning_scene | grep position
   # or
   ros2 run tf2_ros tf2_echo base_link tool0
   ```
3. Copy the x/y/z values into `PUZZLE_POSES`

---

## Adding the server to a proper ROS 2 package

```python
# In your package's setup.py, add to entry_points:
entry_points={
    'console_scripts': [
        'puzzle_pose_server = your_package.puzzle_pose_server:main',
        'puzzle_ws_bridge   = your_package.puzzle_ws_bridge:main',
    ],
},
```

Then:
```bash
colcon build --packages-select your_package
source install/setup.bash
ros2 run your_package puzzle_pose_server
ros2 run your_package puzzle_ws_bridge
```

---

## Quick topic test (without GUI)

```bash
# Send a goal directly from terminal:
ros2 topic pub --once /puzzle_goal std_msgs/msg/String "data: 'puzzle1'"
ros2 topic pub --once /puzzle_goal std_msgs/msg/String "data: 'home'"

# Watch status:
ros2 topic echo /puzzle_status
ros2 topic echo /puzzle_busy
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `moveit_py not found` | Run in DRY-RUN mode for testing; install moveit2 for real use |
| GUI shows "Disconnected" | Make sure `puzzle_ws_bridge.py` is running before opening browser |
| Robot doesn't move | Check URSim pendant is in PLAY mode with ExternalControl active |
| Planning fails | Check the target pose is within UR3e reach (~500mm from base); verify MoveIt is running |
| `websockets` ImportError | `pip3 install websockets --break-system-packages` |