# Subsystem 1 — Perception and Mapping
## ROS2 Humble + Intel RealSense D435i + ArUco + Eye-in-Hand Calibration

**Author:** Amadee R. Thotawatta  
**Package:** `perception_mapping` · `realsense_sim` · `aruco_ros` · `easy_handeye2`  
**ROS2 Distro:** Humble | **OS:** Ubuntu 22.04 | **OpenCV:** 4.5.4

---

## Overview

This subsystem provides real-time perception for the EscapeXRtist project. It detects ArUco markers and puzzle objects using an Intel RealSense D435i RGB-D camera, performs eye-in-hand calibration to align the camera frame with the UR3e robot base frame, and publishes 3D pose data for consumption by Subsystem 2 (Motion Planning).

The full pipeline:

```
Intel RealSense D435i (physical camera mounted on UR3e)
        ↓  realsense2_camera driver
ROS2 topics  (/camera/camera/color/image_raw, /camera/camera/color/camera_info)
        ↓  aruco_detector / multi_aruco_detector / puzzle_object_detector
Detected marker poses  (/aruco/marker_N/pose, /puzzle_objects/poses)
        ↓  easy_handeye2  (eye-in-hand calibration)
Calibrated camera→robot transform  (/hand_eye_calibration/camera_pose)
        ↓
Subsystem 2 — Motion Planning (MoveIt2 / UR3e)
```

A Gazebo Ignition Fortress simulation (`realsense_sim`) is also available for offline testing when the physical camera is unavailable.

---

## Repository

Clone the workspace into `~/ros2_ws/src/`:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/UTS-RI/aruco_ros.git
git clone https://github.com/IFL-CAMP/easy_handeye2.git
git clone <YOUR_TEAM_REPO_URL>   # contains perception_mapping and realsense_sim
```

> **Note:** Replace `<YOUR_TEAM_REPO_URL>` with the actual EscapeXRtist team repository URL (e.g. `https://github.com/not-the-x-potatoes/escapexrtist.git`).

---

## Workspace Structure

```
~/ros2_ws/
├── src/
│   ├── aruco_ros/                          # Third-party: ArUco ROS2 wrapper
│   ├── easy_handeye2/                      # Third-party: eye-in-hand calibration
│   ├── easy_handeye2_demo/                 # Third-party: calibration demo launch files
│   ├── realsense_sim/                      # Gazebo simulation package
│   │   └── ...                             # SDF world, D435i model, launch files
│   └── perception_mapping/                 # Main perception package
│       ├── launch/
│       │   ├── calibrate.launch.py         # Launches eye-in-hand calibration pipeline
│       │   └── publish_calibration.launch.py  # Publishes saved calibration transform
│       ├── perception_mapping/
│       │   ├── __init__.py
│       │   ├── aruco_detector.py           # Single ArUco marker detection + pose publisher
│       │   ├── aruco_debug_viewer.py       # Visualises ArUco detections in rqt
│       │   ├── charuco_pose_publisher.py   # ChArUco board pose estimation
│       │   ├── charuco_debug_viewer.py     # Visualises ChArUco detections in rqt
│       │   ├── checkerboard_pose_publisher.py  # Checkerboard pose for calibration
│       │   ├── multi_aruco_detector.py     # Detects multiple ArUco markers simultaneously
│       │   ├── puzzle_object_detector.py   # Detects puzzle-specific objects (colour/shape)
│       │   └── webcam_realsense_bridge.py  # Topic remapping bridge for D435i double-namespace
│       ├── resource/
│       ├── test/
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py
├── build/
├── install/
│   ├── perception_mapping/
│   └── realsense_sim/
└── log/
```

---

## Prerequisites

### 1. Verify ROS2 Humble

```bash
echo $ROS_DISTRO
# Expected: humble
```

### 2. Verify OpenCV

```bash
python3 -c "import cv2; print(cv2.__version__)"
# Expected: 4.5.4
```

### 3. Install core ROS2 dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
  python3-opencv \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-rqt-image-view \
  ros-humble-usb-cam \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher
```

### 4. Install the Intel RealSense SDK and ROS2 driver

**Step 4a — SDK:**

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
  | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
  https://librealsense.intel.com/Debian/apt-repo \
  $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt-get update
sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev
```

> **Secure Boot:** If prompted for a MOK password during install, set one, reboot, select "Enroll MOK" at the blue screen, enter your password, and reboot again.

**Step 4b — ROS2 driver:**

```bash
sudo apt-get install -y \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description
```

### 5. Install easy_handeye2 dependencies

```bash
sudo apt-get install -y \
  ros-humble-moveit \
  ros-humble-ur \
  ros-humble-ur-robot-driver
pip3 install transforms3d
```

### 6. Configure ~/.bashrc

Ensure sources are in this exact order:

```bash
# Base ROS2
source /opt/ros/humble/setup.bash

# Lab UR3e workspace (if applicable)
source /home/$USER/41068_ws/install/setup.bash

# This workspace
source ~/ros2_ws/install/setup.bash

# Gazebo model path (for simulation only)
export IGN_GAZEBO_RESOURCE_PATH=~/ros2_ws/src/realsense_sim/models:~/ros2_ws/install/realsense_sim/share/realsense_sim/models
```

Apply changes:

```bash
source ~/.bashrc
```

---

## Building the Workspace

Build all packages together from the workspace root:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select \
  aruco_ros \
  easy_handeye2 \
  easy_handeye2_demo \
  realsense_sim \
  perception_mapping
source install/setup.bash
```

**Expected output:**
```
Starting >>> aruco_ros
Starting >>> easy_handeye2
Starting >>> easy_handeye2_demo
Starting >>> perception_mapping
Starting >>> realsense_sim
Finished <<< aruco_ros [8.3s]
Finished <<< easy_handeye2 [5.1s]
Finished <<< easy_handeye2_demo [2.4s]
Finished <<< perception_mapping [3.2s]
Finished <<< realsense_sim [4.1s]

Summary: 5 packages finished
```

> If you only changed Python scripts (not C++ or CMake), you can rebuild just the perception package:
> ```bash
> colcon build --symlink-install --packages-select perception_mapping
> source install/setup.bash
> ```

---

## Task 1 — Testing ArUco Marker Detection

This tests that the ArUco detector correctly identifies markers and publishes their 3D poses. Run with either the physical D435i or a USB webcam.

### Step 1 — Print a test marker

Generate a marker at: https://chev.me/arucogen/

- **Dictionary:** `DICT_4X4_50`
- **Marker ID:** `0`
- **Marker size:** `50 mm` (print at exact physical size)

### Step 2 — Connect your camera

**Option A — Intel RealSense D435i** (connect to a USB 3.0 port, not USB 2.0):

```bash
realsense-viewer
```

Turn on "RGB Camera" and "Stereo Module" to confirm both streams are live, then close the viewer.

**Option B — USB Webcam** (fallback for offline testing):

Plug in any USB camera. No additional setup required.

---

### Running with USB Webcam (Option B)

**Terminal 1 — start the webcam node:**

```bash
source ~/.bashrc
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r /image_raw:=/camera/color/image_raw \
  -r /camera_info:=/camera/color/camera_info
```

**Expected output:**
```
[usb_cam-1] [INFO] [...] [usb_cam]: using device /dev/video0
[usb_cam-1] [INFO] [...] [usb_cam]: camera calibration file not found
```

**Terminal 2 — start the ArUco detector:**

```bash
source ~/.bashrc
ros2 run perception_mapping aruco_detector
```

**Expected output:**
```
[INFO] [...] [aruco_detector]: ArUco detector node started (OpenCV 4.5.x)
[INFO] [...] [aruco_detector]: Camera intrinsics received
```

Hold your printed marker in front of the webcam. You should see:

```
[INFO] [...] [aruco_detector]: Marker ID 0: x=0.032m  y=-0.015m  z=0.381m
[INFO] [...] [aruco_detector]: Publishing poses on /aruco/marker_0/pose
```

---

### Running with Physical Intel RealSense D435i (Option A)

You need **four terminals** open simultaneously.

**Terminal 1 — start the RealSense camera driver:**

```bash
source ~/.bashrc
ros2 launch realsense2_camera rs_launch.py
```

**Expected output:**
```
[camera-1] [INFO] [...] RealSense ROS v4.x.x
[camera-1] [INFO] [...] Device Serial No: XXXXXXXXXX
[camera-1] [INFO] [...] color stream: 640x480 @ 30fps
[camera-1] [INFO] [...] depth stream: 640x480 @ 30fps
```

Keep this terminal running before opening others.

**Terminal 2 — verify topic names, then start the detector:**

First check what the driver is publishing:

```bash
source ~/.bashrc
ros2 topic list | grep camera
```

**Expected output (double namespace is default):**
```
/camera/camera/color/camera_info
/camera/camera/color/image_raw
/camera/camera/depth/camera_info
/camera/camera/depth/image_rect_raw
```

Because the driver uses a double namespace (`/camera/camera/...`) but the detector subscribes to `/camera/...`, use the bridge node or remap directly:

**Option A1 — use the bridge node (recommended):**

```bash
source ~/.bashrc
ros2 run perception_mapping webcam_realsense_bridge
```

Then in a separate sub-terminal, run the detector without remapping:

```bash
source ~/.bashrc
ros2 run perception_mapping aruco_detector
```

**Option A2 — remap manually at launch:**

```bash
source ~/.bashrc
ros2 run perception_mapping aruco_detector --ros-args \
  -r /camera/color/image_raw:=/camera/camera/color/image_raw \
  -r /camera/color/camera_info:=/camera/camera/color/camera_info
```

**Expected detector output:**
```
[INFO] [...] [aruco_detector]: ArUco detector node started (OpenCV 4.5.x)
[INFO] [...] [aruco_detector]: Camera intrinsics received
```

Hold your printed DICT_4X4_50 ID 0 marker in front of the D435i:

```
[INFO] [...] [aruco_detector]: Marker ID 0: x=-0.004m  y=-0.011m  z=0.167m
[INFO] [...] [aruco_detector]: Publishing poses on /aruco/marker_0/pose
```

Multiple markers are detected simultaneously, each on its own topic:

```
[INFO] [...] [aruco_detector]: Marker ID 17: x=-0.627m  y=0.318m  z=1.115m
[INFO] [...] [aruco_detector]: Publishing poses on /aruco/marker_17/pose
```

**Terminal 3 — echo the pose topic:**

```bash
source ~/.bashrc
ros2 topic echo /aruco/marker_0/pose
```

**Expected output:**
```yaml
header:
  frame_id: camera_color_optical_frame
pose:
  position:
    x: -0.004    # metres left/right of camera centre
    y: -0.011    # metres up/down of camera centre
    z: 0.167     # metres away from the lens
  orientation:
    x: 0.012
    y: 0.704
    z: -0.008
    w: 0.710
```

**Terminal 4 — view the annotated debug image:**

```bash
source ~/.bashrc
ros2 run rqt_image_view rqt_image_view
```

Select `/aruco/debug_image` from the dropdown. You will see:
- A coloured border drawn around each detected marker
- Three axes at the marker centre: **Red** = X, **Green** = Y, **Blue** = Z (toward camera)

---

### Multi-Marker Detection

To detect all markers in the scene simultaneously:

```bash
source ~/.bashrc
ros2 run perception_mapping multi_aruco_detector
```

Each detected marker publishes to `/aruco/marker_<ID>/pose`. Confirmed IDs will be logged to the terminal.

---

### Puzzle Object Detection

To detect puzzle-specific objects (colour/shape-based, no marker required):

```bash
source ~/.bashrc
ros2 run perception_mapping puzzle_object_detector
```

Output is published to `/puzzle_objects/poses`.

---

### Detection Checklist

```
[ ]  Camera driver starts and streams without errors
[ ]  ros2 topic list shows /camera/.../image_raw publishing
[ ]  aruco_detector starts and prints "Camera intrinsics received"
[ ]  Holding ID 0 marker produces: Marker ID 0: x=X.XXXm y=X.XXXm z=X.XXXm
[ ]  ros2 topic echo /aruco/marker_0/pose shows valid PoseStamped data
[ ]  z value physically matches the real distance to the camera (within ~2 cm)
[ ]  rqt_image_view shows feed with axes drawn on marker
[ ]  Multiple marker IDs detected and published simultaneously
```

---

## Task 2 — Eye-in-Hand Calibration

Eye-in-hand calibration computes the fixed transform between the camera (mounted on the UR3e wrist) and the robot's end-effector frame. This transform is required for the robot to accurately reach targets detected by the camera.

**Prerequisites:**
- UR3e connected and running via `ur_robot_driver` (or MoveIt2 in simulation)
- Camera driver running (Terminal 1 from Task 1)
- A ChArUco calibration board or checkerboard visible to the camera

---

### Step 1 — Print a calibration board

Print a ChArUco board from: https://calib.io/pages/camera-calibration-pattern-generator

Recommended settings:
- **Type:** ChArUco
- **Rows:** 5, **Columns:** 7
- **Square size:** 30 mm, **Marker size:** 22 mm
- **Dictionary:** ARUCO_4X4

Fix the board flat on a rigid surface (not hand-held).

---

### Step 2 — Start all required nodes

You need **four terminals.**

**Terminal 1 — RealSense camera driver:**

```bash
source ~/.bashrc
ros2 launch realsense2_camera rs_launch.py
```

**Terminal 2 — ChArUco pose publisher:**

```bash
source ~/.bashrc
ros2 run perception_mapping charuco_pose_publisher
```

This publishes the board pose to `/charuco/pose` and a debug image to `/charuco/debug_image`.

Optionally, open the debug viewer to confirm the board is detected:

```bash
ros2 run perception_mapping charuco_debug_viewer
```

**Terminal 3 — UR3e robot and MoveIt2:**

If using the physical robot:
```bash
source ~/.bashrc
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e \
  robot_ip:=<ROBOT_IP>
```

If using MoveIt2 in simulation:
```bash
source ~/.bashrc
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e use_fake_hardware:=true
```

**Terminal 4 — Eye-in-hand calibration pipeline:**

```bash
source ~/.bashrc
ros2 launch perception_mapping calibrate.launch.py
```

This launches the `easy_handeye2` calibration node configured for eye-in-hand mode.

**Expected output:**
```
[handeye_calibration-1] [INFO] [...] HandEye calibration server started
[handeye_calibration-1] [INFO] [...] Waiting for robot pose and target pose...
```

---

### Step 3 — Collect calibration samples

Open RViz or the easy_handeye2 GUI. Move the robot arm to **at least 15 different poses** ensuring:

- The ChArUco board is visible in each pose
- Poses include varied orientations (tilted, rotated), not just translations
- No pose is in a joint singularity

At each pose, click **"Take Sample"** in the easy_handeye2 GUI. The terminal will confirm:

```
[handeye_calibration-1] [INFO] [...] Sample 1/15 recorded
[handeye_calibration-1] [INFO] [...] Sample 2/15 recorded
...
```

**Minimum recommended:** 15 samples. More samples = better accuracy.

---

### Step 4 — Compute and save the calibration

Once enough samples are collected, click **"Compute"** in the GUI. The result is the camera-to-end-effector transform:

```
[handeye_calibration-1] [INFO] [...] Calibration result:
  Translation: [x, y, z]
  Rotation (quaternion): [qx, qy, qz, qw]
```

Then click **"Save"**. The calibration is stored to:

```
~/.ros/easy_handeye2/<calibration_name>.yaml
```

---

### Step 5 — Publish the saved calibration transform

In a new terminal, publish the calibration so other nodes can use it via TF:

```bash
source ~/.bashrc
ros2 launch perception_mapping publish_calibration.launch.py
```

**Expected output:**
```
[static_transform_publisher-1] [INFO] [...] Spinning until stopped
[static_transform_publisher-1] [INFO] [...] Publishing transform: camera_color_optical_frame → tool0
```

Verify the transform is visible in the TF tree:

```bash
source ~/.bashrc
ros2 run tf2_tools view_frames
evince frames.pdf
```

The output PDF should show a chain from `base_link` → ... → `tool0` → `camera_color_optical_frame`.

---

### Calibration Checklist

```
[ ]  Camera driver running and streaming
[ ]  charuco_pose_publisher detects board (debug image shows corners)
[ ]  UR3e connected and MoveIt2 accepting motion goals
[ ]  calibrate.launch.py starts HandEye calibration server
[ ]  Minimum 15 samples collected across varied arm poses
[ ]  "Compute" returns a valid translation and quaternion
[ ]  "Save" writes .yaml file to ~/.ros/easy_handeye2/
[ ]  publish_calibration.launch.py starts without errors
[ ]  ros2 run tf2_tools view_frames shows camera frame in TF tree
[ ]  ros2 topic echo /aruco/marker_0/pose shows poses in robot base frame
```

---

## Running with Gazebo Ignition Fortress (Simulation Only)

When the physical camera is unavailable, the `realsense_sim` package provides a simulated D435i inside Gazebo.

**Terminal 1 — launch the full simulation:**

```bash
source ~/.bashrc
ros2 launch realsense_sim sim_perception_launch.py
```

This starts Gazebo, bridges camera topics to ROS2, and runs the ArUco detector automatically.

**Terminal 2 — verify topics:**

```bash
source ~/.bashrc
ros2 topic list | grep -E "camera|aruco"
```

**Expected:**
```
/aruco/debug_image
/camera/color/camera_info
/camera/color/image_raw
/camera/depth/color/points
/camera/depth/image_rect_raw
```

**Terminal 3 — echo pose:**

```bash
source ~/.bashrc
ros2 topic echo /aruco/marker_0/pose
```

---

## Node and Topic Reference

| Node | Launch / Run | Publishes | Subscribes |
|------|-------------|-----------|------------|
| `aruco_detector` | `ros2 run perception_mapping aruco_detector` | `/aruco/marker_N/pose`, `/aruco/debug_image` | `/camera/color/image_raw`, `/camera/color/camera_info` |
| `multi_aruco_detector` | `ros2 run perception_mapping multi_aruco_detector` | `/aruco/marker_N/pose` (all visible IDs) | `/camera/color/image_raw`, `/camera/color/camera_info` |
| `puzzle_object_detector` | `ros2 run perception_mapping puzzle_object_detector` | `/puzzle_objects/poses` | `/camera/color/image_raw` |
| `charuco_pose_publisher` | `ros2 run perception_mapping charuco_pose_publisher` | `/charuco/pose`, `/charuco/debug_image` | `/camera/color/image_raw`, `/camera/color/camera_info` |
| `charuco_debug_viewer` | `ros2 run perception_mapping charuco_debug_viewer` | — | `/charuco/debug_image` |
| `checkerboard_pose_publisher` | `ros2 run perception_mapping checkerboard_pose_publisher` | `/checkerboard/pose` | `/camera/color/image_raw`, `/camera/color/camera_info` |
| `webcam_realsense_bridge` | `ros2 run perception_mapping webcam_realsense_bridge` | `/camera/color/image_raw`, `/camera/color/camera_info` | `/camera/camera/color/image_raw`, `/camera/camera/color/camera_info` |
| `aruco_debug_viewer` | `ros2 run perception_mapping aruco_debug_viewer` | — | `/aruco/debug_image` |
| `calibrate.launch.py` | `ros2 launch perception_mapping calibrate.launch.py` | HandEye calibration GUI | robot TF + `/charuco/pose` |
| `publish_calibration.launch.py` | `ros2 launch perception_mapping publish_calibration.launch.py` | TF: `camera_color_optical_frame` → `tool0` | — |

---

## Troubleshooting

| Error | Cause | Fix |
|-------|-------|-----|
| `package 'perception_mapping' not found` | Not built or not sourced | Run `colcon build --symlink-install` then `source install/setup.bash` |
| `package 'realsense2_camera' not found` | Wrong source order in `~/.bashrc` | Ensure `source /opt/ros/humble/setup.bash` is first |
| `Camera intrinsics not received` | Topic namespace mismatch | Run `ros2 topic list \| grep camera` and use `webcam_realsense_bridge` or remap args |
| No marker detections | Wrong dictionary or marker not flat | Confirm marker is DICT_4X4_50, printed at correct size, held flat |
| `DetectorParameters() has no attribute` | OpenCV version mismatch | Confirm `python3 -c "import cv2; print(cv2.__version__)"` shows 4.5.x |
| D435i not detected in realsense-viewer | USB 2.0 port or MOK not enrolled | Use a USB 3.0 port; re-enroll MOK if needed |
| `easy_handeye2` calibration GUI not appearing | MoveIt2 not running | Ensure UR3e + MoveIt2 are up before launching calibrate.launch.py |
| Calibration accuracy poor | Too few samples or poses too similar | Collect ≥15 samples; ensure varied orientations, not just translations |
| TF chain broken after calibration | publish_calibration.launch.py not running | Launch it in a dedicated terminal before running perception nodes |
| `Unable to resolve uri[model://realsense_d435i]` | IGN_GAZEBO_RESOURCE_PATH not set | `source ~/.bashrc` and verify `echo $IGN_GAZEBO_RESOURCE_PATH` |
