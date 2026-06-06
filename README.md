# Welcome to the C.I.P.H.E.R. Project
 
> **C.I.P.H.E.R.** — *Collaborative Intelligence for Physical and Human Escape Room*
 
A VR escape room experience powered by a **UR3e collaborative robot arm**, built in Unity and controlled through a Meta Quest headset. Players solve physical puzzles inside a virtual world — with every action triggering real-world robot responses.

<p align="center">
<img width="250" height="300" alt="IMG_1134" src="https://github.com/user-attachments/assets/90fe3eb6-5fc8-4f72-868d-db967b8797d2" />
<img width="250" height="300" alt="IMG_1135" src="https://github.com/user-attachments/assets/c5651cc9-69eb-4b95-9384-bd43062cda1f" />
<img width="250" height="300" alt="Screenshot 2026-05-28 at 12 16 28 am" src="https://github.com/user-attachments/assets/10997d37-e53f-44e9-977f-be8aebc2b86e" />
</p>
<p align="center">
<em>Physical Puzzle Board and UR3e Robot Setup</em>
</p>

<p align="center">
  <img width="700" height="400" alt="Screenshot 2026-05-28 at 12 13 53 am" src="https://github.com/user-attachments/assets/d32eee45-ec41-43bd-bb57-3d8d7ce53544" /><br><em>VR World with Unity Editor (Main Room with Robot  + Puzzle Board)</em>
</p>


---

<p align="center">
<img width="400" height="280" alt="ezgif-4462f011c39bb6e9" src="https://github.com/user-attachments/assets/9c3997f8-fc3f-4602-af5c-7635af25bf5f" />
<img width="400" height="280" alt="teleop gif" src="https://github.com/user-attachments/assets/0def9149-4cf2-46db-84f7-b5977ab30ce0" />


</p>

---
 
## Project Context

You're trapped. The only way out?
A robot arm, a VR headset and your wits.
 
C.I.P.H.E.R. is an XR escape room where every puzzle you solve in the virtual world triggers a real robot arm in the physical one.
 
**Crack the code.**
**Trace the path.**
**Sort the eggs.**
**Tap the card.**
**Escape.**
 
**Expect to be met by:**
1. 🐸 Fellow frog friends
2. 🔢 Intuitive puzzle solving
3. 🤖 Engineering a trusty robot helper with your own controls
*Can you complete it before it's too late?*
---
 
## System Architecture
 
```
┌─────────────────────────────────────────────────────────────┐
│                     Player (Meta Quest VR)                  │
│              Unity VR World — Subsystem 3                   │
└───────────┬──────────────────────────────┬──────────────────┘
            |                              |
            v                              v
┌───────────────────┐          ┌───────────────────────┐
│   Subsystem 2     │          │     Subsystem 4       │
│ Motion & Planning │  ----->  │  XR / ROS Interface   │
│  MoveIt2, UR3e    │          │ Arduino, ROS Bridge   │
└───────────────────┘          └───────────┬───────────┘
                                           |
                                           v
                               ┌───────────────────────┐
                               │     Subsystem 1       │
                               │ Perception & Mapping  │
                               │  ArUco, Camera Feed   │
                               └───────────────────────┘
```
 
---
 
## The Subsystems
 
| Subsystem | Name | Responsibility |
|---|---|---|
| [Subsystem 1](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-1:-Perception-and-Mapping) | Perception and Mapping | Detects real-world robot and puzzle state via ArUco markers and camera |
| [Subsystem 2](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-2:-Motion-and-Planning) | Motion and Planning | Controls UR3e teleoperation, joint planning, collision avoidance via MoveIt2 |
| [Subsystem 3](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-3:-Interaction-and-Execution) | Interaction and Execution | Designs and runs the Unity VR environment, puzzles, UI and player interaction |
| [Subsystem 4](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-4:-Advance-Capabilities-(XR-ROS)) | Advanced Capabilities (XR/ROS) | Communication bridge between Unity and the robot — Arduino, ROS nodes, puzzle board |
 
---
 
## Quick Start
 
> [!IMPORTANT]
> All four subsystems must be running together for the full experience. For VR and real robot setup, start the subsystems in this order (for different modes of running the system, i.e. through Unity Editor/simulated robot, see [Subsystem 4](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-4:-Advance-Capabilities-(XR-ROS))):

Please view [these detailed robot control launch instructions](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-4:-Advance-Capabilities-(XR-ROS)#launch-commands-for-xr-teleoperation-default-with-gripper) first, before going through the following steps for better understanding.
 
| Step | Action | Subsystem |
|---|---|---|
| 1 | Open the XR Unity Project folder in Unity 6.3 Editor | [Subsystem 3](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-3:-Interaction-and-Execution) |
| 2 | Set Robotics -> ROS Settings -> IP Address to your ROS machine host IP | [Subsystem 4](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-4:-Advance-Capabilities-(XR-ROS)#link-to-hostname) |
| 3 | Ensure that Build Profile is set to Android and VR headset is connected via USB, and hit "Build and Run" to generate the apk and launch the world inside the headset | [Subsystem 3](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-3:-Interaction-and-Execution) |
| 4 | Start UR ROS Driver through the `ur_onrobot_control` package (for use with gripper): `ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 robot_ip:=<robot_ip>`| [Subsystem 4](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-4:-Advance-Capabilities-(XR-ROS)) |
| 5 | Start `ur_onrobot_group.launch.py` for MoveIt/Servo, ROS_TCP and controller switching nodes: `ros2 launch rs2_xr_puzzle ur_onrobot_group.launch.py ros_tcp_host:=<your_ros_host_machine_ip>`| [Subsystem 4](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-4:-Advance-Capabilities-(XR-ROS)) |
| 6 | Start Arduino puzzle bridge node to share puzzle solutions/current states over ROS to the headset| [Subsystem 4](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-4:-Advance-Capabilities-(XR-ROS)) |
| 7 | Start robot task node (`puzzle_task.py`) | [Subsystem 2](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-2:-Motion-and-Planning) |
| 8 | Start perception node | [Subsystem 1](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-1:-Perception-and-Mapping) |
 
> [!TIP]
> Each subsystem can also be run and tested independently — see each subsystem wiki page for standalone instructions.
 
---
 
## Wiki Pages
 
### Subsystem 1 — Perception and Mapping
- [Intro to Subsystem 1](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-1:-Perception-and-Mapping)
- [Detecting Multiple ArUco Marker IDs](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Detecting-Multiple-ArUco-Marker-IDs)
- [Eye-in-Hand Calibration](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Eye%E2%80%90in%E2%80%90Hand-Calibration)
- [Physical to VR Environment Synchronisation](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Physical-to-VR-Environment-Synchronisation)
- [Filtered Pose — Moving Average](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Filtered-Pose-%E2%80%90-Moving-Average)
### Subsystem 2 — Motion and Planning
- [Intro to Subsystem 2](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-2:-Motion-and-Planning)
### Subsystem 3 — Interaction and Execution
- [Intro to Subsystem 3](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-3:-Interaction-and-Execution)
- [Puzzle Rooms Overview](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Puzzle-Rooms-Overview)
### Subsystem 4 — Advanced Capabilities (XR/ROS)
- [Intro to Subsystem 4](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Subsystem-4:-Advance-Capabilities-(XR-ROS))
- [Puzzle Board Schematics & Assembly](https://github.com/Didi416/RoboticsStudio2_XRProject/wiki/Puzzle-Board-Schematics-Assembly)
---
 
## Repository
 
```bash
git clone https://github.com/Didi416/RoboticsStudio2_XRProject.git
```
 
**Built with:**
Unity 6.3 LTS · ROS2 Humble · Meta Quest · UR3e · OnRobot RG2 · Arduino Mega
 
---
 
*Autumn 2026 · 41069 UTS Robotics Studio 2 · EscapeXRtists*
 
