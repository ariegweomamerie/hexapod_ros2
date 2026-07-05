# Hexapod Robot — ROS 2

A six-legged (hexapod) walking robot built on **ROS 2 Jazzy** and **Gazebo Sim 8**.
It stands, balances, and **walks with a tripod gait** driven by per-leg inverse
kinematics — all commandable through a standard `/cmd_vel` velocity interface.

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Sim_8-FF6600?logo=gazebo&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green.svg)
![Status](https://img.shields.io/badge/status-walking_in_sim-brightgreen.svg)

---

## Demo

<p align="center">
  <img src="demo/hexapod_walking.gif" alt="Hexapod walking in Gazebo" width="640"><br>
  <em><b>Figure 1.</b> Gazebo camera view — the hexapod walking forward (−Y, toward the
  face) with the alternating tripod gait. <b>Placeholder:</b> record the Gazebo camera
  with its video button (or <code>peek</code>/<code>byzanz</code>) and save it as
  <code>demo/hexapod_walking.gif</code>.</em>
</p>

<p align="center">
  <img src="demo/forward_walk.png" alt="Forward-walk performance plot" width="680"><br>
  <em><b>Figure 2.</b> Forward-walk performance, measured live from Gazebo: forward
  distance vs. time for a <code>linear.x</code> command. The robot holds a steady,
  near-linear cruise of ≈ 1.5 cm/s.</em>
</p>

---

## Overview

This project turns a CAD-exported hexapod model into a fully actuated, walking
robot in simulation, structured so the same control stack can later drive real
servos.

**Features**
- 🦿 **20 degrees of freedom** — 6 legs × 3 joints (coxa / femur / tibia) + a 2-DOF pan/tilt "face".
- 🎮 **`ros2_control`** integration via `gz_ros2_control` (position-controlled joints).
- 🕷️ **Tripod walking gait** — two alternating tripods, D-shaped foot trajectories.
- 📐 **Per-leg inverse kinematics** — damped least-squares (KDL Jacobian), robust to the model's non-orthogonal joint axes.
- 🧭 **Velocity interface** — drive forward/strafe/turn with `geometry_msgs/Twist` on `/cmd_vel`.
- 👁️ **RViz + Gazebo** launch files for visualization and physics simulation.

---

## The robot

| Property | Value |
|---|---|
| Legs | 6 (3 per side), 3 DOF each |
| Leg joints | `coxa` (yaw) → `femur` (lift) → `tibia` (knee) |
| Extra DOF | `face_pan`, `face_tilt` |
| Total DOF | 20 |
| Leg segment (foot reach) | ≈ 0.179 m |
| Ride height (stance) | ≈ 0.13 m |
| **Front direction** | **−Y** (toward the face/head) |

**Body frame** (`base_footprint`): front = −Y, rear = +Y, lateral = ±X.
Front legs `l1`/`r1`, mid legs `l2`/`r2`, rear legs `l3`/`r3`.

---

## Repository structure

```text
hexapod_ros_robot_ws/
├── src/
│   ├── Hexapod_Robot_description/   # URDF/xacro, meshes, ros2_control, Gazebo, RViz, launch
│   │   ├── urdf/                    # robot model + .ros2control + .gazebo + materials
│   │   ├── meshes/                  # STL visual/collision meshes
│   │   ├── config/                  # controllers.yaml, RViz configs, gz bridge
│   │   └── launch/                  # display.launch.py, gazebo.launch.py
│   └── hexapod_gait/                # walking gait + kinematics (Python)
│       ├── hexapod_gait/
│       │   ├── kinematics.py        # FK/IK for all six legs (PyKDL)
│       │   └── gait_node.py         # tripod gait -> JointTrajectory
│       └── launch/                  # gait.launch.py
├── demo/                            # recorded walk data + performance plot
├── run_display.sh                   # RViz launch helper (see Troubleshooting)
└── run_gazebo.sh                    # Gazebo launch helper (see Troubleshooting)
```

---

## Prerequisites

- **Ubuntu 24.04** with **ROS 2 Jazzy**
- **Gazebo Sim 8** (Harmonic) + `ros_gz`
- `ros2_control`, `gz_ros2_control`, `joint_trajectory_controller`, `joint_state_broadcaster`
- Python: `PyKDL`, `urdfdom_py`, `numpy`

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-publisher-gui python3-pykdl ros-jazzy-urdfdom-py
```

---

## Build

```bash
cd ~/hexapod_ros_robot_ws
colcon build
source install/setup.bash
```

---

## Usage

### 1. Visualize in RViz
```bash
ros2 launch Hexapod_Robot_description display.launch.py
```
Move the joint sliders to pose the robot.

### 2. Simulate in Gazebo (robot stands under `ros2_control`)
```bash
ros2 launch Hexapod_Robot_description gazebo.launch.py
```

### 3. Start the walking gait
In a second terminal:
```bash
ros2 launch hexapod_gait gait.launch.py
```

### 4. Drive it
Publish a `geometry_msgs/Twist` on `/cmd_vel` using the standard convention
(`linear.x` = forward, `linear.y` = strafe left, `angular.z` = turn):
```bash
# walk forward (toward the face)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.06}}'

# strafe left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {y: 0.06}}'

# turn in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.3}}'

# stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

Or drive it with the keyboard (the `i`/`,`/`j`/`l` keys map to `linear.x`/`angular.z`):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## How it works

```text
/cmd_vel ──► gait_node ──► per-leg foot targets ──► IK ──► 18 joint angles
   (Twist)      │  (tripod phase clock)                        │
               HOME                                             ▼
          (neutral stance)                 /leg_controller/joint_trajectory ──► Gazebo
```

1. **Description** (`Hexapod_Robot_description`) — the URDF/xacro defines links,
   joints, inertials, meshes, six mesh-derived **foot frames**, the `ros2_control`
   interfaces, and the Gazebo/`gz_ros2_control` plugin.
2. **Kinematics** (`kinematics.py`) — builds a KDL chain per leg
   (`base_footprint → leg_*_foot`) and solves position-only IK with a
   step-clamped, joint-limit-aware damped least-squares iteration.
3. **Gait** (`gait_node.py`) — an alternating **tripod** (A = `l1,l3,r2`,
   B = `r1,r3,l2`). Each foot follows a D-shaped cycle: a ground stroke (stance,
   propels the body) and a lifted arc (swing). Foot targets are converted to
   joint angles by IK and streamed to the `leg_controller` at 50 Hz.

**Controllers:** `joint_state_broadcaster`, `leg_controller` (18 leg joints) and
`face_controller` (2 joints), all `JointTrajectoryController`.

---

## Roadmap

- [x] URDF model + RViz visualization
- [x] `ros2_control` + Gazebo — robot stands
- [x] Foot frames + per-leg inverse kinematics
- [x] Tripod gait — **walks forward in simulation**
- [x] Intuitive `/cmd_vel` mapping (`linear.x` = forward)
- [ ] Faster gait + reliable turning
- [ ] Keyboard/joystick teleop
- [ ] Body pose control (lean, height, orientation)
- [ ] Real-hardware servo interface

---

## Known limitations

- Walking speed is conservative (~1.5 cm/s); most of the commanded stride is lost
  to foot slip. Better foot contact geometry and gait timing would increase it.
- Forward walking and turning both work, but show some sideways/heading drift
  (the gait is open-loop — no body-pose feedback yet).
- `step_height` is capped ~0.035 m by the femur joint's limited range.

---

## Troubleshooting

**RViz/Gazebo crash with `undefined symbol: __libc_pthread_init` (snap terminals).**
If you launch from a VS Code **snap** integrated terminal, snap-injected library
paths can crash native GUI apps. Launch from a normal terminal, or use the
provided helpers that scrub the environment:
```bash
./run_display.sh     # RViz
./run_gazebo.sh      # Gazebo
```

---

## Contributing

Issues and pull requests are welcome. Please keep changes focused and match the
existing code style. For gait/kinematics changes, verify in Gazebo before
submitting.

---

## License

Released under the **MIT License**. See [LICENSE](LICENSE).

## Acknowledgements

Built with [ROS 2](https://www.ros.org/), [Gazebo](https://gazebosim.org/),
[`ros2_control`](https://control.ros.org/), and [Orocos KDL](https://www.orocos.org/kdl.html).
