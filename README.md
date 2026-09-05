# SO-101 Forward Kinematics & MoveIt Workspace

A ROS 2 (Jazzy) workspace for the 5-DoF (+ gripper) **SO-101 robot arm**:
URDF/kinematic description, `ros2_control` simulation, MoveIt 2 motion
planning, and a real-hardware bridge that lets MoveIt plan and execute
directly against the physical Feetech-servo arm.

---

## 📂 Repository Structure

```text
.
├── README.md                     # This guide
├── requirements.txt               # Python dependencies
├── setup.py / setup.sh / activate.sh   # Environment setup
├── test_installation.sh           # Installation verification utility
├── inspect_urdf.py                # URDF structure inspector tool
├── live_diagnostic.py             # Real-time ROS2 TF & joint states monitor
│
├── config/
│   └── so101_params.yaml          # Kinematic offsets and joint limit definitions
│
├── labs/
│   └── lab1_1_visualize_urdf.py   # URDF visualization walkthrough
│
└── ros2_ws/                       # ROS 2 workspace
    └── src/
        ├── lerobot_description/   # URDF/xacro, meshes, Gazebo launch
        ├── lerobot_controller/    # ros2_control controllers for sim/mock hardware
        ├── lerobot_moveit/        # MoveIt config, RViz, named poses, pose automation
        └── lerobot_hardware/      # Real-hardware bridge (Feetech servos over serial)
```

Package-level docs:
[`lerobot_moveit/README.md`](ros2_ws/src/lerobot_moveit/README.md) and
[`lerobot_hardware/README.md`](ros2_ws/src/lerobot_hardware/README.md).

---

## 🛠️ System Prerequisites

- **Operating System:** Ubuntu 22.04 LTS (Humble) or Ubuntu 24.04 LTS (Jazzy)
- **Python:** 3.10+
- **ROS 2:** Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04)
- **Git:** `sudo apt install git`
- For real-hardware use: an SO-101 arm on Feetech STS3215 servos, connected
  over USB serial (default `/dev/ttyACM0`)

---

## 🚀 Setup & Installation

### Step 1: Clone the Repository
```bash
git clone https://github.com/sruthihsr/Lerobot_forwardKinematics_moveit.git
cd Lerobot_forwardKinematics_moveit
```

### Step 2: Run the Setup Script
Detects your ROS 2 install, installs system dependencies, creates a Python
virtual environment, runs `rosdep`, and builds `ros2_ws`:
```bash
chmod +x setup.sh
./setup.sh
```
*You may be prompted for your sudo password to install system packages.*

### Step 3: Activate the Environment
Run this in every new terminal:
```bash
source activate.sh
```

### Step 4: Verify Installation
```bash
./test_installation.sh
```

---

## 📖 URDF Visualization

Understand the URDF, inspect the SO-101's physical dimensions and kinematic
links, and visualize joints/coordinate frames in RViz.

```bash
source activate.sh
ros2 launch lerobot_description so101_display.launch.py
```
A Joint State Publisher GUI and RViz window open — use the sliders to move
joints and watch the 3D model and TF tree update live.

Or inspect the parsed joint limits/offsets directly:
```bash
python3 labs/lab1_1_visualize_urdf.py
python3 inspect_urdf.py
```

---

## 🧮 Forward Kinematics (MuJoCo)

Standalone forward-kinematics lab — no ROS 2 needed, just `pip install
mujoco`. Loads the SO-101 MJCF model
(`ros2_ws/src/lerobot_description/mujoco/`, reusing the same meshes as the
URDF), sets joint angles, runs MuJoCo's FK pass, and prints the resulting
pose of every link plus the end-effector.

```bash
python3 labs/lab1_2_test_fk_mujoco.py            # run the built-in test poses
python3 labs/lab1_2_test_fk_mujoco.py --view      # also open an interactive 3D viewer
```

---

## 🦾 MoveIt Planning & Real-Hardware Control

Bring up MoveIt with mock hardware (no physical arm needed):
```bash
ros2 launch lerobot_moveit so101_moveit.launch.py
```

Bring up MoveIt against the **real physical arm**:
```bash
ros2 launch lerobot_moveit so101_moveit.launch.py \
  use_real_hardware:=True dry_run:=False hardware_port:=/dev/ttyACM0
```
See [`lerobot_hardware/README.md`](ros2_ws/src/lerobot_hardware/README.md)
for calibration and safety notes before running with real hardware, and
[`lerobot_moveit/README.md`](ros2_ws/src/lerobot_moveit/README.md) for the
named poses (`home`, `pose_1`-`pose_4`, `vertical`, `vertical_down`) and the
`pose_loop.py` script that cycles through them unattended with per-move
verification.

---

## 🔍 Diagnostic & Inspection Tools

**URDF structure** — kinematic chain, parent-child links, joint axes/limits:
```bash
python3 inspect_urdf.py
```

**Live ROS 2 diagnostic** — confirms `/joint_states` publishing, QoS, and
live TF transforms (run alongside an active RViz session):
```bash
python3 live_diagnostic.py
```

---

## 📚 References
- **Robot Arm Design:** [SO-ARM100 GitHub](https://github.com/TheRobotStudio/SO-ARM100)
- **ROS 2 Documentation:** [ROS 2 Documentation Portal](https://docs.ros.org/)
