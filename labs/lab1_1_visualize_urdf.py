#!/usr/bin/env python3
"""
Lab 1.1: Visualize SO-101 URDF
==============================

This lab introduces URDF visualization using ROS2 and RViz.

URDF (Unified Robot Description Format) is an XML format that describes:
    - Robot links (rigid bodies)
    - Joints (connections between links)
    - Visual meshes
    - Collision geometry
    - Inertial properties

Prerequisites:
    1. ROS2 Jazzy installed
    2. lerobot_ws workspace built
    
Usage:
    # Method 1: Launch file (recommended)
    ros2 launch lerobot_description so101_display.launch.py
    
    # Method 2: Manual launch (for learning)
    python labs/lab1_1_visualize_urdf.py

Learning Objectives:
    1. Understand URDF structure and components
    2. Visualize robot structure in RViz
    3. Explore joint states and TF tree

Author: SO-101 Robotics Course
"""

import sys
import os

print("=" * 70)
print("Lab 1.1: Visualize SO-101 URDF")
print("=" * 70)

# Check for ROS2
ROS2_AVAILABLE = False
try:
    import rclpy
    ROS2_AVAILABLE = True
except ImportError:
    pass

if not ROS2_AVAILABLE:
    print("""
ROS2 is required for this lab. Here's how to proceed:

Option 1: Install and use ROS2 (Full Experience)
------------------------------------------------
1. Install ROS2:
   - Ubuntu 22.04: Install ROS2 Humble
     https://docs.ros.org/en/humble/Installation.html
   - Ubuntu 24.04: Install ROS2 Jazzy
     https://docs.ros.org/en/jazzy/Installation.html

2. Clone this workspace:
   git clone https://github.com/sruthihsr/Lerobot_forwardKinematics_moveit.git
   cd Lerobot_forwardKinematics_moveit

3. Build the workspace:
   cd ..
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   source install/setup.bash

4. Launch the visualization:
   ros2 launch lerobot_description so101_display.launch.py

5. In RViz:
   - The robot model should appear automatically
   - Use the Joint State Publisher GUI to move joints
   - Observe TF frames in the TF display
   - Explore the robot structure

Option 2: View URDF Structure (No ROS2 Required)
------------------------------------------------
The SO-101 URDF structure is explained below.
""")
    
    print("\n" + "-" * 70)
    print("SO-101 URDF Structure")
    print("-" * 70)
    
    urdf_structure = """
    SO-101 Robot URDF Structure
    ===========================
    
    Links (Rigid Bodies):
    ---------------------
    base_link           - Fixed base attached to world
    link0               - Base rotation platform
    link1               - First arm segment (shoulder)
    link2               - Upper arm
    link3               - Forearm
    link4               - Wrist body
    link5               - Wrist end
    gripper_base        - Gripper mount
    fixed_jaw           - Stationary gripper finger
    moving_jaw          - Actuated gripper finger
    
    Joints:
    -------
    shoulder_pan        - Rotates about Z (vertical axis)
                         Range: -180° to +180°
                         
    shoulder_lift       - Rotates about Y (horizontal axis)
                         Range: -90° to +90°
                         
    elbow_flex          - Rotates about Y
                         Range: -135° to +135°
                         
    wrist_flex          - Rotates about Y
                         Range: -90° to +90°
                         
    wrist_roll          - Rotates about X (along forearm)
                         Range: -180° to +180°
                         
    gripper             - Prismatic joint (sliding)
                         Range: 0mm to 34mm opening
    
    Kinematic Chain:
    ----------------
    world → base_link → link0 → link1 → link2 → link3 → link4 → link5 → gripper
                  ↓        ↓        ↓        ↓        ↓        ↓        ↓
              (fixed)   (pan)   (lift)  (elbow) (wrist) (roll)  (grip)
    
    Coordinate Frames:
    ------------------
    - Base frame: At the bottom of the robot base
    - Each joint has an associated frame
    - Tool frame: At the center of the gripper
    - Frames follow right-hand rule:
      X = Red, Y = Green, Z = Blue
    
    Key Dimensions (meters):
    ------------------------
    - Base height: 0.0624
    - Upper arm: 0.11257 (vertical)
    - Forearm: 0.1349 (horizontal)
    - Wrist: 0.0611
    - Gripper: 0.1034
    - Total reach: ~0.45 (extended)
    """
    
    print(urdf_structure)
    
    print("\n" + "-" * 70)
    print("URDF XML Example")
    print("-" * 70)
    
    urdf_example = '''
    <!-- Example URDF snippet for shoulder_pan joint -->
    <joint name="shoulder_pan" type="revolute">
        <parent link="link0"/>
        <child link="link1"/>
        <origin xyz="0 0 -0.0303992" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>  <!-- Rotation about Z -->
        <limit lower="-3.14159" upper="3.14159" effort="10" velocity="1"/>
    </joint>
    
    <link name="link1">
        <visual>
            <geometry>
                <mesh filename="package://lerobot_description/meshes/link1.stl"/>
            </geometry>
            <material name="robot_color"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.05 0.04 0.06"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.1"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
    </link>
    '''
    
    print(urdf_example)
    
    print("\n" + "-" * 70)
    print("What You'll See in RViz")
    print("-" * 70)
    print("""
    When you launch the visualization, RViz will show:
    
    1. Robot Model
       - 3D visualization of all links
       - Colored based on URDF materials
       - Updates as joints move
    
    2. TF Tree
       - Coordinate frame for each link
       - Shows parent-child relationships
       - Frames update in real-time
    
    3. Joint State Publisher GUI
       - Sliders for each joint
       - Move sliders to see robot respond
       - Values shown in radians
    
    4. RViz Panels
       - Displays: Configure what to show
       - Views: Change camera angle
       - Tools: Interact with the scene
    
    Try these exercises:
    - Move shoulder_pan slider to rotate the base
    - Move shoulder_lift to see upper arm move
    - Observe how TF frames rotate with joints
    - Look at the tool frame position as you move joints
    """)
    
    sys.exit(0)


# If ROS2 is available, we can do more
import rclpy
from rclpy.node import Node
import subprocess
import time

def check_urdf_exists():
    """Check if the URDF file exists."""
    possible_paths = [
        os.path.expanduser('~/lerobot_ws/src/lerobot_description/urdf/so101.urdf'),
        '/opt/ros/jazzy/share/lerobot_description/urdf/so101.urdf',
        os.path.join(os.path.dirname(__file__), '..', 'ros2_ws', 'src', 
                     'lerobot_description', 'urdf', 'so101.urdf'),
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            return path
    return None


def main():
    print("\nROS2 detected! Let's visualize the robot.\n")
    
    urdf_path = check_urdf_exists()
    
    if urdf_path:
        print(f"Found URDF at: {urdf_path}")
        print("\nLaunching RViz visualization...")
        print("(If the launch file isn't available, we'll try manual launch)\n")
    else:
        print("URDF not found. You need to build the ros2_ws workspace first.")
        print("\nSteps:")
        print("  1. cd ros2_ws")
        print("  2. colcon build")
        print("  3. source install/setup.bash")
        print("  6. ros2 launch lerobot_description so101_display.launch.py")
        return
    
    # Try to launch the display
    try:
        print("Running: ros2 launch lerobot_description so101_display.launch.py")
        subprocess.run([
            'ros2', 'launch', 'lerobot_description', 'so101_display.launch.py'
        ], check=True)
    except FileNotFoundError:
        print("\nLaunch file not found. Make sure you've built and sourced the workspace.")
    except KeyboardInterrupt:
        print("\nVisualization closed.")


if __name__ == "__main__":
    main()
