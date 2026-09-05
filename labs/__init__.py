"""
SO-101 Robotics Course Labs
===========================

Lab exercises for lectures 1-3 covering classical robotics fundamentals.

Lecture 1: Forward Kinematics
-----------------------------
- Lab 1.1: Visualize SO-101 URDF (lab1_1_visualize_urdf.py)
- Lab 1.2: Implement and test FK (lab1_2_test_fk_mujoco.py, lab1_2_test_fk_rviz.py)

Lecture 2: Inverse Kinematics
-----------------------------
- Lab 2.1: Numerical IK implementation (lab2_1_test_ik_mujoco.py)
- Lab 2.2: LeRobot's built-in kinematics (lab2_2_lerobot_kinematics.py)

Lecture 3: Simulation & Motion Planning
---------------------------------------
- Lab 3.x: Pick and place (lab3_pick_place_mujoco.py, lab3_pick_place_gazebo.py)

Quick Start:
-----------
# MuJoCo (standalone, no ROS2 needed):
python labs/lab1_2_test_fk_mujoco.py
python labs/lab2_1_test_ik_mujoco.py
python labs/lab3_pick_place_mujoco.py

# ROS2/RViz (requires ROS2 installation):
ros2 launch lerobot_description so101_display.launch.py
python labs/lab1_2_test_fk_rviz.py
python labs/lab3_pick_place_gazebo.py
"""

__all__ = [
    'lab1_1_visualize_urdf',
    'lab1_2_test_fk_mujoco',
    'lab1_2_test_fk_rviz',
    'lab2_1_test_ik_mujoco',
    'lab2_2_lerobot_kinematics',
    'lab3_pick_place_mujoco',
    'lab3_pick_place_gazebo',
]
