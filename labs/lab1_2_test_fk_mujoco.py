#!/usr/bin/env python3
"""
Lab 1.2: Forward Kinematics with MuJoCo
========================================

Computes forward kinematics (joint angles -> end-effector pose) for the
SO-101 arm using MuJoCo, standalone (no ROS2 needed).

Forward kinematics answers: given a set of joint angles, where is the
gripper (and every link in between) in 3D space? MuJoCo computes this via
its kinematic tree the same way any FK library would -- this lab exposes
that computation directly instead of hiding it behind a planning library.

Prerequisites:
    pip install mujoco

Usage:
    python3 labs/lab1_2_test_fk_mujoco.py            # run the built-in test poses
    python3 labs/lab1_2_test_fk_mujoco.py --view      # also open an interactive 3D viewer

Learning Objectives:
    1. Load a MuJoCo model (MJCF) and identify its kinematic tree
       (bodies, joints, sites).
    2. Set joint positions and run mujoco's forward-kinematics pass.
    3. Read the resulting pose of any link (here: the gripper) out of the
       computed model state.
"""

import argparse
import os

import mujoco
import mujoco.viewer
import numpy as np

MODEL_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "ros2_ws", "src", "lerobot_description", "mujoco", "scene.xml",
)

# Order matches the model's qpos layout (see JOINT_NAMES below) -- these are
# the same 6 joints the ROS side calls "1".."6" (lerobot_moveit/so101.srdf).
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
END_EFFECTOR_SITE = "gripperframe"

# A handful of representative joint configurations, in radians, ordered per
# JOINT_NAMES. All zeros is the model's home/zero pose.
TEST_POSES = {
    "home (all zero)": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "elbow bent up": [0.0, -0.3, 0.9, 0.0, 0.0, 0.0],
    "reach left, wrist rolled": [0.6, -0.2, 0.5, -0.3, 1.2, 0.3],
    "near shoulder_pan limit": [1.8, 0.0, 0.0, 0.0, 0.0, 0.0],
}


def quat_to_rpy(quat_wxyz):
    """MuJoCo quaternions are (w, x, y, z); returns (roll, pitch, yaw) in radians."""
    w, x, y, z = quat_wxyz
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def forward_kinematics(model, data, joint_positions):
    """Sets joint_positions (radians, ordered per JOINT_NAMES), runs FK, and
    returns (position_xyz, quaternion_wxyz) of the end-effector site."""
    for name, value in zip(JOINT_NAMES, joint_positions):
        data.qpos[model.joint(name).qposadr[0]] = value

    mujoco.mj_forward(model, data)  # the actual FK computation

    site_id = model.site(END_EFFECTOR_SITE).id
    return data.site_xpos[site_id].copy(), data.xquat[model.site_bodyid[site_id]].copy()


def print_link_chain(model, data):
    """Prints the position of every body in the kinematic chain -- shows FK
    isn't just one number, it's a pose computed for every link."""
    print(f"{'link':<14}{'x':>10}{'y':>10}{'z':>10}")
    for body_id in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if not name:
            continue
        x, y, z = data.xpos[body_id]
        print(f"{name:<14}{x:>10.4f}{y:>10.4f}{z:>10.4f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--view", action="store_true", help="open an interactive 3D viewer")
    args = parser.parse_args()

    print("=" * 70)
    print("Lab 1.2: Forward Kinematics with MuJoCo")
    print("=" * 70)

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    print(f"Loaded model: {MODEL_PATH}")
    print(f"Joints ({model.njnt}): {JOINT_NAMES}")
    print(f"End-effector site: '{END_EFFECTOR_SITE}'\n")

    for pose_name, joint_positions in TEST_POSES.items():
        print("-" * 70)
        print(f"Pose: {pose_name}")
        print(f"  joint angles (rad): {dict(zip(JOINT_NAMES, joint_positions))}")

        position, quat = forward_kinematics(model, data, joint_positions)
        roll, pitch, yaw = quat_to_rpy(quat)
        print(f"  end-effector position (m):     x={position[0]:+.4f} y={position[1]:+.4f} z={position[2]:+.4f}")
        print(f"  end-effector orientation (rad): roll={roll:+.4f} pitch={pitch:+.4f} yaw={yaw:+.4f}")
        print()
        print_link_chain(model, data)
        print()

    if args.view:
        print("Opening interactive viewer -- drag joints or close the window to exit.")
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()


if __name__ == "__main__":
    main()
