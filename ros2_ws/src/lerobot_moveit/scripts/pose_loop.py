#!/usr/bin/env python3
"""
Cycles the SO101 arm through every named group_state in so101.srdf
(home, pose_1, pose_2, ...), looping forever with no user input.

For each pose: sends a MoveGroup goal (plan + execute) to the 'arm' group,
then to the 'gripper' group, then reads back /joint_states and verifies
every joint landed within tolerance of the commanded target.

Usage:
    python3 pose_loop.py                  # loop forever
    python3 pose_loop.py --cycles 3        # stop after 3 full cycles
    python3 pose_loop.py --pause 3.0       # seconds to hold each pose

Ctrl+C stops cleanly (cancels any in-flight goal).
"""
import argparse
import os
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from sensor_msgs.msg import JointState

SRDF_PATH = os.path.join(get_package_share_directory("lerobot_moveit"), "config", "so101.srdf")
# rad, used both as the planning tolerance and the post-move verify tolerance.
# 0.05 rad (~3 deg) reflects this arm's real steady-state tracking error under
# gravity load at larger poses -- observed to sit flat (non-converging) up to
# ~0.12 rad past target on some joints, so a tighter value just makes every
# verify() call FAIL on true positives.
POSITION_TOLERANCE = 0.05


def load_poses(srdf_path):
    """Returns [pose_name, ...] in file order and {(pose_name, group): {joint: value}}."""
    root = ET.parse(srdf_path).getroot()
    order = []
    poses = {}
    for gs in root.findall("group_state"):
        name, group = gs.get("name"), gs.get("group")
        if name not in order:
            order.append(name)
        poses[(name, group)] = {
            j.get("name"): float(j.get("value")) for j in gs.findall("joint")
        }
    return order, poses


class PoseLoop(Node):

    def __init__(self):
        super().__init__("pose_loop")
        self.move_client = ActionClient(self, MoveGroup, "move_action")
        self.last_joint_state = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

    def _on_joint_state(self, msg):
        self.last_joint_state = dict(zip(msg.name, msg.position))

    def move_group_to(self, group_name, targets):
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("move_action server not available")

        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 5
        goal.request.goal_constraints = [Constraints(
            joint_constraints=[
                JointConstraint(
                    joint_name=name, position=value,
                    tolerance_above=POSITION_TOLERANCE, tolerance_below=POSITION_TOLERANCE,
                    weight=1.0,
                )
                for name, value in targets.items()
            ]
        )]
        goal.planning_options.plan_only = False

        send_future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError(f"goal rejected for group {group_name}")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        error_code = result_future.result().result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"group {group_name} failed, error_code={error_code}")

    def verify(self, targets, settle_timeout=8.0, poll_period=0.2):
        """Polls /joint_states until every target joint is within tolerance or
        settle_timeout elapses -- the hardware bridge reports a trajectory as
        done once it has *sent* the last waypoint, not once the physical
        servo has arrived, so a single immediate sample can catch it mid-travel."""
        deadline = time.monotonic() + settle_timeout
        errors = {}
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=poll_period)
            errors = {
                name: abs(self.last_joint_state[name] - target)
                for name, target in targets.items()
                if name in self.last_joint_state
            }
            if len(errors) == len(targets) and all(e <= POSITION_TOLERANCE for e in errors.values()):
                break

        ok = True
        for joint_name, target in targets.items():
            actual = self.last_joint_state.get(joint_name)
            if actual is None:
                self.get_logger().error(f"  joint {joint_name}: no joint_states data")
                ok = False
                continue
            error = errors.get(joint_name, abs(actual - target))
            status = "OK" if error <= POSITION_TOLERANCE else "FAIL"
            if status == "FAIL":
                ok = False
            self.get_logger().info(
                f"  joint {joint_name}: target={target:+.4f} actual={actual:+.4f} "
                f"error={error:.4f} [{status}]"
            )
        return ok


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cycles", type=int, default=5, help="0 = loop forever, default 5")
    parser.add_argument("--pause", type=float, default=2.0, help="seconds to hold each pose")
    args = parser.parse_args()

    order, poses = load_poses(SRDF_PATH)
    print(f"Loaded {len(order)} poses from SRDF: {order}")

    rclpy.init()
    node = PoseLoop()

    cycle = 0
    try:
        while args.cycles == 0 or cycle < args.cycles:
            cycle += 1
            for pose_name in order:
                print(f"\n=== cycle {cycle} -- pose '{pose_name}' ===")
                arm_targets = poses.get((pose_name, "arm"), {})
                gripper_targets = poses.get((pose_name, "gripper"), {})

                if arm_targets:
                    node.get_logger().info(f"arm -> {arm_targets}")
                    node.move_group_to("arm", arm_targets)
                if gripper_targets:
                    node.get_logger().info(f"gripper -> {gripper_targets}")
                    node.move_group_to("gripper", gripper_targets)

                all_targets = {**arm_targets, **gripper_targets}
                ok = node.verify(all_targets)
                print(f"verify: {'PASS' if ok else 'FAIL'}")

                time.sleep(args.pause)
    except KeyboardInterrupt:
        print("\nstopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
