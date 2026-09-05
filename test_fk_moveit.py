#!/usr/bin/env python3
"""
Move + Forward Kinematics loop via MoveIt
==========================================

Cycles through a list of joint configurations for the "arm" planning group:
for each one, sends a MoveGroup goal (plan + execute) to actually move the
robot/simulation, then calls MoveIt's GetPositionFK service to print the
resulting end-effector pose. Repeats forever until Ctrl+C.

The robot is bolted to a fixed clamp, so joint "1" (base yaw) must never
rotate: it is read once from /joint_states at startup and then held fixed
via both a goal constraint AND a path constraint on every MoveGroup request,
so the planner can't pass through other base angles mid-trajectory either.
Only joints "2".."5" are varied. Override the detected value with
--base-position if you want to lock to something other than the arm's
current physical angle.

Prerequisites:
    1. Build/source this workspace (see activate.sh)
    2. Start move_group (+ controllers) in another terminal:
           ros2 launch lerobot_moveit so101_moveit.launch.py
       and make sure the controller manager / fake or real hardware is up
       so that FollowJointTrajectory goals on arm_controller are accepted.

Usage:
    python3 test_fk_moveit.py [--once] [--pause SECONDS] [--base-position RAD]

    --once            run through the configuration list a single time
    --pause SECONDS   seconds to wait after each move before the next one
                       (default: 1.0)
    --base-position   lock joint "1" to this value (rad) instead of reading
                       its current position from /joint_states at startup
"""

import argparse
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, Constraints, JointConstraint
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionFK

# Joint order matches the URDF/SRDF joint names (numeric, not the
# shoulder_pan/shoulder_lift/... aliases used by the lerobot hardware bridge).
ARM_GROUP = "arm"
ARM_JOINT_NAMES = ["1", "2", "3", "4", "5"]

# Base yaw is physically clamped down and must stay fixed. Everything below
# only varies the remaining 4 joints; joint "1"'s value is filled in from
# LOCKED_JOINT_POSITION (set at startup -- see lock_base_joint()).
LOCKED_JOINT_NAME = "1"
FREE_JOINT_NAMES = ["2", "3", "4", "5"]
LOCKED_JOINT_TOLERANCE = 0.001  # rad, applied to both goal and path constraint

# A handful of arm configurations spanning the joint limits (radians), taken
# from so101_base.xacro's <limit> tags, for joints 2-5 only. Gripper joint
# "6" is left untouched.
CONFIGURATIONS = {
    "home": [0.0, 0.0, 0.0, 0.0],
    "reach_forward": [0.7, -0.9, 0.3, 0.0],
    "reach_up": [-1.2, 1.2, 0.5, 0.0],
    "twisted_wrist": [0.4, -0.6, -0.8, 1.5],
}

# Links to query the pose of via FK after each move.
FK_LINK_NAMES = ["gripper", "jaw"]

# Must match the MoveIt planning frame (the robot model's root link).
PLANNING_FRAME = "world"

JOINT_TOLERANCE = 0.001


def quaternion_to_euler_deg(x, y, z, w):
    """Convert a quaternion to roll/pitch/yaw in degrees."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def _locked_joint_constraint(locked_position):
    jc = JointConstraint()
    jc.joint_name = LOCKED_JOINT_NAME
    jc.position = locked_position
    jc.tolerance_above = LOCKED_JOINT_TOLERANCE
    jc.tolerance_below = LOCKED_JOINT_TOLERANCE
    jc.weight = 1.0
    return jc


class MoveAndFK(Node):

    def __init__(self):
        super().__init__("move_and_fk")
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self._joint_state = None
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

    def _joint_state_callback(self, msg):
        self._joint_state = msg

    def wait_for_servers(self, timeout_sec=10.0):
        ok = True
        if not self.move_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("/move_action action server not available.")
            ok = False
        if not self.fk_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error("/compute_fk service not available.")
            ok = False
        if not ok:
            self.get_logger().error(
                "Start move_group first:\n"
                "    ros2 launch lerobot_moveit so101_moveit.launch.py"
            )
        return ok

    def get_current_joint_position(self, joint_name, timeout_sec=10.0):
        """Spin until a /joint_states message containing joint_name arrives,
        return its current position (rad)."""
        end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
        while self.get_clock().now() < end_time:
            if self._joint_state is not None and joint_name in self._joint_state.name:
                idx = self._joint_state.name.index(joint_name)
                return self._joint_state.position[idx]
            rclpy.spin_once(self, timeout_sec=0.2)
        return None

    def move_to(self, free_positions, locked_position):
        """Send a plan+execute MoveGroup goal: FREE_JOINT_NAMES set to
        free_positions, LOCKED_JOINT_NAME pinned at locked_position for the
        whole path (not just the goal). Blocks until the goal finishes.
        Returns the MoveItErrorCodes value."""
        goal = MoveGroup.Goal()
        goal.request.group_name = ARM_GROUP
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5

        goal_constraints = Constraints()
        goal_constraints.joint_constraints.append(_locked_joint_constraint(locked_position))
        for name, position in zip(FREE_JOINT_NAMES, free_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = JOINT_TOLERANCE
            jc.tolerance_below = JOINT_TOLERANCE
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(goal_constraints)

        # Path constraint keeps joint "1" pinned at every waypoint, not just
        # at the final state -- a goal constraint alone would let the
        # planner swing the clamped base joint mid-trajectory.
        path_constraints = Constraints()
        path_constraints.joint_constraints.append(_locked_joint_constraint(locked_position))
        goal.request.path_constraints = path_constraints

        goal.planning_options.plan_only = False

        send_future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal rejected.")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            return None
        return result.result.error_code.val

    def compute_fk(self, free_positions, locked_position):
        request = GetPositionFK.Request()
        request.header.frame_id = PLANNING_FRAME
        request.header.stamp = self.get_clock().now().to_msg()
        request.fk_link_names = FK_LINK_NAMES

        joint_state = JointState()
        joint_state.name = [LOCKED_JOINT_NAME] + FREE_JOINT_NAMES
        joint_state.position = [locked_position] + list(free_positions)
        request.robot_state = RobotState(joint_state=joint_state)

        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def print_fk(self, response):
        if response is None:
            print("  FK service call failed (no response).")
            return
        if response.error_code.val != 1:  # moveit_msgs/MoveItErrorCodes.SUCCESS
            print(f"  FK failed, error_code={response.error_code.val}")
            return
        for link_name, pose_stamped in zip(response.fk_link_names, response.pose_stamped):
            p = pose_stamped.pose.position
            q = pose_stamped.pose.orientation
            roll, pitch, yaw = quaternion_to_euler_deg(q.x, q.y, q.z, q.w)
            print(f"  {link_name}:")
            print(f"    position (m):     x={p.x:.4f}  y={p.y:.4f}  z={p.z:.4f}")
            print(f"    orientation (deg): roll={roll:.1f}  pitch={pitch:.1f}  yaw={yaw:.1f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--once", action="store_true",
                         help="run through the configurations a single time instead of looping forever")
    parser.add_argument("--pause", type=float, default=1.0,
                         help="seconds to wait after each move before computing FK / moving on")
    parser.add_argument("--base-position", type=float, default=None,
                         help="lock joint \"1\" to this value (rad) instead of reading its "
                              "current position from /joint_states at startup")
    args = parser.parse_args()

    rclpy.init()
    node = MoveAndFK()

    if not node.wait_for_servers():
        node.destroy_node()
        rclpy.shutdown()
        return

    if args.base_position is not None:
        locked_position = args.base_position
        print(f"Locking joint \"{LOCKED_JOINT_NAME}\" to --base-position={locked_position:.4f} rad.")
    else:
        print(f"Reading current joint \"{LOCKED_JOINT_NAME}\" position from /joint_states ...")
        locked_position = node.get_current_joint_position(LOCKED_JOINT_NAME)
        if locked_position is None:
            node.get_logger().error(
                "Timed out waiting for /joint_states. Is joint_state_broadcaster "
                "(sim) or the hardware bridge (real) publishing? "
                "Alternatively pass --base-position to skip this."
            )
            node.destroy_node()
            rclpy.shutdown()
            return
        print(f"Locking joint \"{LOCKED_JOINT_NAME}\" to its current position: "
              f"{locked_position:.4f} rad.")

    print("=" * 70)
    print("Move + Forward Kinematics loop via MoveIt")
    print(f"Base joint \"{LOCKED_JOINT_NAME}\" locked at {locked_position:.4f} rad "
          f"(clamp-mounted, never moves).")
    print("Press Ctrl+C to stop.")
    print("=" * 70)

    iteration = 0
    try:
        while True:
            iteration += 1
            print(f"\n########## Iteration {iteration} ##########")

            for name, positions in CONFIGURATIONS.items():
                print(f"\n--- {name} ---")
                print(f"target joints ({', '.join(FREE_JOINT_NAMES)}): "
                      f"{[round(p, 3) for p in positions]}  "
                      f"(joint \"{LOCKED_JOINT_NAME}\" held at {locked_position:.3f})")

                error_code = node.move_to(positions, locked_position)
                if error_code != 1:  # SUCCESS
                    print(f"  Move failed, error_code={error_code}")
                    continue
                print("  Move succeeded.")

                node.get_clock().sleep_for(rclpy.duration.Duration(seconds=args.pause))

                response = node.compute_fk(positions, locked_position)
                node.print_fk(response)

            if args.once:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
