#!/usr/bin/env python3
"""
Real-hardware bridge for the SO101 Feetech-servo follower arm.

Publishes /joint_states from the live servos and exposes two
FollowJointTrajectory action servers (arm_controller, gripper_controller)
with the exact names/types moveit_simple_controller_manager expects (see
lerobot_moveit/config/moveit_controllers.yaml). This lets MoveIt plan and
execute against the physical arm with no ros2_control involved at all --
test_fk_moveit.py and the MoveGroup/RViz wiring in so101_moveit.launch.py
need no changes.

Torque stays enabled at all times, including on shutdown, so the arm holds
its last commanded pose.

ROS params:
    port           (string, default /dev/ttyACM0)
    dry_run        (bool, default True) -- when True, connects read-only and
                    never writes Goal_Position (or any config register) to
                    the servos; trajectory execution just logs intended raw
                    ticks. Must be explicitly set False to move the arm.
    acceleration   (int 0-254, default 20)  -- conservative ramp, see feetech_bus.py
    goal_velocity  (int, default 150)       -- conservative speed cap
    joint_state_rate (float Hz, default 30.0)
"""

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

from feetech_bus import FeetechBus, ClampExceeded

JOINT_NAMES = ["1", "2", "3", "4", "5", "6"]
SERVO_IDS = [1, 2, 3, 4, 5, 6]
ARM_JOINT_NAMES = JOINT_NAMES[:5]
GRIPPER_JOINT_NAMES = JOINT_NAMES[5:]

NAME_TO_ID = dict(zip(JOINT_NAMES, SERVO_IDS))


class So101HardwareBridge(Node):

    def __init__(self):
        super().__init__("so101_hardware_bridge")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("acceleration", 20)
        self.declare_parameter("goal_velocity", 150)
        self.declare_parameter("joint_state_rate", 30.0)

        port = self.get_parameter("port").value
        self.dry_run = self.get_parameter("dry_run").value
        acceleration = self.get_parameter("acceleration").value
        goal_velocity = self.get_parameter("goal_velocity").value
        joint_state_rate = self.get_parameter("joint_state_rate").value

        self.bus_lock = threading.Lock()
        self.bus = FeetechBus(
            port, servo_ids=SERVO_IDS,
            acceleration=acceleration, goal_velocity=goal_velocity,
            logger=self.get_logger(),
        )

        if self.dry_run:
            self.get_logger().warn(
                "dry_run=True: connecting read-only, no servo writes will be "
                "sent (trajectory execution will only log intended commands)."
            )
            self.bus.connect_read_only()
        else:
            self.get_logger().warn(
                f"dry_run=False: LIVE hardware mode on {port}. "
                f"Torque will be enabled and Goal_Position writes will move "
                f"the physical arm."
            )
            self.bus.connect()

        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(1.0 / joint_state_rate, self._publish_joint_state)

        callback_group = ReentrantCallbackGroup()
        self.arm_action_server = ActionServer(
            self, FollowJointTrajectory, "arm_controller/follow_joint_trajectory",
            execute_callback=self._make_execute_callback(ARM_JOINT_NAMES),
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )
        self.gripper_action_server = ActionServer(
            self, FollowJointTrajectory, "gripper_controller/follow_joint_trajectory",
            execute_callback=self._make_execute_callback(GRIPPER_JOINT_NAMES),
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )

        self.get_logger().info("so101_hardware_bridge ready.")

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _publish_joint_state(self):
        with self.bus_lock:
            positions_by_id = self.bus.read_positions_rad()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [positions_by_id[NAME_TO_ID[name]] for name in JOINT_NAMES]
        self.joint_state_pub.publish(msg)

    def _make_execute_callback(self, controller_joint_names):
        def execute_callback(goal_handle):
            trajectory = goal_handle.request.trajectory
            start_time = self.get_clock().now()

            for point in trajectory.points:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = FollowJointTrajectory.Result()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "canceled"
                    return result

                target_time = start_time + Duration(
                    seconds=point.time_from_start.sec,
                    nanoseconds=point.time_from_start.nanosec,
                )
                sleep_for = (target_time - self.get_clock().now()).nanoseconds / 1e9
                if sleep_for > 0:
                    time.sleep(sleep_for)

                targets_rad = {}
                for joint_name, position in zip(trajectory.joint_names, point.positions):
                    if joint_name in controller_joint_names:
                        targets_rad[NAME_TO_ID[joint_name]] = position

                try:
                    with self.bus_lock:
                        self.bus.write_positions_rad(targets_rad, dry_run=self.dry_run)
                except ClampExceeded as exc:
                    self.get_logger().error(str(exc))
                    goal_handle.abort()
                    result = FollowJointTrajectory.Result()
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = str(exc)
                    return result

                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = trajectory.joint_names
                feedback.desired = point
                goal_handle.publish_feedback(feedback)

            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        return execute_callback

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = So101HardwareBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
