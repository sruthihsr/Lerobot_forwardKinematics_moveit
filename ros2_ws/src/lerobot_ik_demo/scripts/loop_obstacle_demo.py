#!/usr/bin/env python3
"""
Continuously sweeps the "arm" group's base joint (1) left and right at the
obstacle_cylinder's height, so each leg's plan visibly detours around it in
RViz -- a direct joint-1 sweep with joint 3 held fixed passes straight
through the cylinder around joint1=0 (verified via FK; see START_JOINTS /
GOAL_JOINTS below), forcing the planner to route around it instead.

Usage:
    ros2 run lerobot_ik_demo loop_obstacle_demo.py                # loop forever
    ros2 run lerobot_ik_demo loop_obstacle_demo.py --cycles 3
    ros2 run lerobot_ik_demo loop_obstacle_demo.py --pause 1.0

Ctrl+C stops cleanly. Requires so101_moveit_obstacle.launch.py (or
so101_moveit.launch.py + add_obstacle_scene.py) already running.
"""
import argparse
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, MoveItErrorCodes, CollisionObject, PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# Kept in sync by hand with scripts/add_obstacle_scene.py -- see that
# file's comments for the real dimensions, buffer margin, and the TODO on
# OBJECT_CENTER_XY (needs your measured placement before real hardware).
PLANNING_FRAME = "base"
REAL_HEIGHT = 0.18
REAL_RADIUS = 0.14 / 2.0
BUFFER_MARGIN = 0.03
COLLISION_RADIUS = REAL_RADIUS + BUFFER_MARGIN
COLLISION_HEIGHT = REAL_HEIGHT + BUFFER_MARGIN
OBJECT_CENTER_XY = (0.0, -0.22)  # TODO: estimate only, replace with your measured position
OBJECT_POSITION = (OBJECT_CENTER_XY[0], OBJECT_CENTER_XY[1], COLLISION_HEIGHT / 2.0)

# Both verified valid (no self- or scene-collision) via /check_state_validity,
# with margin from the collision boundary (invalid for roughly -0.6 <= joint1
# <= 0.8 at this joint3); the straight joint-1 sweep between them, e.g.
# joint1=0.0, is not valid -- that's the detour this demo shows.
START_POSE = "left of object"
GOAL_POSE = "right of object"
START_JOINTS = {"1": -0.9, "2": 0.0, "3": 1.0, "4": 0.0, "5": 0.0}
GOAL_JOINTS = {"1": 1.0, "2": 0.0, "3": 1.0, "4": 0.0, "5": 0.0}


class LoopDemo(Node):
    def __init__(self):
        super().__init__("loop_obstacle_demo")
        self.move_client = ActionClient(self, MoveGroup, "move_action")
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def wait_ready(self):
        if not self.move_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("move_action not available")
        if not self.scene_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/apply_planning_scene not available")

    def ensure_box(self):
        primitive = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[COLLISION_HEIGHT, COLLISION_RADIUS])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = OBJECT_POSITION
        pose.orientation.w = 1.0

        obj = CollisionObject()
        obj.header.frame_id = PLANNING_FRAME
        obj.id = "obstacle_cylinder"
        obj.operation = CollisionObject.ADD
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]

        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [obj]
        future = self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError("apply_planning_scene failed while adding obstacle_cylinder")
        self.get_logger().info("obstacle_cylinder present in planning scene")

    def move_to(self, targets):
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.allowed_planning_time = 8.0
        goal.request.num_planning_attempts = 10
        goal.request.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=v, tolerance_above=0.02, tolerance_below=0.02, weight=1.0)
            for n, v in targets.items()
        ])]
        goal.planning_options.plan_only = False

        send_future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            raise RuntimeError("goal rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.error_code.val


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cycles", type=int, default=0, help="0 = loop forever (default), else stop after N cycles")
    parser.add_argument("--pause", type=float, default=2.0, help="seconds to hold each pose")
    args = parser.parse_args()

    start_targets = START_JOINTS
    goal_targets = GOAL_JOINTS

    rclpy.init()
    node = LoopDemo()
    node.wait_ready()
    node.ensure_box()

    cycle = 0
    try:
        while args.cycles == 0 or cycle < args.cycles:
            cycle += 1
            print(f"\n=== cycle {cycle} ===")

            node.get_logger().info(f"-> '{GOAL_POSE}' (around the cylinder)")
            code = node.move_to(goal_targets)
            ok = code == MoveItErrorCodes.SUCCESS
            print(f"  {GOAL_POSE}: {'OK' if ok else f'FAILED (error_code={code})'}")
            time.sleep(args.pause)

            node.get_logger().info(f"-> '{START_POSE}'")
            code = node.move_to(start_targets)
            ok = code == MoveItErrorCodes.SUCCESS
            print(f"  {START_POSE}: {'OK' if ok else f'FAILED (error_code={code})'}")
            time.sleep(args.pause)
    except KeyboardInterrupt:
        print("\nstopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
