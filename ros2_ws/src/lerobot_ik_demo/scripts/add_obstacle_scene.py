#!/usr/bin/env python3
"""
Adds a single cylinder CollisionObject to MoveIt's planning scene via the
/apply_planning_scene service, modeling a real physical obstacle placed in
front of the SO-101 (sim-to-real: the collision geometry here is padded
beyond the object's real measured size, since planning against exact
real-world dimensions leaves zero margin for calibration error, tape-measure
imprecision, or servo backlash before this runs against real hardware).

Run once move_group is up (needs /apply_planning_scene):
    ros2 run lerobot_ik_demo add_obstacle_scene.py [--object-xy X Y]

Or let launch/so101_moveit_obstacle.launch.py do it automatically (sim only
-- for real hardware, launch lerobot_moveit's so101_moveit.launch.py with
use_real_hardware:=True yourself, then run this script separately).
"""
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene

PLANNING_FRAME = "base"

# Real, measured dimensions of the physical cylinder.
REAL_HEIGHT = 0.18   # m
REAL_DIAMETER = 0.14  # m
REAL_RADIUS = REAL_DIAMETER / 2.0

# Safety margin added on top of the real dimensions before this becomes a
# CollisionObject, so the planner keeps extra clearance instead of grazing
# the object's true surface. Applied to the radius (all horizontal
# directions) and to the top only -- the base stays flush with the table,
# since there's nothing to pad underneath a resting object.
BUFFER_MARGIN = 0.03  # m (3 cm)

COLLISION_RADIUS = REAL_RADIUS + BUFFER_MARGIN
COLLISION_HEIGHT = REAL_HEIGHT + BUFFER_MARGIN
COLLISION_CENTER_Z = COLLISION_HEIGHT / 2.0  # object rests on the table (z=0 in the base frame)

# TODO: measure and fill in exactly. (x, y) center of the cylinder in the
# "base" frame -- x=0 assumes it's centered directly in front of the arm;
# y=-0.22 is only an estimate ("~20-25cm from the base"), not a measured
# value. Replace both with your actual tape-measure numbers before running
# against real hardware. See lerobot_ik_demo/README.md "Sim to real".
OBJECT_CENTER_XY = (0.0, -0.22)

OBJECT_POSITION = (OBJECT_CENTER_XY[0], OBJECT_CENTER_XY[1], COLLISION_CENTER_Z)


def main():
    # Optional override so a measured position needn't be edited into this file.
    args = [a for a in sys.argv[1:] if not a.startswith("--ros-args")]
    object_position = OBJECT_POSITION
    if args[:1] == ["--object-xy"] and len(args) >= 3:
        object_position = (float(args[1]), float(args[2]), COLLISION_CENTER_Z)

    rclpy.init()
    node = Node("add_obstacle_scene")

    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("/apply_planning_scene not available -- is move_group running?")
        rclpy.shutdown()
        sys.exit(1)

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.CYLINDER
    primitive.dimensions = [COLLISION_HEIGHT, COLLISION_RADIUS]

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = object_position
    pose.orientation.w = 1.0

    obstacle = CollisionObject()
    obstacle.header.frame_id = PLANNING_FRAME
    obstacle.id = "obstacle_cylinder"
    obstacle.operation = CollisionObject.ADD
    obstacle.primitives = [primitive]
    obstacle.primitive_poses = [pose]

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [obstacle]

    request = ApplyPlanningScene.Request()
    request.scene = scene

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if result is not None and result.success:
        node.get_logger().info(
            f"Added '{obstacle.id}' (radius={COLLISION_RADIUS:.3f}m incl. {BUFFER_MARGIN:.3f}m buffer, "
            f"height={COLLISION_HEIGHT:.3f}m) at {object_position} to the planning scene."
        )
    else:
        node.get_logger().error("apply_planning_scene call failed")
        rclpy.shutdown()
        sys.exit(1)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
