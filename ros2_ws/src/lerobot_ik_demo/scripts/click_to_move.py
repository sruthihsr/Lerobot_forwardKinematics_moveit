#!/usr/bin/env python3
"""
Click a point in RViz, the arm goes there -- routed over the cylinder
whenever the straight line would hit it.

In RViz use the "Publish Point" tool (toolbar; already configured in
lerobot_moveit's moveit.rviz, topic /clicked_point) and click in the 3D view.
The gripper-frame origin is sent to the clicked point lifted by --hover
(the click lands on a surface; this keeps the gripper above it), never
below --min-z. Planned paths show up in RViz's MotionPlanning "Planned Path".

    ros2 run lerobot_ik_demo click_to_move.py                 # move on every click
    ros2 run lerobot_ik_demo click_to_move.py --plan-only     # just plan + preview
    ros2 run lerobot_ik_demo click_to_move.py --object-xy 0.0 -0.25

Only the latest click is kept while the arm is moving. Ctrl+C to quit.
Against the real arm, start lerobot_moveit with dry_run:=True first and
confirm the paths in RViz before letting it move (see README).
"""
import argparse
import sys

import numpy as np
import rclpy
from rclpy.time import Time

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

from arm_client import ArmClient, Cylinder, add_cylinder_args
from add_obstacle_scene import PLANNING_FRAME


class ClickToMove(ArmClient):
    def __init__(self, args):
        super().__init__("click_to_move", Cylinder.from_args(args), args.clearance, args.speed)
        self.args = args
        self.pending = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(PointStamped, "/clicked_point", self._on_click, 10)

    def _on_click(self, msg):
        self.pending = msg
        self.get_logger().info(f"click received ({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f}) in '{msg.header.frame_id}'")

    def to_base(self, msg):
        if msg.header.frame_id in ("", PLANNING_FRAME, "world"):   # world == base (identity base_joint)
            return np.array([msg.point.x, msg.point.y, msg.point.z])
        tf = self.tf_buffer.lookup_transform(PLANNING_FRAME, msg.header.frame_id, Time())
        p = do_transform_point(msg, tf).point
        return np.array([p.x, p.y, p.z])

    def process_click(self, msg):
        try:
            click = self.to_base(msg)
        except Exception as e:  # tf failure
            self.get_logger().error(f"can't transform click from '{msg.header.frame_id}' to '{PLANNING_FRAME}': {e}")
            return
        target = click + np.array([0.0, 0.0, self.args.hover])
        target[2] = max(target[2], self.args.min_z)
        ok, text, _ = self.go(target, execute=not self.args.plan_only)
        line = f"target ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}): {text}"
        if ok:
            self.get_logger().info(line)
        else:
            self.get_logger().warn(line)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_cylinder_args(ap)
    ap.add_argument("--plan-only", action="store_true", help="plan and preview in RViz, never execute")
    ap.add_argument("--hover", type=float, default=0.05, help="m above the clicked point to send the gripper (default 0.05)")
    ap.add_argument("--min-z", type=float, default=0.06, help="lowest gripper-origin height allowed, m (default 0.06)")
    args = ap.parse_args()

    rclpy.init()
    node = ClickToMove(args)
    try:
        node.wait_ready()
        node.ensure_cylinder()
        c = node.cyl
        node.get_logger().info(
            f"ready: cylinder at ({c.cx:.3f}, {c.cy:.3f}), padded r={c.radius:.3f} h={c.height:.3f}; "
            f"{'PLAN-ONLY' if args.plan_only else 'will move'} -- click points with RViz's Publish Point tool")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pending is not None:
                msg, node.pending = node.pending, None
                node.process_click(msg)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
