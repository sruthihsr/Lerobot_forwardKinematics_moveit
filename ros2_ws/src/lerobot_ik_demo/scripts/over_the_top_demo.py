#!/usr/bin/env python3
"""
Carries the SO-101's gripper OVER THE TOP of the cylinder obstacle: start on
one side, lift above the padded top, cross over, come down on the other side
-- instead of letting a sampling planner swing the base joint around it.

The crossing is an explicit Cartesian arch (see arm_client.py): the gripper
origin stays --clearance above the padded top while inside the footprint, every
point is solved with the arm's own IK and validated against MoveIt's planning
scene, then executed via /execute_trajectory. Same script for mock hardware,
Gazebo and the real arm (start move_group first; see README).

    ros2 run lerobot_ik_demo over_the_top_demo.py                    # plan + validate only
    ros2 run lerobot_ik_demo over_the_top_demo.py --execute          # also move
    ros2 run lerobot_ik_demo over_the_top_demo.py --execute --cycles 0   # loop until Ctrl+C
    ros2 run lerobot_ik_demo over_the_top_demo.py --direction radial --object-xy 0.0 -0.20

--direction lateral crosses left->right in front of the arm (default, well inside
reach); radial crosses near side -> far side and needs the far side within reach.
"""
import argparse
import sys
import time

import rclpy

from arm_client import ArmClient, Cylinder, add_cylinder_args


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_cylinder_args(ap)
    ap.add_argument("--execute", action="store_true",
                    help="actually move the arm (default: plan + validate only)")
    ap.add_argument("--direction", choices=["lateral", "radial"], default="lateral")
    ap.add_argument("--run-out", type=float, default=0.06,
                    help="m beyond the padded radius where the crossing starts/ends (default 0.06)")
    ap.add_argument("--cycles", type=int, default=1, help="with --execute: repeat N times (0 = forever)")
    ap.add_argument("--pause", type=float, default=2.0)
    args = ap.parse_args()

    rclpy.init()
    cyl = Cylinder.from_args(args)
    node = ArmClient("over_the_top_demo", cyl, args.clearance, args.speed)
    try:
        node.wait_ready()
        node.ensure_cylinder()
        half = cyl.radius + args.run_out
        z = node.z_arch
        if args.direction == "lateral":
            a, b = (cyl.cx - half, cyl.cy, z), (cyl.cx + half, cyl.cy, z)
        else:
            a, b = (cyl.cx, cyl.cy + half, z), (cyl.cx, cyl.cy - half, z)
        node.get_logger().info(
            f"cylinder at ({cyl.cx:.3f}, {cyl.cy:.3f}) padded r={cyl.radius:.3f} h={cyl.height:.3f}; "
            f"crossing {args.direction} at z={z:.3f}: A={tuple(round(v, 3) for v in a)} B={tuple(round(v, 3) for v in b)}")

        if not args.execute:   # chain the legs from each planned end state instead of the (unmoved) arm
            q = None
            for name, tgt in (("-> A", a), ("A -> B over the top", b), ("B -> A over the top", a)):
                ok, msg, q = node.go(tgt, execute=False, start_q=q)
                node.get_logger().info(f"{name}: {'OK' if ok else 'FAILED'} -- {msg}")
                if not ok:
                    return 1
            node.get_logger().info("plan-only: nothing moved (pass --execute to move).")
            return 0

        cycle = 0
        while args.cycles == 0 or cycle < args.cycles:
            cycle += 1
            for name, tgt in (("-> A", a), ("A -> B over the top", b), ("B -> A over the top", a)):
                ok, msg, _ = node.go(tgt)
                node.get_logger().info(f"[{cycle}] {name}: {'OK' if ok else 'FAILED'} -- {msg}")
                if not ok:
                    return 1
                time.sleep(args.pause)
        return 0
    except KeyboardInterrupt:
        print("\nstopped by user")
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
