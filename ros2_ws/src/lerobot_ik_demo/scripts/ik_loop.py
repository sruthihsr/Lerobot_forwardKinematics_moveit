#!/usr/bin/env python3
"""
Cycles the gripper through a list of Cartesian positions using the arm's own
inverse kinematics, forever (or --cycles N). Hops that would clip the cylinder
arch over its padded top automatically.

Positions (gripper-frame origin, base frame, metres) come from
config/loop_points.yaml (left / top / right of the cylinder; loop_points_extended.yaml has 7), or from repeated --point X Y Z on the command line.

Pre-flight: before anything moves, the whole loop is planned end-to-end (each
hop from the previous hop's planned end state, and back to the first point).
Points that can't be reached / are in collision are reported and dropped.
After each executed hop the reached tip position (from joint feedback via FK)
is compared with the IK target.

    ros2 run lerobot_ik_demo ik_loop.py                       # pre-flight plan only (default)
    ros2 run lerobot_ik_demo ik_loop.py --execute             # move, loop until Ctrl+C
    ros2 run lerobot_ik_demo ik_loop.py --execute --cycles 3 --pause 1.0
    ros2 run lerobot_ik_demo ik_loop.py --point 0.2 -0.2 0.1 --point -0.2 -0.2 0.1 --execute

Against the real arm: launch lerobot_moveit with dry_run:=True first, check the
previews in RViz, and pass your measured --object-xy.
"""
import argparse
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory

from arm_client import ArmClient, Cylinder, add_cylinder_args


def load_points(args, cyl):
    """--point X Y Z are absolute; in the YAML, `xyz` is absolute and `rel: [dx, dy, z]` is
    an offset from the cylinder centre (so the loop follows --object-xy)."""
    if args.point:
        return [(f"point_{i + 1}", tuple(p)) for i, p in enumerate(args.point)]
    path = args.points or os.path.join(get_package_share_directory("lerobot_ik_demo"), "config", "loop_points.yaml")
    with open(path) as f:
        entries = yaml.safe_load(f)["points"]
    out = []
    for p in entries:
        if "rel" in p:
            dx, dy, z = (float(v) for v in p["rel"])
            out.append((p["name"], (cyl.cx + dx, cyl.cy + dy, z)))
        else:
            out.append((p["name"], tuple(float(v) for v in p["xyz"])))
    return out


def preflight(node, points, start_q=None):
    """Plan the closed loop without moving; returns the subset of points that chain together."""
    keep, q = [], start_q
    for name, xyz in points:
        ok, msg, q_new = node.go(xyz, execute=False, start_q=q)
        if ok:
            keep.append((name, xyz))
            q = q_new
            node.get_logger().info(f"  {name:16s} OK -- {msg}")
        else:
            node.get_logger().warn(f"  {name:16s} SKIPPED -- {msg}")
    if len(keep) >= 2:   # closing hop back to the first point
        ok, msg, _ = node.go(keep[0][1], execute=False, start_q=q)
        if ok:
            node.get_logger().info(f"  {'-> ' + keep[0][0]:16s} OK -- {msg}")
        else:
            node.get_logger().warn(f"  closing hop to {keep[0][0]} failed: {msg}")
            return []
    return keep


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_cylinder_args(ap)
    ap.add_argument("--execute", action="store_true", help="actually move (default: pre-flight plan only)")
    ap.add_argument("--points", help="YAML file of points (default: config/loop_points.yaml)")
    ap.add_argument("--point", type=float, nargs=3, action="append", metavar=("X", "Y", "Z"),
                    help="a position to visit; repeat for several (overrides the YAML)")
    ap.add_argument("--cycles", type=int, default=0, help="loops to run (0 = until Ctrl+C, default)")
    ap.add_argument("--pause", type=float, default=1.0, help="s to hold at each point")
    ap.add_argument("--escape-only", action="store_true", help="just do the --escape retreat, then exit (implies --escape)")
    ap.add_argument("--escape", action="store_true",
                    help="if the arm starts inside the cylinder's safety pad, first pan the base away "
                         "(pan-only, only while moving away from the cylinder, refused if near the real cylinder)")
    args = ap.parse_args()
    args.escape = args.escape or args.escape_only

    rclpy.init()
    node = ArmClient("ik_loop", Cylinder.from_args(args), args.clearance, args.speed)
    log = node.get_logger()
    try:
        node.wait_ready()
        node.ensure_cylinder()
        c = node.cyl
        if args.escape:
            esc, msg = node.plan_escape()
            if esc is None:
                log.info(f"escape: {msg}")
            else:
                log.info(f"escape: {msg}")
                if not args.execute:
                    log.info("escape: not executed (plan-only). Pass --execute to perform it.")
                else:
                    traj = node.to_trajectory(esc)
                    node.show(traj)
                    if not node.execute(traj):
                        log.error("escape execution failed; stopping.")
                        return 1
                    ok = node.settle(esc[-1], tol=0.02, timeout=10.0)
                    time.sleep(args.pause)
                    log.info(f"escape: executed; {'settled' if ok else 'did NOT settle within 0.02 rad'}; now valid in scene: {node.state_valid(node.current_q())}")
        if args.escape_only:
            return 0
        points = load_points(args, c)
        log.info(f"cylinder at ({c.cx:.3f}, {c.cy:.3f}) padded r={c.radius:.3f} h={c.height:.3f}; "
                 f"{len(points)} points; pre-flight:")
        # plan-only escape leaves the arm where it is, so chain the preflight from the escape's end pose
        start = esc[-1] if (args.escape and esc is not None and not args.execute) else None
        points = preflight(node, points, start_q=start)
        if len(points) < 2:
            log.error("fewer than 2 usable points -- nothing to loop.")
            return 1
        if not args.execute:
            log.info(f"pre-flight OK for {len(points)} points; nothing moved (pass --execute to loop).")
            return 0

        cycle = 0
        while args.cycles == 0 or cycle < args.cycles:
            cycle += 1
            for name, xyz in points:
                ok, msg, q_end = node.go(xyz)
                if not ok:
                    log.error(f"[{cycle}] {name}: FAILED -- {msg}; stopping.")
                    return 1
                err = np.linalg.norm(node.kin.position(node.current_q()) - np.asarray(xyz)) * 1000
                log.info(f"[{cycle}] {name:16s} {msg}; tip error {err:.1f} mm")
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
