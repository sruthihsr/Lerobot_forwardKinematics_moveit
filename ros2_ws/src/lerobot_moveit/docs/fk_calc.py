#!/usr/bin/env python3
"""
Forward kinematics of the SO-101, computed from the robot's own URDF -- the worked example behind
docs/KINEMATICS.md.  ROS-free (numpy only) unless --moveit is given.

    python3 docs/fk_calc.py                         # joint table + FK of the SRDF "home" pose, step by step
    python3 docs/fk_calc.py --q 0 0 0 0 0 0         # FK of any pose (radians, joints 1-6; joint 6 optional)
    python3 docs/fk_calc.py --planar                # the closed-form planar formula, checked against the full 3D chain
    python3 docs/fk_calc.py --moveit                # compare with MoveIt's /compute_fk (needs a running move_group)

Needs `xacro` and the sourced workspace (for lerobot_description), or pass --urdf FILE.
Use the SYSTEM python (/usr/bin/python3) for --moveit -- rclpy is ABI-locked to 3.12.
"""
import argparse
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

np.set_printoptions(precision=5, suppress=True, linewidth=120)

JOINTS = ["1", "2", "3", "4", "5", "6"]
CHILD_LINK = {"1": "shoulder", "2": "upper_arm", "3": "lower_arm", "4": "wrist", "5": "gripper", "6": "jaw"}
HOME = [0.015343553863686413, -0.015343553863686413, 0.23475637411440212, -0.01227484309094913,
        -0.006137421545474565, 0.007671776931843207]      # "home" in lerobot_moveit/config/so101.srdf


def rpy_matrix(r, p, y):
    """URDF convention: R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def rot_z4(q):
    c, s = math.cos(q), math.sin(q)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


class Chain:
    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        by_name = {j.get("name"): j for j in root.findall("joint")}
        self.origin, self.rpy, self.xyz, self.lower, self.upper = {}, {}, {}, {}, {}
        for n in JOINTS:
            j = by_name[n]
            o = j.find("origin")
            self.xyz[n] = [float(v) for v in o.get("xyz").split()]
            self.rpy[n] = [float(v) for v in o.get("rpy").split()]
            t = np.eye(4)
            t[:3, :3] = rpy_matrix(*self.rpy[n])
            t[:3, 3] = self.xyz[n]
            self.origin[n] = t
            axis = j.find("axis").get("xyz").split() if j.find("axis") is not None else ["0", "0", "1"]
            assert [float(a) for a in axis] == [0.0, 0.0, 1.0], f"joint {n} axis is {axis}, expected local z"
            lim = j.find("limit")
            self.lower[n], self.upper[n] = float(lim.get("lower")), float(lim.get("upper"))

    def frames(self, q):
        """{link: 4x4 pose in the base frame} for the child link of each joint. q: joints 1..6 (6 optional)."""
        q = list(q) + [0.0] * (6 - len(q))
        t, out = np.eye(4), {}
        for n, qi in zip(JOINTS, q):
            t = t @ self.origin[n] @ rot_z4(qi)       # T_i = T_{i-1} . T_origin_i . Rz(q_i)
            out[CHILD_LINK[n]] = t.copy()
        return out


def load_chain(args):
    if args.urdf:
        return Chain(open(args.urdf).read())
    from ament_index_python.packages import get_package_share_directory
    xacro = os.path.join(get_package_share_directory("lerobot_description"), "urdf", "so101.urdf.xacro")
    return Chain(subprocess.run(["xacro", xacro], capture_output=True, text=True, check=True).stdout)


def print_table(ch):
    print("Joint origins from the URDF (parent -> child), all joint axes are the child's local +z:\n")
    print(" joint  child       xyz [m]                             rpy [rad]                         limits [rad]")
    for n in JOINTS:
        x, r = ch.xyz[n], ch.rpy[n]
        print(f"  {n}     {CHILD_LINK[n]:10s} ({x[0]:+.5f}, {x[1]:+.5f}, {x[2]:+.5f})   "
              f"({r[0]:+.5f}, {r[1]:+.5f}, {r[2]:+.5f})   [{ch.lower[n]:+.4f}, {ch.upper[n]:+.4f}]")
    print()


def print_fk(ch, q, steps):
    q = list(q) + [0.0] * (6 - len(q))
    print(f"q [rad] = {[round(v, 5) for v in q]}")
    print(f"q [deg] = {[round(math.degrees(v), 2) for v in q]}\n")
    t, t_gripper = np.eye(4), None
    for n, qi in zip(JOINTS, q):
        t = t @ ch.origin[n] @ rot_z4(qi)
        if n == "5":
            t_gripper = t.copy()
        if steps:
            print(f"--- after joint {n}  (T_{n} = T_{int(n) - 1} . Torigin_{n} . Rz(q{n}))   link '{CHILD_LINK[n]}'")
            print(t)
            print()
        else:
            print(f"  {CHILD_LINK[n]:10s} origin = ({t[0, 3]:+.5f}, {t[1, 3]:+.5f}, {t[2, 3]:+.5f}) m")
    print(f"\nGripper link (end of the 'arm' group, after joint 5): position = {np.round(t_gripper[:3, 3], 5)} m")
    print(f"Jaw link (after joint 6):                              position = {np.round(t[:3, 3], 5)} m")
    return ch.frames(q)


def planar(ch, n_check=200):
    """Closed-form planar FK for the gripper origin (pan q1 = 0, roll q5 irrelevant to the position)."""
    z = [0.0] * 6
    # fix the arm's vertical plane: at q1 = 0 it contains the joint-1 axis (x ~ pan_x); rho = -(y - pan_y) forward, z up
    pan = ch.origin["1"][:2, 3]

    def rz_(p):   # (rho, z) of a base-frame point
        return np.array([-(p[1] - pan[1]), p[2]])

    def joint_origin(q, link):
        return rz_(ch.frames(q)[link][:3, 3])

    p2 = joint_origin(z, "upper_arm")            # joint-2 origin (a frame's origin doesn't move when its own joint turns)
    p3 = joint_origin(z, "lower_arm"); p4 = joint_origin(z, "wrist"); pg = joint_origin(z, "gripper")
    a, b, c = p3 - p2, p4 - p3, pg - p4          # link vectors at q2=q3=q4=0
    # sign of the rotation sense: does +q2 rotate (rho, z) counter-clockwise or clockwise?
    eps = 1e-3
    d0 = joint_origin([0, 0, 0, 0, 0, 0], "gripper") - p2
    d1 = joint_origin([0, eps, 0, 0, 0, 0], "gripper") - p2
    sense = 1.0 if (d0[0] * d1[1] - d0[1] * d1[0]) > 0 else -1.0

    def rot(th):
        c_, s_ = math.cos(sense * th), math.sin(sense * th)
        return np.array([[c_, -s_], [s_, c_]])

    def planar_fk(q2, q3, q4):
        return p2 + rot(q2) @ a + rot(q2 + q3) @ b + rot(q2 + q3 + q4) @ c

    # the arm's vertical plane sits a hair beside the pan axis (URDF joint-1 origin vs joint-2 origin)
    lam = float(ch.frames(z)["gripper"][0, 3] - pan[0])

    def gripper_xyz(q1, q2, q3, q4):
        """Gripper-frame origin in the base frame from q1..q4 (q5 = wrist roll does not move it)."""
        rho, zz = planar_fk(q2, q3, q4)
        fwd = np.array([-math.sin(q1), -math.cos(q1)])      # +q1 swings the tool toward -x
        lat = np.array([math.cos(q1), -math.sin(q1)])
        xy = pan + rho * fwd + lam * lat
        return np.array([xy[0], xy[1], zz])

    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(n_check):
        q = [rng.uniform(ch.lower[n], ch.upper[n]) for n in "1234"] + [rng.uniform(-1, 1), 0.0]
        worst = max(worst, np.linalg.norm(ch.frames(q)["gripper"][:3, 3] - gripper_xyz(*q[:4])))
    return dict(p2=p2, a=a, b=b, c=c, sense=sense, fk=planar_fk, worst=worst, pan=pan, lam=lam, xyz=gripper_xyz)


def print_planar(ch):
    pl = planar(ch)
    p2, a, b, c, lam, pan = pl["p2"], pl["a"], pl["b"], pl["c"], pl["lam"], pl["pan"]
    print("Closed-form FK of the gripper origin.  Plane coordinates: rho = forward distance from the pan axis, z = up.\n")
    print(f"  pan axis (joint 1, vertical) passes through base-frame (x, y) = ({pan[0]:.5f}, {pan[1]:.5f})")
    print(f"  arm plane's lateral offset from the pan axis           lambda = {lam * 1000:+.3f} mm")
    print(f"  joint-2 origin (shoulder) in the plane                 p2 = ({p2[0]:+.5f}, {p2[1]:+.5f})")
    print(f"  link vectors at q2=q3=q4=0:  a = ({a[0]:+.5f}, {a[1]:+.5f})  |a| = {np.linalg.norm(a):.5f}   joint 2 -> joint 3")
    print(f"                               b = ({b[0]:+.5f}, {b[1]:+.5f})  |b| = {np.linalg.norm(b):.5f}   joint 3 -> joint 4")
    print(f"                               c = ({c[0]:+.5f}, {c[1]:+.5f})  |c| = {np.linalg.norm(c):.5f}   joint 4 -> gripper origin")
    print(f"  a positive joint angle turns (rho, z) {'counter-' if pl['sense'] > 0 else ''}clockwise\n")
    print("  (1) plane:   P(rho,z) = p2 + R(q2)a + R(q2+q3)b + R(q2+q3+q4)c        R(t) = 2D rotation by "
          f"{'+' if pl['sense'] > 0 else '-'}t")
    print("  (2) pan:     x = pan_x - rho sin q1 + lambda cos q1")
    print("               y = pan_y - rho cos q1 - lambda sin q1")
    print("               z = z\n")
    print(f"  checked against the full 3D chain on 200 random poses (q1..q4 over their limits): max error = {pl['worst']:.1e} m\n")
    q = HOME
    q1, q2, q3, q4 = q[:4]
    print(f"  Worked example, SRDF home: q1..q4 = {[round(v, 5) for v in q[:4]]} rad")
    def rot(t):
        cs, sn = math.cos(pl["sense"] * t), math.sin(pl["sense"] * t)
        return np.array([[cs, -sn], [sn, cs]])
    t1, t2, t3 = rot(q2) @ a, rot(q2 + q3) @ b, rot(q2 + q3 + q4) @ c
    print(f"    p2               = ({p2[0]:+.5f}, {p2[1]:+.5f})")
    print(f"    R(q2)·a          = ({t1[0]:+.5f}, {t1[1]:+.5f})      angle q2 = {q2:+.5f}")
    print(f"    R(q2+q3)·b       = ({t2[0]:+.5f}, {t2[1]:+.5f})      angle    = {q2 + q3:+.5f}")
    print(f"    R(q2+q3+q4)·c    = ({t3[0]:+.5f}, {t3[1]:+.5f})      angle    = {q2 + q3 + q4:+.5f}")
    rho, zz = p2 + t1 + t2 + t3
    print(f"    sum              -> rho = {rho:+.5f}, z = {zz:+.5f}")
    x = pan[0] - rho * math.sin(q1) + lam * math.cos(q1)
    y = pan[1] - rho * math.cos(q1) - lam * math.sin(q1)
    print(f"    pan by q1={q1:+.5f}: x = {pan[0]:.5f} - {rho:.5f}·{math.sin(q1):+.5f} + ({lam:+.6f})·{math.cos(q1):.5f} = {x:+.5f}")
    print(f"                       y = {pan[1]:+.5f} - {rho:.5f}·{math.cos(q1):.5f} - ({lam:+.6f})·{math.sin(q1):+.5f} = {y:+.5f}")
    print(f"    gripper origin   = ({x:+.5f}, {y:+.5f}, {zz:+.5f}) m")
    full = ch.frames(q)["gripper"][:3, 3]
    print(f"    full 4x4 chain   = ({full[0]:+.5f}, {full[1]:+.5f}, {full[2]:+.5f}) m\n")
    for label, qq in (("zero pose", [0, 0, 0, 0]), ("q = (0.4, 0.5, -0.8, 0.3)", [0.4, 0.5, -0.8, 0.3])):
        r, zz2 = pl["fk"](*qq[1:])
        xyz = pl["xyz"](*qq)
        print(f"  {label:26s} rho = {r:+.5f}, z = {zz2:+.5f}  ->  ({xyz[0]:+.5f}, {xyz[1]:+.5f}, {xyz[2]:+.5f}) m")


def compare_moveit(ch, poses):
    import rclpy
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetPositionFK
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = rclpy.create_node("fk_calc")
    cli = node.create_client(GetPositionFK, "/compute_fk")
    if not cli.wait_for_service(timeout_sec=10.0):
        sys.exit("/compute_fk not available -- is move_group running (same ROS_DOMAIN_ID)?")
    links = list(CHILD_LINK.values())
    worst = 0.0
    print(f"{'pose':28s} {'link':10s} {'mine (x, y, z) [m]':32s} {'MoveIt /compute_fk':32s} {'|diff| [m]':>10s}")
    for name, q in poses:
        q = list(q) + [0.0] * (6 - len(q))
        req = GetPositionFK.Request()
        req.header.frame_id = "base"
        req.fk_link_names = links
        req.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=[float(v) for v in q]))
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut)
        res = {n: p.pose.position for n, p in zip(fut.result().fk_link_names, fut.result().pose_stamped)}
        mine = ch.frames(q)
        for link in ("gripper", "jaw"):
            m, p = mine[link][:3, 3], res[link]
            d = float(np.linalg.norm(m - [p.x, p.y, p.z]))
            worst = max(worst, d)
            print(f"{name:28s} {link:10s} ({m[0]:+.5f}, {m[1]:+.5f}, {m[2]:+.5f})   ({p.x:+.5f}, {p.y:+.5f}, {p.z:+.5f})   {d:10.2e}")
        for link in links[:-2]:
            m, p = mine[link][:3, 3], res[link]
            worst = max(worst, float(np.linalg.norm(m - [p.x, p.y, p.z])))
    print(f"\nworst position difference over all six links and {len(poses)} poses: {worst:.2e} m")
    node.destroy_node()
    rclpy.shutdown()
    return worst


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", help="URDF file (default: xacro of lerobot_description's so101.urdf.xacro)")
    ap.add_argument("--q", type=float, nargs="+", metavar="Q", help="joint values in radians (1-6 values; default: SRDF home)")
    ap.add_argument("--steps", action="store_true", help="print every cumulative 4x4 transform")
    ap.add_argument("--planar", action="store_true", help="derive/check the closed-form planar formula")
    ap.add_argument("--moveit", action="store_true", help="compare with MoveIt's /compute_fk")
    args = ap.parse_args()

    ch = load_chain(args)
    if args.planar:
        print_planar(ch)
        return
    if args.moveit:
        rng = np.random.default_rng(7)
        poses = [("zero", [0] * 6), ("home", HOME)]
        for i in range(4):
            poses.append((f"random {i + 1}", [rng.uniform(ch.lower[n], ch.upper[n]) for n in JOINTS]))
        sys.exit(0 if compare_moveit(ch, poses) < 1e-6 else 1)
    print_table(ch)
    print_fk(ch, args.q if args.q else HOME, args.steps)


if __name__ == "__main__":
    main()
