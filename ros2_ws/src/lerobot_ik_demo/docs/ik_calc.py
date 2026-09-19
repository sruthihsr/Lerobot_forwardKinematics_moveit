#!/usr/bin/env python3
"""
Inverse kinematics of the SO-101 in closed form -- the worked calculation behind docs/INVERSE_KINEMATICS.md.
ROS-free (numpy only) unless --moveit is given.  Uses scripts/so101_kin.py for forward kinematics and as the
solver the planner actually runs, and compares the closed form with it.

    python3 docs/ik_calc.py                              # worked example: recover the SRDF "home" pose from its tip position
    python3 docs/ik_calc.py --target 0.211 -0.173 0.12 --pitch 30     # any target (metres, pitch in degrees)
    python3 docs/ik_calc.py --check                      # statistics: round trip, both elbow branches, agreement with so101_kin
    python3 docs/ik_calc.py --moveit                     # compare with MoveIt's /compute_ik (KDL); needs a running move_group

Target = gripper-frame origin xyz in the base frame; pitch = q2+q3+q4 (the one free tool-orientation angle).
Needs `xacro` and the sourced workspace (for lerobot_description), or --urdf FILE.
"""
import argparse
import math
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
from so101_kin import Kin  # noqa: E402

np.set_printoptions(precision=5, suppress=True, linewidth=120)
HOME = [0.015343553863686413, -0.015343553863686413, 0.23475637411440212, -0.01227484309094913, -0.006137421545474565]


def load_kin(args):
    if args.urdf:
        return Kin(open(args.urdf).read())
    from ament_index_python.packages import get_package_share_directory
    xacro = os.path.join(get_package_share_directory("lerobot_description"), "urdf", "so101.urdf.xacro")
    return Kin(subprocess.run(["xacro", xacro], capture_output=True, text=True, check=True).stdout)


def rot(t):
    """2-D rotation by -t: a positive joint angle turns the (rho, z) plane clockwise."""
    return np.array([[math.cos(t), math.sin(t)], [-math.sin(t), math.cos(t)]])


def ang(v):
    return math.atan2(v[1], v[0])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Planar:
    """The constants of the closed form, read off the URDF through so101_kin's forward kinematics."""

    def __init__(self, K):
        self.K = K
        self.pan = np.array(K.pan_xy)
        z = [0.0] * 5
        pts = self._points(z)
        self.p2, p3, p4, pg = pts
        self.a, self.b, self.c = p3 - self.p2, p4 - p3, pg - p4
        self.lam = float(K.fk(z)[0, 3] - self.pan[0])          # arm plane's lateral offset from the pan axis

    def _points(self, q):
        """(rho, z) of the joint-2, joint-3, joint-4 origins and the gripper origin, for pan q1 = 0."""
        f = self.K._fixed
        m, out = np.eye(4), []
        for i, qi in enumerate(q):
            c, s = math.cos(qi), math.sin(qi)
            rz = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            m = m @ f[i] @ rz
            out.append(m[:3, 3].copy())
        pl = lambda p: np.array([-(p[1] - self.pan[1]), p[2]])
        return [pl(out[1]), pl(out[2]), pl(out[3]), pl(out[4])]   # joints 2, 3, 4 origins, gripper origin


def solve(P, xyz, pitch, verbose=False):
    """All closed-form solutions for (gripper origin xyz, pitch = q2+q3+q4).

    Up to four: two pan angles (arm reaching *forward* of the pan axis, rho > 0, or *behind* it, rho < 0)
    x two elbow branches.  Returns dicts {q, pan, elbow, ok, why}; ok = inside the joint limits.
    """
    K = P.K
    out = lambda *a: print(*a) if verbose else None
    x, y, z = xyz
    d = np.array([x - P.pan[0], y - P.pan[1]])
    A = float(np.linalg.norm(d))
    out(f"  step 1  pan.   d = target_xy - pan_xy = ({d[0]:+.5f}, {d[1]:+.5f}),  |d| = {A:.5f}")
    if A < 0.02 or A < abs(P.lam):
        return []
    # d = rho*f(q1) + lam*l(q1),  f = (-sin q1, -cos q1), l = (cos q1, -sin q1)  ->  cos(q1 + alpha) = lam / |d|
    alpha = math.atan2(d[1], d[0])
    pans = []
    for s in (1.0, -1.0):
        q1 = wrap(-alpha + s * math.acos(max(-1.0, min(1.0, P.lam / A))))
        pans.append((q1, -math.sin(q1) * d[0] - math.cos(q1) * d[1]))
    pans.sort(key=lambda t: -t[1])                       # the forward (rho > 0) solution first
    q1_simple = -math.atan2(d[0], -d[1])
    out(f"          forward solution:  q1 = {pans[0][0]:+.5f} rad, rho = {pans[0][1]:+.5f} m    (approx. -atan2(dx, -dy) = {q1_simple:+.5f}; differs by the {P.lam * 1000:+.3f} mm plane offset)")
    out(f"          behind solution:   q1 = {pans[1][0]:+.5f} rad, rho = {pans[1][1]:+.5f} m    (arm reaches back over the pan axis)")

    la, lb = np.linalg.norm(P.a), np.linalg.norm(P.b)
    offset = ang(P.b) - ang(P.a)
    sols = []
    for pi, (q1, rho) in enumerate(pans):
        verb = verbose and pi == 0
        pr = (lambda *a: print(*a)) if verb else (lambda *a: None)
        T = np.array([rho, z])
        wr = T - P.p2 - rot(pitch) @ P.c
        pr(f"  step 2  wrist point.  W = (rho, z) - p2 - R(pitch)·c = ({T[0]:+.5f}, {T[1]:+.5f}) - ({P.p2[0]:+.5f}, {P.p2[1]:+.5f}) - R({pitch:+.4f})·({P.c[0]:+.5f}, {P.c[1]:+.5f})")
        pr(f"          W = ({wr[0]:+.5f}, {wr[1]:+.5f}),  |W| = {np.linalg.norm(wr):.5f} m")
        D2 = float(wr @ wr)
        C = (D2 - la ** 2 - lb ** 2) / (2 * la * lb)
        pr(f"  step 3  elbow (law of cosines).  cos(theta) = (|W|² - |a|² - |b|²) / (2|a||b|) = ({D2:.5f} - {la ** 2:.5f} - {lb ** 2:.5f}) / {2 * la * lb:.5f} = {C:+.5f}")
        if 1.0 < abs(C) < 1.0 + 1e-6:        # fully stretched / folded elbow: rounding pushes cos a hair past 1
            C = math.copysign(1.0, C)
        if abs(C) > 1.0:
            pr("          |cos| > 1: the wrist point is out of reach for this pitch")
            continue
        th = math.acos(C)
        for k, sg in enumerate((+1.0, -1.0)):
            q3 = wrap(offset - sg * th)
            v = P.a + rot(q3) @ P.b
            q2 = wrap(ang(v) - ang(wr))
            q4 = wrap(pitch - q2 - q3)
            q = [q1, q2, q3, q4, 0.0]
            ok = K.in_limits(q)
            why = "; ".join(f"q{i + 1}={v_:+.3f} not in [{K.lower[i]:+.3f}, {K.upper[i]:+.3f}]" for i, v_ in enumerate(q)
                            if not (K.lower[i] - 1e-6 <= v_ <= K.upper[i] + 1e-6))
            sols.append(dict(q=q, pan="fwd" if rho > 0 else "back", elbow="AB"[k], ok=ok, why=why, C=C))
            pr(f"  step 4  elbow {'AB'[k]}:  q3 = (angle b - angle a) {'-' if sg > 0 else '+'} acos = {math.degrees(offset):+.3f}° {'-' if sg > 0 else '+'} {math.degrees(th):.3f}° = {q3:+.5f}")
            pr(f"                    q2 = angle(a + R(q3)b) - angle(W) = {math.degrees(ang(v)):+.3f}° - {math.degrees(ang(wr)):+.3f}° = {q2:+.5f}")
            pr(f"                    q4 = pitch - q2 - q3 = {pitch:+.5f} - ({q2:+.5f}) - ({q3:+.5f}) = {q4:+.5f}")
    return sols


def report(P, xyz, pitch, seed=None):
    K = P.K
    print(f"target gripper origin = ({xyz[0]:+.5f}, {xyz[1]:+.5f}, {xyz[2]:+.5f}) m,  pitch q2+q3+q4 = {pitch:+.5f} rad ({math.degrees(pitch):+.2f}°)\n")
    print(f"constants (from the URDF):  pan axis ({P.pan[0]:.5f}, {P.pan[1]:.5f}),  lambda = {P.lam * 1000:+.3f} mm,  p2 = ({P.p2[0]:+.5f}, {P.p2[1]:+.5f})")
    print(f"                            a = ({P.a[0]:+.5f}, {P.a[1]:+.5f}) |a|={np.linalg.norm(P.a):.5f}   b = ({P.b[0]:+.5f}, {P.b[1]:+.5f}) |b|={np.linalg.norm(P.b):.5f}   c = ({P.c[0]:+.5f}, {P.c[1]:+.5f}) |c|={np.linalg.norm(P.c):.5f}\n")
    sols = solve(P, xyz, pitch, verbose=True)
    if not sols:
        print("\nno solution: unreachable (too close to the pan axis, wrist point out of reach, or behind the base)")
        return sols
    print("\nresults (q5 = wrist roll is free; shown as 0):")
    print("  pan   elbow      q1        q2        q3        q4      limits   FK error [m]")
    for s in sols:
        q = s["q"]
        err = np.linalg.norm(K.position(q) - np.asarray(xyz))
        pe = abs(wrap(Kin.pitch(q) - pitch))
        print(f"  {s['pan']:4s}    {s['elbow']}    {q[0]:+.5f} {q[1]:+.5f} {q[2]:+.5f} {q[3]:+.5f}   {'ok ' if s['ok'] else 'OUT'}     {err:.1e}   (pitch err {pe:.1e})" + (f"   <- {s['why']}" if not s["ok"] else ""))
    if seed is not None:
        r = K.ik(xyz, pitch, seed)
        print("\n  planner's solver so101_kin.Kin.ik (damped Newton, seeded):",
              "none" if r is None else f"{np.round(r[:4], 5)}  (nearest branch: "
              + (lambda b: f"{b['pan']}/{b['elbow']}")(min(sols, key=lambda s: np.linalg.norm(np.array(s['q'][:4]) - np.array(r[:4])))) + ")")
    return sols


def check(P, n=2000):
    """Statistics over random poses.  'Working region' = the arm reaches forward of the pan axis and the elbow is
    not within ~11 deg of straight (|cos theta| < 0.98) -- where the loop actually operates."""
    K = P.K
    rng = np.random.default_rng(11)
    used = found = recovered = multi = 0
    worst_fk, rec_err = 0.0, []
    seeded, step = {"all": [0, 0], "region": [0, 0]}, {"all": [0, 0], "region": [0, 0]}
    tip = []
    for _ in range(n):
        q = [rng.uniform(K.lower[i], K.upper[i]) for i in range(4)] + [0.0]
        xyz, pitch = K.position(q), Kin.pitch(q)
        if math.hypot(xyz[0] - P.pan[0], xyz[1] - P.pan[1]) < 0.03:
            continue                                            # too close to the pan axis: azimuth undefined
        used += 1
        good = [s for s in solve(P, xyz, pitch) if s["ok"]]
        if not good:
            continue
        found += 1
        multi += len(good) > 1
        worst_fk = max(worst_fk, max(float(np.linalg.norm(K.position(s["q"]) - xyz)) for s in good))
        e = min(float(np.linalg.norm(np.array(s["q"][:4]) - np.array(q[:4]))) for s in good)
        rec_err.append(e)
        recovered += e < 1e-3
        fwd = [s for s in good if s["pan"] == "fwd"]
        region = bool(fwd) and abs(fwd[0]["C"]) < 0.98
        keys = ["all"] + (["region"] if region else [])
        # (a) planner's solver, seeded ~3 deg off the true answer
        r = K.ik(xyz, pitch, [v + rng.uniform(-0.05, 0.05) for v in q])
        for k in keys:
            seeded[k][1] += 1
            seeded[k][0] += r is not None
        if r is not None:
            tip.append(float(np.linalg.norm(K.position(r) - xyz)))
        # (b) one 10 mm path step: target moved 1 cm in a random direction, seed = the previous solution
        dv = rng.normal(size=3)
        t = xyz + 0.01 * dv / np.linalg.norm(dv)
        if any(s["ok"] for s in solve(P, t, pitch)):
            r2 = K.ik(t, pitch, q)
            for k in keys:
                step[k][1] += 1
                step[k][0] += r2 is not None
    rec_err, tip = np.array(rec_err), np.array(tip)
    pct = lambda a: f"{a[0]}/{a[1]} = {100 * a[0] / max(1, a[1]):.1f} %"
    print(f"{used} random poses (q1..q4 uniform over their joint limits, away from the pan axis)\n")
    print("closed form (this document)")
    print(f"  finds a limit-respecting solution:                          {found}/{used}")
    print(f"  one solution is the original joint vector (< 1e-3 rad):     {recovered}/{found}   (worst joint error {rec_err.max():.1e} rad)")
    print(f"  more than one solution inside the limits (a real choice):   {multi}/{found}")
    print(f"  worst FK error of any solution:                             {worst_fk:.1e} m\n")
    print("planner's solver so101_kin.Kin.ik (damped Newton, needs a seed)")
    print(f"  seeded ~3 deg off the answer, all poses:                    {pct(seeded['all'])}")
    print(f"  seeded ~3 deg off the answer, working region:               {pct(seeded['region'])}")
    print(f"  1 cm path step from the previous solution, all poses:       {pct(step['all'])}")
    print(f"  1 cm path step from the previous solution, working region:  {pct(step['region'])}")
    print(f"  tip error when it does solve: median {np.median(tip) * 1000:.3f} mm, max {tip.max() * 1000:.3f} mm (tolerance 1 mm)")


def compare_moveit(P, n=80):
    import rclpy
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetPositionIK
    from sensor_msgs.msg import JointState
    K = P.K
    rclpy.init()
    node = rclpy.create_node("ik_calc")
    cli = node.create_client(GetPositionIK, "/compute_ik")
    if not cli.wait_for_service(timeout_sec=10.0):
        sys.exit("/compute_ik not available -- is move_group running (same ROS_DOMAIN_ID)?")
    rng = np.random.default_rng(5)
    names = ["1", "2", "3", "4", "5"]

    def quat(m):
        w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
        if w > 1e-6:
            return ((m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w), w)
        x = math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2.0
        return (x, (m[0, 1] + m[1, 0]) / (4 * x), (m[0, 2] + m[2, 0]) / (4 * x), (m[2, 1] - m[1, 2]) / (4 * x))

    def kdl(pose_t, pose_r, seed):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name, r.ik_link_name, r.avoid_collisions = "arm", "gripper", False
        r.timeout.nanosec = 100_000_000
        r.robot_state = RobotState(joint_state=JointState(name=names, position=[float(v) for v in seed]))
        r.pose_stamped.header.frame_id = "base"
        p = Pose()
        p.position.x, p.position.y, p.position.z = (float(v) for v in pose_t)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in quat(pose_r))
        r.pose_stamped.pose = p
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut)
        return fut.result().error_code.val == 1

    stats = {"near": 0, "home": 0, "closed": 0}
    tot = 0
    for _ in range(n):
        q = [rng.uniform(K.lower[i], K.upper[i]) * 0.8 for i in range(4)] + [0.0]
        T = K.fk(q)
        xyz, pitch = T[:3, 3], Kin.pitch(q)
        if math.hypot(xyz[0] - P.pan[0], xyz[1] - P.pan[1]) < 0.03:
            continue                                            # on the pan axis: azimuth undefined, closed form declines
        tot += 1
        stats["near"] += kdl(xyz, T[:3, :3], [v + rng.uniform(-0.05, 0.05) for v in q])
        stats["home"] += kdl(xyz, T[:3, :3], HOME)
        stats["closed"] += any(s["ok"] for s in solve(P, xyz, pitch))
    print(f"{tot} random reachable targets: the full 6-D pose (position + orientation) is taken from FK, so an exact solution exists")
    print(f"  MoveIt /compute_ik (KDL), seeded 3° from the answer : {stats['near']}/{tot} solved")
    print(f"  MoveIt /compute_ik (KDL), seeded at the home pose    : {stats['home']}/{tot} solved")
    print(f"  closed form (position + pitch, no seed needed)       : {stats['closed']}/{tot} solved")
    node.destroy_node()
    rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf")
    ap.add_argument("--target", type=float, nargs=3, metavar=("X", "Y", "Z"))
    ap.add_argument("--pitch", type=float, help="q2+q3+q4 in degrees (default: 0)")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--moveit", action="store_true")
    args = ap.parse_args()

    K = load_kin(args)
    P = Planar(K)
    if args.check:
        check(P)
    elif args.moveit:
        compare_moveit(P)
    elif args.target:
        report(P, args.target, math.radians(args.pitch or 0.0), seed=HOME)
    else:
        print("Worked example: recover the SRDF 'home' pose from its gripper position and pitch.\n"
              f"home q1..q4 = {[round(v, 5) for v in HOME[:4]]}\n")
        xyz, pitch = K.position(HOME), Kin.pitch(HOME)
        report(P, xyz, pitch, seed=HOME)


if __name__ == "__main__":
    main()
