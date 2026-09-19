"""
ROS-free kinematics for the SO-101 "arm" group (joints 1-5), read from the
robot's own URDF so it can't drift from lerobot_description.

Why not MoveIt's KDL IK: this is a 5-DOF arm, and KDL's 6D numerical IK only
converges on poses that are already exactly consistent with the kinematics,
so it fails on most targets. Here the reduced problem is solved directly:

  q1      pan, from the target's azimuth about the joint-1 axis
  q2,q3   2-link planar solve for the gripper-frame origin (radius, height)
  q4      = pitch - q2 - q3, where pitch = q2+q3+q4 is the free tool-pitch
  q5      wrist roll, held at the caller's value (it doesn't move the origin)

Joints 2-4 are parallel, so tool pitch is just their sum.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

ARM_JOINTS = ["1", "2", "3", "4", "5"]


def _rpy(a, b, c):
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])
    ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
    rz = np.array([[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]])
    return rz @ ry @ rx


class Kin:
    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        joints = {j.get("name"): j for j in root.findall("joint")}
        self._fixed = []   # 4x4 origin transform per joint 1-5
        self.lower, self.upper = [], []
        for name in ARM_JOINTS:
            j = joints[name]
            o = j.find("origin")
            m = np.eye(4)
            m[:3, :3] = _rpy(*[float(v) for v in o.get("rpy").split()])
            m[:3, 3] = [float(v) for v in o.get("xyz").split()]
            self._fixed.append(m)
            lim = j.find("limit")
            self.lower.append(float(lim.get("lower")))
            self.upper.append(float(lim.get("upper")))
        # joint-1 axis is vertical and passes through this (x, y) in the base frame
        self.pan_xy = (self._fixed[0][0, 3], self._fixed[0][1, 3])

    def fk(self, q):
        """4x4 pose of the 'gripper' link (child of joint 5) in the base frame."""
        m = np.eye(4)
        for f, qi in zip(self._fixed, q):
            c, s = math.cos(qi), math.sin(qi)
            rz = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            m = m @ f @ rz
        return m

    def position(self, q):
        return self.fk(q)[:3, 3]

    @staticmethod
    def pitch(q):
        return q[1] + q[2] + q[3]

    def in_limits(self, q, tol=1e-6):
        return all(lo - tol <= v <= hi + tol for v, lo, hi in zip(q, self.lower, self.upper))

    def ik(self, xyz, pitch, seed, tol=1e-3):
        """Joint values placing the gripper origin at xyz with tool pitch `pitch`, or None.

        Prefers the elbow branch the seed already sits on; otherwise the solution closest to `seed`.
        q5 is copied from `seed`.
        """
        px, py = self.pan_xy
        dx, dy = xyz[0] - px, xyz[1] - py
        if math.hypot(dx, dy) < 0.02:      # too close to the pan axis: azimuth undefined
            return None
        # ahead is -y and joint 1 turns clockwise from above, so +x needs negative q1
        q1 = -math.atan2(dx, -dy)
        if not self.lower[0] <= q1 <= self.upper[0]:
            return None

        target = np.asarray(xyz, dtype=float)
        best, best_cost = None, None
        starts = [(seed[1], seed[2])] + [(a, b) for a in (-1.2, -0.4, 0.4, 1.2) for b in (-1.2, -0.4, 0.4, 1.2)]
        for a, b in starts:
            q23 = np.array([a, b], dtype=float)
            ok = False
            for _ in range(40):
                q = [q1, q23[0], q23[1], pitch - q23[0] - q23[1], seed[4]]
                r = self.position(q) - target
                if np.linalg.norm(r) < tol:
                    ok = True
                    break
                jac = np.empty((3, 2))
                for k in range(2):
                    d = q23.copy()
                    d[k] += 1e-6
                    qd = [q1, d[0], d[1], pitch - d[0] - d[1], seed[4]]
                    jac[:, k] = (self.position(qd) - self.position(q)) / 1e-6
                # damped least squares
                step = np.linalg.solve(jac.T @ jac + 1e-6 * np.eye(2), jac.T @ r)
                q23 = q23 - np.clip(step, -0.5, 0.5)
            if not ok:
                continue
            q = [q1, float(q23[0]), float(q23[1]), float(pitch - q23[0] - q23[1]), float(seed[4])]
            if not self.in_limits(q):
                continue
            cost = sum((x - y) ** 2 for x, y in zip(q, seed))
            if best is None or cost < best_cost - 1e-9:
                best, best_cost = q, cost
            if a == seed[1] and b == seed[2]:
                break   # the seed's own basin converged: keeps the path on one elbow branch, and is fast
        return best


def densify(waypoints, step):
    """waypoints: [(xyz, pitch), ...] -> list of (xyz, pitch) spaced <= `step` m apart (linear)."""
    out = [waypoints[0]]
    for (p0, s0), (p1, s1) in zip(waypoints, waypoints[1:]):
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        n = max(1, int(math.ceil(np.linalg.norm(p1 - p0) / step)))
        for i in range(1, n + 1):
            t = i / n
            out.append((tuple(p0 + t * (p1 - p0)), s0 + t * (s1 - s0)))
    return out


def solve_path(kin, dense, q_start, max_step=0.3):
    """IK along a densified path, warm-started from the previous solution. max_step (rad per path point)
    only has to catch elbow flips (~1 rad); holding tool pitch fixed already gives ~0.07 rad per cm.

    q_start is used verbatim as the first point (so the trajectory begins exactly at the
    current state). Returns (joint_list, None) or (None, reason).
    """
    qs = [list(q_start)]
    for i, (xyz, pitch) in enumerate(dense[1:], start=1):
        q = kin.ik(xyz, pitch, qs[-1])
        if q is None:
            return None, f"no IK solution at path point {i}/{len(dense) - 1} xyz=({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) pitch={math.degrees(pitch):.0f} deg"
        jump = max(abs(a - b) for a, b in zip(q, qs[-1]))
        if jump > max_step:
            return None, f"joint jump of {jump:.3f} rad at path point {i} (elbow/branch flip)"
        qs.append(q)
    return qs, None


def time_parameterize(qs, v_max, a_max=None):
    """[(time_s, q, qdot)] with a trapezoid-ish velocity profile: per-segment time is set by the
    fastest joint at <= v_max rad/s, and the profile ramps in/out so start/end velocities are 0.
    """
    a_max = a_max or 3.0 * v_max
    q = np.asarray(qs)
    seg = np.max(np.abs(np.diff(q, axis=0)), axis=1)                # per-segment lead-joint travel
    total = float(np.sum(seg))
    if total < 1e-9:
        return None
    cum = np.concatenate([[0.0], np.cumsum(seg)])                   # path coordinate s (rad of lead joint)
    # 1-D speed profile along s: accelerate at a_max, cruise at v_max, decelerate
    t_acc = v_max / a_max
    s_acc = 0.5 * a_max * t_acc ** 2
    if 2 * s_acc >= total:      # never reaches cruise
        t_acc = math.sqrt(total / a_max)
        v_peak, s_acc = a_max * t_acc, total / 2
        t_cruise = 0.0
    else:
        v_peak = v_max
        t_cruise = (total - 2 * s_acc) / v_max

    def t_of_s(s):
        if s <= s_acc:
            return math.sqrt(2 * s / a_max)
        if s <= total - s_acc:
            return t_acc + (s - s_acc) / v_peak
        return t_acc + t_cruise + t_acc - math.sqrt(max(0.0, 2 * (total - s) / a_max))

    def v_of_s(s):
        if s <= s_acc:
            return math.sqrt(2 * a_max * s)
        if s <= total - s_acc:
            return v_peak
        return math.sqrt(max(0.0, 2 * a_max * (total - s)))

    out = []
    for i, s in enumerate(cum):
        if i == 0 or i == len(cum) - 1:
            qdot = np.zeros(q.shape[1])
        else:
            tang = (q[i + 1] - q[i - 1]) / max(1e-9, cum[i + 1] - cum[i - 1])
            qdot = tang * v_of_s(s)
        out.append((t_of_s(s), q[i], qdot))
    # strictly increasing times
    for i in range(1, len(out)):
        if out[i][0] <= out[i - 1][0]:
            out[i] = (out[i - 1][0] + 1e-3, out[i][1], out[i][2])
    return out
