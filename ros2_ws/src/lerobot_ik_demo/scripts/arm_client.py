"""
Shared plumbing for over_the_top_demo.py and click_to_move.py.

Plans gripper-origin paths with so101_kin (own IK), validates every point
against MoveIt's planning scene (padded cylinder + self-collision) via
/check_state_validity, then executes through move_group's /execute_trajectory
-- so the same code drives mock hardware, Gazebo, and the real Feetech bridge.

Routing rule: if the straight line from the current tip to the target would
pass through the cylinder's padded footprint below the "arch" height, the path
is lifted to the arch height first, carried over the top, and lowered onto
the target. Otherwise it goes straight.
"""
import math
import subprocess
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import CollisionObject, DisplayTrajectory, MoveItErrorCodes, PlanningScene, RobotState, RobotTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

from add_obstacle_scene import COLLISION_HEIGHT, COLLISION_RADIUS, OBJECT_CENTER_XY, PLANNING_FRAME, REAL_RADIUS
from so101_kin import ARM_JOINTS, Kin, densify, solve_path, time_parameterize

GROUP = "arm"
PATH_STEP = 0.01           # m between IK'd path points
LATERAL_MARGIN = 0.03      # m beyond the padded radius that still counts as "would hit the cylinder" (gripper body width)
OVER_MARGIN = 0.01         # m the gripper origin must stay above the padded top while over the footprint
PITCH_STEP = math.radians(10)
PITCH_TRIES = 9            # target pitch candidates: current +/- k*10 deg, k=0..9


def add_cylinder_args(parser):
    parser.add_argument("--object-xy", type=float, nargs=2, metavar=("X", "Y"),
                        help="cylinder centre in the base frame (m), overriding OBJECT_CENTER_XY in "
                             "add_obstacle_scene.py -- use your tape-measure values on the real setup")
    parser.add_argument("--clearance", type=float, default=0.05,
                        help="m the gripper origin stays above the PADDED cylinder top when crossing (default 0.05)")
    parser.add_argument("--speed", type=float, default=0.4, help="peak joint speed, rad/s (default 0.4)")


@dataclass
class Cylinder:
    cx: float
    cy: float
    radius: float   # padded
    height: float   # padded

    @classmethod
    def from_args(cls, args):
        cx, cy = args.object_xy if args.object_xy else OBJECT_CENTER_XY
        return cls(cx, cy, COLLISION_RADIUS, COLLISION_HEIGHT)


def _dist_point_segment(c, a, b):
    a, b, c = np.asarray(a, float), np.asarray(b, float), np.asarray(c, float)
    ab = b - a
    t = 0.0 if np.dot(ab, ab) < 1e-12 else float(np.clip(np.dot(c - a, ab) / np.dot(ab, ab), 0.0, 1.0))
    return float(np.linalg.norm(c - (a + t * ab)))


class ArmClient(Node):
    def __init__(self, name, cyl, clearance, speed):
        super().__init__(name)
        self.cyl, self.clearance, self.speed = cyl, clearance, speed
        self.z_arch = cyl.height + clearance
        self.scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")
        self.exec = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self.display = self.create_publisher(DisplayTrajectory, "/display_planned_path", 10)
        self.joints = {}
        self.create_subscription(JointState, "/joint_states", lambda m: self.joints.update(zip(m.name, m.position)), 10)
        xacro = f"{get_package_share_directory('lerobot_description')}/urdf/so101.urdf.xacro"
        self.kin = Kin(subprocess.run(["xacro", xacro], capture_output=True, text=True, check=True).stdout)

    # ---- setup ----------------------------------------------------------
    def wait_ready(self):
        for name, c in (("apply_planning_scene", self.scene), ("check_state_validity", self.validity)):
            if not c.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f"/{name} not available -- is move_group running?")
        if not self.exec.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("execute_trajectory not available -- is move_group running?")
        deadline = time.time() + 10.0
        while not all(j in self.joints for j in ARM_JOINTS) and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not all(j in self.joints for j in ARM_JOINTS):
            raise RuntimeError("no /joint_states for the arm joints")

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def ensure_cylinder(self):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.cyl.cx, self.cyl.cy, self.cyl.height / 2.0
        pose.orientation.w = 1.0
        obj = CollisionObject(id="obstacle_cylinder", operation=CollisionObject.ADD)
        obj.header.frame_id = PLANNING_FRAME
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[self.cyl.height, self.cyl.radius])]
        obj.primitive_poses = [pose]
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [obj]
        res = self.call(self.scene, ApplyPlanningScene.Request(scene=scene))
        if res is None or not res.success:
            raise RuntimeError("failed to add obstacle_cylinder to the planning scene")

    def current_q(self):
        return [self.joints[j] for j in ARM_JOINTS]

    # ---- validation -----------------------------------------------------
    def state_valid(self, q):
        req = GetStateValidity.Request(group_name=GROUP)
        req.robot_state = RobotState(joint_state=JointState(name=list(ARM_JOINTS), position=[float(v) for v in q]))
        res = self.call(self.validity, req)
        return res is not None and res.valid

    # ---- planning -------------------------------------------------------
    def plan_to(self, target, start_q=None):
        """Plan tip -> target (gripper-origin xyz, base frame), from the current state unless
        `start_q` is given (for chaining plan-only legs). Returns (qs, None) or (None, reason)."""
        c = self.cyl
        q0 = list(start_q) if start_q is not None else self.current_q()
        p0, s0 = self.kin.position(q0), Kin.pitch(q0)
        tgt = np.asarray(target, float)

        keep_out = c.radius + LATERAL_MARGIN
        if math.hypot(tgt[0] - c.cx, tgt[1] - c.cy) < keep_out and tgt[2] < self.z_arch - 1e-9:
            return None, (f"target is inside the cylinder's padded footprint below the arch height "
                          f"(z < {self.z_arch:.3f} m)")

        crosses = (_dist_point_segment((c.cx, c.cy), p0[:2], tgt[:2]) < keep_out
                   and min(p0[2], tgt[2]) < self.z_arch - 1e-9)

        ik_reason = collision_reason = None   # first of each kind: IK failures alone hide a blocked-but-solvable path
        for k in sorted(range(-PITCH_TRIES, PITCH_TRIES + 1), key=abs):
            st = s0 + k * PITCH_STEP
            wps = [(tuple(p0), s0)]
            if crosses:
                # ease toward the target pitch during the lift: holding the start pitch can run out
                # of joint range (e.g. a steeply down-pointing tool) before reaching arch height
                wps.append(((p0[0], p0[1], max(self.z_arch, p0[2])), 0.5 * (s0 + st)))
                wps.append(((tgt[0], tgt[1], max(self.z_arch, tgt[2])), st))
            wps.append((tuple(tgt), st))
            dense = densify(wps, PATH_STEP)
            qs, why = solve_path(self.kin, dense, q0)
            if qs is None:
                ik_reason = ik_reason or why
                continue
            bad = self._check_path(dense, qs)
            if bad:
                collision_reason = collision_reason or f"{bad} (at tool pitch {math.degrees(st):+.0f} deg)"
                continue
            return qs, None
        reason = collision_reason or ik_reason
        return None, f"{reason} (tried tool pitches within +/-{math.degrees(PITCH_TRIES * PITCH_STEP):.0f} deg of current)"

    def plan_escape(self, max_deg=60, real_margin=0.02, extra_deg=8):
        """If the arm currently sits inside the cylinder's safety pad (MoveIt says invalid), plan a
        base-pan-only retreat to the first valid pose. Deliberately narrow: only joint 1 moves, and a
        step is accepted only if the gripper origin gets strictly farther from the cylinder axis.
        Refuses if the tip is already within `real_margin` of the REAL (unpadded) cylinder.
        Returns (qs, message); qs is None if there is nothing to do or it can't be done safely."""
        c, q0 = self.cyl, self.current_q()
        if self.state_valid(q0):
            return None, "current pose is already collision-free; no escape needed"
        d0 = math.hypot(*(self.kin.position(q0)[:2] - np.array([c.cx, c.cy])))
        if d0 < REAL_RADIUS + real_margin:
            return None, (f"gripper origin is only {d0:.3f} m from the cylinder axis (real radius "
                          f"{REAL_RADIUS:.2f} m): too close to escape automatically -- move the arm by hand")
        for sign in (-1, 1):
            qs, last = [q0], d0
            for step in range(1, max_deg + 1):
                q = list(q0)
                q[0] = q0[0] + sign * math.radians(step)
                if not self.kin.in_limits(q):
                    break
                d = math.hypot(*(self.kin.position(q)[:2] - np.array([c.cx, c.cy])))
                if d <= last + 1e-4:   # not moving away from the cylinder: wrong direction
                    break
                qs.append(q)
                last = d
                if self.state_valid(q):
                    # the real arm tracks a few degrees short (sag/deadband), so retreat well past the
                    # first valid pose, still only while moving away and staying valid
                    for extra in range(1, extra_deg + 1):
                        q = list(q0)
                        q[0] = q0[0] + sign * math.radians(step + extra)
                        d2 = math.hypot(*(self.kin.position(q)[:2] - np.array([c.cx, c.cy])))
                        if not self.kin.in_limits(q) or d2 <= last + 1e-4 or not self.state_valid(q):
                            break
                        qs.append(q)
                        last = d2
                    return qs, f"pan {'-' if sign < 0 else '+'}{len(qs) - 1} deg (tip {d0:.3f} -> {last:.3f} m from the cylinder axis)"
        return None, "no pan-only retreat found that keeps moving away from the cylinder"

    def _check_path(self, dense, qs):
        c = self.cyl
        for i, (xyz, _) in enumerate(dense):
            if math.hypot(xyz[0] - c.cx, xyz[1] - c.cy) < c.radius and xyz[2] < c.height + OVER_MARGIN:
                return f"path point {i} passes through the cylinder (z={xyz[2]:.3f})"
        for i in list(range(1, len(qs), 2)) + [len(qs) - 1]:
            if not self.state_valid(qs[i]):
                return f"path point {i}/{len(qs) - 1} is in collision in MoveIt's planning scene"
        return None

    # ---- execution ------------------------------------------------------
    def to_trajectory(self, qs):
        profile = time_parameterize(qs, self.speed)
        if profile is None:
            return None
        traj = RobotTrajectory()
        traj.joint_trajectory.joint_names = list(ARM_JOINTS)
        for t, q, qd in profile:
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in q]
            p.velocities = [float(v) for v in qd]
            p.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            traj.joint_trajectory.points.append(p)
        return traj

    def show(self, traj):
        msg = DisplayTrajectory(model_id="so101")
        msg.trajectory_start = RobotState(joint_state=JointState(name=list(ARM_JOINTS), position=self.current_q()))
        msg.trajectory = [traj]
        self.display.publish(msg)

    def execute(self, traj):
        fut = self.exec.send_goal_async(ExecuteTrajectory.Goal(trajectory=traj))
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return False
        rfut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rfut)
        return rfut.result().result.error_code.val == MoveItErrorCodes.SUCCESS

    def settle(self, target_q, tol=0.05, timeout=6.0):
        """The real bridge reports success when the last waypoint is *sent* (see lerobot_hardware
        README), so wait for the arm to actually get there before planning from its state."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if max(abs(a - b) for a, b in zip(self.current_q(), target_q)) < tol:
                return True
        return False

    def go(self, target, execute=True, start_q=None, retries=2):
        """Plan + (optionally) execute to target. Returns (ok, message, end_q).

        On the real arm the joints keep sagging/drifting while the path is being planned and validated
        (2-3 s), so MoveIt can reject the trajectory up front ("start point deviates from current robot
        state more than 0.01") before anything moves. If the arm has NOT moved from the planned start,
        re-plan from the fresh state and retry (at most `retries` times); anything else fails.
        """
        for attempt in range(retries + 1):
            qs, why = self.plan_to(target, start_q)
            if qs is None:
                return False, why, None
            traj = self.to_trajectory(qs)
            if traj is None:
                return True, "already at the target", qs[-1]
            dur = traj.joint_trajectory.points[-1].time_from_start
            self.show(traj)
            info = f"{len(qs)} points, {dur.sec + dur.nanosec * 1e-9:.1f} s"
            if not execute:
                return True, f"planned {info} (not executed)", qs[-1]
            if self.execute(traj):
                settled = self.settle(qs[-1])
                retried = f" after {attempt} re-plan(s)" if attempt else ""
                return True, f"executed {info}{retried}" + ("" if settled else " (arm did not settle within tolerance)"), qs[-1]
            rclpy.spin_once(self, timeout_sec=0.2)   # refresh joint state
            moved = max(abs(a - b) for a, b in zip(self.current_q(), qs[0]))
            if moved > 0.05 or attempt == retries:
                why = "execution failed" + (" after the arm had started moving" if moved > 0.05 else f" {retries + 1} times")
                return False, why, None
            self.get_logger().warn(f"execution rejected with the arm still at the plan start (drifted {moved:.3f} rad); re-planning")
            start_q = None   # plan from the fresh measured state
        return False, "execution failed", None
