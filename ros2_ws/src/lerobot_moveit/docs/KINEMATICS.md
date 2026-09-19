# SO-101 forward kinematics — worked calculation

How the position and orientation of every link is computed from the six joint angles, derived from the
robot's own URDF and **checked against MoveIt's `/compute_fk`**. Everything below is reproducible with
[`fk_calc.py`](fk_calc.py) (numpy only; no ROS needed except for the MoveIt comparison).

```bash
python3 docs/fk_calc.py                       # joint table + FK of the SRDF "home" pose
python3 docs/fk_calc.py --q 0 0 0 0 0 0       # any pose, radians (joints 1–6; joint 6 optional)
python3 docs/fk_calc.py --steps               # every cumulative 4×4 transform
python3 docs/fk_calc.py --planar              # the closed-form formula and its worked example
python3 docs/fk_calc.py --moveit              # compare against a running move_group's /compute_fk
```
(Source the workspace first so `xacro` and `lerobot_description` are found, or pass `--urdf FILE`.
`--moveit` needs the system `python3` and a running stack on the same `ROS_DOMAIN_ID`.)

## 1. Setup and conventions

* **Frame `base`** (= `world`; the URDF's `base_joint` is an identity fixed joint).
  **+x = the arm's left, −y = forward, +z = up**, viewed from behind the arm.
* Six revolute joints, named `1`…`6`. Joints 1–5 form the `arm` MoveIt group; joint 6 is the gripper's moving jaw.
* The URDF gives, for every joint, a **fixed origin transform** from the parent link (`xyz` + `rpy`) and a
  rotation about the joint's **local +z axis** (every joint here has `<axis xyz="0 0 1"/>`).
* URDF `rpy` means `R = Rz(yaw) · Ry(pitch) · Rx(roll)`.

### Joint table (from `so101.urdf.xacro`)

| joint | parent → child | origin `xyz` [m] | origin `rpy` [rad] | limits [rad] |
|---|---|---|---|---|
| 1 pan | base → shoulder | (+0.02079, −0.02307, +0.09488) | (−π, 0, +π/2) | ±1.9199 |
| 2 shoulder lift | shoulder → upper_arm | (−0.03040, −0.01828, −0.05420) | (−π/2, −π/2, 0) | ±1.7453 |
| 3 elbow | upper_arm → lower_arm | (−0.11257, −0.02800, 0) | (0, 0, +π/2) | −1.7453 … +1.5708 |
| 4 wrist pitch | lower_arm → wrist | (−0.13490, +0.00520, 0) | (0, 0, −π/2) | ±1.6581 |
| 5 wrist roll | wrist → gripper | (0, −0.06110, +0.01810) | (+π/2, 0, +π) | ±2.7925 |
| 6 jaw | gripper → jaw | (+0.02020, +0.01880, −0.02340) | (+π/2, 0, 0) | −0.1745 … +1.7453 |

## 2. The forward-kinematics equation

Each joint contributes one homogeneous transform: its fixed origin, then the rotation about its own z:

$$
T_{\text{origin},i}=\begin{bmatrix} R(\text{rpy}_i) & \mathbf{p}_i \\ \mathbf{0}^{\top} & 1 \end{bmatrix},
\qquad
R_z(q)=\begin{bmatrix}\cos q & -\sin q & 0 & 0\\ \sin q & \cos q & 0 & 0\\ 0 & 0 & 1 & 0\\ 0 & 0 & 0 & 1\end{bmatrix}
$$

$$
\boxed{\,T_i \;=\; T_{i-1}\; T_{\text{origin},i}\; R_z(q_i)\,},\qquad T_0=I,\quad i=1\ldots 6
$$

where $\mathbf{p}_i$ is the joint's `xyz` and $R(\text{rpy}) = R_z(\text{yaw})\,R_y(\text{pitch})\,R_x(\text{roll})$.

`T_i` is the pose of link *i*'s frame in `base`; its last column is the link origin, its top-left 3×3 the
orientation. The **gripper link** (the tip frame of the `arm` group, and the point the
[`lerobot_ik_demo`](../../lerobot_ik_demo/README.md) planner drives) is `T_5`; the jaw is `T_6`.

**Direction of joint 1, from the numbers.** Its origin has `rpy = (−π, 0, +π/2)`, so
`R = Rz(π/2)·Rx(−π)` and the joint's z axis in `base` is `R·(0,0,1) = (0, 0, −1)` — pointing **down**.
A positive `q1` is therefore a rotation about −z: **clockwise seen from above**, swinging the tool toward
**−x, the arm's right** (viewed from behind). Checked: `q1 = +0.5` moves the gripper x from +0.0206 to −0.1013.

Joint axes in `base` at `q = 0` (third column of each `T_i`):

| joint | axis in `base` | meaning |
|---|---|---|
| 1 | (0, 0, −1) | vertical (pan) |
| 2, 3, 4 | (+1, 0, 0) | three **parallel** horizontal axes — a planar 3-link arm |
| 5 | (0, +1, 0) | along the forearm/tool direction (wrist roll) |
| 6 | (−1, 0, 0) | jaw hinge |

The parallel axes 2–4 are why *tool pitch = q2 + q3 + q4* (used by the IK, §5).

## 3. Worked example — zero pose (`q = 0`)

All rotations vanish, so every `T_i` is the product of the fixed origins (`python3 docs/fk_calc.py --q 0 0 0 0 0 0`):

| link | origin in `base` [m] |
|---|---|
| shoulder (after joint 1) | (+0.02079, −0.02307, +0.09488) |
| upper_arm | (+0.00251, −0.05347, +0.14908) |
| lower_arm | (+0.00251, −0.08147, +0.26165) |
| wrist | (+0.00251, −0.21637, +0.26685) |
| **gripper** | **(+0.02062, −0.27747, +0.26685)** |
| jaw | (+0.03942, −0.30087, +0.28705) |

With the arm at zero it is stretched **forward and level**: the wrist and gripper both sit at z = 0.26685 m.

## 4. Worked example — the SRDF `home` pose

`home` in `config/so101.srdf`: `q = (0.01534, −0.01534, 0.23476, −0.01227, −0.00614, 0.00767)` rad
= (0.88°, −0.88°, 13.45°, −0.70°, −0.35°, 0.44°).

| link | origin in `base` [m] |
|---|---|
| shoulder | (+0.02079, −0.02307, +0.09488) |
| upper_arm | (+0.00205, −0.05319, +0.14908) |
| lower_arm | (+0.00165, −0.07946, +0.26207) |
| wrist | (−0.00039, −0.21224, +0.23778) |
| **gripper** | **(+0.01679, −0.27230, +0.22522)** |
| jaw | (+0.03505, −0.29966, +0.24028) |

The full gripper pose is

```
T_5 = | −0.00929   0.99984   0.01500   +0.01679 |
      | −0.20553  −0.01659   0.97851   −0.27230 |
      |  0.97861   0.00600   0.20565   +0.22522 |
      |  0         0         0          1       |
```

## 5. Closed form: the arm is a planar 3-link chain plus a pan

Because axes 2–4 are parallel and axis 1 is vertical, the gripper origin has a compact closed form
(`python3 docs/fk_calc.py --planar` derives the constants from the URDF and checks the formula).

Work in the arm's vertical plane: **ρ** = forward distance from the pan axis, **z** = up. The pan axis passes
through base-frame (0.02079, −0.02307). Constants, all read off the URDF at `q2 = q3 = q4 = 0`:

| symbol | value | meaning |
|---|---|---|
| **p₂** | (+0.03040, +0.14908) | joint-2 origin in the (ρ, z) plane |
| **a** | (+0.02800, +0.11257), \|a\| = **0.11600** m | joint 2 → joint 3 (upper arm) |
| **b** | (+0.13490, +0.00520), \|b\| = **0.13500** m | joint 3 → joint 4 (forearm) |
| **c** | (+0.06110, 0), \|c\| = **0.06110** m | joint 4 → gripper origin |
| **λ** | −0.176 mm | the plane sits this far beside the pan axis (a URDF construction detail) |

A positive joint angle turns the (ρ, z) plane clockwise, so $R(t)$ is the 2-D rotation matrix by $-t$:

$$
\begin{pmatrix}\rho\\ z\end{pmatrix}
= \mathbf{p}_2 + R(q_2)\,\mathbf{a} + R(q_2+q_3)\,\mathbf{b} + R(q_2+q_3+q_4)\,\mathbf{c}
\qquad (1)
$$

$$
\begin{aligned}
x &= p_x - \rho\,\sin q_1 + \lambda\,\cos q_1\\
y &= p_y - \rho\,\cos q_1 - \lambda\,\sin q_1\\
z &= z
\end{aligned}
\qquad (2)
$$

with $R(t)=\begin{bmatrix}\cos t & \sin t\\ -\sin t & \cos t\end{bmatrix}$ (rotation by $-t$) and $(p_x,p_y)$ the pan axis.

`q5` (wrist roll) and `q6` (jaw) do not appear: the gripper origin lies on the roll axis.
Over 200 random poses (q1…q4 across their full limits) this agrees with the full 4×4 chain to
**4·10⁻⁶ m** (4 µm; one servo tick is ~0.4 mm at this reach).

### Numeric walk-through for `home`

`q1..q4 = (0.01534, −0.01534, 0.23476, −0.01227)`:

| term | value (ρ, z) | rotation angle |
|---|---|---|
| p₂ | (+0.03040, +0.14908) | — |
| R(q2)·a | (+0.02627, +0.11299) | q2 = −0.01534 |
| R(q2+q3)·b | (+0.13280, −0.02429) | q2+q3 = +0.21941 |
| R(q2+q3+q4)·c | (+0.05979, −0.01257) | q2+q3+q4 = +0.20714 |
| **sum** | **ρ = +0.24926, z = +0.22522** | |

then the pan step, with `sin q1 = 0.01534`, `cos q1 = 0.99988`:

```
x = 0.02079 − 0.24926·0.01534 + (−0.000176)·0.99988 = +0.01679
y = −0.02307 − 0.24926·0.99988 − (−0.000176)·0.01534 = −0.27230
z = +0.22522
```

→ gripper origin **(+0.01679, −0.27230, +0.22522)** m — identical to the 4×4 chain in §4.

Two more, from the script: zero pose ρ = +0.25440, z = +0.26685 → (+0.02062, −0.27747, +0.26685);
`q = (0.4, 0.5, −0.8, 0.3)` → ρ = +0.29738, z = +0.27928 → (−0.09518, −0.29691, +0.27928).

## 6. Verification against MoveIt

`python3 docs/fk_calc.py --moveit` calls `/compute_fk` for all six links at six poses (zero, `home`, four
random poses inside the joint limits) and compares positions:

| pose | link | `fk_calc.py` (x, y, z) [m] | MoveIt `/compute_fk` | \|diff\| [m] |
|---|---|---|---|---|
| zero | gripper | (+0.02062, −0.27747, +0.26685) | same | 1.8·10⁻¹⁶ |
| zero | jaw | (+0.03942, −0.30087, +0.28705) | same | 2.4·10⁻¹⁶ |
| home | gripper | (+0.01679, −0.27230, +0.22522) | same | 2.2·10⁻¹⁶ |
| home | jaw | (+0.03505, −0.29966, +0.24028) | same | 2.4·10⁻¹⁶ |
| random 1 | gripper | (−0.01897, −0.09901, −0.02783) | same | 1.2·10⁻¹⁶ |
| random 2 | gripper | (+0.08678, −0.00074, −0.00861) | same | 1.3·10⁻¹⁶ |
| random 3 | gripper | (+0.20351, −0.15639, +0.31041) | same | 2.7·10⁻¹⁶ |
| random 4 | gripper | (−0.11574, −0.29205, +0.05244) | same | 1.9·10⁻¹⁶ |

Worst difference over all links and poses: **3.2·10⁻¹⁶ m** — i.e. equal to machine precision, as expected:
MoveIt evaluates the same URDF chain. To query it yourself:

```bash
ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK \
  "{header: {frame_id: base}, fk_link_names: [gripper, jaw],
    robot_state: {joint_state: {name: ['1','2','3','4','5','6'], position: [0.0153,-0.0153,0.2348,-0.0123,-0.0061,0.0077]}}}"
```

## 7. Reach

Sampling the joint limits (60 000 random poses), the **gripper origin** stays within
x ∈ [−0.320, +0.362], y ∈ [−0.364, +0.257], z ∈ [−0.094, +0.461] m, and at most **0.342 m** from the pan axis.
The fingertips extend roughly another 10 cm beyond the gripper-frame origin (by the gripper meshes' extent);
that offset is not part of the joint chain and is not modelled here.

## 8. Inverse kinematics, briefly

* The arm has **5 joints**, so it can reach a 3-D position plus one free *tool pitch* (= q2+q3+q4), not an
  arbitrary 6-D pose. MoveIt's default solver (`config/kinematics.yaml`: KDL, `position_only_ik: False`)
  asks for the full pose and, for a 5-DOF arm, **only converges when the requested orientation is already
  exactly reachable** — in practice it failed on most targets tried, including reachable positions.
* [`lerobot_ik_demo/scripts/so101_kin.py`](../../lerobot_ik_demo/scripts/so101_kin.py) inverts the closed
  form above: `q1` from the target's azimuth about the pan axis, `q2, q3` from a damped-Newton 2-link solve
  of equation (1) for the given pitch, and `q4 = pitch − q2 − q3`. It reads the joint origins from the same
  URDF, is unit-tested (round-trip error < 1 mm), and agrees with `/compute_fk` at the home pose.

## 9. What FK does *not* include

* **Gravity sag and servo error.** FK is the *modelled* geometry. On the real arm the measured joints
  differ from the commanded ones by a few degrees under load (joints 2–3 most), so the physical tip lands
  1–5 cm from the FK of the commanded pose. Always plan from measured `/joint_states`.
* **Calibration.** The URDF's zero is defined by the bridge's per-servo `home_raw` ticks
  (`lerobot_hardware/config/so101_calibration.yaml`); a wrong calibration shifts every FK result.
* **Collision geometry** (link meshes, padded obstacles) — see the MoveIt planning scene.
