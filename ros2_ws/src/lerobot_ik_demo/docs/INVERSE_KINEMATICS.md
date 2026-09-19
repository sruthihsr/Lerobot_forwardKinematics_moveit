# SO-101 inverse kinematics — worked calculation

Given where the gripper should be, find the joint angles. This document derives a **closed-form solution**
from the robot's URDF, works two of the loop's own targets through it by hand, and states how it relates to the
solver the planner actually runs (`scripts/so101_kin.py`) and to MoveIt's KDL — with measured numbers.
Everything is reproducible with [`ik_calc.py`](ik_calc.py) (numpy only; `--moveit` needs a running `move_group`).

```bash
python3 docs/ik_calc.py                                        # worked example: recover the SRDF "home" pose
python3 docs/ik_calc.py --target 0.2108 -0.1731 0.12 --pitch 30   # any target: metres, pitch in degrees
python3 docs/ik_calc.py --check                                # statistics over 2 000 random poses
python3 docs/ik_calc.py --moveit                               # compare with MoveIt's /compute_ik (isolated ROS domain advised)
```
(Source the workspace first so `xacro` and `lerobot_description` are found, or pass `--urdf FILE`.)

The forward kinematics this inverts, including the joint table and the planar constants used below, are derived in
[`lerobot_moveit/docs/KINEMATICS.md`](../../lerobot_moveit/docs/KINEMATICS.md).

## 1. The problem

The arm has **five** joints, so it cannot reach an arbitrary 6-D pose. The loop asks for what it can control:

| Input | Symbol | Meaning |
|---|---|---|
| gripper-frame origin | **(x, y, z)** in `base` | where the gripper is (the point the planner drives) |
| tool pitch | **φ = q2 + q3 + q4** | the one free orientation angle (joints 2–4 are parallel) |

| Output | Determined by |
|---|---|
| q1 (pan) | the azimuth of (x, y) about the pan axis |
| q2, q3 (shoulder, elbow) | the wrist point's reach and height |
| q4 (wrist pitch) | φ − q2 − q3 |
| q5 (wrist roll) | **free** — it does not move the gripper origin; the planner holds its current value |

Counting: 3 position + 1 pitch = 4 constraints, 4 unknowns (q1…q4), so solutions are isolated points
(a few discrete branches), not a continuum.

## 2. Setup from the forward kinematics

With the arm in its vertical plane (coordinates **ρ** = forward from the pan axis, **z** = up), and
`R(t)` = 2-D rotation by **−t** (a positive joint angle turns the plane clockwise):

```
P(ρ, z) = p₂ + R(q2)·a + R(q2+q3)·b + R(q2+q3+q4)·c                       (1)

x = pan_x − ρ·sin q1 + λ·cos q1
y = pan_y − ρ·cos q1 − λ·sin q1                                            (2)
```

| constant | value (from the URDF) |
|---|---|
| pan axis (pan_x, pan_y) | (0.02079, −0.02307) m |
| plane offset λ | −0.176 mm |
| p₂ (joint 2 in the plane) | (+0.03040, +0.14908) |
| **a** (joint 2 → 3) | (+0.02800, +0.11257), \|a\| = 0.11600 m, direction 76.032° |
| **b** (joint 3 → 4) | (+0.13490, +0.00520), \|b\| = 0.13500 m, direction 2.207° |
| **c** (joint 4 → gripper origin) | (+0.06110, 0), \|c\| = 0.06110 m |

## 3. Derivation

**Step 1 — pan (q1).** Let **d** = (x − pan_x, y − pan_y). Equation (2) says **d** = ρ·**f** + λ·**l** with the
orthonormal pair **f** = (−sin q1, −cos q1) (forward) and **l** = (cos q1, −sin q1). Taking the **l** component,
`λ = d·l = |d|·cos(q1 + α)` with `α = atan2(d_y, d_x)`, hence

```
q1 = −α ± arccos(λ / |d|),        ρ = d·f = −sin q1·d_x − cos q1·d_y
```

Two pan solutions: the **forward** one (ρ > 0, the arm reaches out in front) and a **behind** one
(ρ < 0, the arm reaches back over its own pan axis; q1 ≈ ±π away). Because |λ| ≈ 0.18 mm ≪ |d|,
the forward solution is `q1 ≈ −atan2(d_x, −d_y)`, in error by at most about `λ/|d|` ≈ 0.0007 rad (0.04°, 0.2 mm at 0.25 m)
— the approximation the planner's solver uses. Undefined when **d** ≈ 0 (target on the pan axis): declined below 2 cm.

**Step 2 — wrist point.** Only `c` depends on q4 and the pitch, so subtract it (with `φ` known) to get the point that
joints 2 and 3 alone must reach:

```
W = (ρ, z) − p₂ − R(φ)·c  =  R(q2)·a + R(q2+q3)·b
```

**Step 3 — elbow (q3), by the law of cosines.** Factor out `R(q2)`: `W = R(q2)·[ a + R(q3)·b ]`; rotating doesn't
change length, so `|W|² = |a + R(q3)·b|² = |a|² + |b|² + 2|a||b|·cos(∠b − ∠a − q3)`, where `∠a` = 76.032° and
`∠b` = 2.207° are the directions of **a** and **b**. Solving,

```
cos θ = ( |W|² − |a|² − |b|² ) / ( 2|a||b| ) ,         q3 = (∠b − ∠a) ∓ arccos(cos θ)  =  −73.825° ∓ θ
```

The ∓ gives the **two elbow branches** (A: elbow folded far the other way, B: the usual pose).
**Reachable only if |cos θ| ≤ 1**, i.e. `0.019 m = | |a|−|b| | ≤ |W| ≤ |a|+|b| = 0.251 m`.

**Step 4 — shoulder (q2).** With q3 known, `v = a + R(q3)·b` is a fixed vector and `W = R(q2)·v`, so

```
q2 = angle(v) − angle(W)            (angle(·) = atan2(vertical, ρ))
```

**Step 5 — wrist pitch (q4).**  `q4 = φ − q2 − q3`.

**Step 6 — wrist roll (q5)** is unconstrained.

Finally **filter by the joint limits** (`q1 ±1.920`, `q2 ±1.745`, `q3 −1.745…+1.571`, `q4 ±1.658` rad); every
candidate outside them is discarded. Up to 2 (pan) × 2 (elbow) = 4 candidates; typically one or two survive.

## 4. Worked example 1 — recover the SRDF `home` pose

Target: gripper origin (+0.01679, −0.27230, +0.22522) m and pitch φ = +0.20714 rad (11.87°) — i.e. the tip position
and pitch *of* the `home` pose, so the answer is known: q = (0.01534, −0.01534, 0.23476, −0.01227).

| step | calculation | result |
|---|---|---|
| 1 pan | **d** = (−0.00400, −0.24923), \|d\| = 0.24926 | forward: q1 = **+0.01534**, ρ = +0.24926 m (approx. formula: +0.01605) |
| 2 wrist point | **W** = (0.24926, 0.22522) − (0.03040, 0.14908) − R(0.2071)·(0.06110, 0) | **W** = (+0.15907, +0.08870), \|W\| = 0.18213 |
| 3 elbow | cos θ = (0.03317 − 0.01346 − 0.01823) / 0.03132 | = **+0.04754**, θ = 87.275° |
| 4 branch B | q3 = −73.825° + 87.275° | **q3 = +13.45° = +0.23476** |
| | q2 = angle(**v**) − angle(**W**) = +28.266° − 29.145° | **q2 = −0.88° = −0.01534** |
| | q4 = 0.20714 − (−0.01534) − 0.23476 | **q4 = −0.01227** |
| 4 branch A | q3 = −73.825° − 87.275° = −161.1° (−2.81173) | q2 = +1.65201, q4 = +1.36685 |

Branch B reproduces `home` exactly. Branch A reaches the same point with the elbow folded the other way, but
**q3 = −2.812 rad is outside [−1.745, +1.571]**, so it is discarded. The "behind" pan solution (q1 = −3.125) is
outside the q1 limits. FK of branch B lands within 1.5·10⁻⁷ m of the target.

## 5. Worked example 2 — a target from the loop

The default loop's `left_side` point with the cylinder at (0.0708, −0.2731): offset (+0.14, +0.10) → gripper
origin (0.2108, −0.1731, 0.12) m; take pitch φ = 30° (0.5236 rad).

| step | calculation | result |
|---|---|---|
| 1 pan | **d** = (+0.19001, −0.15003), \|d\| = 0.24210 | q1 = **−0.90317** rad (−51.7°), ρ = +0.24210 (approx. formula: −0.90245) |
| 2 wrist point | **W** = (0.24210, 0.12000) − (0.03040, 0.14908) − R(0.5236)·(0.06110, 0) | **W** = (+0.15878, +0.00147), \|W\| = 0.15879 |
| 3 elbow | cos θ = (0.02521 − 0.01346 − 0.01823) / 0.03132 | = **−0.20647**, θ = 101.915° |
| 4 branch B | q3 = −73.825° + 101.915° | **q3 = +28.09° = +0.49027** |
| | q2 = +19.742° − 0.530° | **q2 = +19.21° = +0.33532** |
| | q4 = 0.52360 − 0.33532 − 0.49027 | **q4 = −0.30200** |
| 4 branch A | q3 = −175.74° (−3.06725), q2 = +2.30022, q4 = +1.29063 | **discarded**: q2 and q3 outside their limits |

Answer: **q = (−0.90317, +0.33532, +0.49027, −0.30200)** rad, FK error 7.7·10⁻⁷ m. The planner's solver returns
(−0.90245, +0.33503, +0.49055, −0.30198): the same branch, within 0.0007 rad per joint (the q1 approximation above),
and a tip error under its 1 mm tolerance.

The loop's `top` point, (0.0708, −0.2731, 0.30) at φ = 20°, gives q = (−0.19810, +0.20245, −0.68474, +0.83135)
(branch A out of limits: q3 = −1.892); the planner's solver returns (−0.19741, +0.19953, −0.67942, +0.82895).

**Unreachable example.** (0, −0.60, 0.10) at φ = 0: cos θ = +6.60 > 1 — the wrist point is 0.49 m from the shoulder, far beyond
|a| + |b| = 0.251 m — so there is no solution.

## 6. The solver the planner actually runs

`so101_kin.Kin.ik(xyz, pitch, seed)` solves the same equations **numerically**: `q1 = −atan2(d_x, −d_y)`, then a
damped Newton iteration on (q2, q3) — with `q4 = φ − q2 − q3` — that drives the FK position error below 1 mm,
started from the previous solution so a path stays on one elbow branch, then a joint-limit check; `q5` is copied
from the seed. Around a 1 cm path step it is warm-started from the last point (`solve_path`), and steps whose
joints jump more than 0.3 rad are rejected as elbow flips.

Because it is the same geometry, it lands on the closed form's branch (§5). Measured over random poses drawn
uniformly across the joint limits (`ik_calc.py --check`, 1 823 poses):

| | closed form | planner's solver, **all** poses | planner's solver, **working region** |
|---|---|---|---|
| finds a limit-respecting solution | 1 820 / 1 823 | seeded 3° off: 82.2 % | seeded 3° off: **99.9 %** (1 358/1 360) |
| one 1 cm path step from the previous solution | — | 86.6 % | **99.7 %** (1 230/1 234) |
| accuracy | FK error ≤ 4.5 µm | tip error median 0.20 mm, max 0.99 mm | same |

"Working region" = the arm reaches forward of the pan axis and the elbow is not within ~11° of straight
(|cos θ| < 0.98) — where the loop operates. The solver's misses fall almost entirely **outside** it, for two reasons:
it only ever returns the **forward** pan solution, and Newton converges poorly near the stretched-elbow singularity.
The closed form has neither weakness. On the real arm, the successful loop cycles ran without IK failures; the IK
refusals seen on hardware were where the joint limits are genuinely exhausted (for example a steeply
down-pointing tool pitch of ~95° during a lift, which needs more wrist range than exists), not solver misses.

The closed form declines targets within 2 cm of the pan axis (azimuth undefined; excluded from the table above). All
seven of its misses seen before adding the rounding clamp were at the fully-stretched elbow (`q3 = −73.8°`, cos θ = 1),
where the two branches merge and rounding can push `|cos θ|` past 1; the clamp (within 10⁻⁶) removed most of them.
The 3 remaining misses out of 1 823 were not individually examined.

*(The closed form is documented here as a reference and check; the planner still uses the Newton solver — nothing in
`scripts/` was changed for this document.)*

## 7. Comparison with MoveIt's KDL

`config/kinematics.yaml` selects KDL with `position_only_ik: False`, so `/compute_ik` needs a **full 6-D pose**.
A 5-joint arm can realise only a 2-parameter family of orientations at each position (free pitch and roll; the
heading is fixed by the pan). Measured with `ik_calc.py --moveit` on 76 random reachable targets whose full pose
comes from FK (so an exact solution exists):

| solver | solved |
|---|---|
| MoveIt `/compute_ik` (KDL), seeded 3° from the answer | 73 / 76 |
| MoveIt `/compute_ik` (KDL), seeded at the `home` pose | 71 / 76 |
| closed form (position + pitch, no seed) | 76 / 76 |

(KDL is randomised; a run of 60 gave 60/60 and 59/60 — expect a few percent of misses.) So KDL **works when the
requested orientation is exactly reachable, which is what the numbers above supply**; it does not when the orientation
is merely *approximately* consistent — as happened when the loop's poses were first built from a hand-made orientation
family — or when the goal comes from an RViz interactive marker with an arbitrary orientation. Asking for position
and pitch, as the closed form does, avoids the issue.

## 8. How the planner chooses the pitch

The loop does not fix φ. For each hop `arm_client.plan_to` tries target pitches `φ_now + k·10°` (k = 0, ±1 … ±9,
nearest first), solves IK along a 1 cm-spaced path with the pitch interpolated between waypoints, and keeps the first
pitch whose whole path is IK-feasible and collision-free (MoveIt `/check_state_validity`). The lift leg eases the
pitch toward the target pitch so the joints do not run out of range while rising.

## 9. Accuracy and limits

* **Numerical:** closed form ≤ 4.5 µm against the full 4×4 chain; planner's solver ≤ 1 mm by construction (median 0.2 mm).
  One servo tick is ≈ 0.09° ≈ 0.4 mm at 0.25 m reach, so both are far below the hardware's resolution.
* **Physical:** the *modelled* joint angles are exact for the URDF, but the real arm sags a few degrees under load
  (joints 2–3 most), so the tip lands 1–5 cm from the IK target. Always re-plan from measured `/joint_states`.
* **Singularities:** stretched elbow (`q3 = −73.8°`, `|cos θ| = 1`) and the pan axis (ρ → 0).
* **Not modelled:** the fingertips (≈ 10 cm beyond the gripper-frame origin), the gripper joint, collisions
  (handled separately by MoveIt).

## 10. Code map

| Piece | Where |
|---|---|
| closed form (this document) | `docs/ik_calc.py` → `solve()` |
| planner's IK | `scripts/so101_kin.py` → `Kin.ik`, `solve_path` |
| pitch search, routing over the cylinder | `scripts/arm_client.py` → `plan_to` |
| unit tests for FK/IK round trip | `test/test_kinematics.py` |
| forward kinematics | `lerobot_moveit/docs/KINEMATICS.md`, `fk_calc.py` |
