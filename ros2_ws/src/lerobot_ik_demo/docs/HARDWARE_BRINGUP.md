# Running on the real SO-101 arm

Follow this order. **Each step must pass before the next.** Design background:
[ARCHITECTURE.md](ARCHITECTURE.md). Servo-level details: `lerobot_hardware/README.md`.

> ⚠️ Live mode moves a real arm next to a real object. Keep a hand on the power switch, keep the workspace
> clear, and watch the first cycles. The arm's tip lands 1–5 cm off target (gravity sag, see
> [ARCHITECTURE §7](ARCHITECTURE.md#7-sim-to-real-gap-known-limitations)); the safety pad is 3 cm.

## 0. Prerequisites

* ROS 2 Jazzy workspace built (`colcon build`), including `lerobot_hardware` — `run.sh` does this.
* `/usr/bin/python3` used, not conda (`run.sh` handles `PATH`). The bridge needs `feetech-servo-sdk`
  (`/usr/bin/python3 -m pip install -r requirements.txt`).
* Arm powered (servo bus supply **and** USB), `ls /dev/ttyACM0` exists, user in the `dialout` group.
* Servos calibrated: `lerobot_hardware/config/so101_calibration.yaml` holds each servo's raw tick at the
  URDF zero pose ("home": arm straight, wrist level). See `lerobot_hardware/README.md` → Calibration.

## 1. Dry run — connect read-only (nothing moves)

```bash
./run.sh real            # dry_run:=True, plan-only
```

Check in the log: all 6 servos `ping OK`, `dry_run=True: connecting read-only`. Then check the readings
match the physical arm (`ros2 topic echo /joint_states --once --field position`, radians):

* All five arm joints must be **inside the URDF limits** (±110°, ±100°, −100…+90°, ±95°, ±160°). A folded
  storage pose reads outside them and MoveIt refuses to plan ("Start state is outside bounds").
* Compare RViz's robot model with the real arm — they should match, wrist roll included. A large mismatch on
  one joint means that servo needs recalibrating. (One joint reading ~90° off is *not* necessarily a bug:
  check whether the arm is physically rolled that far before recalibrating.)
* Readings are quantised (~0.09°): a still arm gives identical numbers every time; a hand-moved arm changes.

## 2. Move the arm to a clear start pose

While the bridge is in dry-run, torque is off and you can pose the arm by hand. Put the tip **well clear of
the cylinder** — ≥ 22° of pan away is enough, ~+35° is comfortable (positive pan = the arm's right, away
from a cylinder on its left). The plan starts from wherever the arm is and refuses a start inside the
cylinder's safety pad. (`--escape`, on by default in `run.sh`, can retreat a short pan-only distance if the
tip is outside the real cylinder — don't rely on it.)

## 3. Place and measure the cylinder

Measure from the **shoulder-pan axis** (the vertical axis joint 1 turns about), to the middle of the cylinder:

* `forward` metres in front of the pan axis, `left` metres to the arm's left (negative = right).
* Convert to the base frame: `x = 0.0208 + left`, `y = −0.0231 − forward`.

  Example: 25 cm forward, 5 cm left → `x = 0.0708`, `y = −0.2731` (the run.sh defaults).

Pass them as `--x X --y Y`. The default loop points are offsets from the cylinder centre, so they follow.
A wrong position means the plan avoids empty space and not the object.

## 4. Plan-only check with the real readings

```bash
./run.sh real --x 0.0708 --y -0.2731        # replace with your numbers
```

Expect `pre-flight OK for 3 points; nothing moved`. Common refusals:

| Message | Meaning / fix |
|---|---|
| `path point 0 passes through the cylinder` / `path point 1 … in collision` | Start pose is inside the cylinder's pad. Pan the arm away and re-run. |
| `no IK solution at path point …` | A point (or the lift) is out of reach at every tool pitch tried. Move the object closer or edit `config/loop_points.yaml`. |
| `fewer than 2 usable points` | Everything was refused; fix the start pose or the object position first. |

The stack stays up in plan-only mode: open RViz and confirm the green cylinder sits where the real one does.

## 5. Live run

```bash
./run.sh real --live --execute --x 0.0708 --y -0.2731
```

`run.sh` prints a checklist and asks you to type `yes`. What happens: the bridge sets each servo's goal to
its present position, then enables torque (the arm holds its pose); after `start_delay` the loop pre-flights
and starts. The first run: add `--cycles 1` and watch every hop.

* **Stop:** Ctrl+C. The loop exits, the launch shuts down, and the arm **holds its last pose with torque on**.
  A trajectory already handed to the bridge finishes first. Power switch for an emergency.
* **Do not** start a second `ros2 launch lerobot_moveit …` or a second bridge — two bridges on one serial
  port crash both. `run.sh` refuses if it sees one.
* **Do not** press *Plan and Execute* in RViz while the loop runs; it sends its own trajectory to the same arm.
* If the arm ends a hop inside the safety pad the next hop is refused and the loop stops — that is the
  intended failure mode, not a crash. Re-pose and restart.
* Torque stays on after the stack exits. To move the arm by hand again, disable torque
  (README of `lerobot_hardware`, Calibration step 2) or power-cycle.

## 6. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `no /joint_states for the arm joints` | No bridge is publishing: the stack isn't running, it crashed, or two stacks fought over the port. Stop everything, run one `./run.sh`. |
| Bridge died: `sync_read failed: There is no status packet` / `Bad file descriptor` | Serial link lost (USB dropped, a second process opened the port, servo bus unpowered). Replug/power, check `lsof /dev/ttyACM0`, relaunch. |
| `ModuleNotFoundError: rclpy._rclpy_pybind11` | conda python first on `PATH`. Use `run.sh`, or `export PATH=/usr/bin:$PATH`. |
| `ClampExceeded` in the bridge log | A commanded joint is outside that servo's calibrated range (see `lerobot_hardware/README.md`). |
| Cylinder not visible in RViz | It exists only while `move_group` holds it. `ik_loop.py` adds it on start; or `ros2 run lerobot_ik_demo add_obstacle_scene.py --object-xy X Y`. Check MotionPlanning ▸ Scene Geometry is ticked. |
| `Invalid Trajectory: start point deviates from current robot state more than 0.01 at joint …` | The arm drifted (gravity sag) between planning and execution. Nothing moved. The loop now re-plans and retries up to twice when the arm hasn't moved; if it still fails it stops — restart it. |
| Arm sags / ends 2–4 cm low | Expected (gravity, steady-state servo error). See ARCHITECTURE §7. |
| Wrist roll reads far from 0 | Not used by the planner; only matters for the jaw's collision shape, which follows the reading. |

## 7. What has and hasn't been verified

* **Mock hardware:** `run.sh --execute --cycles 1` runs start-to-finish, tip error < 1 mm; plan-only default,
  guards (`real` while another stack runs, `--live` on mock, bad args) tested.
* **Real arm** (dev machine, arm at the measured cylinder position): the loop ran many consecutive cycles at
  0.2 rad/s with no bridge errors (8 in the longest run); tip error 11–47 mm per hop. Two runs stopped
  themselves, by design: one when the arm ended inside the safety pad, one (cycle 9) when MoveIt rejected a
  trajectory because joint 2 had drifted > 0.01 rad during planning — nothing moved. The re-plan/retry for
  that case was added afterwards and has been run on mock hardware only (its retry branch has not yet
  triggered on the real arm). The pan-only `--escape` was used successfully. There is **no automated
  test against hardware**, and no measurement of the *physical* clearance to the real cylinder.
* **Not tested:** Gazebo with the new scripts; other arms/calibrations; long unattended runs.
