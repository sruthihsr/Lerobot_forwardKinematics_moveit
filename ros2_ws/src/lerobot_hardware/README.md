# lerobot_hardware

Real-hardware bridge for the SO-101 Feetech-servo follower arm. It talks
directly to the servos over serial (no `ros2_control` involved) and exposes
two `FollowJointTrajectory` action servers — `arm_controller` and
`gripper_controller` — under the exact names `lerobot_moveit`'s
`moveit_controllers.yaml` expects, so MoveIt can plan and execute against the
physical robot with no other changes to the MoveIt config.

## Contents

| File | Purpose |
|---|---|
| `scripts/feetech_bus.py` | Low-level STS3215 serial driver (register map, raw-tick ↔ radian conversion, calibration). |
| `scripts/so101_hardware_bridge.py` | ROS 2 node: publishes `/joint_states`, runs the two `FollowJointTrajectory` action servers. |
| `scripts/feetech_probe.py` | Standalone diagnostic — pings servos, dumps live EEPROM limits, read-only. |
| `config/so101_calibration.yaml` | Per-servo raw-tick zero reference (see **Calibration** below). |

## Running

```bash
ros2 run lerobot_hardware so101_hardware_bridge.py --ros-args \
  -p port:=/dev/ttyACM0 -p dry_run:=True
```
Normally you don't run this directly — `lerobot_moveit`'s
`so101_moveit.launch.py` launches it for you when `use_real_hardware:=True`.

**Params:**
- `port` (string, default `/dev/ttyACM0`)
- `dry_run` (bool, default `True`) — `True`: connects read-only, never writes
  to the servos (no torque, no `Goal_Position`); trajectory execution just
  logs what it *would* send. Must be explicitly set to `False` to move the
  real arm.
- `acceleration` / `goal_velocity` (ints, default `20` / `150`) — conservative
  raw-unit speed/ramp caps. Deliberately slow; see **Known limitations**.
- `joint_state_rate` (Hz, default `30.0`)

**⚠️ Only ever run one instance against a given serial port at a time.**
Two bridges (e.g. two full `so101_moveit.launch.py` launches) fighting over
the same servos causes unpredictable motion and command failures — always
check `ps aux | grep hardware_bridge` before starting a new launch.

## Calibration

Each STS3215 servo reports `Present_Position` as a raw tick in `[0, 4095]`
relative to *its own* internal zero — which has no relationship to the URDF's
zero pose. `so101_calibration.yaml` maps each servo's raw ticks to the
radian values MoveIt/the URDF expect, by recording what raw tick each servo
reports when the arm is physically posed at the URDF's zero ("home") pose.

Without this file, `feetech_bus.py` falls back to using each servo's live
EEPROM min/max-limit *midpoint* as zero — which does not generally match the
URDF's zero and will likely make MoveIt immediately reject planning (`Start
state is outside bounds`) because the reported joint values fall outside the
URDF's declared limits.

**To (re)calibrate** (e.g. after re-mounting a servo, or building a new
arm):

1. Stop any running launch so nothing else is writing to the servos.
2. Disable torque on all 6 servos so the arm can be moved by hand:
   ```python
   from feetech_bus import FeetechBus, ADDR_TORQUE_ENABLE
   bus = FeetechBus("/dev/ttyACM0")
   bus.port_handler.openPort(); bus.port_handler.setBaudRate(1_000_000)
   for sid in bus.servo_ids:
       bus.packet_handler.ping(bus.port_handler, sid)
       bus._write(sid, ADDR_TORQUE_ENABLE, 0)
   bus.close()
   ```
3. Physically pose the arm at the URDF's zero/home position (all 5 arm
   joints + gripper at 0 rad).
4. Read each servo's raw `Present_Position` at that exact pose (see
   `feetech_bus.read_positions_rad`'s raw-read pattern, or use
   `GroupSyncRead` on `ADDR_PRESENT_POSITION` directly) and write the 6
   values into `config/so101_calibration.yaml` under `home_raw:`, keyed by
   servo id (`1`-`6`).
5. Relaunch (or just restart the hardware bridge). `/joint_states` should
   now read ~0 rad on every joint at that pose.

**Important — this changes what "0 rad" means for every joint**, so any
`group_state` poses already recorded in `so101.srdf` (see
`lerobot_moveit/README.md`) will need to be re-checked/re-recorded after a
recalibration; they were captured relative to the *previous* calibration.

### Per-joint raw headroom is not symmetric

The URDF declares each joint's limits (e.g. `±1.745` rad), but a servo's
*physical* single-turn encoder only spans `[0, 4095]` raw ticks, and the
calibrated zero from the procedure above can land anywhere in that range —
not necessarily centered. That means a joint can have much less real
headroom in one direction than the URDF limit alone would suggest, especially
if the zero pose happens to sit close to the encoder's `0`/`4095` wrap
boundary on one side. `write_positions_rad()` raises `ClampExceeded` rather
than silently doing something unpredictable when a commanded position would
need more than ~13 raw ticks (~0.02 rad) of clamping to fit the servo's live
limits — treat that exception as "this pose isn't actually reachable at the
current calibration," not as a bug to route around.

## Known limitations

- **Execution reports success once the last trajectory waypoint is *sent*,
  not once the servo has physically arrived.** The conservative
  `goal_velocity`/`acceleration` defaults mean a large move can still be
  travelling for several seconds after `FollowJointTrajectory` reports
  `SUCCESS`. Anything reading `/joint_states` right after execution should
  poll for a few seconds rather than sampling once.
- **Steady-state tracking error under gravity load.** For larger joint
  excursions (especially with the arm extended away from a well-balanced
  pose), the arm has been observed to settle up to ~0.1-0.15 rad away from
  the commanded target and not converge further — a real limitation of this
  actuator/load combination, not a software bug.

## Troubleshooting

- **`ros2: command not found` in a script/non-interactive shell**: most
  `~/.bashrc`s exit early for non-interactive shells, before the ROS
  `setup.bash` sourcing — source `/opt/ros/jazzy/setup.bash` and this
  workspace's `install/setup.bash` explicitly in whatever runs the command.
- **`ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`**: a
  Python other than the system interpreter ROS was built against (e.g. a
  Conda `base` env auto-activated ahead of it on `PATH`) is running the
  script. Put `/usr/bin` first on `PATH` (or otherwise ensure `python3`
  resolves to the system interpreter) before running.
