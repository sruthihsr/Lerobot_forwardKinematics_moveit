# lerobot_moveit

MoveIt 2 configuration and launch for the SO-101 arm — planning (`move_group`),
RViz visualization/control, and (with real hardware) the servo bridge from
`lerobot_hardware`.

## Launching

```bash
ros2 launch lerobot_moveit so101_moveit.launch.py \
  use_real_hardware:=True dry_run:=False hardware_port:=/dev/ttyACM0
```

**Args:**
- `is_sim` (default `False`) — `True` if Gazebo (launched separately) is
  providing `robot_state_publisher`/`ros2_control`; leave `False` otherwise.
- `use_real_hardware` (default `False`) — `True`: skip the mock/Gazebo
  `ros2_control` stack and launch `lerobot_hardware`'s servo bridge instead,
  to plan/execute against the physical arm.
- `dry_run` (default `True`) — only used when `use_real_hardware:=True`.
  Must be explicitly set to `False` to actually move the real arm; see
  `lerobot_hardware/README.md`.
- `hardware_port` (default `/dev/ttyACM0`) — serial port for the real arm.

**⚠️ Only run one instance of this launch (i.e. one `hardware_bridge`) against
the arm at a time** — see the warning in `lerobot_hardware/README.md`.

In RViz, use the **MotionPlanning** panel's **Planning** tab: drag the
interactive markers (or pick a named goal state from the dropdown) and hit
**Plan**, then **Execute**.

## Named poses

`config/so101.srdf` defines named `group_state`s MoveIt can plan straight to
(via RViz's goal-state dropdown, or `move_group.setNamedTarget("...")` /
the `pose_loop.py` script below). Current poses:

| Name | Description |
|---|---|
| `home` | Calibration reference pose (see `lerobot_hardware/README.md`). |
| `pose_1`..`pose_4` | Small variations around `home`, each with a different gripper opening. |
| `vertical` | Elbow (joint 3) swung to `+1.40` rad. |
| `vertical_down` | Elbow swung to `-0.85` rad (kept short of `-1.40` for table clearance). |

**To add a pose:** either use MoveIt Setup Assistant's "Robot Poses" tab, or
edit `config/so101.srdf` directly — add one `<group_state name="..."
group="arm">` block (joints 1-5) and one `group="gripper"` block (joint 6),
e.g.:

```xml
<group_state name="my_pose" group="arm">
    <joint name="1" value="0.20" />
    <joint name="2" value="-0.20" />
    <joint name="3" value="0.40" />
    <joint name="4" value="-0.15" />
    <joint name="5" value="0.10" />
</group_state>
<group_state name="my_pose" group="gripper">
    <joint name="6" value="0.20" />
</group_state>
```

Read live joint values to use as targets with:
```bash
ros2 topic echo /joint_states --once
```

**Before committing a new pose that pushes any joint far from `home`**,
double-check it against each servo's live raw-tick headroom (see
`lerobot_hardware/README.md` → *Per-joint raw headroom is not symmetric*) —
the URDF's declared limits alone don't guarantee a value is actually
reachable at the current calibration, and don't account for physical
obstructions (e.g. the arm's mounting table).

## `scripts/pose_loop.py`

Cycles the arm through every named pose in `so101.srdf`, with no user input,
verifying each move by polling `/joint_states` against the commanded target
afterward.

```bash
ros2 run lerobot_moveit pose_loop.py                 # 5 cycles (default)
ros2 run lerobot_moveit pose_loop.py --cycles 0        # loop forever
ros2 run lerobot_moveit pose_loop.py --cycles 1 --pause 3.0
```
(Or run the file directly with `python3` — it doesn't depend on being
installed as long as the workspace's `install/setup.bash` is sourced.)

Ctrl+C stops it cleanly. Requires the launch above already running
(reads the SRDF live and talks to `move_group`'s `/move_action` server — it
doesn't start MoveIt itself).

**Verify tolerance** is `0.05` rad (~3°) — calibrated to this arm's observed
real-world tracking accuracy under load (see `lerobot_hardware/README.md` →
*Known limitations*), not an arbitrary number. A `FAIL` past that means a
genuinely large discrepancy, not settling noise.
