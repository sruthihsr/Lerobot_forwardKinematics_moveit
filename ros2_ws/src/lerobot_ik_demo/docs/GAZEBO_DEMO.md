# Gazebo obstacle demo (original demo, kept for reference)

This is the package's first demo: a static cylinder in **Gazebo** plus the matching object in MoveIt's
planning scene, so OMPL routes around it. It is independent of the IK loop described in the main
[README](../README.md) (which plans its own paths and needs no Gazebo).

> **Status:** these files are unchanged from the original demo. The newer scripts (`ik_loop.py`,
> `click_to_move.py`, `over_the_top_demo.py`) were **not** tested against Gazebo — they were tested on
> mock hardware and on the real arm. Treat Gazebo + the new scripts as unverified.

Everything needed lives in this package — `lerobot_description`, `lerobot_moveit`, and
`lerobot_controller` are used as-is, not modified.

Gazebo and MoveIt don't share geometry automatically, so the object is defined in two places, kept in
sync by hand: `worlds/obstacle_world.sdf` (what Gazebo renders/simulates, at the object's *real* size)
and `scripts/add_obstacle_scene.py` (what MoveIt plans against, at a *padded* size). Both are expressed
in the world frame, which coincides with the `base` link frame because `so101_gazebo_obstacle.launch.py`
spawns the robot with no pose offset — if you move the object or give the robot a spawn offset, update
both files.

## Running

```bash
# Terminal 1: Gazebo, with the obstacle world
ros2 launch lerobot_ik_demo so101_gazebo_obstacle.launch.py

# Terminal 2: MoveIt against that Gazebo instance (unmodified lerobot_moveit, is_sim:=True),
# auto-adding the same cylinder to the planning scene after move_group comes up
ros2 launch lerobot_ik_demo so101_moveit_obstacle.launch.py
```

In RViz, drag the interactive marker to a goal pose on the far side of the cylinder and hit **Plan** —
the planned path arcs around it instead of going straight through. **Execute** drives the simulated arm
through that same collision-free path in Gazebo.

`scripts/loop_obstacle_demo.py` sweeps the base joint left and right at the cylinder's height in a loop
(`ros2 run lerobot_ik_demo loop_obstacle_demo.py [--cycles N] [--pause S]`).

## Files

| Path | Purpose |
|---|---|
| `worlds/obstacle_world.sdf` | Gazebo world: ground plane, sun, and the static `obstacle_cylinder` model. |
| `launch/so101_gazebo_obstacle.launch.py` | Gazebo bring-up (self-contained copy of `lerobot_description`'s, pointed at the obstacle world). |
| `launch/so101_moveit_obstacle.launch.py` | Includes `lerobot_moveit`'s launch file unmodified + schedules the obstacle-scene node. |
| `scripts/add_obstacle_scene.py` | Publishes the `CollisionObject` to MoveIt's planning scene (`--object-xy X Y` overrides the position). |
| `scripts/loop_obstacle_demo.py` | Loops a base-joint sweep so each leg visibly detours around the cylinder. |

## Sim to real

The obstacle models a real cylinder (18 cm tall, 14 cm diameter). Gazebo shows it at its true size;
MoveIt plans against a **padded** copy (`BUFFER_MARGIN` = 3 cm on the radius and top; the base stays flush
with the table), because planning against exact dimensions leaves no margin for calibration error, tape
measurement, or servo backlash on the real arm. For the real-arm procedure see
[HARDWARE_BRINGUP.md](HARDWARE_BRINGUP.md).
