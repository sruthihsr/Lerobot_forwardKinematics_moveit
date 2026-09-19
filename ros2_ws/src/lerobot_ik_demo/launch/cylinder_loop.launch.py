"""
One launch file for the whole demo: MoveIt (+ mock hardware or the real Feetech bridge),
then the IK loop around the cylinder.

    ros2 launch lerobot_ik_demo cylinder_loop.launch.py                         # mock arm, plan-only
    ros2 launch lerobot_ik_demo cylinder_loop.launch.py execute:=True           # mock arm, loops
    ros2 launch lerobot_ik_demo cylinder_loop.launch.py hardware:=real          # real arm, read-only dry run
    ros2 launch lerobot_ik_demo cylinder_loop.launch.py hardware:=real dry_run:=False execute:=True

Normally started through ../run.sh, which adds the pre-flight checks and the confirmation
prompt for live hardware. See README.md / docs/.
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# rclpy is ABI-locked to the system Python 3.12; a conda "base" python earlier on PATH breaks it.
SYSTEM_PYTHON = "/usr/bin/python3"

ARGS = [
    ("hardware", "mock", "'mock' (ros2_control mock hardware, no arm needed) or 'real' (Feetech bridge on the serial port)"),
    ("dry_run", "True", "real only: True = bridge connects read-only and only logs servo writes; False = MOVES THE ARM"),
    ("port", "/dev/ttyACM0", "real only: serial port of the arm"),
    ("execute", "False", "True = actually run the loop (plan-only pre-flight otherwise)"),
    ("object_x", "0.0708", "cylinder centre x in the base frame, m (+x is the arm's left) -- MEASURE THIS"),
    ("object_y", "-0.2731", "cylinder centre y in the base frame, m (-y is forward) -- MEASURE THIS"),
    ("clearance", "0.07", "m the gripper stays above the padded cylinder top when crossing"),
    ("speed", "0.2", "peak joint speed, rad/s"),
    ("cycles", "0", "loops to run; 0 = until Ctrl+C"),
    ("pause", "2.0", "seconds to hold at each point"),
    ("points", "", "optional YAML file of loop points (default: config/loop_points.yaml)"),
    ("start_delay", "10.0", "seconds to wait for move_group before starting the loop"),
]


def _setup(context):
    cfg = {name: LaunchConfiguration(name).perform(context) for name, _, _ in ARGS}
    real = cfg["hardware"] == "real"
    execute = cfg["execute"].lower() in ("true", "1", "yes")

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("lerobot_moveit"), "launch", "so101_moveit.launch.py")),
        launch_arguments={
            "is_sim": "False",
            "use_real_hardware": "True" if real else "False",
            "dry_run": cfg["dry_run"],
            "hardware_port": cfg["port"],
        }.items(),
    )

    script = os.path.join(get_package_prefix("lerobot_ik_demo"), "lib", "lerobot_ik_demo", "ik_loop.py")
    cmd = [SYSTEM_PYTHON, script,
           "--object-xy", cfg["object_x"], cfg["object_y"],
           "--clearance", cfg["clearance"], "--speed", cfg["speed"],
           "--cycles", cfg["cycles"], "--pause", cfg["pause"], "--escape"]
    if cfg["points"]:
        cmd += ["--points", cfg["points"]]
    if execute:
        cmd.append("--execute")
    loop = ExecuteProcess(cmd=cmd, name="ik_loop", output="screen")

    actions = [moveit, TimerAction(period=float(cfg["start_delay"]), actions=[loop])]
    if execute:
        # A loop that exits (error, Ctrl+C, --cycles reached) takes the stack down with it. The real arm keeps
        # torque on and holds its last pose. Plan-only leaves the stack up so the plan/cylinder can be inspected in RViz.
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=loop, on_exit=[EmitEvent(event=Shutdown(reason="ik_loop exited"))])))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc) for name, default, desc in ARGS]
        + [OpaqueFunction(function=_setup)])
