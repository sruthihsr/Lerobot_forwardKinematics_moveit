"""
Standalone copy of lerobot_description/launch/so101_gazebo.launch.py that
loads worlds/obstacle_world.sdf instead of gz-sim's built-in empty world --
kept as a separate file (rather than a 'world' arg on the original) so this
demo package doesn't need to modify the base bring-up packages.
"""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lerobot_description = get_package_share_directory("lerobot_description")
    lerobot_ik_demo = get_package_share_directory("lerobot_ik_demo")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(lerobot_description, "urdf", "so101.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )

    # lerobot_description's meshes are referenced as
    # model://lerobot_description/meshes/... in the URDF, so this needs to
    # point at lerobot_description's *parent* share dir, same as
    # so101_gazebo.launch.py.
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(lerobot_description).parent.resolve())],
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_sim:=True",
        ]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}],
    )

    world_path = os.path.join(lerobot_ik_demo, "worlds", "obstacle_world.sdf")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments=[
            ("gz_args", [f" -v 4 -r {world_path} "])
        ],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "so101"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])
