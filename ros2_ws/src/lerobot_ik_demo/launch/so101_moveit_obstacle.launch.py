"""
Includes lerobot_moveit's so101_moveit.launch.py unmodified (is_sim:=True),
then adds the 'obstacle_box' CollisionObject to the planning scene so OMPL
plans around the box that so101_gazebo_obstacle.launch.py spawns in Gazebo.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("lerobot_moveit"),
                "launch",
                "so101_moveit.launch.py",
            ])
        ]),
        launch_arguments={"is_sim": "True"}.items(),
    )

    # move_group needs a few seconds to come up and advertise
    # /apply_planning_scene before this can succeed.
    add_obstacle_node = TimerAction(
        period=5.0,
        actions=[Node(
            package="lerobot_ik_demo",
            executable="add_obstacle_scene.py",
            output="screen",
        )],
    )

    return LaunchDescription([
        moveit_launch,
        add_obstacle_node,
    ])
