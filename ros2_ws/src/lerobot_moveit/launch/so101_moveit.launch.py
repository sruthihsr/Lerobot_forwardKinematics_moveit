import os
import tempfile
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Declare arguments
    is_sim_arg = DeclareLaunchArgument(
        name="is_sim",
        default_value="False",
        description=(
            "True: assume Gazebo (launched separately via "
            "lerobot_description/so101_gazebo.launch.py) is providing "
            "robot_state_publisher and ros2_control. "
            "False (default): bring up robot_state_publisher and "
            "ros2_control_node here with mock hardware, so MoveGroup goals "
            "can be planned and executed without Gazebo."
        )
    )

    is_sim = LaunchConfiguration("is_sim")

    use_real_hardware_arg = DeclareLaunchArgument(
        name="use_real_hardware",
        default_value="False",
        description=(
            "True: skip the mock/Gazebo ros2_control controller stack and "
            "instead launch the Feetech hardware bridge "
            "(lerobot_hardware/so101_hardware_bridge.py) to talk to the "
            "real SO101 arm directly over serial."
        )
    )
    dry_run_arg = DeclareLaunchArgument(
        name="dry_run",
        default_value="True",
        description=(
            "Only used when use_real_hardware is True. True (default): the "
            "hardware bridge connects read-only and only logs intended "
            "servo writes instead of sending them. Must be explicitly set "
            "to False to actually move the real arm."
        )
    )
    hardware_port_arg = DeclareLaunchArgument(
        name="hardware_port",
        default_value="/dev/ttyACM0",
        description="Serial port for the real SO101 arm (only used when use_real_hardware is True)."
    )

    use_real_hardware = LaunchConfiguration("use_real_hardware")
    dry_run = LaunchConfiguration("dry_run")
    hardware_port = LaunchConfiguration("hardware_port")

    # Brings up robot_state_publisher + ros2_control_node + controller
    # spawners (arm_controller, gripper_controller, joint_state_broadcaster)
    # against mock/Gazebo hardware. Skipped entirely when use_real_hardware
    # is True (the hardware bridge below takes over execution instead).
    # When is_sim is True these nodes are skipped (Gazebo provides them
    # instead); see lerobot_controller/launch/so101_controller.launch.py.
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("lerobot_controller"),
                "launch",
                "so101_controller.launch.py"
            ])
        ]),
        launch_arguments={"is_sim": is_sim}.items(),
        condition=UnlessCondition(use_real_hardware)
    )

    # Talks directly to the real Feetech servos and exposes the same
    # FollowJointTrajectory action names moveit_controllers.yaml expects, so
    # move_group can execute against it with no ros2_control involved.
    hardware_bridge_node = Node(
        package="lerobot_hardware",
        executable="so101_hardware_bridge.py",
        output="screen",
        parameters=[{
            "port": hardware_port,
            "dry_run": dry_run,
        }],
        condition=IfCondition(use_real_hardware)
    )

    # Get URDF via xacro using Command (let ROS2 handle it)
    robot_description_content = Command([
        PathJoinSubstitution([FindPackageShare("ros2"), "libexec", "ros2", "xacro"]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("lerobot_description"),
            "urdf",
            "so101.urdf.xacro"
        ]),
    ])

    # For MoveIt, we need the URDF as a string at build time
    # So we process xacro separately
    lerobot_description_dir = get_package_share_directory("lerobot_description")
    xacro_file = os.path.join(lerobot_description_dir, "urdf", "so101.urdf.xacro")
    
    # Use os.system to process xacro and save to temp file
    temp_urdf = "/tmp/so101_moveit.urdf"
    os.system(f"xacro {xacro_file} > {temp_urdf}")

    # MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("so101", package_name="lerobot_moveit")
        .robot_description(file_path=temp_urdf)
        .robot_description_semantic(file_path="config/so101.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # controller_launch (so101_controller.launch.py) already brings up
    # robot_state_publisher for the mock/Gazebo path, but that whole include
    # is skipped when use_real_hardware is True, so publish TF here instead
    # -- otherwise RViz has no transforms and can't render the robot.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
        condition=IfCondition(use_real_hardware)
    )

    # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_sim},
            {"publish_robot_description_semantic": True}
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )

    # RViz node
    rviz_config_path = os.path.join(
        get_package_share_directory("lerobot_moveit"),
        "config",
        "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
        ]
    )

    return LaunchDescription([
        is_sim_arg,
        use_real_hardware_arg,
        dry_run_arg,
        hardware_port_arg,
        controller_launch,
        hardware_bridge_node,
        robot_state_publisher_node,
        move_group_node,
        rviz_node
    ])