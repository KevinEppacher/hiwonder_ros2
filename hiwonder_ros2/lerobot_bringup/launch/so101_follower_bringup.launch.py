import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")

    description_package = get_package_share_directory(
        "so101_follower_description"
    )

    bringup_package = get_package_share_directory(
        "lerobot_bringup"
    )

    xacro_file = os.path.join(
        description_package,
        "urdf",
        "so101_follower.urdf.xacro",
    )

    calibration_file = os.path.join(
        description_package,
        "config",
        "so101_follower_calibration.yaml",
    )

    controllers_file = os.path.join(
        bringup_package,
        "config",
        "so101_follower_controllers.yaml",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " port:=",
                port,
                " calibration_file:=",
                calibration_file,
            ]
        ),
        value_type=str,
    )

    port_argument = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyACM0",
        description="Serial port of the HiWonder servo bus",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
            },
            controllers_file,
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            port_argument,
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster,
        ]
    )