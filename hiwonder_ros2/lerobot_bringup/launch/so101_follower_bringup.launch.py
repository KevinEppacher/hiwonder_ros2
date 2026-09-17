import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # ---------------------- Arguments ------------------------------#
    
    port_argument = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyACM0",
        description="Serial port of the HiWonder servo bus",
    )
        
    port = LaunchConfiguration("port")
    
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Flag to enable joint_state_publisher_gui",
    )

    gui = LaunchConfiguration("gui")
    
    # ---------------------- Paths ------------------------------#

    xacro_file = os.path.join(
        get_package_share_directory("so101_follower_description"),
        "urdf",
        "so101_follower.urdf.xacro",
    )

    calibration_file = os.path.join(
        get_package_share_directory("so101_follower_description"),
        "config",
        "so101_follower_calibration.yaml",
    )

    controllers_file = os.path.join(
        get_package_share_directory("lerobot_bringup"),
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
    
    rviz_config_file = os.path.join(
        get_package_share_directory("so101_follower_description"),
        "rviz",
        "rviz.rviz"
    )
    
    # ---------------------- Nodes ------------------------------#

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
    
    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )
    
    # ---------------------- Launch Description ------------------------#
    
    ld = LaunchDescription()
    ld.add_action(port_argument)
    ld.add_action(gui_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(controller_manager)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(position_controller)
    ld.add_action(rviz)
    return ld