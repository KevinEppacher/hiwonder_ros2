import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # ---------------------- Paths ------------------------------#
    
    follower_camera_config_path = os.path.join(
        get_package_share_directory("lerobot_bringup"),
        "config",
        "follower_wrist_camera.yaml",
    )

    # ---------------------- Arguments ------------------------------#

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="follower",
        description="Namespace for the robot",
    )
    
    video_device_arg = DeclareLaunchArgument(
        "video_device",
        default_value="/dev/video0",
        description=(
            "Video device for the follower wrist camera. "
            "List devices with 'v4l2-ctl --list-devices'."
        ),
    )

    namespace = LaunchConfiguration("namespace")
    video_device = LaunchConfiguration("video_device")

    # ---------------------- Nodes ------------------------------#

    cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="follower_wrist_camera",
        namespace=namespace,
        output="screen",
        parameters=[
            follower_camera_config_path,
            {"video_device": video_device}
        ],
    )

    # ---------------------- Launch Description ------------------------------#

    ld = LaunchDescription()
    ld.add_action(namespace_arg)
    ld.add_action(video_device_arg)
    ld.add_action(cam_node)
    return ld
