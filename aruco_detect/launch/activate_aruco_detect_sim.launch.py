from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # 定義引數，用來在命令行中動態設置 debug 模式，預設為 "False"
    debug_mode = DeclareLaunchArgument(
        "debug",
        default_value="False",  # 預設值必須是字串
        description="Enable debug mode",
    )

    show_image = DeclareLaunchArgument(
        "show_image",
        default_value="True",  # 預設值必須是字串
        description="Enable camera image display",
    )
    # yaml檔案的路徑
    aruco_markers_file = os.path.join(
        get_package_share_directory("aruco_detect"),
        "config",
        "platform_markers.yaml",
    )
    camera_config_file = os.path.join(
        get_package_share_directory("aruco_detect"),
        "config",
        "simulation_camera_parameter.yaml",
    )

    return LaunchDescription(
        [
            debug_mode,
            show_image,
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                output="screen",
                arguments=[f"/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image"],
            ),
            Node(
                package="aruco_detect",
                executable="aruco_detector_node.py",
                output="screen",
                parameters=[
                    {"simulation": False},
                    {"aruco_marker_config_file": aruco_markers_file},
                    {"camera_config_file": camera_config_file},
                    {"show_image": LaunchConfiguration("show_image")},
                ],
            ),
        ]
    )
