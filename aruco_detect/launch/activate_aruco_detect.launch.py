from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 定義引數，用來在命令行中動態設置 debug 模式，預設為 "False"
    debug_mode = DeclareLaunchArgument(
        "debug",
        default_value="False",  # 預設值必須是字串
        description="Enable debug mode",
    )

    show_image = DeclareLaunchArgument(
        "show_image",
        default_value="False",  # 預設值必須是字串
        description="Enable camera image display",
    )

    return LaunchDescription(
        [
            debug_mode,
            show_image,

            Node(
                package="aruco_detect",
                executable="camera_node.py",
                output="screen",
                parameters=[
                    {"exposure": 157},
                    {"debug": LaunchConfiguration("debug")},  # 將debug傳入節點參數
                ],
            ),
            Node(
                package="aruco_detect",
                executable="detect_aruco_node.py",
                output="screen",
                parameters=[
                    {"show_image": LaunchConfiguration("show_image")},
                    {"debug": LaunchConfiguration("debug")},  # 將debug傳入節點參數
                ],
            ),
            Node(
                package="aruco_detect",
                executable="kalman_node.py",
                output="screen",
                parameters=[
                    {"debug": LaunchConfiguration("debug")},  # 將debug傳入節點參數
                ],
            ),
            Node(
                package="aruco_detect",
                executable="closest_marker_publisher.py",
                output="screen",
                parameters=[
                    {"debug": LaunchConfiguration("debug")},  # 將debug傳入節點參數
                ],
            ),
        ]
    )
