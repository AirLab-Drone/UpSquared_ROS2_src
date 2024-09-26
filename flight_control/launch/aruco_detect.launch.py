import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    aruco_markers_file = os.path.join(
        get_package_share_directory("flight_control"),
        "config",
        "aruco_markers.yaml",
    )
    return LaunchDescription(
        [
            Node(
                package="flight_control",
                executable="aruco_detector_node.py",
                output="screen",
                name="aruco_detector_node",
                parameters=[{"config_file": aruco_markers_file}],
            ),
        ]
    )
