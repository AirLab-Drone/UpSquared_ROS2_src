import os
import subprocess
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    try:
        aruco_markers_file = os.path.join(
            get_package_share_directory("flight_control"),
            "config",
            "aruco_markers.yaml",
        )
    except PackageNotFoundError:
        print("Cannot find the package 'flight_control'.")
        return

    return LaunchDescription(
        [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                output="screen",
                arguments=[f"/drone_camera@sensor_msgs/msg/Image@gz.msgs.Image"],
            ),
            Node(
                package="flight_control",
                executable="aruco_detector_node.py",
                output="screen",
                name="aruco_detector_node",
                parameters=[{"config_file": aruco_markers_file}, {"simulation": True}],
            ),
        ]
        
    )
