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

    try:
        result = subprocess.run(["ign", "topic", "-l"], capture_output=True, text=True)
    except FileNotFoundError:
        print("Cannot find the command 'ign'.")
        return

    image_topics = [line for line in result.stdout.split("\n") if "image" in line]

    gz_bridge_nodes = []
    for topic in image_topics:
        gz_bridge_nodes.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[f"{topic}@sensor_msgs/msg/Image@gz.msgs.Image"],
            )
        )

    return LaunchDescription(
        [
            Node(
                package="flight_control",
                executable="aruco_detector_node.py",
                output="screen",
                name="aruco_detector_node",
                parameters=[{"config_file": aruco_markers_file}, {"simulation": True}],
            ),
        ]
        + gz_bridge_nodes
    )
