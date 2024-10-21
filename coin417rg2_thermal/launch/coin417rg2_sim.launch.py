import launch
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

import os
import platform


def generate_launch_description():
    return launch.LaunchDescription(
        [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                output="screen",
                arguments=[f"/thermal_camera@sensor_msgs/msg/Image@gz.msgs.Image"],
            ),
            Node(
                package="coin417rg2_thermal",
                executable="simulator_thermal.py",
                name="coin417rg2_thermal",
            ),
            Node(
                package="coin417rg2_thermal",
                executable="thermal_frame_to_drone_frame.py",
                name="thermal_frame_to_drone_frame",
            ),
        ]
    )


# if __name__ == '__main__':
#     generate_launch_description()
