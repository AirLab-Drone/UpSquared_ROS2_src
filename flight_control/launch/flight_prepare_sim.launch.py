from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python import get_package_share_directory


def changeMavrosPublishRate(msg_id):
    return TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "mavros",
                    "mav",
                    "sys",
                    "message-interval",
                    "--id",
                    f"{msg_id}",
                    "--rate",
                    "100",
                ],
                output="screen",
            )
        ],
    )


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("flight_control"), "launch/apm.launch"
                    )
                ),
                launch_arguments={
                    "fcu_url": "udp://:14550@",
                }.items(),
            ),
            ExecuteProcess(
                cmd=[
                    "gz",
                    "sim",
                    "-v4",
                    "-r",
                    "iris_aruco.sdf",
                    "--render-engine",
                    "ogre",
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "sim_vehicle.py",
                    "-v",
                    "ArduCopter",
                    "-f",
                    "gazebo-iris",
                    "--model",
                    "JSON",
                    "--out",
                    "127.0.0.1:14551",
                    "--map",
                    "--console",
                    "--mavproxy-args=--streamrate=-1",
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                output="screen",
                arguments=[f"/lidar1@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                output="screen",
                arguments=[f"/lidar2@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"],
            ),
            changeMavrosPublishRate(24),
            changeMavrosPublishRate(33),
            changeMavrosPublishRate(132),
        ]
    )
