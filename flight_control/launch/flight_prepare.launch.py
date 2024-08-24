from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python import get_package_share_directory


def changeMavrosPublishRate():
    return TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "mavros",
                    "mav",
                    "sys",
                    "rate",
                    "--all",
                    "100",
                ],
                output="screen",
            )
        ],
    )
# def changeMavrosPublishRate(msg_id):
#     return TimerAction(
#         period=10.0,
#         actions=[
#             ExecuteProcess(
#                 cmd=[
#                     "ros2",
#                     "run",
#                     "mavros",
#                     "mav",
#                     "sys",
#                     "message-interval",
#                     "--id",
#                     f"{msg_id}",
#                     "--rate",
#                     "100",
#                 ],
#                 output="screen",
#             )
#         ],
#     )


def generate_launch_description():
    return LaunchDescription(
        [
            # Node(
            #     package="flight_control",
            #     executable="aruco_detector_node.py",
            #     output="screen",
            # ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("mavros"), "launch/apm.launch"
                    )
                ),
                launch_arguments={
                    "fcu_url": "/dev/ttyACM0",
                }.items(),
            ),
            # changeMavrosPublishRate(24),
            # changeMavrosPublishRate(33),
            # changeMavrosPublishRate(132),
            changeMavrosPublishRate(),
        ]
    )
