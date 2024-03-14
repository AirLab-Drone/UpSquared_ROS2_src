from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='flight_control',
            # namespace='turtlesim1',
            executable='cpp_executable',
            # name='sim'
            output='screen'
        ),
        Node(
            package='flight_control',
            # namespace='turtlesim2',
            executable='py_node.py',
            # name='sim'
            output='screen'
        )
    ])