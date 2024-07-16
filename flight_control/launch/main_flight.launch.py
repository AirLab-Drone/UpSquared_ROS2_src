import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def launch_setup(context, *args, **kwargs):
    parm_simulation = LaunchConfiguration('simulation').perform(context)
    
    aruco_markers_file = os.path.join(
        get_package_share_directory('flight_control'),
        'config',
        'aruco_markers.yaml',
    )
    
    if parm_simulation == True:
        # 在这里添加你想在 simulation 模式下执行的操作
        print("Simulation mode is enabled")
    else:
        # 在这里添加你想在非 simulation 模式下执行的操作
        print("Simulation mode is disabled")

    return [
        Node(
            package='flight_control',
            executable='main_flight_node.py',
            output='screen',
            name='main_flight_node',
            parameters=[
                {'config_file': aruco_markers_file},
                {'simulation': parm_simulation},
            ],
        ),
    ]

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'simulation',
                default_value="False",
                description='simulation mode',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
