import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def launch_setup(context, *args, **kwargs):
    parm_simulation_str = LaunchConfiguration('simulation').perform(context)
    parm_simulation = parm_simulation_str.lower() == 'true'  # 将字符串解析为布尔值

    aruco_markers_file = os.path.join(
        get_package_share_directory('aruco_detect'),
        'config',
        'aruco_markers.yaml',
    )
    base_position_file = ""
    
    if parm_simulation:
        # 在这里添加你想在 simulation 模式下执行的操作
        print("Simulation mode is enabled")
        base_position_file = os.path.join(
            get_package_share_directory('flight_control'),
            'config',
            'base_position_sim.yaml',
        )
    else:
        # 在这里添加你想在非 simulation 模式下执行的操作
        print("Simulation mode is disabled")
        base_position_file = os.path.join(
            get_package_share_directory('flight_control'),
            'config',
            'base_position.yaml',
        )

    return [
        Node(
            package='flight_control',
            executable='main_flight_node.py',
            output='screen',
            name='main_flight_node',
            parameters=[
                {'config_file': aruco_markers_file},
                {'simulation': parm_simulation},
                {'base_position_config': base_position_file},
            ],
        ),
    ]

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'simulation',
                default_value='False',  # 设置默认值为字符串形式
                description='simulation mode',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
