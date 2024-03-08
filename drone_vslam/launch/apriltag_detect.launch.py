
import os

from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from typing import Text
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args,  **kwargs):     
    
    apriltag_yaml = os.path.join(
        get_package_share_directory('apriltag_ros'), 'apriltag_ros', 'cfg', 'tags_36h11.yaml')   

    return [

        
        Node(
            package='apriltag_ros', executable='apriltag_node', output='screen',
            remappings=[
                ('/apriltag/image_rect', '/camera/infra1/image_rect_raw'),
                ('/apriltag/camera_info', '/camera/infra1/camera_info')
            ],
            parameters=[{'from': apriltag_yaml}],
            # extra_arguments=['use_intra_process_comms:=True']
        )

                
    ]


def generate_launch_description():
    

    return LaunchDescription([
        OpaqueFunction(function=launch_setup)

    ])