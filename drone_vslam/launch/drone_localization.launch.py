from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('rtabmap_args',   default_value='',                   
                              description='Can be used to pass RTAB-Map\'s parameters or other flags like --udebug and --delete_db_on_start/-d'),
        DeclareLaunchArgument('mode', default_value='true', 
                              description='Launch in localization mode.'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',  
                              description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz', default_value='true', 
                              description='Launch RVIZ (optional).'),                              


        # Realsense D435i launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('drone_vslam'),
                    'launch/realsenseD435i.launch.py'
                )
            )            
        ),

        # RTAB-Map launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('drone_vslam'),
                    'launch/rtabmap.launch.py'
                )
            ),
            launch_arguments={
                'args': LaunchConfiguration("rtabmap_args"),
                'localization': LaunchConfiguration("mode"),
                'rtabmap_viz': LaunchConfiguration("rtabmap_viz"),
                'rviz': LaunchConfiguration("rviz"),
                }.items()  # 传递参数到引用的启动文件
        )

    ])


    


