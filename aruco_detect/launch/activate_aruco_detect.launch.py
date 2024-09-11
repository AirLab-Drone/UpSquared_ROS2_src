from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_detect',
            executable="camera_node.py",
            # output="screen",
            name="camera_node",            
            parameters=[
                {'exposure': 5000},
            ],
        ),
        Node(
            package='aruco_detect',
            executable='detect_aruco_node.py',
            # output='screen',
            name='detect_aruco',          
            parameters=[
                {'show_image': True},
            ],
        ),
        Node(
            package='aruco_detect',
            executable='kalman_node.py',
            # output='screen',
            name='kalman_node'
        ),
        Node(
            package='aruco_detect',
            executable='closest_marker_publisher.py',
            # output='screen',
            name='closest_marker'
        ),
    ])