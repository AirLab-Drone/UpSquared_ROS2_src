import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='coin417rg2_thermal',
            executable='COIN417RG2_ros2_node',
            name='coin417rg2_thermal'),
  ])