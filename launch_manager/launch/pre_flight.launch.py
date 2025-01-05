import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import TimerAction


def flight_controller_setup(context, *args, **kwargs):
    parm_simulation_str = LaunchConfiguration("simulation").perform(context)
    parm_simulation = parm_simulation_str.lower() == "true"  # 将字符串解析为布尔值

    if parm_simulation:
        # 在这里添加你想在 simulation 模式下执行的操作
        print("flight controller Simulation mode is enabled")
        return [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("flight_control"),
                        "launch/flight_controller_setup_sim.launch.py",
                    )
                ),
            )
        ]
    else:
        # 在这里添加你想在非 simulation 模式下执行的操作
        print("flight controller Simulation mode is disabled")
        return [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("flight_control"),
                        "launch/flight_controller_setup.launch.py",
                    )
                ),
            )
        ]


def camera_setup(context, *args, **kwargs):
    parm_simulation_str = LaunchConfiguration("simulation").perform(context)
    parm_simulation = parm_simulation_str.lower() == "true"  # 将字符串解析为布尔值

    if parm_simulation:
        # 在这里添加你想在 simulation 模式下执行的操作
        print("camera setup Simulation mode is enabled")
        return [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("aruco_detect"),
                        "launch/activate_aruco_detect_sim.launch.py",
                    )
                ),
            )
        ]
    else:
        # 在这里添加你想在非 simulation 模式下执行的操作
        print("camera setup Simulation mode is disabled")
        return [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("aruco_detect"),
                        "launch/activate_aruco_detect.launch.py",
                    )
                ),
            )
        ]


def thermal_camera_setup(context, *args, **kwargs):
    parm_simulation_str = LaunchConfiguration("simulation").perform(context)
    parm_simulation = parm_simulation_str.lower() == "true"  # 将字符串解析为布尔值

    if parm_simulation:
        # 在这里添加你想在 simulation 模式下执行的操作
        print("thermal camera setup Simulation mode is enabled")
        return [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("coin417rg2_thermal"),
                        "launch/coin417rg2_sim.launch.py",
                    )
                ),
            )
        ]
    else:
        # 在这里添加你想在非 simulation 模式下执行的操作
        print("thermal camera setup Simulation mode is disabled")
        return [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("coin417rg2_thermal"),
                        "launch/coin417rg2.launch.py",
                    )
                ),
            )
        ]


def payload_setup(context, *args, **kwargs):
    parm_simulation_str = LaunchConfiguration("simulation").perform(context)
    parm_simulation = parm_simulation_str.lower() == "true"  # 将字符串解析为布尔值

    if parm_simulation:
        # 在这里添加你想在 simulation 模式下执行的操作
        print("payload setup Simulation mode is enabled")
        return
    else:
        # 在这里添加你想在非 simulation 模式下执行的操作
        print("payload setup Simulation mode is disabled")
        return [
            Node(
                package="payload",
                executable="JY_modbus_service.py",
                output="screen",
            )
        ]


def platform_setup(context, *args, **kwargs):
    parm_simulation_str = LaunchConfiguration("simulation").perform(context)
    parm_simulation = parm_simulation_str.lower() == "true"  # 将字符串解析为布尔值

    if parm_simulation:
        # 在这里添加你想在 simulation 模式下执行的操作
        print("platform setup Simulation mode is enabled")
        return
    else:
        # 在这里添加你想在非 simulation 模式下执行的操作
        print("platform setup Simulation mode is disabled")
        return [
            Node(
                package="platform_communication",
                executable="platform_communication_node.py",
                output="screen",
            )
        ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "simulation",
                default_value="False",  # 设置默认值为字符串形式
                description="simulation mode",
            ),
            OpaqueFunction(function=flight_controller_setup),
            OpaqueFunction(function=camera_setup),
            OpaqueFunction(function=thermal_camera_setup),
            OpaqueFunction(function=payload_setup),
            OpaqueFunction(function=platform_setup),
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package="drone_status",
                        executable="USBChecker.py",
                        output="screen",
                    )
                ],
            )
        ]
    )
