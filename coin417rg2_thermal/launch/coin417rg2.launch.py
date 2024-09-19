import launch
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

import os
import platform


def generate_launch_description():

    # 獲取lib路徑的相對路徑
    package_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    # 根據系統架構設置 lib 路徑
    print(f"[Info] {platform.machine()}")
    if platform.machine() == "x86_64":
        lib_path = os.path.join(package_path, "lib", "x86_64")
    elif platform.machine() in ["arm", "aarch64"]:
        lib_path = os.path.join(package_path, "lib", "arm")
    else:
        print(f"Unknown architecture: {platform.machine()}")
        return None

    # 設置LD_LIBRARY_PATH
    ld_library_path_current = os.environ.get("LD_LIBRARY_PATH", "")

    return launch.LaunchDescription(
        [
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH", value=f"{lib_path}:{ld_library_path_current}"
            ),
            Node(
                package="coin417rg2_thermal",
                executable="COIN417RG2_ros2_node",
                name="coin417rg2_thermal",
            ),
            Node(
                package="coin417rg2_thermal",
                executable="thermal_frame_to_drone_frame",
                name="thermal_frame_to_drone_frame",
            ),
        ]
    )


# if __name__ == '__main__':
#     generate_launch_description()
