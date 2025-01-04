from ament_index_python.packages import get_package_share_directory
import os
import yaml


def get_yaml_config(
    package_name: str = "", config_file_name: str = "", config_file_path: str = ""
):
    if package_name and config_file_name and not config_file_path:
        package_share_directory = get_package_share_directory(package_name)
        config_file_path = os.path.join(
            package_share_directory, "config", config_file_name
        )
    elif not config_file_path:
        print("Please provide package_name and config_file_name")
        return
    # 檢查文件是否存在
    if not os.path.exists(config_file_path):
        print(f"Cannot load Aruco markers config file: {config_file_path}")
        return

    # 加载配置文件
    with open(config_file_path, "r") as config_file:
        return yaml.safe_load(config_file)
