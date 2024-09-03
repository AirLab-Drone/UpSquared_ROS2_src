#!/usr/bin/env python3

"""
此節點用於偵測aruco, 並發布位置
"""


import cv2
import numpy as np
import yaml
import os
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterType, ParameterDescriptor, FloatingPointRange, SetParametersResult
from sensor_msgs.msg import Image



class ArucoDetect(Node):
    def __init__(self):
        super().__init__("aruco_detect")

        package_share_directory = get_package_share_directory('aruco_detect')
        config_file_path = os.path.join(package_share_directory, 'config', 'aruco_markers.yaml')
        try:
            # 加载配置文件
            with open(config_file_path, 'r') as config_file:
                self.aruco_markers = yaml.safe_load(config_file)
            # 使用 yaml.dump 格式化输出
            formatted_yaml = yaml.dump(self.aruco_markers, sort_keys=False, indent=2)
            self.get_logger().info(f"Loaded Aruco markers:\n{formatted_yaml}")

        except:
            self.get_logger().error(f"Cannot load Aruco markers config file: {config_file_path}")
            self.destroy_node()
            rclpy.shutdown()
            return
            

        # # VGA 180fps
        self.mtx = np.array(
            [
                [479.23864074, 0.0, 322.41904053],
                [0.0, 478.87010769, 208.59056289],
                [0.0, 0.0, 1.0],
            ]
        )
        self.dist = np.array(
            [[-0.04673894, 0.12198613, 0.00533764, 0.00095581, -0.15779023]]
        )





def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ArucoDetect()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
