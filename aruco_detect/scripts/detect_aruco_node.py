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
from sensor_msgs.msg import Image



class ArucoDetect(Node):
    def __init__(self):
        super().__init__("aruco_detect")

        package_share_directory = get_package_share_directory('aruco_detect')
        config_file_path = os.path.join(package_share_directory, 'config', 'aruco_markers.yaml')

        # 检查文件是否存在
        if not os.path.exists(config_file_path):
            self.get_logger().error(f"Cannot load Aruco markers config file: {config_file_path}")
            self.destroy_node()
            rclpy.shutdown()
            return

        # 加载配置文件
        with open(config_file_path, 'r') as config_file:
            self.aruco_markers = yaml.safe_load(config_file)
        # 使用 yaml.dump 格式化输出
        formatted_yaml = yaml.dump(self.aruco_markers, sort_keys=False, indent=2)
        self.get_logger().info(f"Loaded Aruco markers:\n{formatted_yaml}")

    
        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()

        # VGA 180fps
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

    def image_callback(self, msg):
        # 将ROS图像消息转换为OpenCV图像
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 转换为灰度图像
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 加载Aruco字典
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()

        # 检测Aruco标记
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if str(marker_id) in self.aruco_markers['aruco_markers']:
                    marker_length = self.aruco_markers['aruco_markers'][str(marker_id)]['marker_length']
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_length, self.mtx, self.dist)

                    # 在图像上绘制Aruco标记及其轴
                    cv2.aruco.drawDetectedMarkers(frame, [corners[i]])
                    # cv2.aruco.drawAxis(frame, self.mtx, self.dist, rvec, tvec, 0.1)

                    self.get_logger().info(f"Detected Aruco ID: {marker_id}, Position: {tvec.flatten()}, Rotation: {rvec.flatten()}")

        # 显示图像 (调试时使用)
        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ArucoDetect()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
