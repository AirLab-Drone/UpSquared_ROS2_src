#!/usr/bin/env python3

"""
此節點用於偵測aruco, 並發布Marker消息 
"""

import cv2
import numpy as np
import math
import yaml
import os
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from aruco_msgs.msg import Marker


class ArucoDetect(Node):
    def __init__(self):
        super().__init__("aruco_detect")

        self.declare_parameter(
            "show_image",
            False,
            ParameterDescriptor(
                description="Boolean to decide whether to display the OpenCV image or not"
            ),
        )
        self.show_image = (
            self.get_parameter("show_image").get_parameter_value().bool_value
        )

        package_share_directory = get_package_share_directory("aruco_detect")
        config_file_path = os.path.join(
            package_share_directory, "config", "aruco_markers.yaml"
        )

        # 檢查文件是否存在
        if not os.path.exists(config_file_path):
            self.get_logger().error(
                f"Cannot load Aruco markers config file: {config_file_path}"
            )
            self.destroy_node()
            rclpy.shutdown()
            return

        # 加载配置文件
        with open(config_file_path, "r") as config_file:
            self.aruco_markers = yaml.safe_load(config_file)
        self.get_logger().info(
            f"Loaded Aruco markers:\n{yaml.dump(self.aruco_markers, sort_keys=False, indent=2)}"
        )

        # 創建Publisher
        self.marker_publishers = {
            0: self.create_publisher(Marker, "/aruco/marker/id_0", 10),
            6: self.create_publisher(Marker, "/aruco/marker/id_6", 10),
        }

        self.closest_marker_publisher = self.create_publisher(Marker, '/closest_marker', 10)

        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )

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

    def draw_axis_and_id(self, frame, rvec, tvec, marker_length, marker_id):
        axis_length = marker_length * 0.8  # 坐標軸長度
        axis = np.float32(
            [
                [0, 0, 0],  # 原點
                [axis_length, 0, 0],  # X 軸
                [0, axis_length, 0],  # Y 軸
                [0, 0, axis_length],  # Z 軸
            ]
        ).reshape(-1, 3)

        # 將3D點投影到2D圖像上
        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, self.mtx, self.dist)

        imgpts = np.int32(imgpts).reshape(-1, 2)

        # 繪製坐標軸
        origin = tuple(imgpts[0].ravel())
        frame = cv2.line(
            frame, origin, tuple(imgpts[1].ravel()), (0, 0, 255), 5
        )  # X 紅色
        frame = cv2.line(
            frame, origin, tuple(imgpts[2].ravel()), (0, 255, 0), 5
        )  # Y 綠色
        frame = cv2.line(
            frame, origin, tuple(imgpts[3].ravel()), (255, 0, 0), 5
        )  # Z 藍色

        # 在原点旁边添加 Aruco ID 标签
        self._draw_aruco_id(frame, origin, marker_id)

        return frame

    def _draw_aruco_id(self, frame, origin, marker_id):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.8
        thickness = 2

        # 繪製黑色邊框 (用較粗的線條)
        cv2.putText(
            frame,
            f"{marker_id}",
            (origin[0] + 10, origin[1] + 20),
            font,
            font_scale,
            (0, 0, 0),
            thickness + 2,
            cv2.LINE_AA,
        )
        # 繪製白色文字 (用較細的線條)
        cv2.putText(
            frame,
            f"{marker_id}",
            (origin[0] + 10, origin[1] + 20),
            font,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )


    def image_callback(self, msg):
        # 将ROS图像消息转换为OpenCV图像
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 检测Aruco标记
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
        

        # 檢查是否有檢測到Aruco標記
        if ids is not None:
            found_valid_marker = False

            for i, marker_id in enumerate(ids.flatten()):
                # 檢查是否是ID 0或6
                if str(marker_id) in self.aruco_markers["aruco_markers"]:
                    marker_length = self.aruco_markers["aruco_markers"][str(marker_id)]["marker_length"]
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i], marker_length, self.mtx, self.dist
                    )

                    # 在图像上绘制Aruco标记及其轴
                    frame = self.draw_axis_and_id(
                        frame, rvec, tvec, marker_length, marker_id
                    )

                    # 发布有效的Aruco标记
                    self.publish_marker(marker_id, tvec, rvec)
                    found_valid_marker = True  # 表示找到了一個有效的標記

                # else:
                #     self.get_logger().info(f"Detected marker ID {marker_id}, but it's not 0 or 6")
            
            if not found_valid_marker:
                # 如果沒有找到有效的標記，發布空的Marker消息
                self.closest_marker_publisher.publish(self.create_empty_marker())
                self.get_logger().info(f"No valid Aruco markers (ID 0 or 6) detected in the image")

        else:
            # 如果沒有找到有效的標記，發布空的Marker消息
            self.closest_marker_publisher.publish(self.create_empty_marker())
            self.get_logger().info(f"No valid Aruco markers in the image")


        # 根據參數是否顯示圖像
        if self.show_image:
            # 在圖片中間畫一點
            cv2.circle(frame, (320, 240), 5, (255, 255, 255), -1)
            cv2.imshow("Aruco Detection", frame)
            cv2.waitKey(1)


    def rvec_to_euler_angles(self, rvec):
        R_mat, _ = cv2.Rodrigues(rvec[0])
        sy = math.sqrt(R_mat[0, 0] ** 2 + R_mat[1, 0] ** 2)
        singular = sy < 1e-6
        if not singular:
            roll = math.atan2(R_mat[2, 1], R_mat[2, 2])
            pitch = math.atan2(-R_mat[2, 0], sy)
            yaw = math.atan2(R_mat[1, 0], R_mat[0, 0])
        else:
            roll = math.atan2(-R_mat[2, 0], sy)
            pitch = math.atan2(R_mat[0, 2], R_mat[1, 1])
            yaw = 0
        return yaw, pitch, roll

    def publish_marker(self, marker_id, tvec, rvec):
        marker_msg = Marker()
        marker_msg.header = Header()
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.header.frame_id = "camera"
        marker_msg.id = int(marker_id)
        marker_msg.x = tvec[0][0][0]
        marker_msg.y = tvec[0][0][1]
        marker_msg.z = tvec[0][0][2]
        _yaw, _pitch, _roll = self.rvec_to_euler_angles(rvec)
        marker_msg.roll = math.degrees(_roll)
        marker_msg.pitch = math.degrees(_pitch)
        marker_msg.yaw = math.degrees(_yaw)
        marker_msg.confidence = 1.0

        if marker_id in self.marker_publishers:
            self.marker_publishers[marker_id].publish(marker_msg)
            self.get_logger().info(
                f"Published marker for ID {marker_id}: x={marker_msg.x:.4f}, y={marker_msg.y:.4f}, z={marker_msg.z:.4f}, roll={marker_msg.roll:.4f}, pitch={marker_msg.pitch:.4f}, yaw={marker_msg.yaw:.4f}"
            )

    def create_empty_marker(self):
        """創建confidence為0的空Marker消息"""
        empty_marker = Marker()
        empty_marker.header = Header()
        empty_marker.header.stamp = self.get_clock().now().to_msg()
        empty_marker.header.frame_id = 'camera_frame'
        empty_marker.id = 0  
        empty_marker.x = 0.0
        empty_marker.y = 0.0
        empty_marker.z = 0.0
        empty_marker.roll = 0.0
        empty_marker.pitch = 0.0
        empty_marker.yaw = 0.0
        empty_marker.confidence = 0.0
        return empty_marker

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ArucoDetect()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
