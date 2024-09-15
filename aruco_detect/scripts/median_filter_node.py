#!/usr/bin/env python3

"""
此節點用於濾波aruco (median filter with window size 5), 並發布Marker消息
"""
import numpy as np

import rclpy
from rclpy.node import Node
from aruco_msgs.msg import Marker
from std_msgs.msg import Header
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from collections import deque

class MedianFilterNode(Node):
    def __init__(self):
        super().__init__('median_filter_node')

        self.declare_parameter(
            "debug",
            False, 
            ParameterDescriptor(
                description="Enable debug mode",
                type=ParameterType.PARAMETER_BOOL
            ),
        )
        self.debug = self.get_parameter("debug").get_parameter_value().bool_value

        # 初始化兩個ID的窗口，窗口大小為5
        self.windows = {
            0: deque(maxlen=5),
            6: deque(maxlen=5)
        }

        # 訂閱Aruco ID 0 和 ID 6 的檢測消息
        self.subscription_handlers = {
            0: self.create_subscription(Marker, '/aruco/marker/id_0', self.create_marker_callback(0), 10),
            6: self.create_subscription(Marker, '/aruco/marker/id_6', self.create_marker_callback(6), 10)
        }

        # 分別發布濾波後的Marker消息
        self.marker_publishers = {
            0: self.create_publisher(Marker, '/aruco/marker_filtered/id_0', 10),
            6: self.create_publisher(Marker, '/aruco/marker_filtered/id_6', 10)
        }

    def create_marker_callback(self, marker_id):
        """
        返回一個針對指定Marker ID的回調函數，處理訂閱到的Marker消息。
        """
        def marker_callback(msg):
            # 提取位姿數據 (x, y, z, roll, pitch, yaw)
            current_values = np.array([msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw])

            # 添加當前值到窗口
            self.windows[marker_id].append(current_values)

            # 如果窗口中數據不足5個，則直接返回未濾波的值
            if len(self.windows[marker_id]) < 5:
                filtered_values = current_values
            else:
                # 計算每個軸的中位數
                filtered_values = np.median(self.windows[marker_id], axis=0)

            # 創建新的Marker消息
            filtered_marker = Marker()
            filtered_marker.header = Header()
            filtered_marker.header.stamp = self.get_clock().now().to_msg()
            filtered_marker.header.frame_id = msg.header.frame_id
            filtered_marker.id = msg.id
            filtered_marker.x, filtered_marker.y, filtered_marker.z = filtered_values[0:3]
            filtered_marker.roll, filtered_marker.pitch, filtered_marker.yaw = filtered_values[3:6]
            filtered_marker.confidence = msg.confidence

            # 發布濾波後的Marker消息
            self.marker_publishers[marker_id].publish(filtered_marker)
            if self.debug:
                self.log_filtered_marker(marker_id, filtered_marker)

        return marker_callback

    def log_filtered_marker(self, marker_id, marker_msg):
        """
        記錄濾波後的Marker消息。
        """
        self.get_logger().info(
            f"\n"
            f"Median Filter Marker\n"
            f"  ID = {marker_id}\n"
            f"  x = {marker_msg.x:.4f}, y = {marker_msg.y:.4f}, z = {marker_msg.z:.4f}\n"
            f"  roll = {marker_msg.roll:.4f}, pitch = {marker_msg.pitch:.4f}, yaw = {marker_msg.yaw:.4f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MedianFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
