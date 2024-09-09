#!/usr/bin/env python3

"""
此節點用於濾波aruco (kalman filter), 並發布Marker消息
"""

import rclpy
from rclpy.node import Node
from aruco_msgs.msg import Marker
from std_msgs.msg import Header
import numpy as np
from aruco_detect.kalman_filter import KalmanFilter

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # 初始化卡爾曼濾波器
        self.kalman_filter_0 = KalmanFilter()
        self.kalman_filter_6 = KalmanFilter()

        # 訂閱Aruco ID 0 和 ID 6 的檢測消息
        self.subscription_0 = self.create_subscription(
            Marker,
            '/aruco/marker/id_0',
            self.marker_callback_0,
            10
        )
        self.subscription_6 = self.create_subscription(
            Marker,
            '/aruco/marker/id_6',
            self.marker_callback_6,
            10
        )

        # 分別發布濾波後的Marker消息
        self.publisher_0 = self.create_publisher(Marker, '/aruco/marker_filtered/id_0', 10)
        self.publisher_6 = self.create_publisher(Marker, '/aruco/marker_filtered/id_6', 10)

    def marker_callback_0(self, msg):
        # 提取位姿數據 (x, y, z, roll, pitch, yaw)
        z = np.array([msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw])

        # 預測和更新卡爾曼濾波
        self.kalman_filter_0.predict()
        filtered_values = self.kalman_filter_0.update(z)

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
        self.publisher_0.publish(filtered_marker)
        self.get_logger().info(f'Published filtered marker for ID 0: {filtered_marker.id}')

    def marker_callback_6(self, msg):
        # 提取位姿數據 (x, y, z, roll, pitch, yaw)
        z = np.array([msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw])

        # 預測和更新卡爾曼濾波
        self.kalman_filter_6.predict()
        filtered_values = self.kalman_filter_6.update(z)

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
        self.publisher_6.publish(filtered_marker)
        self.get_logger().info(f'Published filtered marker for ID 6: {filtered_marker.id}')

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
