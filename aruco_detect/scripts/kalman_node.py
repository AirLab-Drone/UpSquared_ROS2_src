#!/usr/bin/env python3

"""
此節點用於濾波aruco (kalman filter), 並發布Marker消息
"""
import numpy as np

import rclpy
from rclpy.node import Node
from aruco_msgs.msg import Marker
from std_msgs.msg import Header

from aruco_detect.kalman_filter import KalmanFilter

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # 初始化兩個卡爾曼濾波器
        self.kalman_filters = {
            0: KalmanFilter(),
            6: KalmanFilter()
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
            z = np.array([msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw])

            # 預測和更新卡爾曼濾波
            self.kalman_filters[marker_id].predict()
            filtered_values = self.kalman_filters[marker_id].update(z)

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
            self.log_filtered_marker(marker_id, filtered_marker)

        return marker_callback

    def log_filtered_marker(self, marker_id, marker_msg):
        """
        記錄濾波後的Marker消息。
        """
        self.get_logger().info(
            f"Filter marker for ID {marker_id}: x={marker_msg.x:.4f}, y={marker_msg.y:.4f}, z={marker_msg.z:.4f}, roll={marker_msg.roll:.4f}, pitch={marker_msg.pitch:.4f}, yaw={marker_msg.yaw:.4f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
