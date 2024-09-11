#!/usr/bin/env python3

"""
    這個節點訂閱濾波後的aruco標記消息，判斷距離最近的標記並輸出
    發布的marker.msg以無人機中心為原點，x軸向右，y軸向後，z軸向上 (pixel座標)
"""


import math
import yaml
import os

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from aruco_msgs.msg import Marker
from std_msgs.msg import Header


class MarkerDistanceNode(Node):
    def __init__(self):
        super().__init__('marker_distance_node')


        package_share_directory = get_package_share_directory("aruco_detect")
        config_file_path = os.path.join(
            package_share_directory, "config", "camera_frame_to_drone_frame.yaml"
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
            self.camera_to_drone_config = yaml.safe_load(config_file)
        self.get_logger().info(
            f"Loaded frame transformation config file:\n{yaml.dump(self.camera_to_drone_config, sort_keys=False, indent=2)}"
        )

        self.camera_x = self.camera_to_drone_config['camera_x']
        self.camera_y = self.camera_to_drone_config['camera_y']
        self.camera_z = self.camera_to_drone_config['camera_z']
        self.camera_roll = self.camera_to_drone_config['camera_roll']
        self.camera_pitch = self.camera_to_drone_config['camera_pitch']
        self.camera_yaw = self.camera_to_drone_config['camera_yaw']
        self.get_logger().info(f"camera_x: {self.camera_x}")
        self.get_logger().info(f"camera_y: {self.camera_y}")
        self.get_logger().info(f"camera_z: {self.camera_z}")
        self.get_logger().info(f"camera_roll: {self.camera_roll}")
        self.get_logger().info(f"camera_pitch: {self.camera_pitch}")
        self.get_logger().info(f"camera_yaw: {self.camera_yaw}")


        # 訂閱兩個話題: /aruco/marker_filtered/id_0 和 /aruco/marker_filtered/id_6
        self.create_subscription(Marker, '/aruco/marker_filtered/id_0', self.marker_callback, 10)
        self.create_subscription(Marker, '/aruco/marker_filtered/id_6', self.marker_callback, 10)

        
        # 儲存接收到的標記數據即時間
        self.markers = {0: (None, self.get_clock().now()), 6: (None, self.get_clock().now())}
        self.timeout = Duration(seconds=0.1) # 設定超時閾值（0.1秒）

        # 創建一個新的Publisher，發布最近的標記
        self.closest_marker_publisher = self.create_publisher(Marker, '/closest_marker', 10)
        



    def marker_callback(self, msg):

        """處理來自標記的訊息"""
        current_time = self.get_clock().now()

        # 創建標記的副本以避免直接修改接收到的訊息
        marker_copy = Marker()
        marker_copy.header = msg.header
        marker_copy.id = msg.id
        marker_copy.x = msg.x
        marker_copy.y = msg.y
        marker_copy.z = msg.z
        marker_copy.roll = msg.roll
        marker_copy.pitch = msg.pitch
        marker_copy.yaw = msg.yaw
        marker_copy.confidence = 1.0  # 設置信任度

        # 更新標記和時間
        self.markers[msg.id] = (marker_copy, current_time)

        # 檢查標記是否超時
        for marker_id in self.markers:
            marker, last_received_time = self.markers[marker_id]
            if current_time - last_received_time > self.timeout:
                self.markers[marker_id] = (None, last_received_time)  # 清除超時的標記

        # 計算距離並比較
        closest_marker = self.camera_frame_to_drone_frame(self.calculate_closest_marker())
        self.publish_marker(closest_marker, f"{closest_marker.id} is the closest marker")

    def calculate_closest_marker(self):
        """計算距離並回傳最近的標記"""


        distance_0 = float("inf")
        distance_6 = float("inf")

        marker_0_visible = self.markers[0][0] is not None
        marker_6_visible = self.markers[6][0] is not None

        # 計算兩個標記的距離
        if marker_0_visible:
            distance_0 = self.calculate_distance(self.markers[0][0])
        if marker_6_visible:
            distance_6 = self.calculate_distance(self.markers[6][0])

        # 檢查是否只看到一個標記
        if marker_0_visible and not marker_6_visible:
            return self.markers[0][0]
            # self.publish_marker(self.markers[0][0], "Only see ID: 0")
        elif marker_6_visible and not marker_0_visible:
            return self.markers[6][0]
            # self.publish_marker(self.markers[6][0], "Only see ID: 6")

        elif marker_0_visible and marker_6_visible:
            # 比較距離並發布最近的標記
            if distance_0 < distance_6:
                return self.markers[0][0]
                # self.publish_marker(self.markers[0][0], "Marker ID 0 is closer")
            else:
                return self.markers[6][0]
                # self.publish_marker(self.markers[6][0], "Marker ID 6 is closer")


    def camera_frame_to_drone_frame(self, marker):
        """將標記從相機座標轉換為無人機中心座標"""

        # 校正相機的偏移量，將相機座標轉換為無人機中心座標
        transformed_marker = Marker()
        
        # x, y, z 座標校正
        transformed_marker.x = marker.x + self.camera_x
        transformed_marker.y = marker.y + self.camera_y
        transformed_marker.z = marker.z + self.camera_z

        # 如果需要考慮旋轉，這裡可以加入旋轉修正
        # 此處假設相機的旋轉角度為 0，所以只需保持原值
        transformed_marker.roll = marker.roll + self.camera_roll
        transformed_marker.pitch = marker.pitch + self.camera_pitch
        transformed_marker.yaw = marker.yaw + self.camera_yaw

        # 保留其他屬性
        transformed_marker.id = marker.id
        transformed_marker.header = marker.header
        transformed_marker.confidence = marker.confidence

        return transformed_marker


    def calculate_distance(self, marker):
        """計算標記到相機的距離"""
        return math.sqrt(marker.x ** 2 + marker.y ** 2 + marker.z ** 2)

    def publish_marker(self, marker, log_message):
        """發布Marker消息並記錄日誌"""
        self.closest_marker_publisher.publish(marker)
        self.get_logger().info(log_message)



def main(args=None):
    rclpy.init(args=args)
    node = MarkerDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
