#!/usr/bin/env python3

"""
    這個節點訂閱濾波後的aruco標記消息，判斷距離最近的標記並輸出
    發布的marker.msg以無人機中心為原點，x軸向右，y軸向後，z軸向上 (pixel座標)
"""

import rclpy
from rclpy.node import Node
from aruco_msgs.msg import Marker
from std_msgs.msg import Header
import math

class MarkerDistanceNode(Node):
    def __init__(self):
        super().__init__('marker_distance_node')

        # 訂閱兩個話題: /aruco/marker_filtered/id_0 和 /aruco/marker_filtered/id_6
        self.create_subscription(Marker, '/aruco/marker_filtered/id_0', self.marker_callback, 10)
        self.create_subscription(Marker, '/aruco/marker_filtered/id_6', self.marker_callback, 10)
        
        # 儲存接收到的標記數據
        self.markers = {0: None, 6: None}

        # 創建一個新的Publisher，發布最近的標記
        self.closest_marker_publisher = self.create_publisher(Marker, '/closest_marker', 10)

    #! 這裡有bug 這裡有bug 這裡有bug
    #! 若不發布 會拿舊資料去比較
    def marker_callback(self, msg):
        """處理來自標記的訊息"""
        self.markers[msg.id] = msg
        self.markers[msg.id].confidence = 1.0  # 如果接收到標記，將信任度設為1.0
        distance_0 = float("inf")
        distance_6 = float("inf")

        # 確認標記是否接收到，並分別處理
        if self.markers[0] is not None and self.markers[6] is not None:
            # 同時接收到兩個標記，計算距離並比較
            distance_0 = self.calculate_distance(self.markers[0])
            distance_6 = self.calculate_distance(self.markers[6])
            print("distance_0: ", distance_0)
            print("distance_6: ", distance_6)
        
        elif self.markers[0] is not None and self.markers[6] is None:
            # 只接收到ID 0
            distance_0 = self.calculate_distance(self.markers[0])
            distance_6 = float("inf")
            print("distance_6: ", distance_6)

        elif self.markers[6] is not None and self.markers[0] is None:
            # 只接收到ID 6
            distance_0 = float("inf")
            distance_6 = self.calculate_distance(self.markers[6])
            print("distance_0: ", distance_0)

        # if distance_0 < distance_6:
        #     self.publish_marker(self.markers[0], "Marker ID 0 is closer")
        # else:
        #     self.publish_marker(self.markers[6], "Marker ID 6 is closer")



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
