import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformListener
import tf2_ros
import numpy as np

 

class DistanceCalculator(Node):
    def __init__(self):
        super().__init__('distance_calculator')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


    def find_apriltag(self, parent_frame, tag_id):
        try:
            transform = self.tf_buffer.lookup_transform(parent_frame, tag_id, rclpy.time.Time().to_msg(), rclpy.duration.Duration(seconds=1))
            return transform.transform.translation
        except Exception as e:
            self.get_logger().error('Error finding apriltag: {}'.format(e))
            return None

    def get_distance(self, parent_frame, child_frame):
        try:
            # 获取父坐标系到子坐标系的变换
            transform = self.tf_buffer.lookup_transform(parent_frame, child_frame, rclpy.time.Time().to_msg(), rclpy.duration.Duration(seconds=1))

            # 提取变换的平移分量
            translation = transform.transform.translation

            # 计算距离
            distance = np.sqrt(translation.x**2 + translation.y**2 + translation.z**2)

            return distance

            # return translation.x, translation.y, translation.z
        except Exception as e:
            self.get_logger().error('Error calculating distance: {}'.format(e))
            return None


def main(args=None):
    rclpy.init(args=args)
    distance_calculator = DistanceCalculator()

    parent_frame = 'camera_link'  # 父坐标系的名称
    child_frame = 'Mario'    # 子坐标系的名称


    # while rclpy.ok():
    #     apriltag_pos = distance_calculator.find_apriltag(parent_frame, child_frame)
    #     if apriltag_pos is not None:
    #         print(f"{apriltag_pos}")
    #     rclpy.spin_once(distance_calculator)

    # distance_calculator.destroy_node()
    # rclpy.shutdown()


    while rclpy.ok():
        distance = distance_calculator.get_distance(parent_frame, child_frame)
        if distance is not None:
            # print(f"x: {distance[0]:.3f}, y: {distance[1]:.3f}, z: {distance[2]:.3f}")
            distance_calculator.get_logger().info('distance: {}'.format(distance))
        rclpy.spin_once(distance_calculator)

    distance_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()