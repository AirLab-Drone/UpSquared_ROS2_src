import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class TFTransformer(Node):
    def __init__(self):
        super().__init__('tf_transformer')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 订阅需要转换的TF变换
        self.subscription = self.create_subscription(
            TransformStamped,
            '/tf',
            self.tf_callback,
            10
        )
        self.subscription  # 防止Python清除subscription对象

    def tf_callback(self, msg):
        try:
            # 获取原始的TF变换
            original_tf = self.tf_buffer.lookup_transform(
                msg.header.frame_id,
                msg.child_frame_id,
                msg.header.stamp
            )

            # 创建新的TF变换，将父子坐标系互换
            new_tf = TransformStamped()
            new_tf.header = original_tf.header
            new_tf.header.frame_id = original_tf.child_frame_id
            new_tf.child_frame_id = original_tf.header.frame_id
            new_tf.transform = original_tf.transform

            # 发布新的TF变换
            self.tf_broadcaster.sendTransform(new_tf)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Transform lookup failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    tf_transformer = TFTransformer()
    rclpy.spin(tf_transformer)
    tf_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
