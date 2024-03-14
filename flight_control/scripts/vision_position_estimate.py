import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from sensor_msgs.msg import Range

def setVisionPositionEstimateMsg(x, y, z, pitch, roll, yaw, w):
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.x = pitch
    msg.pose.orientation.y = roll
    msg.pose.orientation.z = yaw
    msg.pose.orientation.w = w
    return msg
def setPoseCovarianceMsg(x, y, z, pitch, roll, yaw, w):
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.x = pitch
    msg.pose.pose.orientation.y = roll
    msg.pose.pose.orientation.z = yaw
    msg.pose.pose.orientation.w = w

    return msg
def odom_callback(msg):
    print(msg.pose.pose.position.x)
    print(msg.pose.pose.position.y)
    print(msg.pose.pose.position.z)
    print(msg.pose.pose.orientation.x)
    print(msg.pose.pose.orientation.y)
    print(msg.pose.pose.orientation.z)
    print(msg.pose.pose.orientation.w)
    print('-----------------------')
    msg = setVisionPositionEstimateMsg(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    pub.publish(msg)
if __name__ == '__main__':
    rclpy.init()
    node = Node('vision_position_estimate')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', 10)
    # pub = node.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
    # sub = node.create_subscription(Odometry, '/odom', odom_callback, 10)
    rate = node.create_rate(10)
    while rclpy.ok():
        msg = setPoseCovarianceMsg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        print(msg)
        pub.publish(msg)
        rclpy.spin_once(node)
        rate.sleep()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    