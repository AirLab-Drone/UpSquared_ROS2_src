import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoCaptureFromRos2:
    def __init__(self, path, node=None):
        #'/world/iris_runway/model/iris_with_ardupilot_camera/model/camera/linkcamera_link/sensor/camera1/image'
        if not rclpy.ok():
            rclpy.init()
        self.cv_image = None
        self.node = node if node is not None else rclpy.create_node('video_capture_from_ros2')
        self.image_sub = self.node.create_subscription(Image, path, self.image_callback, 10)
    def image_callback(self, msg):
        self.cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        # 显示图像
        # cv2.imshow("Image", self.cv_image)
        # cv2.waitKey(1)
    def read(self):
        rclpy.spin_once(self.node)
        if(self.cv_image is None):
            return False, None
        return True, self.cv_image
    def release(self):
        self.__delattr__()
    def __delattr__(self):
        self.node.destroy_node()

if __name__ == "__main__":
    video = VideoCaptureFromRos2("/world/iris_runway/model/iris_with_ardupilot_camera/model/camera/link/camera_link/sensor/camera1/image")
    while True:
        ret, frame = video.read()
        print(ret, frame.shape)
        if ret:
            cv2.imshow("Image", frame)
            cv2.waitKey(1)
    # pass