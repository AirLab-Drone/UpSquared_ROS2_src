#!/usr/bin/env python3


"""
此節點用於捕捉圖像並發布校正畸變後的圖像
發布圖像為 sensor_msgs.msg.Image 型態
"""

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CaptureImage(Node):

    def __init__(self):
        super().__init__("camera_node")
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

        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        self.msg = Image()


        self.camera_image_punlisher = self.create_publisher(Image, "/camera/image_raw", 10)
        timer_period =  1.0 / 180.0 # seconds 180fps
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        # Capture frame-by-frame
        ret, frame = self.cap.read()

        # undistort
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
        dst = cv2.undistort(frame, self.mtx, self.dist, None, newcameramtx)

        # 發布圖片
        self.msg = self.bridge.cv2_to_imgmsg(dst, "bgr8")
        self.camera_image_punlisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CaptureImage()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
