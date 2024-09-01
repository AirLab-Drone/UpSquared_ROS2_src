#!/usr/bin/env python3


"""
此節點用於捕捉圖像並發布校正畸變後的圖像
發布圖像為 sensor_msgs.msg.Image 型態
"""

import cv2
import numpy as np
import time
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CaptureImage(Node):

    def __init__(self):
        super().__init__("camera_node")

        # # VGA 180fps
        # self.mtx = np.array(
        #     [
        #         [479.23864074, 0.0, 322.41904053],
        #         [0.0, 478.87010769, 208.59056289],
        #         [0.0, 0.0, 1.0],
        #     ]
        # )
        # self.dist = np.array(
        #     [[-0.04673894, 0.12198613, 0.00533764, 0.00095581, -0.15779023]]
        # )

        # ----------------------------------- 相機設定 ----------------------------------- #
        self.cap = cv2.VideoCapture(1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 180)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        # 取得當前FPS
        self.actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"Actual FPS: {self.actual_fps}")



        self.bridge = CvBridge()
        self.msg = Image()


        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0

        timer_period = 1.0 / 180.0  # 計時器周期設為相機實際 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # 捕捉每一幀
        ret, frame = self.cap.read()

        # 如果相機無法啟動，則記錄信息並返回
        # TODO: 若相機無法啟動, 無人機要做出相應的處理
        if not ret:
            self.get_logger().info("Can't receive frame (stream end?). Exiting ...")
            return

        # FPS 計算
        self.frame_count += 1
        elapsed_time = time.time() - self.start_time
        if elapsed_time > 1:
            self.fps = self.frame_count / elapsed_time
            self.get_logger().info(f"Current FPS: {self.fps:.2f}")
            self.frame_count = 0
            self.start_time = time.time()

        # 在影像上顯示 FPS (可選)
        cv2.putText(frame, f'FPS: {self.fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 發布圖片
        self.msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.camera_image_publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CaptureImage()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
