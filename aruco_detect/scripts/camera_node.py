#!/usr/bin/env python3


"""
此節點用於捕捉圖像並發布圖像
發布圖像為 sensor_msgs.msg.Image 型態

設定曝光值:
    ros2 param set /camera_node exposure 100.0
"""

import cv2
import numpy as np
import time
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType, ParameterDescriptor, FloatingPointRange, SetParametersResult
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool


class CaptureImage(Node):

    def __init__(self):
        super().__init__("camera_node")


        # ---------------------- Define the parameter descriptor --------------------- #
        exposure_descriptor = ParameterDescriptor(
            description=(
                "camera's exposure value as a positive double (ms)"
            ),
            type = ParameterType.PARAMETER_DOUBLE,
            # floating_point_range = [FloatingPointRange(
            #     from_value = 0.0,   # 曝光值最小值
            #     to_value = 1000000.0    # 曝光值最大值
            # )],
        )

        # Define the parameter with a descriptor
        self.declare_parameter(
            "exposure", 20.0, exposure_descriptor
        ) 
        # Register a callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)




        # ----------------------------------- 相機設定 ----------------------------------- #
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 180)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3.0)
        self.update_exposure(self.get_parameter("exposure").get_parameter_value().double_value)

        # 取得當前FPS
        self.actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"Actual FPS: {self.actual_fps}")

        self.bridge = CvBridge()

        self.camera_image_publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        
        self.frame_count = 0
        self.start_time = self.get_clock().now()
        self.fps = 0

        timer_period = 1.0 / 180.0  # 計時器周期設為相機實際 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.set_auto_exposure_srv = self.create_service(SetBool, 'set_auto_exposure', self.set_auto_exposure_callback)
        

    def parameter_callback(self, params):

        for param in params:
            print(f"Received param: {param.name}, type: {param.type_}, value: {param.value}")
            if param.name == "exposure":
                self.update_exposure(param.value)
        return SetParametersResult(successful=True)


    def update_exposure(self, exposure_value):
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3.0)  
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)   # 關閉自動曝光
        self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure_value)
        self.cap.read()  # 捕获一帧以确保设置生效
        self.get_logger().info(f"Exposure set to: {exposure_value}")


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
        current_time = self.get_clock().now()
        elapsed_time = current_time - self.start_time
        elapsed_seconds = elapsed_time.nanoseconds / 1e9  # Convert nanoseconds to seconds

        if elapsed_seconds > 0.5:
            self.fps = self.frame_count / elapsed_seconds
            self.get_logger().info(f"Current FPS: {self.fps:.2f}")
            self.frame_count = 0
            self.start_time = current_time

        # 在影像上顯示 FPS (可選)
        # cv2.putText(frame, f'FPS: {self.fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 發布圖片
        self.msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.camera_image_publisher.publish(self.msg)


    def set_auto_exposure_callback(self, request, response):
        if request.data:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3.0)  # 開啟自動曝光
            self.get_logger().info("Auto exposure enabled.")
        else:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3.0)  
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)   # 關閉自動曝光
            self.get_logger().info("Auto exposure disabled.")
        response.success = True
        response.message = "Auto exposure setting updated."
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CaptureImage()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
