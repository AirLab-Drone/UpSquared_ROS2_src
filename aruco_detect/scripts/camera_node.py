#!/usr/bin/env python3


"""
ubuntu查相機設定
    v4l2-ctl -d /dev/video0 --all

User Controls

                     brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=0
                       contrast 0x00980901 (int)    : min=0 max=64 step=1 default=32 value=32
                     saturation 0x00980902 (int)    : min=0 max=128 step=1 default=0 value=0
                            hue 0x00980903 (int)    : min=-40 max=40 step=1 default=0 value=0
        white_balance_automatic 0x0098090c (bool)   : default=1 value=1
                          gamma 0x00980910 (int)    : min=72 max=500 step=1 default=100 value=100
                           gain 0x00980913 (int)    : min=0 max=100 step=1 default=0 value=0
           power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=1 value=2 (60 Hz)
                        0: Disabled
                        1: 50 Hz
                        2: 60 Hz
      white_balance_temperature 0x0098091a (int)    : min=2800 max=6500 step=1 default=4600 value=4600 flags=inactive
                      sharpness 0x0098091b (int)    : min=0 max=6 step=1 default=3 value=3
         backlight_compensation 0x0098091c (int)    : min=0 max=2 step=1 default=1 value=1

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=3 (Aperture Priority Mode)
                        1: Manual Mode
                        3: Aperture Priority Mode
         exposure_time_absolute 0x009a0902 (int)    : min=1 max=5000 step=1 default=157 value=5000 flags=inactive
     exposure_dynamic_framerate 0x009a0903 (bool)   : default=0 value=0

此節點用於捕捉圖像並發布圖像
發布圖像為 sensor_msgs.msg.Image 型態

設定曝光值:
    /camera_node exposure(int)  :min=1 max=5000 step=1 default=157
    ros2 param set /camera_node exposure 2

"""

import cv2
import numpy as np
import time
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType, ParameterDescriptor, IntegerRange, SetParametersResult
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool


class CaptureImage(Node):

    def __init__(self):
        super().__init__("camera_node")


        # ---------------------- Define the parameter descriptor --------------------- #

        # 創建參數描述，並設置類型為布林值
        self.declare_parameter(
            "debug",
            False, 
            ParameterDescriptor(
                description="Enable debug mode",
                type=ParameterType.PARAMETER_BOOL
            ),
        )
        self.debug = self.get_parameter("debug").get_parameter_value().bool_value


        exposure_descriptor = ParameterDescriptor(
            description=(
                "camera's exposure value as an integer (ms)"
            ),
            type=ParameterType.PARAMETER_INTEGER,
            integer_range=[IntegerRange(
                from_value=1,   # 曝光值最小值
                to_value=5000,  # 曝光值最大值
                step=1          # 步長為 1
            )],
        )

        # Define the parameter with a descriptor
        self.declare_parameter(
            "exposure", 157, exposure_descriptor
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
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3.0)
        self.update_exposure(self.get_parameter("exposure").get_parameter_value().integer_value)

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
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)   # 關閉自動曝光
        self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure_value)
        self.cap.read()  # 捕获一帧以确保设置生效
        self.get_logger().info(f"Exposure set to: {exposure_value}")


    def timer_callback(self):
        # 捕捉每一幀
        ret, frame = self.cap.read()

        # 如果相機無法啟動，則記錄信息並返回
        # TODO: 若相機無法啟動, 無人機要做出相應的處理
        if not ret:
            self.get_logger().error("Can't receive frame (stream end?). Exiting ...")
            return

        # FPS 計算
        self.frame_count += 1
        current_time = self.get_clock().now()
        elapsed_time = current_time - self.start_time
        elapsed_seconds = elapsed_time.nanoseconds / 1e9  # Convert nanoseconds to seconds

        if elapsed_seconds > 0.5:
            self.fps = self.frame_count / elapsed_seconds
            if self.debug:
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
            # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3.0)  # 開啟自動曝光
            self.get_logger().info("Auto exposure enabled.")
        else:
            # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3.0)  
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
