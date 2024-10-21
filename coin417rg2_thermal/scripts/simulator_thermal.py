#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from thermal_msgs.msg import ThermalAlert


class SimulatorThermal(Node):
    def __init__(self):
        super().__init__("simulator_thermal")
        self.get_logger().info("Simulator Thermal Node has been started")
        self.create_subscription(Image, "/thermal_camera", self.image_callback, 10)
        self.hot_spot_publisher = self.create_publisher(
            ThermalAlert, "/coin417rg2_thermal/hot_spot", 10
        )

    def image_callback(self, msg):
        self.bridge = CvBridge()
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(cv_image)
        image_center_x = cv_image.shape[1] / 2
        image_center_y = cv_image.shape[0] / 2
        thermal_alert_temp = ThermalAlert()
        thermal_alert_temp.x = float(max_loc[0] - image_center_x)
        thermal_alert_temp.y = float(max_loc[1] - image_center_y)
        thermal_alert_temp.temperature = max_val
        self.hot_spot_publisher.publish(thermal_alert_temp)
        cv2.circle(
            cv_image,
            max_loc,
            10,
            (0, 255, 0),
            2,
        )
        cv2.imshow("thermal_image", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    simulator_thermal = SimulatorThermal()
    rclpy.spin(simulator_thermal)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
