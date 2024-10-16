#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from thermal_msgs.msg import ThermalAlert
from sensor_msgs.msg import Range
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import math


class ThermalFrameToDroneFrame(Node):

    def __init__(self):
        super().__init__("thermal_frame_to_drone_frame")
        # -------------------------------- subscriptions -------------------------------- #
        self.subscription_thermal = self.create_subscription(
            ThermalAlert,
            "/coin417rg2_thermal/hot_spot",
            self.hot_spot_pose_callback,
            10,
        )
        self.subscription_range = self.create_subscription(
            Range,
            "/mavros/rangefinder_pub",
            self.rangefinder_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        # -------------------------------- publishers -------------------------------- #
        self.hot_spot_drone_frame_pub = self.create_publisher(
            ThermalAlert, "/coin417rg2_thermal/hot_spot_drone_frame", 10
        )
        # --------------------------------- variables -------------------------------- #
        # hot spot in thermal frame
        self.x_pixel = 0
        self.y_pixel = 0
        self.temperature = 0.0
        # rangefinder altitude
        self.rangefinder_alt = 0.0
        # thermal camera parameters
        self.config = self.get_yaml_config("coin417rg2_thermal", "thermal_camera.yaml")

    def hot_spot_pose_callback(self, msg):
        self.x_pixel = msg.x
        self.y_pixel = msg.y
        self.temperature = msg.temperature
        hot_spot_drone_frame = ThermalAlert()
        # convert pixel to meter
        hot_spot_drone_frame.x, hot_spot_drone_frame.y = self.pixel_to_meter(
            self.x_pixel, self.y_pixel
        )
        # rotate and offset the thermal coordinate to drone coordinate
        hot_spot_drone_frame.x, hot_spot_drone_frame.y = (
            self.rotate_and_offset_thermal_coordinate(
                hot_spot_drone_frame.x, hot_spot_drone_frame.y
            )
        )
        hot_spot_drone_frame.temperature = self.temperature
        self.get_logger().info(f"temperature: {hot_spot_drone_frame.temperature}")
        self.hot_spot_drone_frame_pub.publish(hot_spot_drone_frame)

    def rangefinder_callback(self, msg):
        self.rangefinder_alt = msg.range

    def get_yaml_config(self, package_name, config_file_name):
        package_share_directory = get_package_share_directory(package_name)
        config_file_path = os.path.join(
            package_share_directory, "config", config_file_name
        )

        if not os.path.exists(config_file_path):
            self.get_logger().error(
                f"Cannot load Aruco markers config file: {config_file_path}"
            )
            return None

        with open(config_file_path, "r") as file:
            config = yaml.safe_load(file)
        return config

    def pixel_to_meter(self, x_pixel, y_pixel):
        """
        @param x_pixel: x pixel in thermal image
        @param y_pixel: y pixel in thermal image
        @return: x, y in meter
        """
        self.thermal_fov = self.config["thermal_camera_params"]["fov"]
        self.thermal_image_width = self.config["thermal_camera_params"]["width"]
        fov = math.radians(self.thermal_fov)
        image_width = self.thermal_image_width
        meter_per_pixel = 2 * self.rangefinder_alt * math.tan(fov / 2) / image_width
        return x_pixel * meter_per_pixel, y_pixel * meter_per_pixel

    def rotate_and_offset_thermal_coordinate(self, x_meter, y_meter):
        """
        @param x_meter: x pixel in thermal image
        @param y_meter: y pixel in thermal image
        @return: x, y in meter, the same unit as input
        """
        rotate_deg = math.radians(self.config["thermal_camera_pose"]["yaw"])
        offset_x = self.config["thermal_camera_pose"]["x"]
        offset_y = self.config["thermal_camera_pose"]["y"]
        temp_x = x_meter
        temp_y = y_meter
        x_meter = temp_x * math.cos(rotate_deg) + temp_y * math.sin(rotate_deg)
        y_meter = -temp_x * math.sin(rotate_deg) + temp_y * math.cos(rotate_deg)
        x_meter += offset_x
        y_meter += offset_y
        return x_meter, y_meter


def main(args=None):
    rclpy.init(args=args)
    node = ThermalFrameToDroneFrame()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
