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
        super().__init__('thermal_frame_to_drone_frame')
        # -------------------------------- subscriptions -------------------------------- #
        self.subscription_thermal = self.create_subscription(
            ThermalAlert,
            '/coin417rg2_thermal/hot_spot',
            self.hot_spot_pose_callback,
            10)
        self.subscription_range = self.create_subscription(
            Range,
            '/mavros/rangefinder_pub',
            self.rangefinder_callback,
            rclpy.qos.qos_profile_sensor_data)
        # -------------------------------- publishers -------------------------------- #
        self.hot_spot_drone_frame_pub = self.create_publisher(ThermalAlert, '/coin417rg2_thermal/hot_spot_drone_frame', 10)
        # --------------------------------- variables -------------------------------- #
        # hot spot in thermal frame
        self.x_pixel = 0
        self.y_pixel = 0
        self.temperature = 0.0
        # rangefinder altitude
        self.rangefinder_alt = 0.0
        # thermal camera parameters
        config = self.get_yaml_config('coin417rg2_thermal', 'thermal_camera.yaml')
        self.thermal_fov = config['thermal_camera_params']['fov']
        self.thermal_image_width = config['thermal_camera_params']['width']

    def hot_spot_pose_callback(self, msg):
        meter_per_pixel = self.compute_meter_per_pixel()
        if meter_per_pixel == 0.0: #如果還沒有算出meter_per_pixel，就不要publish
            return

        self.x_pixel = msg.x
        self.y_pixel = msg.y
        self.temperature = msg.temperature
        hot_spot_drone_frame = ThermalAlert()
        hot_spot_drone_frame.x = self.x_pixel * meter_per_pixel
        hot_spot_drone_frame.y = self.y_pixel * meter_per_pixel
        hot_spot_drone_frame.temperature = self.temperature
        self.get_logger().info(f'temperature: {hot_spot_drone_frame.temperature}')
        self.hot_spot_drone_frame_pub.publish(hot_spot_drone_frame)

    def rangefinder_callback(self, msg):
        self.rangefinder_alt = msg.range

    def get_yaml_config(self, package_name, config_file_name):
        package_share_directory = get_package_share_directory(package_name)
        config_file_path = os.path.join(package_share_directory, 'config', config_file_name)

        if not os.path.exists(config_file_path):
            self.get_logger().error(f'Cannot load Aruco markers config file: {config_file_path}')
            return None

        with open(config_file_path, 'r') as file:
            config = yaml.safe_load(file)
        return config

    def compute_meter_per_pixel(self):
        fov = math.radians(self.thermal_fov)
        image_width = self.thermal_image_width
        return 2 * self.rangefinder_alt * math.tan(fov / 2) / image_width

def main(args=None):
    rclpy.init(args=args)
    node = ThermalFrameToDroneFrame()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()