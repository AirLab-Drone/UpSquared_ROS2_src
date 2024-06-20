#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from flight_control_py.flight.flight_controller_info import FlightInfo


class FlightInfoNode(Node):
    def __init__(self):
        super().__init__("flight_info_node")
        self.declare_parameter("simulation", False)
        self.flight_info = FlightInfo(self)
        self.create_timer(0.1, self.mainDetectCallback)
    
    def mainDetectCallback(self):
        print(f'UWB coordinate: x={self.flight_info.uwb_coordinate.x}, y={self.flight_info.uwb_coordinate.y}, z={self.flight_info.uwb_coordinate.z}')
        print(f'rangefinder distance: {self.flight_info.rangefinder_alt}')


def main():
    if not rclpy.ok():
        rclpy.init()
    flight_info_node = FlightInfoNode()
    rclpy.spin(flight_info_node)
    flight_info_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
