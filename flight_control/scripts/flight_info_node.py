#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from flight_control_py.flight.flight_controller_info import FlightInfo


class FlightInfoNode(Node):
    def __init__(self):
        super().__init__("flight_info_node")
        self.declare_parameter("simulation", False)
        self.flight_info = FlightInfo(self)


def main():
    if not rclpy.ok():
        rclpy.init()
    flight_info_node = FlightInfoNode()
    rclpy.spin(flight_info_node)
    flight_info_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
