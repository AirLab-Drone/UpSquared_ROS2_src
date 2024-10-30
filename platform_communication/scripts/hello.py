#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class Hello(Node):

    def __init__(self):
        super().__init__('hello')
        self.get_logger().info('Hello World')
    def __del__(self):
        self.get_logger().info('Goodbye World')
        self.destroy_node()
        rclpy.shutdown()
def main(args=None):
    rclpy.init(args=args)
    hello = Hello()
    rclpy.spin(hello)
    del hello

if __name__ == '__main__':
    main()