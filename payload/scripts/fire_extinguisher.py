#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from payload.srv import Spry, HoldPayload, CheckPayload
import time
from periphery import GPIO


class FireExtinguisher(Node):
    def __init__(self):
        super().__init__("fire_extinguisher")
        self.srv = self.create_service(Spry, "/fire_extinguisher/spry", self.spry_callback)
        self.srv = self.create_service(HoldPayload, "/fire_extinguisher/hold_fire_extinguisher", self.hold_fire_extinguisher)
        self.srv = self.create_service(CheckPayload, "/fire_extinguisher/check_fire_extinguisher", self.check_fire_extinguisher)
        # ------------------------------ gpio pin setup ------------------------------ #
        self.spry_pin = GPIO(23, "out")
        self.hold_pin = GPIO(24, "out")
        self.check_write_pin = GPIO(22, "out")
        self.check_read_pin = GPIO(27, "in")

    def spry_callback(self, request, response):
        self.get_logger().info("fire extinguisher service is called")
        try:
            self.spry_pin.write(True)
            time.sleep(2)
            self.spry_pin.write(False)
            response.success = True
        except Exception as e:
            self.get_logger().info(f"error: {e}")
            response.success = False
        self.get_logger().info(f"response: {response}")
        return response

    def hold_fire_extinguisher(self, request, response):
        self.get_logger().info("HoldFireExtinguisher service is called")
        try:
            if request.hold:
                self.hold_pin.write(False)
            else:
                self.hold_pin.write(True)
            response.success = True
        except Exception as e:
            self.get_logger().info(f"error: {e}")
            response.success = False
        return response

    def check_fire_extinguisher(self, request, response):
        self.get_logger().info("check_fire_extinguisher service is called")
        try:
            self.check_write_pin.write(True)
            time.sleep(0.5)
            response.success = self.check_read_pin.read()
            self.check_write_pin.write(False)
        except Exception as e:
            self.get_logger().info(f"error: {e}")
            response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)

    fire_extinguisher = FireExtinguisher()

    rclpy.spin(fire_extinguisher)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
