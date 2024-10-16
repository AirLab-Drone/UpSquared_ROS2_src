#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from payload.srv import Spry
import time
import gpio

class FireExtinguisher(Node):
    def __init__(self):
        super().__init__('fire_extinguisher')
        self.srv = self.create_service(Spry, 'spry', self.spry_callback)
        # ------------------------------ gpio pin setup ------------------------------ #
        self.spry_pin = gpio.GPIOPin(14, gpio.OUT)

        
    def spry_callback(self, request, response):
        self.get_logger().info('fire extinguisher service is called')
        try:
            self.spry_pin.write(gpio.HIGH)
            time.sleep(2)
            self.spry_pin.write(gpio.LOW)
            response.success = True
        except Exception as e:
            self.get_logger().info(f'error: {e}')
            response.success = False
        self.get_logger().info(f"response: {response}")
        return response

def main(args=None):
    rclpy.init(args=args)

    fire_extinguisher = FireExtinguisher()

    rclpy.spin(fire_extinguisher)

    rclpy.shutdown()

if __name__ == '__main__':
    main()