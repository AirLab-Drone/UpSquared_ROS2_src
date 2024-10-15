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

        
    def spry_callback(self, request, response):
        try:
            pin = gpio.GPIOPin(14, gpio.OUT)
            pin.write(14, gpio.HIGH)
            time.sleep(2)
            pin.write(14, gpio.LOW)
            response.success = True
        except:
            response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)

    fire_extinguisher = FireExtinguisher()

    rclpy.spin(fire_extinguisher)

    rclpy.shutdown()

if __name__ == '__main__':
    main()