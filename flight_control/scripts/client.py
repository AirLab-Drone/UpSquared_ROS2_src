#!/usr/bin/env python3

from rclpy.node import Node
import rclpy
from flight_control.srv import GetCloestAruco

class Client(Node):
    def __init__(self):
        super().__init__('client')
        self.cli = self.create_client(GetCloestAruco, 'test')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetCloestAruco.Request()
        

    def send_request(self):
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = Client()
    client.send_request()
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()