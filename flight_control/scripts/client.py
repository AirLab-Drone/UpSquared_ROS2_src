#!/usr/bin/env python3

from rclpy.node import Node
import rclpy
from flight_control.srv import GetCloestAruco

class Client(Node):
    def __init__(self):
        super().__init__('client')
        self.cli = self.create_client(GetCloestAruco, 'get_cloest_aruco')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetCloestAruco.Request()
        

    def send_request(self):
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    if(not rclpy.ok()):
        rclpy.init()
    start_time = rclpy.clock.Clock().now()
    client = Client()
    for i in range(1):
        result = client.send_request()
    print(f'during time: {(rclpy.clock.Clock().now() - start_time).nanoseconds/1e9}s')
    print(f'cloest aruco: {result.aruco}')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()