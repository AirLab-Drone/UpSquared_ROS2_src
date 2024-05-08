#!/usr/bin/python3
import time
import rclpy
from rclpy.node import Node
import threading
from flight_control_py.flight.base_control import BaseControl as FlightControl
from flight_control_py.flight.flight_controller_info import FlightInfo
from flight_control_py.flight.mission import Mission


class MainFlightNode(Node):

    def __init__(self):
        super().__init__("main_flight_node")
        self.controller = FlightControl(self)
        self.flight_info = FlightInfo(self)
        self.mission = Mission(self.controller, self.flight_info, self)

    def wait(self):
        self.controller.setZeroVelocity()
        time.sleep(5)

    def takeOff(self, alt=2):
        while not self.controller.armAndTakeoff(alt=alt):
            print("armAndTakeoff fail")

    def arucoLandingTest(self):
        # while not self.controller.armAndTakeoff(alt=2):  # Takeoff
        #     print("armAndTakeoff fail")
        # time.sleep(5)
        self.mission.setMode(self.mission.LANDIND_ON_PLATFORM)
        threading.Thread(target=self.mission.landedOnPlatform).start()

    def goToFireTest(self):
        # while not self.controller.armAndTakeoff(alt=2):
        #     print("armAndTakeoff fail")
        # time.sleep(5)
        self.mission.setMode(self.mission.NAVIGATION)
        threading.Thread(target=self.mission.navigateTo, args=[0, 0, 0, 0]).start()


def main():
    if not rclpy.ok():
        rclpy.init()
    flight_node = MainFlightNode()
    # flight_node.arucoLandingTest()
    flight_node.goToFireTest()
    print("main_flight_node is running")
    rclpy.spin(flight_node)
    flight_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
