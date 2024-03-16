#!/usr/bin/python3
import time
import rclpy
from rclpy.node import Node
from flight_control_py.flight.base_control import BaseControl as FlightControl
from flight_control_py.flight.flight_controller_info import FlightInfo
from flight_control_py.flight.mission import Mission


class MainFlightNode(Node):
    def __init__(self):
        super().__init__("main_flight_node")

    def wait(self, controller):
        controller.setZeroVelocity()
        time.sleep(5)

    def velocityTest(self):
        rclpy.init()
        node = rclpy.create_node("flight_control")
        flightController = flight_control.FlightControl(node)
        time.sleep(5)
        isTakeoffSuccess = False
        while isTakeoffSuccess == False:
            isTakeoffSuccess = flightController.armAndTakeoff()
        # flightController.simpleFlight(1.0, 0.0, 0.0, 5)
        # wait(flightController)
        flightController.simpleFlight(-1.0, 0.0, 0.0, 5)
        wait(flightController)
        # flightController.simpleFlight(0.0, 0.5, 0.0, 5)
        # wait(flightController)
        # flightController.simpleFlight(0.0, -0.5, 0.0, 5)
        # wait(flightController)
        isLandSuccess = False
        while isLandSuccess == False:
            isLandSuccess = flightController.land()
        flightController.destroy()

    def arucoLandingTest(self):
        # flight_info_node = rclpy.create_node("flight_info")
        controller = FlightControl(self)
        flight_info = FlightInfo(self)
        while not controller.armAndTakeoff(alt=2):
            print("armAndTakeoff fail")
        time.sleep(5)
        mission = Mission(controller, flight_info,self)
        mission.landedOnPlatform()
        controller.destroy()


def main():
    if not rclpy.ok():
        rclpy.init()
    flight_node = MainFlightNode()
    flight_node.arucoLandingTest()
    rclpy.spin(flight_node)
    flight_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
