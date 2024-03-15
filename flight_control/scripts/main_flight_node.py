# Python
import time
import rclpy
import flight_control
from mission import Mission

def wait(controller):
    controller.setZeroVelocity()
    time.sleep(5)

def velocityTest():
    rclpy.init()
    node = rclpy.create_node("flight_control")
    flightController = flight_control.FlightControl(node)
    time.sleep(5)
    isTakeoffSuccess = False
    while(isTakeoffSuccess == False):
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
    while(isLandSuccess == False):
        isLandSuccess = flightController.land()
    flightController.destroy()
def arucoLandingTest():
    rclpy.init()
    flight_controller_node = rclpy.create_node("flight_control")
    # flight_info_node = rclpy.create_node("flight_info")
    controller = flight_control.FlightControl(flight_controller_node)
    flight_info = flight_control.FlightInfo(flight_controller_node)
    while not controller.armAndTakeoff(alt=2):
        print("armAndTakeoff fail")
    time.sleep(5)
    mission = Mission(controller,flight_info)
    mission.landedOnPlatform()
    controller.destroy()
if __name__ == '__main__':
    arucoLandingTest()