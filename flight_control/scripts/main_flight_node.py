#!/usr/bin/python3
import time
import rclpy
from rclpy.node import Node
import threading
from flight_control_py.flight.base_control import BaseControl as FlightControl
from flight_control_py.flight.flight_controller_info import FlightInfo
from flight_control_py.flight.mission import Mission
from thermal_msgs.msg import ThermalAlert


class MainFlightNode(Node):

    # flow mode
    STOP_FLOW = 0
    FLOW1_FLOW = 1
    TEST_FLOW = 2

    # incoming message
    thermal_alert_msg = ThermalAlert()

    def __init__(self):
        super().__init__("main_flight_node")
        # --------------------------------ros2 Parameters -------------------------------- #
        self.declare_parameter("simulation", False)
        self.declare_parameter("bcn_orient_yaw", 0.0)
        self.declare_parameter("config_file", "")

        # 創建控制實例
        self.controller = FlightControl(self)
        self.flight_info = FlightInfo(self)
        self.mission = Mission(self.controller, self.flight_info, self)
        # 接收火源警報
        self.create_subscription(
            ThermalAlert,
            "/thermal_alert",
            self.thermalAlertCallback,
            rclpy.qos.qos_profile_sensor_data,
        )

        # --------------------------------- 控制飛行流程偵測器 -------------------------------- #
        self.flow_thread = threading.Thread()  # 建立空執行緒
        self.flow_mode = self.STOP_FLOW  # 暫時使用的流程模式
        self.create_timer(1, self.mainDetectCallback)

    # ---------------------------------------------------------------------------- #
    #                                   main loop                                  #
    # ---------------------------------------------------------------------------- #
    def mainDetectCallback(self):
        """
        此為主要的循環
        持續的偵測流程模式，並切換流程
        """

        def flowSwitch(target, target_name):
            """
            detectCallback()的子函數，用來切換流程，並確保只有一個流程在執行
            """
            if not self.flow_thread.is_alive():
                self.flow_thread = threading.Thread(target=target, name=target_name)
                self.flow_thread.start()
                return
            if self.flow_thread.is_alive() and self.flow_thread.name == target_name:
                self.get_logger().info(f"{target_name} is running")
                return
            if self.flow_thread.is_alive() and self.flow_thread.name != target_name:
                self.mission.stopMission()
                self.flow_thread = threading.Thread(target=target, name=target_name)
                self.flow_thread.start()
                return

        # 判斷使用哪個流程
        if self.flow_mode == self.STOP_FLOW:
            self.mission.stopMission()
            self.get_logger().info("stop flow")
            return
        elif self.flow_mode == self.FLOW1_FLOW:
            flowSwitch(self.flow1, "flow1")
            return
        elif self.flow_mode == self.TEST_FLOW:
            flowSwitch(self.testFlow, "testFlow")
            return

    # ---------------------------------------------------------------------------- #
    #                                  detect rule                                 #
    # ---------------------------------------------------------------------------- #
    def thermalAlertCallback(self, msg):
        """
        火源警報的callback函數
        """
        # self.get_logger().info(f"thermal alert: {msg}")
        # 如果溫度小於60度，則不執行
        if msg.temperature < 60:
            return
        self.thermal_alert_msg = msg
        self.flow_mode = self.FLOW1_FLOW

    # ---------------------------------------------------------------------------- #
    #                                     Flow                                     #
    # ---------------------------------------------------------------------------- #

    def testFlow(self):
        if not self.controller.setMode():
            self.get_logger().info("setMode fail")
            self.flow_mode = self.STOP_FLOW
            return
        time.sleep(4)
        self.get_logger().info("takeoff")
        if not self.mission.simpleTakeoff():
            self.get_logger().info("takeoff fail")
            self.flow_mode = self.STOP_FLOW
            return
        self.get_logger().info("fire distinguish")
        if not self.mission.fireDistinguish():
            self.get_logger().info("fire distinguish fail")
            # self.flow_mode = self.STOP_FLOW
            # return
        if not self.mission.simpleLanding():
            self.get_logger().info("fire distinguish fail")
            self.flow_mode = self.STOP_FLOW
            return
        self.flow_mode = self.STOP_FLOW

    def flow1(self):
        """
        滅火流程
        起飛 --> 飛到指定位置 --> 飛回原點 --> 降落
        """
        if not self.controller.setMode():
            self.get_logger().info("setMode fail")
            self.flow_mode = self.STOP_FLOW
            return
        time.sleep(4)
        self.get_logger().info("takeoff")
        if not self.mission.simpleTakeoff():
            self.get_logger().info("takeoff fail")
            self.flow_mode = self.STOP_FLOW
            return
        self.get_logger().info("navigateTo fire")
        if not self.mission.navigateTo(
            self.thermal_alert_msg.x, self.thermal_alert_msg.y, 2
        ):
            self.get_logger().info("navigateTo fail")
            self.flow_mode = self.STOP_FLOW
            return
        self.get_logger().info("fire distinguish")
        if not self.mission.fireDistinguish():
            self.get_logger().info("fire distinguish fail")
            # self.flow_mode = self.STOP_FLOW
            # return
        self.get_logger().info("navigateTo home")
        if not self.mission.navigateTo(6.87, 0.84, 2):
            self.get_logger().info("navigateTo fail")
            self.flow_mode = self.STOP_FLOW
            return
        self.get_logger().info("landedOnPlatform")
        if not self.mission.landedOnPlatform():
            self.get_logger().info("landedOnPlatform fail")
            self.flow_mode = self.STOP_FLOW
            return
        self.flow_mode = self.STOP_FLOW


def main():
    if not rclpy.ok():
        rclpy.init()
    flight_node = MainFlightNode()
    flight_node.flow_mode = flight_node.STOP_FLOW
    rclpy.spin(flight_node)
    flight_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
