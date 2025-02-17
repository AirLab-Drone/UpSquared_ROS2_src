#!/usr/bin/python3
import time
import rclpy
from rclpy.node import Node
import threading
from flight_control_py.flight.base_control import BaseControl as FlightControl
from flight_control_py.flight.flight_controller_info import FlightInfo
from flight_control_py.flight.mission import Mission
from thermal_msgs.msg import ThermalAlert
from flight_control_py.tool.get_yaml_config import get_yaml_config
import math


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
        self.declare_parameter("config_file", "")  # 這是讀取aruco marker的設定檔
        self.declare_parameter("base_position_config", "")  # 這是讀取基地位置的設定檔

        # 創建控制實例
        self.controller = FlightControl(self)
        self.flight_info = FlightInfo(self)
        self.mission = Mission(self.controller, self.flight_info, self)
        # 讀取設定檔
        base_position_config_path = (
            self.get_parameter("base_position_config")
            .get_parameter_value()
            .string_value
        )
        if base_position_config_path == "":
            self.get_logger().error("Please set base_position_config parameter")
            raise Exception("Configuration file not set")
        self.base_position_config = get_yaml_config(
            config_file_path=base_position_config_path
        )
        # -------------------------------- subscriber -------------------------------- #
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
        if not self.mission.simpleTakeoff(3):
            self.get_logger().info("takeoff fail")
            self.flow_mode = self.STOP_FLOW
            return
        if not self.mission.verticalFlightMission(1.5):
            self.get_logger().info("go down fail")
            self.flow_mode = self.STOP_FLOW
            return
        time.sleep(3)
        if not self.mission.verticalFlightMission(3.0):
            self.get_logger().info("go up fail")
            self.flow_mode = self.STOP_FLOW
            return
        if not self.mission.simpleLanding():
            self.get_logger().info("landing fail")
            self.flow_mode = self.STOP_FLOW
            return
        self.flow_mode = self.STOP_FLOW

    def flow1(self):
        """
        滅火流程
        起飛 --> 飛到指定位置 --> 飛回原點 --> 降落
        """
        try:
            if not self.controller.setMode():
                self.get_logger().info("setMode fail")
                self.flow_mode = self.STOP_FLOW
                return
            time.sleep(4)
            # ----------------------------- prepare takeoff ----------------------------- #
            # self.get_logger().info("loading extinguisher")
            # if not self.mission.loadingExtinguisher():
            #     self.get_logger().info("loading extinguisher fail")
            #     self.flow_mode = self.STOP_FLOW
            #     return
            # self.get_logger().info("prepare takeoff")
            # if not self.mission.prepareTakeoff():
            #     self.get_logger().info("prepareTakeoff fail")
            #     self.flow_mode = self.STOP_FLOW
            #     return
            self.get_logger().info("takeoff")
            if not self.mission.simpleTakeoff():
                self.get_logger().info("takeoff fail")
                self.flow_mode = self.STOP_FLOW
                return
            # -------------------------------- fight fire -------------------------------- #
            self.get_logger().info("navigateTo fire")
            #起飛後先設置平台降落狀態，關閉遮板不等待
            # if not self.mission.prepareLandingNoWait():
            #     self.get_logger().info("prepareLandingNoWait fail")
            #     self.flow_mode = self.STOP_FLOW
            #     return
            if not self.mission.navigateTo(
                self.thermal_alert_msg.x, self.thermal_alert_msg.y, 3.0
            ):
                self.get_logger().info("navigateTo fail")
                self.flow_mode = self.STOP_FLOW
                return
            self.get_logger().info("fire distinguish")
            if not self.mission.fireDistinguish():
                self.get_logger().info("fire distinguish fail")
                # self.flow_mode = self.STOP_FLOW
                # return
            # ----------------------------- throw extinguish ----------------------------- #
            self.get_logger().info("throw extinguisher")
            throwing_position = [
                self.base_position_config["throwing"]["x"],
                self.base_position_config["throwing"]["y"],
            ]
            # go to throw extinguisher position
            if not self.mission.navigateTo(
                throwing_position[0], throwing_position[1], 3.0
            ):
                self.get_logger().info("navigateTo fail")
                self.flow_mode = self.STOP_FLOW
                return
            # down to throw extinguisher
            if not self.mission.verticalFlightMission(1.5):
                self.get_logger().info("go down fail")
                self.flow_mode = self.STOP_FLOW
                return
            time.sleep(3) # 等待3秒在丟滅火器
            # # throw extinguisher
            # if not self.mission.throwingExtinguisher():
            #     self.get_logger().info("throw extinguisher fail")
            #     self.flow_mode = self.STOP_FLOW
            #     return
            # go up
            if not self.mission.verticalFlightMission(3.0):
                self.get_logger().info("go up fail")
                self.flow_mode = self.STOP_FLOW
                return
            # ---------------------------------- go home --------------------------------- #
            self.get_logger().info("navigateTo home")
            home_position = [
                self.base_position_config["home"]["x"],
                self.base_position_config["home"]["y"],
            ]
            if not self.mission.navigateTo(home_position[0], home_position[1], 3.0):
                self.get_logger().info("navigateTo fail")
                self.flow_mode = self.STOP_FLOW
                return
            # self.get_logger().info("prepare landing")
            # if not self.mission.prepareLanding():
            #     self.get_logger().info("prepareLanding fail")
            #     self.flow_mode = self.STOP_FLOW
            #     return
            self.get_logger().info("landedOnPlatform")
            if not self.mission.landedOnPlatform():
                self.get_logger().info("landedOnPlatform fail")
                self.flow_mode = self.STOP_FLOW
                return
            # #! 不要重複對齊
            # time.sleep(5)
            # self.get_logger().info("align drone")
            # if not self.mission.platformAlignment():
            #     self.get_logger().info("align drone fail")
            #     self.flow_mode = self.STOP_FLOW
            #     return

        except Exception as e:
            self.get_logger().info(f"flow1 error: {e}")
            self.flow_mode = self.STOP_FLOW
        self.flow_mode = self.STOP_FLOW


def main():
    if not rclpy.ok():
        rclpy.init()
    flight_node = MainFlightNode()
    flight_node.flow_mode = flight_node.TEST_FLOW
    rclpy.spin(flight_node)
    flight_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
