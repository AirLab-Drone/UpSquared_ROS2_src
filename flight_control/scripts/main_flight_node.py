#!/usr/bin/python3
import time
import rclpy
from rclpy.node import Node
import threading
from flight_control_py.flight.base_control import BaseControl as FlightControl
from flight_control_py.flight.flight_controller_info import FlightInfo
from flight_control_py.flight.mission import Mission


class MainFlightNode(Node):

    # flow mode
    STOP_FLOW = 0
    FLOW1_FLOW = 1

    def __init__(self):
        super().__init__("main_flight_node")
        self.controller = FlightControl(self)
        self.flight_info = FlightInfo(self)
        self.mission = Mission(self.controller, self.flight_info, self)
        # --------------------------------- 控制飛行流程偵測器 -------------------------------- #
        self.flow_thread = threading.Thread()  # 建立空執行緒
        self.flow_mode = self.STOP_FLOW  # 暫時使用的流程模式
        self.create_timer(1, self.detectCallback)

    def detectCallback(self):
        '''
        持續的偵測流程模式，並切換流程
        '''
        def flowSwitch(target, target_name):
            '''
            detectCallback()的子函數，用來切換流程，並確保只有一個流程在執行
            '''
            if not self.flow_thread.is_alive():
                self.flow_thread = threading.Thread(
                    target=target, name=target_name
                ).start()
                return
            if self.flow_thread.is_alive() and self.flow_thread.name == target_name:
                print(f"{target_name} is running")
                return
            if self.flow_thread.is_alive() and self.flow_thread.name != target_name:
                self.mission.stopMission()
                self.flow_thread = threading.Thread(
                    target=target, name=target_name
                ).start()
                return
        # 判斷使用哪個流程
        if self.flow_mode == self.STOP_FLOW:
            self.mission.stopMission()
            return
        elif self.flow_mode == self.FLOW1_FLOW:
            flowSwitch(self.flow1, "flow1")
            return

    def arucoLandingTest(self):
        self.mission.simpleTakeoff()
        self.mission.landedOnPlatform()

    def goToFireTest(self):
        while not self.controller.armAndTakeoff(alt=2):
            print("armAndTakeoff fail")
        time.sleep(5)
        threading.Thread(
            target=self.mission.navigateTo,
            args=[13.054284387619752, 12.375995335838226, 0, 0],
        ).start()

    def flow1(self):
        '''
        起飛 --> 飛到指定位置 --> 降落
        '''
        is_done = self.mission.simpleTakeoff()
        if not is_done:
            print("takeoff fail")
            return
        is_done = self.mission.navigateTo(13.054284387619752, 12.375995335838226, 0, 0)
        if not is_done:
            print("navigateTo fail")
            return
        is_done = self.mission.simpleLanding()
        self.flow_mode = self.STOP_FLOW


def main():
    if not rclpy.ok():
        rclpy.init()
    flight_node = MainFlightNode()
    flight_node.flow_mode = flight_node.FLOW1_FLOW #設定執行flow1
    rclpy.spin(flight_node)
    flight_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
