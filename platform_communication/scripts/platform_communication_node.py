#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time
from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from pymodbus.exceptions import ModbusIOException, ConnectionException
from platform_communication.srv import (
    AlignmentRod,
    PerforatedPlate,
    MovetoChargeTank,
    MovetoExtinguisher,
    VerticalSlider,
    MainsPower,
    CheckTankStatus,
)


class PlatformCommunicationNode(Node):
    def __init__(self):
        super().__init__("platform_communication_node")
        # ------------------------------ define service ------------------------------ #
        # x y 滑桿
        self.create_service(
            AlignmentRod,
            "/platform_communication/alignment_rod",
            self.alignment_rod_callback,
        )
        # 開孔板
        self.create_service(
            PerforatedPlate,
            "/platform_communication/perforated_plate",
            self.perforated_plate_callback,
        )
        # 料筒移動
        self.create_service(
            MovetoChargeTank,
            "/platform_communication/moveto_charge_tank",
            self.moveto_charge_tank_callback,
        )
        self.create_service(
            MovetoExtinguisher,
            "/platform_communication/moveto_extinguisher",
            self.moveto_extinguisher_callback,
        )
        # vertical slider
        self.create_service(
            VerticalSlider,
            "/platform_communication/vertical_slider",
            self.vertical_slider_callback,
        )
        # 市電供電
        self.create_service(
            MainsPower, "/platform_communication/mains_power", self.mains_power_callback
        )
        # 檢查料筒位置
        self.create_service(
            CheckTankStatus,
            "/platform_communication/check_tank_status",
            self.check_tank_status_callback,
        )
        # ------------------------------ io pin setup ------------------------------ #
        self.UNIT = 0x01
        self.client = ModbusClient(
            host="192.168.112.7",
            port=502,
            timeout=3,
        )
        # address
        self.open_alignment_rod_addr = 1500
        self.close_alignment_rod_addr = 1501
        self.open_perforated_plate_addr = 1520
        self.close_perforated_plate_addr = 1521
        self.moveto_charge_tank_addr = 1506
        self.moveto_extinguisher_addr = 1507
        self.rise_vertical_slider_addr = 1509
        self.drop_vertical_slider_addr = 1511
        self.open_mains_power_addr = 1512
        self.close_mains_power_addr = 1513
        self.check_tank_status_addr = 1600
        self.finish_status_addr = 1620
        # check connection
        while not self.client.connect():
            self.get_logger().info("connecting...")
            time.sleep(0.5)
        print("connected ^_^\n", self.client)
    # ---------------------------------------------------------------------------- #
    #                                   callback                                   #
    # ---------------------------------------------------------------------------- #
    def alignment_rod_callback(self, request, response):
        self.get_logger().info("alignment rod service is called")
        for i in range(2):
            response.success = True
            try:
                if request.open:
                    result = self.client.write_coil(
                        self.open_alignment_rod_addr, True, unit=self.UNIT
                    )
                    if result.isError():
                        response.success = False
                else:
                    result = self.client.write_coil(
                        self.close_alignment_rod_addr, True, unit=self.UNIT
                    )
                    if result.isError():
                        response.success = False
                break
            except ConnectionException as e:
                self.get_logger().error(f"Connection error: {e}")
                response.success = False
            except Exception as e:
                self.get_logger().error(f"error: {str(e)}")
                response.success = False
                break
        result = self.client.read_coils(self.open_alignment_rod_addr,2)
        self.get_logger().info(f'result: {result.bits}')
        self.get_logger().info(f"response: {response}")
        if response.success:
            response.success = self.wait_finish()
        return response

    def perforated_plate_callback(self, request, response):
        self.get_logger().info("perforated plate service is called")
        for i in range(2):
            response.success = True
            try:
                if request.open:
                    result = self.client.write_coil(self.open_perforated_plate_addr, True)
                    if result.isError():
                        response.success = False
                else:
                    result = self.client.write_coil(self.close_perforated_plate_addr, True)
                    if result.isError():
                        response.success = False
                break
            except ConnectionException as e:
                self.get_logger().error(f"Connection error: {e}")
                response.success = False
            except Exception as e:
                self.get_logger().error(f"error: {str(e)}")
                response.success = False
                break
        self.get_logger().info(f"response: {response}")
        if response.success:
            response.success = self.wait_finish()
        return response

    def moveto_charge_tank_callback(self, request, response):
        self.get_logger().info("moveto charge tank service is called")
        for i in range(2):
            response.success = True
            try:
                result = self.client.write_coil(
                    self.moveto_charge_tank_addr, True, unit=self.UNIT
                )
                if result.isError():
                    response.success = False
                break
            except ConnectionException as e:
                self.get_logger().error(f"Connection error: {e}")
                response.success = False
            except Exception as e:
                self.get_logger().error(f"error: {str(e)}")
                response.success = False
                break
        self.get_logger().info(f"response: {response}")
        if response.success:
            response.success = self.wait_finish()
        return response

    def moveto_extinguisher_callback(self, request, response):
        if request.num not in [1,2]:
            self.get_logger().info("out of extinguish range")
            response.success = False
            return response
        for i in range(2):
            response.success = True
            self.get_logger().info("moveto extinguisher service is called")
            try:
                result = self.client.write_coil(
                    self.moveto_extinguisher_addr + request.num -1, True, unit=self.UNIT
                )
                if result.isError():
                    response.success = False
                break
            except ConnectionException as e:
                self.get_logger().error(f"Connection error: {e}")
                response.success = False
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")
                response.success = False
                break
        self.get_logger().info(f"response: {response}")
        if response.success:
            response.success = self.wait_finish()
        return response

    def vertical_slider_callback(self, request, response):
        self.get_logger().info("vertical slider service is called")
        for i in range(2):
            response.success = True
            try:
                if request.up:
                    result = self.client.write_coil(
                        self.rise_vertical_slider_addr, True, unit=self.UNIT
                    )
                    if result.isError():
                        response.success = False
                else:
                    result = self.client.write_coil(
                        self.drop_vertical_slider_addr, True, unit=self.UNIT
                    )
                    if result.isError():
                        response.success = False
                break
            except ConnectionException as e:
                self.get_logger().error(f"Connection error: {e}")
                response.success = False
            except Exception as e:
                self.get_logger().error(f"error: {str(e)}")
                response.success = False
                break
        self.get_logger().info(f"response: {response}")
        if response.success:
            response.success = self.wait_finish()
        return response

    def mains_power_callback(self, request, response):
        self.get_logger().info("mains power service is called")
        for i in range(2):
            response.success = True
            try:
                if request.open:
                    result = self.client.write_coil(
                        self.open_mains_power_addr, True, unit=self.UNIT
                    )
                    if result.isError():
                        response.success = False
                else:
                    result = self.client.write_coil(
                        self.close_mains_power_addr, True, unit=self.UNIT
                    )
                    if result.isError():
                        response.success = False
                break
            except ConnectionException as e:
                self.get_logger().error(f"Connection error: {e}")
                response.success = False
            except Exception as e:
                self.get_logger().error(f"error: {str(e)}")
                response.success = False
                break
        self.get_logger().info(f"response: {response}")
        if response.success:
            response.success = self.wait_finish()
        return response

    def check_tank_status_callback(self, request, response):
        self.get_logger().info("check tank status service is called")
        for i in range(2):
            response.success = True
            try:
                result = self.client.read_coils(
                    self.check_tank_status_addr, 3
                )
                if result.isError():
                    self.get_logger().error("ModbusIOException")
                    response.success = False
                    return response
                self.get_logger().info(str(result.bits))
                # 判斷現在在哪一個桶位
                index = -1
                for i in range(len(result.bits)):
                    if result.bits[i] == 1:
                        index = i
                        break
                response.num = index
                break
            except ConnectionException as e:
                self.get_logger().error(f"Connection error: {e}")
                response.success = False
            except Exception as e:
                self.get_logger().error(f"error: {str(e)}")
                response.success = False
                break
        return response
    # ---------------------------------------------------------------------------- #
    #                                   function                                   #
    # ---------------------------------------------------------------------------- #
    def check_finish_status(self):
        try:
            result = self.client.read_coils(self.finish_status_addr, 1)
            if result.isError():
                self.get_logger().error("ModbusIOException")
                return False
            return result.bits[0]
        except Exception as e:
            self.get_logger().error(f"error: {str(e)}")
            return False
    def wait_finish(self, timeout=20):
        start_time = rclpy.clock.Clock().now()
        while rclpy.clock.Clock().now() - start_time < rclpy.time.Duration(seconds=timeout):
            if self.check_finish_status():
                return True
            time.sleep(0.1)
        self.get_logger().info('wait platform status is timeout')
        return False


def main(args=None):
    rclpy.init(args=args)

    platform_communication_node = PlatformCommunicationNode()

    rclpy.spin(platform_communication_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
