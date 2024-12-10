#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from platform_communication.srv import AlignmentRod, PerforatedPlate, MovetoChargeTank, MovetoExtinguisher, VerticalSlider, MainsPower
import time
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusIOException


class PlatformCommunicationNode(Node):
    def __init__(self):
        super().__init__("platform_communication_node")
        # ------------------------------ define service ------------------------------ #
        # x y 滑桿
        self.create_service(
            AlignmentRod, "/platform_communication/alignment_rod", self.alignment_rod_callback
        )
        # 開孔板
        self.create_service(
            PerforatedPlate, "/platform_communication/perforated_plate", self.perforated_plate_callback
        )
        # 補充瓶移動
        self.create_service(
            MovetoChargeTank, "/platform_communication/moveto_charge_tank", self.moveto_charge_tank_callback
        )
        self.create_service(
            MovetoExtinguisher, "/platform_communication/moveto_extinguisher", self.moveto_extinguisher_callback
        )
        # vertical slider
        self.create_service(
            VerticalSlider, "/platform_communication/vertical_slider", self.vertical_slider_callback
        )
        # 市電供電
        self.create_service(
            MainsPower, "/platform_communication/mains_power", self.mains_power_callback
        )
        # ------------------------------ io pin setup ------------------------------ #
        self.UNIT = 0x01
        self.client = ModbusClient(
            method="rtu",  # 通訊模式
            port="/dev/ttyUSB0",  # 串口設備
            baudrate=9600,  # 波特率
            stopbits=2,  # 停止位
            bytesize=8,  # 資料位
            timeout=3,  # 超時時間
        )
        # address
        self.open_alignment_rod_addr = 0x01
        self.close_alignment_rod_addr = 0x02
        self.open_perforated_plate_addr = 0x03
        self.close_perforated_plate_addr = 0x04
        self.moveto_charge_tank_addr = 0x05
        self.moveto_extinguisher_addr = 0x06
        self.rise_vertical_slider_addr = 0x07
        self.drop_vertical_slider_addr = 0x08
        self.open_mains_power_addr = 0x09
        self.close_mains_power_addr = 0x0A
        # check connection
        while not self.client.connect():
            self.get_logger().info("connecting...")
            time.sleep(0.5)

    def alignment_rod_callback(self, request, response):
        self.get_logger().info("alignment rod service is called")
        response.success = True
        try:
            if request.open:
                result = self.client.write_coil(self.open_alignment_rod_addr, True, unit=self.UNIT)
                if result.isError():
                    response.success = False
            else:
                result = self.client.write_coil(self.close_alignment_rod_addr, True, unit=self.UNIT)
                if result.isError():
                    response.success = False
        except Exception as e:
            self.get_logger().error(f"error: {str(e)}")
            response.success = False
        self.get_logger().info(f"response: {response}")
        return response
    def perforated_plate_callback(self, request, response):
        self.get_logger().info("perforated plate service is called")
        response.success = True
        try:
            if request.open:
                result = self.client.write_coil(self.open_perforated_plate_addr, True, unit=self.UNIT)
                if result.isError():
                    response.success = False
            else:
                result = self.client.write_coil(self.close_perforated_plate_addr, True, unit=self.UNIT)
                if result.isError():
                    response.success = False
        except Exception as e:
            self.get_logger().error(f"error: {str(e)}")
            response.success = False
        self.get_logger().info(f"response: {response}")
        return response
    def moveto_charge_tank_callback(self, request, response):
        self.get_logger().info("moveto charge tank service is called")
        response.success = True
        try:
            result = self.client.write_coil(self.moveto_charge_tank_addr, True, unit=self.UNIT)
            if result.isError():
                response.success = False
        except Exception as e:
            self.get_logger().error(f"error: {str(e)}")
            response.success = False
        self.get_logger().info(f"response: {response}")
        return response
    def moveto_extinguisher_callback(self, request, response):
        self.get_logger().info("moveto extinguisher service is called")
        response.success = True
        try:
            result = self.client.write_coil(self.moveto_extinguisher_addr, request.num, unit=self.UNIT)
            if result.isError():
                response.success = False
        except Exception as e:
            self.get_logger().error(f"error: {str(e)}")
            response.success = False
        self.get_logger().info(f"response: {response}")
        return response
    def vertical_slider_callback(self, request, response):
        self.get_logger().info("vertical slider service is called")
        response.success = True
        try:
            if request.up:
                result = self.client.write_coil(self.rise_vertical_slider_addr, True, unit=self.UNIT)
                if result.isError():
                    response.success = False
            else:
                result = self.client.write_coil(self.drop_vertical_slider_addr, True, unit=self.UNIT)
                if result.isError():
                    response.success = False
        except Exception as e:
            self.get_logger().error(f"error: {str(e)}")
            response.success = False
        self.get_logger().info(f"response: {response}")
        return response
    def mains_power_callback(self, request, response):
        self.get_logger().info("mains power service is called")
        response.success = True
        try:
            if request.open:
                result = self.client.write_coil(self.open_mains_power_addr, True, unit=self.UNIT)
                if result.isError():
                    response.success = False
            else:
                result = self.client.write_coil(self.close_mains_power_addr, True, unit=self.UNIT)
                if result.isError():
                    response.success = False
        except Exception as e:
            self.get_logger().error(f"error: {str(e)}")
            response.success = False
        self.get_logger().info(f"response: {response}")
        return response
    



def main(args=None):
    rclpy.init(args=args)

    platform_communication_node = PlatformCommunicationNode()

    rclpy.spin(platform_communication_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
