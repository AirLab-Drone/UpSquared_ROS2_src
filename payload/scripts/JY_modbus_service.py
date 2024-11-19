#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from payload.srv import Spry, HoldPayload, CheckPayload
import time
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusIOException


class FireExtinguisher_JYModbus(Node):
    def __init__(self):
        super().__init__("fire_extinguisher_jy_modbus")
        self.srv = self.create_service(
            Spry, "/fire_extinguisher_jy_modbus/spry", self.spry_callback
        )
        self.srv = self.create_service(
            HoldPayload,
            "/fire_extinguisher_jy_modbus/hold_fire_extinguisher",
            self.hold_fire_extinguisher,
        )
        self.srv = self.create_service(
            CheckPayload,
            "/fire_extinguisher_jy_modbus/check_fire_extinguisher",
            self.check_fire_extinguisher,
        )
        # ------------------------------ io pin setup ------------------------------ #
        self.UNIT = 0x01
        self.client = ModbusClient(
            method="rtu",  # 通訊模式
            port="/dev/ttyUSB4",  # 串口設備
            baudrate=9600,  # 波特率
            stopbits=2,  # 停止位
            bytesize=8,  # 資料位
            timeout=3,  # 超時時間
        )
        self.spry_pin = 0x0000
        self.hold_pin = 0x0002
        self.check_read_pin = 0x0000
        while not self.client.connect():
            self.get_logger().info("connecting...")
            time.sleep(0.5)

    def spry_callback(self, request, response):
        self.get_logger().info("fire extinguisher service is called")
        response.success = True
        try:
            result = self.client.write_coil(self.spry_pin, True, unit=self.UNIT)
            if result.isError():
                response.success = False
            time.sleep(2)
            result = self.client.write_coil(self.spry_pin, False, unit=self.UNIT)
            if result.isError():
                response.success = False
        except Exception as e:
            self.get_logger().info(f"error: {e}")
            response.success = False
        self.get_logger().info(f"response: {response}")
        return response

    def hold_fire_extinguisher(self, request, response):
        self.get_logger().info("HoldFireExtinguisher service is called")
        response.success = True
        try:
            result = self.client.write_coil(self.hold_pin, request.hold, unit=self.UNIT)
            if result.isError():
                response.success = False
        except Exception as e:
            self.get_logger().info(f"error: {e}")
            response.success = False
        return response

    def check_fire_extinguisher(self, request, response):
        self.get_logger().info("check_fire_extinguisher service is called")
        try:
            result = self.client.read_coils(self.check_read_pin, 1, unit=self.UNIT)
            if result.isError():
                self.get_logger().error("ModbusIOException")
                response.success = False
                return response
            response.success = not result.bits[0]
        except Exception as e:
            self.get_logger().info(f"error: {e}")
            response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)

    fire_extinguisher = FireExtinguisher_JYModbus()

    rclpy.spin(fire_extinguisher)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
