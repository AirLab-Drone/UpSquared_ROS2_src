#!/usr/bin/env python3

'''
確認無人機的RGB相機及熱像儀是否已連接
    RGB相機: 0c45:6364:Microdia USB 2.0 Camera   
    熱像儀: 04b4:f8f8:Cypress Semiconductor Corp. GuideCamera
'''


import rclpy
from rclpy.node import Node
from drone_status.srv import CheckUSBDevices  # 使用新建的服務
import subprocess

class USBChecker(Node):
    def __init__(self):
        super().__init__('usb_checker')
        # 創建自定義 service
        self.srv = self.create_service(CheckUSBDevices, 'check_usb_devices', self.check_usb_devices_callback)
        # 使用字典儲存裝置 ID 和名稱
        self.target_devices = {
            '0c45:6364': 'Microdia USB 2.0 Camera',
            '04b4:f8f8': 'Cypress Semiconductor Corp. GuideCamera'
        }
        self.get_logger().info("USBChecker service is ready to check USB devices.")

    def check_usb_devices_callback(self, request, response):
        # 執行 lsusb 命令以檢查當前接上的USB設備
        result = subprocess.run(['lsusb'], stdout=subprocess.PIPE, text=True)
        lsusb_output = result.stdout
        self.get_logger().info("Checking connected USB devices...")

        # 檢查目標裝置是否存在
        missing_devices = []
        for device_id, device_name in self.target_devices.items():
            if device_id not in lsusb_output:
                missing_devices.append(f"{device_name} ({device_id})")  # 加入裝置名稱與 ID
        
        if missing_devices:
            response.success = False
            response.missing_devices = missing_devices  # 傳遞遺失設備陣列
        else:
            response.success = True
            response.missing_devices = []
        
        return response

def main(args=None):
    rclpy.init(args=args)
    usb_checker = USBChecker()
    rclpy.spin(usb_checker)
    usb_checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


