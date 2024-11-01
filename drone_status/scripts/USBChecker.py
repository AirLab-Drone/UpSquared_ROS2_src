#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from drone_status_msgs.srv import CheckUSBDevices  # 使用新建的服務
import subprocess

class USBChecker(Node):
    def __init__(self):
        super().__init__('usb_checker')
        # 創建自定義 service 
        self.srv = self.create_service(CheckUSBDevices, 'check_usb_devices', self.check_usb_devices_callback)
        
        # 使用字典儲存裝置 ID 和名稱，支援多個 ID
        self.target_devices = {
            '0c45:6364': 'Microdia USB 2.0 Camera',
            '04b4:f8f8': 'Cypress Semiconductor Corp. GuideCamera', # USB 2.0
            '04b4:f9f9': 'Cypress Semiconductor Corp. GuideCamera', # USB 3.0
        }
        self.get_logger().info("USBChecker service is ready to check USB devices.")

    def check_usb_devices_callback(self, request, response):
        # 執行 lsusb 命令以檢查當前接上的 USB 設備
        result = subprocess.run(['lsusb'], stdout=subprocess.PIPE, text=True)
        lsusb_output = result.stdout
        self.get_logger().info("Checking connected USB devices...")

        # 檢查目標裝置是否存在
        missing_devices = []
        
        # 檢查每個目標裝置的 ID 是否在 lsusb 的輸出中
        guide_camera_found = False
        for device_id, device_name in self.target_devices.items():
            if device_id in lsusb_output:
                self.get_logger().info(f"{device_name} ({device_id}) 已連接。")
                if device_name == 'Cypress Semiconductor Corp. GuideCamera':
                    guide_camera_found = True
            else:
                if device_name != 'Cypress Semiconductor Corp. GuideCamera':
                    missing_devices.append(f"{device_name} ({device_id})")
        
        # 如果 GuideCamera 的 USB 2.0 或 USB 3.0 任一 ID 未檢測到，則視為缺失
        if not guide_camera_found:
            missing_devices.append("Cypress Semiconductor Corp. GuideCamera (04b4:f8f8 or 04b4:f9f9)")
        
        # 回應結果
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
