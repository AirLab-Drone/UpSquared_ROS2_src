#!/usr/bin/env python3


import numpy as np

class KalmanFilter:
    def __init__(self, process_variance=1e-4, measurement_variance=0.01):
        # 初始化狀態和不確定性
        self.x = np.zeros(6)  # 初始狀態 (x, y, z, roll, pitch, yaw)
        self.P = np.eye(6)  # 初始狀態不確定性

        # 過程噪聲和測量噪聲
        self.Q = process_variance * np.eye(6)
        self.R = measurement_variance * np.eye(6)

    def predict(self):
        # 預測步驟 (無運動模型假設)
        self.P = self.P + self.Q

    def update(self, z):
        # 更新步驟 (z 是觀測值)
        K = self.P @ np.linalg.inv(self.P + self.R)  # 卡爾曼增益
        self.x = self.x + K @ (z - self.x)  # 更新狀態
        self.P = (np.eye(6) - K) @ self.P  # 更新不確定性
        return self.x
