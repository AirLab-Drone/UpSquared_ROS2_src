#!/usr/bin/env python3

import cv2
import numpy as np

class KalmanFilter:
    def __init__(self):
        # 使用矢量化初始化Kalman濾波器的參數
        self.kf = cv2.KalmanFilter(18, 6)
        self.kf.measurementMatrix = np.eye(6, 18, dtype=np.float32)
        self.kf.transitionMatrix = np.eye(18, 18, dtype=np.float32)

        # 設置狀態和測量噪聲協方差矩陣
        self.kf.processNoiseCov = np.eye(18, dtype=np.float32) * 1e-5
        self.kf.measurementNoiseCov = np.eye(6, dtype=np.float32) * 1e-1
        self.kf.errorCovPost = np.eye(18, dtype=np.float32)

        self.kf.statePost = np.zeros(18, dtype=np.float32)

    def predict(self):
        return self.kf.predict()

    def correct(self, measurement):
        return self.kf.correct(measurement)
