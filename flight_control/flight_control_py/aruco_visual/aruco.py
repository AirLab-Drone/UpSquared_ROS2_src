from aruco_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovariance, Point, Quaternion
import numpy as np
import cv2
import math
import rclpy

class LimitedList:
    def __init__(self, initial_size=20):
        self.items = []
        self.initial_size = initial_size

    def add_element(self, element):
        if len(self.items) == self.initial_size:
            self.items.pop(0)  # 删除最早的元素
        self.items.append(element)

    def pop_element_and_getmedian(self):
        if len(self.items) == 0:
            return "null"
        else:
            self.items.pop(0)
            return np.median(self.items)

    def calculate_median(self):
        return np.median(self.items)

    def calculate_std(self):
        return np.std(self.items)


class Aruco:
    mtx = np.array(
        # [[776.31000562, 0, 327.96638128], [0, 775.07173891, 179.57551958], [0, 0, 1]]
        [
            [405.19622214, 0.00000000e00, 340],
            [0.00000000e00, 405.19622214, 240],
            [0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
    )
    dist = np.array(
        # [
        #     [
        #         8.29378271e-02,
        #         1.26989092e-01,
        #         3.86532147e-03,
        #         1.18462078e-03,
        #         -1.87627090e00,
        #     ]
        # ]
        [[0, 0, 0, 0, 0]]
    )
    markerLength = 0.16  # unit: meter
    limit_list_size = 5

    # markerLength = 0.0385
    def __init__(self, id) -> None:
        self.id = id
        self.x_list = LimitedList(self.limit_list_size)
        self.y_list = LimitedList(self.limit_list_size)
        self.z_list = LimitedList(self.limit_list_size)
        self.yaw_list = LimitedList(self.limit_list_size)
        self.pitch_list = LimitedList(self.limit_list_size)
        self.roll_list = LimitedList(self.limit_list_size)

    def checkInList(self, ids):
        if ids is None or self.id not in ids:
            self.x_list.pop_element_and_getmedian()
            self.y_list.pop_element_and_getmedian()
            self.z_list.pop_element_and_getmedian()
            self.yaw_list.pop_element_and_getmedian()
            self.pitch_list.pop_element_and_getmedian()
            self.roll_list.pop_element_and_getmedian()
            return False
        else:
            return True

    def estimatePoseSingleMarkers(self, corner):
        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
            corner, self.markerLength, self.mtx, self.dist)
        self.rvec = rvec
        self.tvec = tvec
        yaw, pitch, roll = self.rvec_to_euler_angles(rvec)
        x = tvec[0][0][0]
        y = tvec[0][0][1]
        z = tvec[0][0][2]
        return x, y, z, yaw, pitch, roll

    def update(self, id, corner):
        if self.id != id:
            return
        x, y, z, yaw, pitch, roll = self.estimatePoseSingleMarkers(corner)
        self.x_list.add_element(x)
        self.y_list.add_element(y)
        self.z_list.add_element(z)
        self.yaw_list.add_element(math.degrees(yaw))
        self.pitch_list.add_element(math.degrees(pitch))
        self.roll_list.add_element(math.degrees(roll))
        return self.getCoordinate()

    def getCoordinate(self):
        """
        @return: x,y,z,yaw,pitch,roll
        """
        if self.checkStd() == False:
            return None, None, None, None, None, None
        x = self.x_list.calculate_median()
        y = self.y_list.calculate_median()
        z = self.z_list.calculate_median()
        yaw = self.yaw_list.calculate_median()
        pitch = self.pitch_list.calculate_median()
        roll = self.roll_list.calculate_median()
        return x, y, z, yaw, pitch, roll

    def getCoordinateWithMarkerMsg(self):
        x, y, z, yaw, pitch, roll = self.getCoordinate()
        if x == None or y == None or z == None or yaw == None or pitch == None or roll == None:
            return Marker()
        marker = Marker()
        marker.header.frame_id = "aruco"
        marker.header.stamp = rclpy.clock.Clock().now().to_msg()
        marker.id = int(self.id)
        marker.pose = PoseWithCovariance()
        marker.pose.pose.position = Point(x=x, y=y, z=z)
        # roll, pitch, yaw to quaternion
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        w = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2) # noqa
        marker.pose.pose.orientation = Quaternion(x=float(roll), y=float(pitch), z=float(yaw), w=float(w))

        return marker

    def checkStd(self):
        x_std = self.x_list.calculate_std()
        y_std = self.y_list.calculate_std()
        z_std = self.z_list.calculate_std()
        yaw_std = self.yaw_list.calculate_std()
        if x_std > 0.02 or y_std > 0.02 or z_std > 0.02 or yaw_std > 2:
            return False
        return True

    def rvec_to_euler_angles(self, rvec):
        rvec_flipped = rvec[0][0] * -1
        R_mat, jacobian = cv2.Rodrigues(rvec_flipped)
        pitch = math.atan2(R_mat[2, 1], R_mat[2, 2])
        sy = math.sqrt((R_mat[0][0] * R_mat[0][0]) + (R_mat[1][0] * R_mat[1][0]))
        singular = sy < 1e-6
        if not singular:
            pitch = math.atan2(R_mat[2, 1], R_mat[2, 2])
            roll = math.atan2(-R_mat[2, 0], sy)
            yaw = math.atan2(R_mat[1, 0], R_mat[0, 0])
        else:
            pitch = math.atan2(-R_mat[1, 2], R_mat[1, 1])
            roll = math.atan2(-R_mat[2, 0], sy)
            yaw = 0
        return yaw, pitch, roll

    def drawAruco(self, frame):
        frame = cv2.drawFrameAxes(
            frame, self.mtx, self.dist, self.rvec, self.tvec, 0.05
        )
        return frame

    def is_empty(self):
        return len(self.x_list.items) == 0
