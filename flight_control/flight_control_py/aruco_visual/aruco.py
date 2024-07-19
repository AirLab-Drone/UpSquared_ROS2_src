from aruco_msgs.msg import Marker
import numpy as np
import cv2
import math
import rclpy
from scipy.spatial.transform import Rotation


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
    # wxsj-AHD-1080p
    mtx = np.array(
        [
            [413.48355918, 0, 314.89857666],
            [0, 414.51193613, 250.2192352],
            [0, 0, 1],
        ]
    )
    dist = np.array([[0.18866341, -0.22605576, 0.00306978, 0.00236462, 0.08722311]])

    # # simulation
    # mtx = np.array([[554.25625995,   0, 960],
    #    [  0, 554.25625995, 540],
    #    [  0,   0,   1]])
    # dist = np.array([[0, 0, 0, 0, 0]])

    limit_list_size = 3

    def __init__(self, marker_id, marker_config) -> None:
        self.id = marker_id
        self.markerLength = marker_config["marker_length"]  # unit: meter
        self.offset_x = marker_config["offset_x"]
        self.offset_y = marker_config["offset_y"]
        self.offset_z = marker_config["offset_z"]
        self.marker_yaw = marker_config["marker_yaw"]
        self.x_list = LimitedList(self.limit_list_size)
        self.y_list = LimitedList(self.limit_list_size)
        self.z_list = LimitedList(self.limit_list_size)
        self.yaw_list = LimitedList(self.limit_list_size)
        self.pitch_list = LimitedList(self.limit_list_size)
        self.roll_list = LimitedList(self.limit_list_size)
        self.rvec = None
        self.tvec = None

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
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            corner, self.markerLength, self.mtx, self.dist
        )
        self.rvec = rvec
        self.tvec = tvec
        yaw, pitch, roll = self.rvec_to_euler_angles(rvec)
        x = tvec[0][0][0]
        y = tvec[0][0][1]
        z = tvec[0][0][2]
        return x, y, z, yaw, pitch, roll

    def update(self, marker_id, corner):
        if self.id != marker_id:
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

    def get_coordinate_with_offset(self):
        x, y, z, yaw, pitch, roll = self.getCoordinate()
        if (
            x == None
            or y == None
            or z == None
            or yaw == None
            or pitch == None
            or roll == None
        ):
            return None, None, None, None, None, None
        # ---------------------------------- offset ---------------------------------- #
        x -= self.offset_x
        y -= self.offset_y
        z -= self.offset_z
        yaw -= self.marker_yaw
        return x, y, z, yaw, pitch, roll

    def getCoordinateWithMarkerMsg(self):
        x, y, z, yaw, pitch, roll = self.getCoordinate()
        if (
            x == None
            or y == None
            or z == None
            or yaw == None
            or pitch == None
            or roll == None
        ):
            return Marker()
        marker = Marker()
        marker.header.frame_id = "aruco"
        marker.header.stamp = rclpy.clock.Clock().now().to_msg()
        marker.id = int(self.id)
        marker.x = x
        marker.y = y
        marker.z = z
        marker.roll = roll
        marker.pitch = pitch
        marker.yaw = yaw
        marker.confidence = 1.0
        return marker

    def checkStd(self):
        x_std = self.x_list.calculate_std()
        y_std = self.y_list.calculate_std()
        z_std = self.z_list.calculate_std()
        yaw_std = self.yaw_list.calculate_std()
        if x_std > 0.2 or y_std > 0.2 or z_std > 0.2 or yaw_std > 20:
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

    def euler_to_quaternion(self, roll, pitch, yaw):
        # 將Euler角轉換為旋轉矩陣
        rotation_matrix = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()

        # 將旋轉矩陣轉換為四元數
        quaternion = Rotation.from_matrix(rotation_matrix).as_quat()

        return quaternion

    def quaternion_to_euler(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)
        return X, Y, Z

    def fromMsgMarker2Aruco(self, marker: Marker):
        """
        Converts a marker message to an Aruco object.

        Args:
            marker (Marker): The marker message to convert.

        Returns:
            Aruco: The converted Aruco object, or None if the marker confidence is below 0.5.
        """
        if marker.confidence < 0.5:
            return None
        self.id = marker.id
        self.x_list.add_element(marker.x)
        self.y_list.add_element(marker.y)
        self.z_list.add_element(marker.z)
        self.yaw_list.add_element(marker.yaw)
        self.pitch_list.add_element(marker.pitch)
        self.roll_list.add_element(marker.roll)
        return self
