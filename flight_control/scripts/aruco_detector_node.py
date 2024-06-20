#!/usr/bin/env python3
import math
import numpy as np
import cv2
from rclpy.node import Node
import rclpy
from flight_control_py.aruco_visual.aruco import Aruco
from flight_control_py.tool import video_capture_from_ros2
from aruco_msgs.msg import Marker, MarkerArray
from flight_control.srv import GetCloestAruco

from aruco_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovariance, Point, Quaternion
import numpy as np
import cv2
import math
import rclpy


class ArucoDetector(Node):
    """
    坐標系:marker相對於相機, x: 向右為正, y: 向後為正, z: 遠離相機為正, marker順時針旋轉為正
    """

    # #-----------------setting-----------------
    # OUTVEDIO = False
    # DRAWTEXT = True
    # SHOWIMAGE = True
    # DRAWARUCO = True
    is_running = True

    def __init__(
        self,
        save_video: bool = False,
        rotate_deg: float = 0,
        offset_x: float = 0,
        offset_y: float = 0,
    ):
        super().__init__("aruco_detector")
        self.declare_parameter("simulation", False)
        self.cap = cv2.VideoCapture(0)
        if self.get_parameter("simulation").get_parameter_value().bool_value:
            video_source = video_capture_from_ros2.VideoCaptureFromRos2(
                "/world/iris_runway/model/iris_with_ardupilot_camera/model/camera/link/camera_link/sensor/camera1/image"
            )
        if type(self.cap) is cv2.VideoCapture:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
        self.frame = None
        self.arucoList = []
        self.count = 0
        self.start_time = self.get_clock().now()
        self.rotate_deg = rotate_deg
        timer_period = 0.05  # seconds
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        if save_video:
            # save with mp4
            ret, frame = self.cap.read()
            while not ret:
                ret, frame = self.cap.read()
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.out = cv2.VideoWriter(
                f"bottom_video/{rclpy.clock.Clock().now().nanoseconds}.mp4",
                fourcc,
                30.0,
                (frame.shape[1], frame.shape[0]),
            )
        # ---------------------------------- service --------------------------------- #
        # -------------------------------- publishers -------------------------------- #
        self.aruco_publisher = self.create_publisher(MarkerArray, "aruco_markers", 10)
        self.cloest_aruco_publisher = self.create_publisher(Marker, "cloest_aruco", 10)
        # ------------------------------- start detect ------------------------------- #
        self.create_timer(timer_period, self.run)
        self.create_timer(timer_period, self.get_cloest_aruco_callback)

    def run(self):
        if self.is_running:
            ret, self.frame = self.cap.read()
            if not ret:
                return
            # -----------------find aruco-----------------
            if self.frame is None:
                return
            (corners, ids, rejected) = self.detector.detectMarkers(self.frame)
            if hasattr(self, "out") and self.out is not None:
                self.frame = cv2.aruco.drawDetectedMarkers(
                    self.frame, corners, ids, (0, 255, 0)
                )
                self.out.write(self.frame)

            if len(corners) > 0:
                for i in range(len(ids)):
                    self.addNewAruco(ids[i][0], corners[i])
            marker_array_temp = MarkerArray()
            for aruco in self.arucoList:
                if aruco.checkInList(ids) and len(corners) > 0:
                    id = aruco.id
                    aruco.update(id, corners[np.where(ids == aruco.id)[0][0]])
                if not aruco.is_empty():
                    marker_array_temp.markers.append(aruco.getCoordinateWithMarkerMsg())
                    # print(aruco.getCoordinate())
            marker_array_temp.header.frame_id = "aruco_list"
            marker_array_temp.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.aruco_publisher.publish(marker_array_temp)
            # self.debug()

    def stop(self):
        self.is_running = False
        self.cap.release()
        cv2.destroyAllWindows()

    def debug(self):
        # -------------print fps-----------------
        self.count += 1
        count_fps = 30
        if self.count % count_fps == 0:
            end_time = self.get_clock().now()
            print(f"FPS: {count_fps / ((end_time - self.start_time).nanoseconds*1e-9)}")
            # for aruco in arucoList:
            #     print(aruco.id,aruco.getCoordinate())
            self.start_time = self.get_clock().now()
        # ------------------------------- aruco status ------------------------------- #
        if len(self.arucoList) > 0:
            print(len(self.arucoList[0].x_list.items))

    def get_cloest_aruco_callback(self):
        closest_aruco = None
        for aruco in self.arucoList:
            id = aruco.id
            x, y, z, yaw, pitch, roll = aruco.getCoordinate()
            if (
                x == None
                or y == None
                or z == None
                or yaw == None
                or pitch == None
                or roll == None
            ):
                continue
            if (
                math.isnan(x)
                or math.isnan(y)
                or math.isnan(z)
                or math.isnan(yaw)
                or math.isnan(pitch)
                or math.isnan(roll)
            ):
                continue
            if closest_aruco == None:
                closest_aruco = aruco
            else:
                aruco_distance = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
                closest_aruco_distance = math.sqrt(
                    math.pow(float(closest_aruco.getCoordinate()[0]), 2)
                    + math.pow(float(closest_aruco.getCoordinate()[1]), 2)
                )
                if aruco_distance < closest_aruco_distance:
                    closest_aruco = aruco
        if closest_aruco == None:
            marker = Marker()
            marker.header.frame_id = "aruco"
            marker.header.stamp = rclpy.clock.Clock().now().to_msg()
            marker.confidence = 0.0
            self.cloest_aruco_publisher.publish(marker)
        else:
            marker = closest_aruco.getCoordinateWithMarkerMsg()
            if self.rotate_deg != 0:
                marker = self.rotateAndOffsetArucoCoordinate(marker, self.rotate_deg)
            self.cloest_aruco_publisher.publish(marker)
        print(
            f"x: {marker.x:.2f}, y: {marker.y:.2f}, z: {marker.z:.2f}, yaw: {marker.yaw:.2f}, pitch: {marker.pitch:.2f}, roll: {marker.roll:.2f}"
        )

    def rotateAndOffsetArucoCoordinate(
        self, marker, rotate_deg=0, offset_x=0, offset_y=0
    ):
        """
        @param marker
        @param rotate_deg: rotate degree, 相機離0度的偏移角度，順時針為正
        @param offset_x: x offset, 相機離中心點的左右偏移, 向右為正
        @param offset_y: y offset, 相機離中心點的前後偏移, 向後為正
        @return: a marker with new coordinate
        """
        marker.yaw = (marker.yaw + rotate_deg) % 360
        rotate_deg = math.radians(rotate_deg)
        temp_x = marker.x
        temp_y = marker.y
        marker.x = temp_x * math.cos(rotate_deg) - temp_y * math.sin(rotate_deg)
        marker.y = temp_x * math.sin(rotate_deg) + temp_y * math.cos(rotate_deg)
        marker.x += offset_x
        marker.y += offset_y
        return marker

    def addNewAruco(self, id, corner):
        for aruco in self.arucoList:
            if aruco.id == id:
                return False
        aruco = Aruco(id)
        aruco.update(id, corner)
        self.arucoList.append(aruco)

    def draw(self, x, y, z, yaw, id, frame, image_y):
        text = "id: {}, x: {:.2f}, y: {:.2f}, z: {:.2f}, yaw: {:.2f}".format(
            id, x, y, z, yaw
        )
        cv2.putText(
            frame, text, (20, image_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
        )


def main():
    if not rclpy.ok():
        rclpy.init()
    video_source = cv2.VideoCapture(0)
    aruco_detector = ArucoDetector(
        save_video=True,
        rotate_deg=90,
        offset_x=0,
        offset_y=0,
    )
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
