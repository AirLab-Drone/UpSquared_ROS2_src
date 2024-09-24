#!/usr/bin/env python3
import math
import numpy as np
import cv2
import yaml
from rclpy.node import Node
import rclpy

from geometry_msgs.msg import PoseWithCovariance, Point, Quaternion

from flight_control_py.aruco_visual.aruco import Aruco
from flight_control_py.tool.video_capture_from_ros2 import VideoCaptureFromRos2
from aruco_msgs.msg import Marker, MarkerArray


class ArucoDetector(Node):
    """
    以無人機中心為原點
    camera coordinate:
    x 軸: 向右為正，向左為負
    y 軸: 向後為正，向前為負
    z 軸: 遠離為正，靠近為負
    rotate deg: 相機離0度的偏移角度，順時針為正
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
        # --------------------------- ros2 global parameter -------------------------- #
        self.declare_parameter("simulation", False)
        self.declare_parameter("config_file", "")
        # ------------------------------ class variable ------------------------------ #
        # cv2 setup
        self.cap = None
        if self.get_parameter("simulation").get_parameter_value().bool_value:
            self.cap = VideoCaptureFromRos2("/camera/image_raw")
        else:
            self.get_logger().info("connecting to camera")
            self.cap = cv2.VideoCapture(0)
            # -------------------------------- 設定以MJPG格式讀取 ------------------------------- #
            self.cap.set(
                cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("m", "j", "p", "g")
            )
            self.cap.set(
                cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G")
            )

            # ----------------------------------- 消除頻閃 ----------------------------------- #
            # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)  # 再關閉自動曝光
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3.0)  # 先開啟自動曝光
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0) # 再關閉自動曝光
            self.cap.set(cv2.CAP_PROP_EXPOSURE, 157)
            # TODO 降落時自動調整曝光值

            self.get_logger().info("connected to camera")
        print(f"cap: {self.cap}")

        self.frame = None
        # FPS debug
        self.count = 0
        self.start_time = self.get_clock().now()
        # real camera setup
        self.rotate_deg = rotate_deg
        self.offset_x = offset_x
        self.offset_y = offset_y
        # aruco setup
        self.arucoList = []
        config_file_path = (
            self.get_parameter("config_file").get_parameter_value().string_value
        )
        if config_file_path == "":
            self.get_logger().error("Please set config_file parameter")
            raise Exception("Configuration file not set")
        with open(config_file_path, "r") as f:
            config = yaml.safe_load(f)
            self.markers_config = config["aruco_markers"]
        # aruco detector
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
        # -------------------------------- publishers -------------------------------- #
        self.aruco_publisher = self.create_publisher(MarkerArray, "aruco_markers", 10)
        self.cloest_aruco_publisher = self.create_publisher(Marker, "closest_aruco", 10)
        # ------------------------------- start detect ------------------------------- #
        timer_period = 0.1  # seconds
        self.create_timer(timer_period, self.run)
        self.create_timer(timer_period, self.get_closest_aruco_callback)

    def run(self):
        if self.is_running:
            ret, self.frame = self.cap.read()
            if not ret:
                return
            # -----------------find aruco-----------------
            if self.frame is None:
                return
            (corners, ids, rejected) = self.detector.detectMarkers(self.frame)

            if len(corners) > 0:
                for i in range(len(ids)):
                    self.addNewAruco(ids[i][0], corners[i])
            marker_array_temp = MarkerArray()
            count = 0
            for aruco in self.arucoList:
                if aruco.checkInList(ids) and len(corners) > 0:
                    id = aruco.id
                    aruco.update(id, corners[np.where(ids == aruco.id)[0][0]])
                if not aruco.is_empty():
                    marker_array_temp.markers.append(aruco.getCoordinateWithMarkerMsg())
                    # print(aruco.getCoordinate())
                coord = aruco.getCoordinate()
                try:
                    formatted_coord = f"{id}: {coord[0]:.3f}, {coord[1]:.3f}, {coord[2]:.3f}, {coord[3]:.3f}, {coord[4]:.3f}, {coord[5]:.3f}"
                    cv2.putText(
                        self.frame,
                        formatted_coord,
                        (10, count * 50 + 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )
                except:
                    pass
                count = count + 1

            if hasattr(self, "out") and self.out is not None:
                self.frame = cv2.aruco.drawDetectedMarkers(
                    self.frame, corners, ids, (0, 255, 0)
                )
                self.out.write(self.frame)
            marker_array_temp.header.frame_id = "aruco_list"
            marker_array_temp.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.aruco_publisher.publish(marker_array_temp)
            self.debug()

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
            self.get_logger().info(
                f"FPS: {count_fps / ((end_time - self.start_time).nanoseconds*1e-9)}"
            )
            # for aruco in arucoList:
            #     print(aruco.id,aruco.getCoordinate())
            self.start_time = self.get_clock().now()
        # ------------------------------- aruco status ------------------------------- #
        # cv2.imshow("frame", self.frame)
        # key = cv2.waitKey(1) & 0xFF

        # if key == ord("q"):
        #     self.stop()
        # elif key == ord("a"):
        #     self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3.0) # 先開啟自動曝光
        #     self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0) # 再關閉自動曝光

    def get_closest_aruco_callback(self):

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
            marker = self.rotateAndOffsetArucoCoordinate(
                marker, self.rotate_deg, self.offset_x, self.offset_y
            )
            self.cloest_aruco_publisher.publish(marker)
        self.get_logger().info(
            f"id: {marker.id} x: {marker.x:.2f}, y: {marker.y:.2f}, z: {marker.z:.2f}, yaw: {marker.yaw:.2f}, pitch: {marker.pitch:.2f}, roll: {marker.roll:.2f}"
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
        marker.x = temp_x * math.cos(rotate_deg) + temp_y * math.sin(rotate_deg)
        marker.y = -temp_x * math.sin(rotate_deg) + temp_y * math.cos(rotate_deg)
        marker.x += offset_x
        marker.y += offset_y
        return marker

    def addNewAruco(self, marker_id, corner):
        for aruco in self.arucoList:
            if aruco.id == marker_id:
                return False
        if str(marker_id) not in self.markers_config:
            return False
        aruco = Aruco(
            marker_id=marker_id, marker_config=self.markers_config[f"{marker_id}"]
        )
        aruco.update(marker_id, corner)
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

    aruco_detector = ArucoDetector(
        save_video=True,
        rotate_deg=180,
        offset_x=0,
        offset_y=0.125,
    )
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
