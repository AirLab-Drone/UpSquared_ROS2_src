#!/usr/bin/env python3
import math
import numpy as np
import cv2
import yaml
from rclpy.node import Node
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


from geometry_msgs.msg import PoseWithCovariance, Point, Quaternion

from aruco_detect_py.aruco import Aruco
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

    def __init__(self):
        super().__init__("aruco_detector")
        # --------------------------- ros2 global parameter -------------------------- #
        self.declare_parameter("simulation", False)
        self.declare_parameter("aruco_marker_config_file", "")
        self.declare_parameter("camera_config_file", "")
        self.declare_parameter("show_image", False)
        # ------------------------------ class variable ------------------------------ #
        # cv2 setup
        self.frame = None
        self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        # FPS debug
        self.count = 0
        self.start_time = self.get_clock().now()
        # camera setup
        camera_config_path = (
            self.get_parameter("camera_config_file").get_parameter_value().string_value
        )
        if camera_config_path == "":
            self.get_logger().error("Please set camera_config_file parameter")
            raise Exception("Configuration file not set")
        with open(camera_config_path, "r") as f:
            config = yaml.safe_load(f)
            # camera offset
            self.rotate_deg = config["camera_offset"]["yaw"]
            self.offset_x = config["camera_offset"]["x"]
            self.offset_y = config["camera_offset"]["y"]
            # camera matrix
            self.camera_parameter = {}
            self.camera_parameter["matrix"] = np.array(
                config["camera_parameter"]["camera_matrix"]
            )
            self.camera_parameter["distortion"] = np.array(
                config["camera_parameter"]["distortion_coefficients"]
            )
        # aruco setup
        self.arucoList = []
        aruco_marker_config_file_path = (
            self.get_parameter("aruco_marker_config_file")
            .get_parameter_value()
            .string_value
        )
        if aruco_marker_config_file_path == "":
            self.get_logger().error("Please set aruco_marker_config_file parameter")
            raise Exception("Configuration file not set")
        with open(aruco_marker_config_file_path, "r") as f:
            config = yaml.safe_load(f)
            self.markers_config = config["aruco_markers"]
        # aruco detector
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        # -------------------------------- publishers -------------------------------- #
        self.aruco_publisher = self.create_publisher(MarkerArray, "aruco_markers", 10)
        self.closest_aruco_publisher = self.create_publisher(
            Marker, "closest_aruco", 10
        )
        # ------------------------------- start detect ------------------------------- #
        timer_period = 0.01  # seconds
        self.create_timer(timer_period, self.run)
        self.create_timer(timer_period, self.get_closest_aruco_callback)

    def image_callback(self, msg):
        self.frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")

    def run(self):
        if self.is_running:
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

            marker_array_temp.header.frame_id = "aruco_list"
            marker_array_temp.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.aruco_publisher.publish(marker_array_temp)
            # -----------------show image-----------------
            if (
                self.frame is not None
                and self.get_parameter("show_image").get_parameter_value().bool_value
            ):
                cv2.imshow("frame", self.frame)
                cv2.waitKey(1)

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
            self.closest_aruco_publisher.publish(marker)
        else:
            marker = closest_aruco.getCoordinateWithMarkerMsg()
            marker = self.rotateAndOffsetArucoCoordinate(
                marker, self.rotate_deg, self.offset_x, self.offset_y
            )
            self.closest_aruco_publisher.publish(marker)
        self.get_logger().info(
            f"\rid: {marker.id} x: {marker.x:.2f}, y: {marker.y:.2f}, z: {marker.z:.2f}, yaw: {marker.yaw:.2f}, pitch: {marker.pitch:.2f}, roll: {marker.roll:.2f}"
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
            marker_id=marker_id,
            marker_config=self.markers_config[f"{marker_id}"],
            camera_parameter=self.camera_parameter,
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

    def stop(self):
        self.is_running = False
        self.cap.release()
        cv2.destroyAllWindows()


def main():
    if not rclpy.ok():
        rclpy.init()

    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
