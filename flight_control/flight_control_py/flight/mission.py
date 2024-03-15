import math
import time
import rclpy
from visual import ArucoDetector
from flight_control import FlightControl, FlightInfo
from tool.PID import PID
import threading
from tool.video_capture_from_ros2 import VideoCaptureFromRos2


class Mission:
    """
    包含都個任務的class，用於導航，降落等功能
    """

    def __init__(self, controller: FlightControl, flight_info: FlightInfo) -> None:
        self.controller = controller
        self.flight_info = flight_info

    def landedOnPlatform(self):
        """
        Function to control the drone to land on a platform using Aruco markers.

        This function continuously checks the closest Aruco marker detected by the ArucoDetector.
        It calculates the distance and orientation between the drone and the marker.
        The drone moves towards the marker until the distance is less than 0.1 meters.
        If the drone's altitude is lower than the specified lowest_high value, it moves down towards the platform.
        Once the drone has landed on the platform, it stops moving and lands completely.

        Returns:
            None
        """
        # --------------------------------- variable --------------------------------- #
        lowest_high = 0.7  # 最低可看到aruco的高度 單位:公尺
        max_speed = 0.5  # 速度 單位:公尺/秒
        max_yaw = 5*3.14/180  # 5度
        downward_distance = -0.2  # the distance to move down
        aruco_detector = ArucoDetector(
            video_source=VideoCaptureFromRos2(
                "/world/iris_runway/model/iris_with_ardupilot_camera/model/camera/link/camera_link/sensor/camera1/image",
                node=self.controller.node,
            )
        )  # 0 for webcam, 1 for ros image
        count = 0  # count of no aruco
        max_count = 200  # max count of no aruco
        pid_x = PID(
            0.5, 0.25, 0, 0, time=self.controller.node.get_clock().now().nanoseconds * 1e-9
        )
        pid_y = PID(
            0.5, 0.25, 0, 0, time=self.controller.node.get_clock().now().nanoseconds * 1e-9
        )
        pid_yaw = PID(
            1, 0.5, 0, 0, time=self.controller.node.get_clock().now().nanoseconds * 1e-9
        )
        while True:
            # rclpy.spin_once(self.flight_info.node)
            # ----------------------- get downward aruco coordinate ---------------------- #
            closest_aruco = aruco_detector.closestAruco()
            if closest_aruco is None:
                count += 1
                if count > max_count:
                    print('move up')
                    self.controller.sendPositionTargetVelocity(0, 0, 0.5, 0)
                    count = 0
                # self.controller.setZeroVelocity()
                continue
            x, y, z, yaw, _, _ = closest_aruco.getCoordinate()
            if x is None or y is None or z is None or yaw is None:
                count += 1
                if count > max_count:
                    print('move up')
                    self.controller.sendPositionTargetVelocity(0, 0, 0.5, 0)
                    count = 0
                # self.controller.setZeroVelocity()
                continue
            count = 0
            # -------------------------------- PID control ------------------------------- #
            move_x = -pid_x.PID(
                x, self.controller.node.get_clock().now().nanoseconds * 1e-9
            )
            move_y = -pid_y.PID(
                y, self.controller.node.get_clock().now().nanoseconds * 1e-9
            )
            print(f'yaw: {yaw}')
            move_yaw = -pid_yaw.PID(
                yaw, self.controller.node.get_clock().now().nanoseconds * 1e-9
            )
            print(f'move_yaw0: {move_yaw}')
            # print(f"move_x:{(move_x)}, move_y:{type(move_y)}, move_yaw:{type(move_yaw)} ")
            # print(f"move_x:{(move_x)}, move_y:{move_y}, move_yaw:{(move_yaw)}  ")
            # print(f"move_pid_x:{move_pid_x}, move_pid_y:{move_pid_y}")

            diffrent_distance = math.sqrt(x**2 + y**2)
            # -------------------------- limit move_x and move_y and move_yaw------------------------- #
            move_x = min(max(x, -max_speed), max_speed)
            move_y = min(max(y, -max_speed), max_speed)
            print(f'move_yaw1: {move_yaw}')
            move_yaw = min(max(move_yaw * 3.14159 / 180, -max_yaw), max_yaw)
            print(f'move_yaw2: {move_yaw}')
            # ----------------------------- send velocity command ----------------------------- #
            # move_x = 0
            # move_y = 0

            if (
                diffrent_distance < 0.03
                and self.flight_info.rangefinder_alt <= lowest_high
               
            ):
                self.controller.setZeroVelocity()
                print(f'landing high:{self.flight_info.rangefinder_alt}')
                break
            self.controller.sendPositionTargetPosition(0, 0, 0, yaw= -move_yaw)
            if self.flight_info.rangefinder_alt > lowest_high:
                self.controller.sendPositionTargetVelocity(
                    -move_y,
                    -move_x,
                    downward_distance,
                    -move_yaw,
                )
            else:
                # when height is lower than lowest_high, stop moving down
                self.controller.sendPositionTargetVelocity(
                    -move_y,
                    -move_x,
                    0,
                    -move_yaw,
                )
            print(f"move_x:{move_x}, move_y:{move_y}, move_yaw:{move_yaw}, diffrent_distance:{diffrent_distance}")
            # time.sleep(1)
        self.controller.setZeroVelocity()
        print("now I want to land=================================")
        while not self.controller.land():
            print("landing")
