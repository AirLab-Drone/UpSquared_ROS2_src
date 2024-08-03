import math
import rclpy
from rclpy.node import Node
from flight_control_py.tool.PID import PID
from flight_control_py.flight.base_control import BaseControl as FlightControl
from flight_control_py.flight.flight_controller_info import FlightInfo
from flight_control_py.aruco_visual.aruco import Aruco
from flight_control.srv import GetCloestAruco
from aruco_msgs.msg import Marker
import time
import yaml


class Mission:
    """
    包含都個任務的class，用於導航，降落等功能
    """

    # define mode name
    WAIT_MODE = -1
    TAKE_OFF_MODE = 0
    LANDING_MODE = 1
    LANDIND_ON_PLATFORM_MODE = 2
    NAVIGATION_MODE = 3

    mode = WAIT_MODE

    def __init__(
        self, controller: FlightControl, flight_info: FlightInfo, node: Node
    ) -> None:
        self.controller = controller
        self.flight_info = flight_info
        self.node = node
        self.cloest_aruco = None
        # self.getCloestArucoClient = self.node.create_client(GetCloestAruco, 'get_cloest_aruco')
        self.sub = self.node.create_subscription(
            Marker, "cloest_aruco", self.cloest_aruco_callback, 10
        )
        # aruco marker config
        config_file_path = (
            self.node.get_parameter("config_file").get_parameter_value().string_value
        )
        if config_file_path == "":
            self.node.get_logger().error("Please set config_file parameter")
            raise Exception("Configuration file not set")
        with open(config_file_path, "r") as f:
            config = yaml.safe_load(f)
            self.markers_config = config["aruco_markers"]

    # ---------------------------------------------------------------------------- #
    #                                   callback                                   #
    # ---------------------------------------------------------------------------- #
    def cloest_aruco_callback(self, msg):
        self.cloest_aruco = Aruco(
            marker_id=msg.id, marker_config=self.markers_config[f"{msg.id}"]
        ).fromMsgMarker2Aruco(msg)

    # ---------------------------------------------------------------------------- #
    #                                   Function                                   #
    # ---------------------------------------------------------------------------- #
    def __setMode(self, mode):
        if mode not in [
            self.WAIT_MODE,
            self.TAKE_OFF_MODE,
            self.LANDING_MODE,
            self.LANDIND_ON_PLATFORM_MODE,
            self.NAVIGATION_MODE,
        ]:
            return False
        self.mode = mode

    def stopMission(self):
        self.__setMode(self.WAIT_MODE)
        self.controller.setZeroVelocity()

    # ---------------------------------------------------------------------------- #
    #                                    Mission                                   #
    # ---------------------------------------------------------------------------- #
    def simpleTakeoff(self, target_hight=2):
        count = 0
        while not self.controller.armAndTakeoff(alt=target_hight):
            print("armAndTakeoff fail")
            count += 1
            if count > 5:
                return False
        # print('takeoff success')
        # while abs(self.flight_info.rangefinder_alt - target_hight) > 0.5:
        #     print(f'rangefinder alt: {self.flight_info.rangefinder_alt} target_hight: {target_hight}')
        #     print(f"hight offset: {self.flight_info.rangefinder_alt - target_hight}")
        time.sleep(7)
        self.mode = self.WAIT_MODE
        return True

    def simpleLanding(self):
        while not self.controller.land():
            print("landing")
        while self.flight_info.state.armed:
            # print(f"landing high:{self.flight_info.rangefinder_alt}")
            pass
        self.mode = self.WAIT_MODE
        return True

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
        # 檢查先前模式是否為等待模式，定且設定目前模式為降落至平台模式
        if self.mode != self.WAIT_MODE:
            return False
        self.__setMode(self.LANDIND_ON_PLATFORM_MODE)
        # --------------------------------- variable --------------------------------- #
        LOWEST_HEIGHT = 0.7  # 最低可看到aruco的高度 單位:公尺
        MAX_SPEED = 0.3  # 速度 單位:公尺/秒
        MAX_YAW = 25 * 3.14 / 180  # 15度/s
        DOWNWARD_SPEED = -0.2  # the distance to move down,必需要為負
        last_moveup_time = rclpy.clock.Clock().now()

        # -------------------------------- PID initial ------------------------------- #
        def init_pid():
            current_time = rclpy.clock.Clock().now().nanoseconds
            pid_move_x = PID(0.6, 0.006, 0.0083, current_time)
            pid_move_y = PID(0.6, 0.006, 0.0083, current_time)
            pid_move_yaw = PID(1, 0.006, 0.0083, current_time)
            # pid_move_yaw = PID(0.4, 0.006, 0.0083, current_time)
            return pid_move_x, pid_move_y, pid_move_yaw

        pid_move_x, pid_move_y, pid_move_yaw = init_pid()
        # ------------------------------- start mission ------------------------------ #
        while True:
            # 設定中斷點，如果不是降落模式就直接結束
            # todo 檢查thread結束後是否會釋放記憶體
            if self.mode != self.LANDIND_ON_PLATFORM_MODE:
                self.controller.setZeroVelocity()
                return False
            # get downward aruco coordinate
            closest_aruco = self.cloest_aruco
            if closest_aruco is None:
                if rclpy.clock.Clock().now() - last_moveup_time > rclpy.time.Duration(
                    seconds=0.5
                ):
                    if self.flight_info.rangefinder_alt < 3:
                        # todo 若看不到aruco水平飛到UWB home position
                        # print('move up')
                        self.controller.sendPositionTargetVelocity(
                            0, 0, -DOWNWARD_SPEED, 0
                        )
                        last_moveup_time = rclpy.clock.Clock().now()
                        pid_move_x, pid_move_y, pid_move_yaw = init_pid()
                continue
            marker_x, marker_y, marker_z, marker_yaw, _, _ = (
                closest_aruco.get_coordinate_with_offset()
            )
            # 處理
            if (
                marker_x is None
                or marker_y is None
                or marker_z is None
                or marker_yaw is None
            ):
                if rclpy.clock.Clock().now() - last_moveup_time > rclpy.time.Duration(
                    seconds=0.5
                ):
                    # todo 若看不到aruco水平飛到UWB home position
                    if self.flight_info.rangefinder_alt < 3:
                        # print('mo = pid_move_y.PID(-marker_x, current_time)ve up')
                        self.controller.sendPositionTargetVelocity(
                            0, 0, -DOWNWARD_SPEED, 0
                        )
                        last_moveup_time = rclpy.clock.Clock().now()
                        pid_move_x, pid_move_y, pid_move_yaw = init_pid()
                continue
            # 無人機和降落點的水平誤差
            different_distance = math.sqrt(marker_x**2 + marker_y**2)
            # limit move_x and move_y and move_yaw #
            # todo改成以無人機為準的座標系
            # todo加入PID控制
            # PID control
            current_time = rclpy.clock.Clock().now().nanoseconds
            move_x = pid_move_x.update(-marker_y, current_time)
            move_y = pid_move_y.update(-marker_x, current_time)

            marker_yaw = (marker_yaw + 180) % 360 - 180
            move_yaw = pid_move_yaw.update(-marker_yaw * 3.14 / 180, current_time)
            different_move = math.sqrt(move_x**2 + move_y**2)
            # 限制最大速度
            max_speed_temp = min(max(different_move, -MAX_SPEED), MAX_SPEED)
            move_x = move_x / different_move * max_speed_temp
            move_y = move_y / different_move * max_speed_temp
            # move_yaw = (move_yaw + 6.28) % 6.28
            # if (6.28 - move_yaw) < move_yaw:
            #     move_yaw = -move_yaw
            # move_yaw = min(max(move_yaw, -MAX_YAW), MAX_YAW)
            print(
                f"move_x:{move_x:.2f}, move_y:{move_y:.2f}, move_yaw:{move_yaw:.2f}, different_distance:{different_distance:.2f}"
            )
            last_moveup_time = rclpy.clock.Clock().now()
            # send velocity command#
            if (
                different_distance < 0.5
                and self.flight_info.rangefinder_alt <= LOWEST_HEIGHT
                and (0 <= marker_yaw <= 5 or 355 <= marker_yaw <= 360)
            ):
                self.controller.setZeroVelocity()
                print(f"landing high:{self.flight_info.rangefinder_alt}")
                print(
                    f"x:{marker_x}, y:{marker_y}, z:{marker_z}, yaw:{marker_yaw}, high:{self.flight_info.rangefinder_alt}"
                )
                break
            if self.flight_info.rangefinder_alt > LOWEST_HEIGHT:
                self.controller.sendPositionTargetVelocity(
                    move_x,
                    move_y,
                    DOWNWARD_SPEED,
                    move_yaw,
                )
            else:
                self.controller.sendPositionTargetVelocity(
                    move_x,
                    move_y,
                    0,
                    move_yaw,
                )
            self.node.get_logger().debug(
                f"[Mission.landedOnPlatform] move_x:{move_x:.2f}, move_y:{move_y:.2f}, move_yaw:{move_yaw:.2f}, different_distance:{different_distance:.2f}"
            )
        self.controller.setZeroVelocity()
        print("now I want to land=================================")
        while not self.controller.land():
            print("landing")
        # while self.flight_info.rangefinder_alt > 0.1:
        #     print(f"landing high:{self.flight_info.rangefinder_alt}")
        self.mode = self.WAIT_MODE
        return True

    def navigateTo(
        self,
        destination_x: float,
        destination_y: float,
        destination_z: float,
    ):
        """
        Function to navigate the drone to a specified location.

        The drone moves towards the specified location until it reaches the location.
        The drone stops moving once it reaches the location.

        Args:
            destination_x (float): The x-coordinate of the location.
            destination_y (float): The y-coordinate of the location.
            destination_z (float): The rangefinder altitude of the location.

        Returns:
            None
        """
        # 檢查先前模式是否為等待模式，定且設定目前模式為導航模式
        if self.mode != self.WAIT_MODE:
            return False
        self.__setMode(self.NAVIGATION_MODE)
        # --------------------------------- variable --------------------------------- #
        MAX_SPEED = 0.3
        MAX_YAW = 15 * 3.14 / 180
        bcn_orient_yaw = (
            self.node.get_parameter("bcn_orient_yaw").get_parameter_value().double_value
        )

        # --------------------------------- function --------------------------------- #
        # 如果距離範圍在threshold內就回傳True
        def around(a, b, threshold=0.2):
            return abs(a - b) < threshold

        # ------------------------------- start mission ------------------------------ #
        # 等待取得uwb座標
        while (
            self.flight_info.uwb_coordinate.x == 0
            and self.flight_info.uwb_coordinate.y == 0
            and self.flight_info.uwb_coordinate.z == 0
        ):
            print("waiting for uwb coordinate")
        # 持續計算距離並設置移動速度，還沒到指定位置時繼續移動
        while not (
            around(self.flight_info.uwb_coordinate.x, destination_x)
            and around(self.flight_info.uwb_coordinate.y, destination_y)
        ):
            # 設定中斷點，如果不是前往火源模式就直接結束
            if self.mode != self.NAVIGATION_MODE:
                self.controller.setZeroVelocity()
                return False
            # 取的目前位置與目標位置的差距
            x_diff = destination_x - self.flight_info.uwb_coordinate.x
            y_diff = destination_y - self.flight_info.uwb_coordinate.y
            z_diff = destination_z - self.flight_info.rangefinder_alt
            yaw_diff = math.atan2(y_diff, x_diff) * 180 / math.pi
            # 計算需要旋轉多少角度
            compass_heading = self.flight_info.compass_heading
            rotate_deg = (90 - yaw_diff - compass_heading + bcn_orient_yaw) % 360
            if 360 - rotate_deg < rotate_deg:
                rotate_deg = rotate_deg - 360
            if abs(rotate_deg) < 5:  # 如果角度小於5度就不旋轉
                rotate_deg = 0
            # 限制最大旋轉角度
            move_yaw = min(max(-rotate_deg * 3.14 / 180, -MAX_YAW), MAX_YAW)
            # 將須往前距離當作速度，並且限制最大速度
            move_forward = math.sqrt(x_diff**2 + y_diff**2)
            move_forward = abs(min(max(move_forward, -MAX_SPEED), MAX_SPEED))
            # 高度移動距離，並且限制最大速度
            move_z = min(max(z_diff, -MAX_SPEED), MAX_SPEED)
            self.node.get_logger().debug(
                f"[Mission.navigateTo] rotate_deg: {rotate_deg}, move_forward: {move_forward}, move_yaw: {move_yaw}"
            )
            if abs(rotate_deg) < MAX_YAW:
                self.controller.sendPositionTargetVelocity(
                    move_forward, 0, move_z, move_yaw
                )
            else:
                self.controller.sendPositionTargetVelocity(0, 0, move_z, move_yaw)
        self.controller.setZeroVelocity()
        self.mode = self.WAIT_MODE
        return True
