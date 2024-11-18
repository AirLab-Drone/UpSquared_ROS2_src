import math
import rclpy
from rclpy.node import Node
from flight_control_py.tool.PID import PID
from flight_control_py.tool.get_yaml_config import get_yaml_config
from flight_control_py.flight.base_control import BaseControl as FlightControl
from flight_control_py.flight.flight_controller_info import FlightInfo
from aruco_detect_py.aruco import Aruco
from aruco_msgs.msg import Marker
import time
import yaml
from std_msgs.msg import Int32MultiArray, Float32
from thermal_msgs.msg import ThermalAlert
from payload.srv import Spry


class Mission:
    """
    包含都個任務的class，用於導航，降落等功能
    """

    # define mode name
    WAIT_MODE = -1
    TAKE_OFF_MODE = 0
    LANDING_MODE = 1
    LANDING_ON_PLATFORM_MODE = 2
    NAVIGATION_MODE = 3
    FIRE_DISTINGUISH_MODE = 4
    # define control parameter
    LIMIT_SEALING_RANGE = 0.8  # 距離天花板的最小距離
    LOWEST_HEIGHT = 0.6  # 最低可看到aruco的高度 單位:公尺
    HIGHEST_HEIGHT = 3.0  # 最高高度 單位:公尺
    MAX_SPEED = 0.5  # 速度 單位:公尺/秒
    MAX_VERTICAL_SPEED = 0.15  # 速度 單位:公尺/秒
    MAX_YAW = 15 * 3.14 / 180  # 15度/s

    # initial mode
    mode = WAIT_MODE

    def __init__(
        self, controller: FlightControl, flight_info: FlightInfo, node: Node
    ) -> None:
        self.controller = controller
        self.flight_info = flight_info
        self.node = node
        # ----------------------------- subscription_data ---------------------------- #
        self.closest_aruco = None
        self.hot_spot = ThermalAlert()  # 小熱像儀的熱點
        self.last_hot_spot_update_time = rclpy.clock.Clock().now()
        # ------------------------------- subscription ------------------------------- #
        self.sub = self.node.create_subscription(
            Marker, "closest_aruco", self.closest_aruco_callback, 10
        )
        self.sub = self.node.create_subscription(
            ThermalAlert,
            "/coin417rg2_thermal/hot_spot_drone_frame",
            self.hot_spot_callback,
            10,
        )
        # ------------------------------ service client ------------------------------ #
        if not self.node.get_parameter("simulation").get_parameter_value().bool_value:
            self.fire_extinguisher_spry_client = self.node.create_client(Spry, "/spry")
            while not self.fire_extinguisher_spry_client.wait_for_service(
                timeout_sec=1.0
            ):
                self.node.get_logger().info("service not available, waiting again...")
        # ---------------------------- aruco marker config --------------------------- #
        self.markers_config = get_yaml_config("aruco_detect", "aruco_markers.yaml")[
            "aruco_markers"
        ]

    # ---------------------------------------------------------------------------- #
    #                                   callback                                   #
    # ---------------------------------------------------------------------------- #
    def closest_aruco_callback(self, msg):
        self.closest_aruco = Aruco(
            marker_id=msg.id, marker_config=self.markers_config[f"{msg.id}"]
        ).fromMsgMarker2Aruco(msg)

    def hot_spot_callback(self, msg):
        self.last_hot_spot_update_time = rclpy.clock.Clock().now()
        self.hot_spot = msg

    # ---------------------------------------------------------------------------- #
    #                                   Function                                   #
    # ---------------------------------------------------------------------------- #
    def __setMode(self, mode):
        if mode not in [
            self.WAIT_MODE,
            self.TAKE_OFF_MODE,
            self.LANDING_MODE,
            self.LANDING_ON_PLATFORM_MODE,
            self.NAVIGATION_MODE,
            self.FIRE_DISTINGUISH_MODE,
        ]:
            self.node.get_logger().error("not a valid mode")
            return False
        self.mode = mode

    def stopMission(self):
        self.__setMode(self.WAIT_MODE)
        self.controller.setZeroVelocity()

    # ---------------------------------------------------------------------------- #
    #                                    Mission                                   #
    # ---------------------------------------------------------------------------- #
    # def templateMission(self):
    #     # 檢查先前模式是否為等待模式，定且設定目前模式為導航模式
    #     if self.mode != self.WAIT_MODE:
    #         self.stopMission()
    #         return False
    #     self.__setMode(self.TEMPLATE_MODE)
    #     # --------------------------------- variable --------------------------------- #
    #     # ----------------------------------- 開始任務 ----------------------------------- #
    #     if self.mode != self.TEMPLATE_MODE:
    #         self.node.get_logger().info("It's not in template mode")
    #         self.stopMission()
    #         return False
    #     # ----------------------------------- 結束任務 ----------------------------------- #
    #     self.stopMission()
    #     return True

    def simpleTakeoff(self, target_hight=2):
        LIMIT_SEALING_RANGE = self.LIMIT_SEALING_RANGE
        LOWEST_HEIGHT = self.LOWEST_HEIGHT
        count = 0
        self.node.get_logger().info(f"initial target_hight: {target_hight}")
        # 檢查先前模式是否為等待模式，定且設定目前模式為起飛模式
        if self.mode != self.WAIT_MODE:
            self.stopMission()
            return False
        pre_range_sealing = (
            self.flight_info.rangefinder2_range - target_hight
        )  # 預估起飛後離天花板的高度
        # 如果離天花板太近，就將目標高度設定為離天花板的最小距離
        if pre_range_sealing < LIMIT_SEALING_RANGE:
            target_hight = self.flight_info.rangefinder2_range - LIMIT_SEALING_RANGE
        # 如果離地面太近，禁止起飛
        self.node.get_logger().info(f"target_hight: {target_hight}")
        if target_hight < LOWEST_HEIGHT:
            self.node.get_logger().error("too low to takeoff")
            self.stopMission()
            return False
        while not self.controller.armAndTakeoff(alt=target_hight):
            print("armAndTakeoff fail")
            count += 1
            if count > 5:
                self.stopMission()
                return False
        start_time = rclpy.clock.Clock().now()
        # 等待起飛高度到達目標高度
        while abs(self.flight_info.rangefinder_alt - target_hight) > 0.5:
            print(f"hight offset: {self.flight_info.rangefinder_alt - target_hight}")
            if rclpy.clock.Clock().now() - start_time > rclpy.time.Duration(seconds=7):
                self.stopMission()
                self.node.get_logger().error("takeoff time out")
                return False
        self.mode = self.WAIT_MODE
        self.stopMission()
        return True

    def simpleLanding(self):
        while not self.controller.land():
            print("landing")
        while self.flight_info.state.armed:
            # print(f"landing high:{self.flight_info.rangefinder_alt}")
            pass
        self.stopMission()
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
            self.stopMission()
            return False
        self.__setMode(self.LANDING_ON_PLATFORM_MODE)
        # --------------------------------- variable --------------------------------- #
        LIMIT_SEALING_RANGE = self.LIMIT_SEALING_RANGE
        HIGHEST_HEIGHT = self.HIGHEST_HEIGHT
        LOWEST_HEIGHT = self.LOWEST_HEIGHT
        MAX_SPEED = self.MAX_SPEED
        MAX_VERTICAL_SPEED = self.MAX_VERTICAL_SPEED
        MAX_YAW = self.MAX_YAW
        last_moveup_time = rclpy.clock.Clock().now()
        last_not_in_range_time = rclpy.clock.Clock().now()

        # -------------------------------- PID initial ------------------------------- #
        def init_pid():
            current_time = rclpy.clock.Clock().now().nanoseconds
            current_time = rclpy.clock.Clock().now().nanoseconds
            # pid_move_x = PID(0.5, 0.01, 0.005, current_time)  # 修改後的 PID 參數
            # pid_move_y = PID(0.5, 0.01, 0.005, current_time)  # 修改後的 PID 參數
            # pid_move_z = PID(0.3, 0.01, 0.005, current_time, LOWEST_HEIGHT)  # 修改後的 PID 參數
            # pid_move_yaw = PID(0.5, 0.01, 0.005, current_time)
            pid_move_x = PID(0.6, 0.0006, 0.00083, current_time)
            pid_move_y = PID(0.6, 0.0006, 0.00083, current_time)
            pid_move_z = PID(0.4, 0.0006, 0.00083, current_time, LOWEST_HEIGHT)
            pid_move_yaw = PID(0.6, 0.0006, 0.00083, current_time)
            return pid_move_x, pid_move_y, pid_move_yaw, pid_move_z

        pid_move_x, pid_move_y, pid_move_yaw, pid_move_z = init_pid()

        # ------------------------------- look for aruco ------------------------------ #
        def lookForAruco():
            """
            當降落時看不到aruco時的動作
            """
            nonlocal last_moveup_time  # 確保使用外部變數
            if rclpy.clock.Clock().now() - last_moveup_time > rclpy.time.Duration(
                seconds=0.5
            ):
                self.node.get_logger().info("looking for aruco")
                if (
                    self.flight_info.rangefinder_alt < HIGHEST_HEIGHT
                    and self.flight_info.rangefinder2_range > LIMIT_SEALING_RANGE
                ):
                    self.controller.sendPositionTargetVelocity(
                        0, 0, MAX_VERTICAL_SPEED, 0
                    )
                    last_moveup_time = rclpy.clock.Clock().now()
            return init_pid()

        # ------------------------------- start mission ------------------------------ #
        while True:
            # 設定中斷點，如果不是降落模式就直接結束
            # todo 檢查thread結束後是否會釋放記憶體
            if self.mode != self.LANDING_ON_PLATFORM_MODE:
                self.stopMission()
                return False
            # get downward aruco coordinate
            closest_aruco = self.closest_aruco
            if closest_aruco is None:
                pid_move_x, pid_move_y, pid_move_yaw, pid_move_z = lookForAruco()
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
                pid_move_x, pid_move_y, pid_move_yaw, pid_move_z = lookForAruco()
                continue
            # 無人機和降落點的水平誤差
            different_distance = math.sqrt(marker_x**2 + marker_y**2)
            # -------------------------------- PID control ------------------------------- #
            current_time = rclpy.clock.Clock().now().nanoseconds
            move_x = pid_move_x.update(marker_y, current_time)
            move_y = pid_move_y.update(marker_x, current_time)
            move_z = pid_move_z.update(marker_z, current_time)
            marker_yaw = (marker_yaw + 180) % 360 - 180  # 轉換成-180~180
            move_yaw = pid_move_yaw.update(marker_yaw * 3.14 / 180, current_time)
            # ---------------------------------- 限制最大速度 ---------------------------------- #
            different_move = math.sqrt(move_x**2 + move_y**2)
            max_speed_temp = min(max(different_move, -MAX_SPEED), MAX_SPEED)
            move_x = (
                move_x / different_move * max_speed_temp if -30 < marker_yaw < 30 else 0
            )  # 如果偏角太大就不移動
            move_y = (
                move_y / different_move * max_speed_temp if -30 < marker_yaw < 30 else 0
            )
            move_z = (
                min(max(move_z, -MAX_VERTICAL_SPEED), MAX_VERTICAL_SPEED)
                if -30 < marker_yaw < 30
                else 0
            )
            move_yaw = min(max(move_yaw, -MAX_YAW), MAX_YAW)
            last_moveup_time = rclpy.clock.Clock().now()

            print("===========================================================")
            print(
                f"move_x:   {move_x:.2f}, move_y:   {move_y:.2f}, move_z:   {move_z:.2f}, move_yaw:   {move_yaw:.2f}"
            )
            # print(
            #     f"id: {closest_aruco.id}, marker_x: {marker_x:.2f}, marker_y: {marker_y:.2f}, marker_z: {marker_z:.2f}, marker_yaw: {marker_yaw:.2f}"
            # )
            # ------------------ check distance and yaw, whether landing ----------------- #
            if (
                different_distance < 0.1
                and marker_z <= LOWEST_HEIGHT * 1.2
                and (-5 <= marker_yaw <= 5)
            ):
                if (
                    rclpy.clock.Clock().now() - last_not_in_range_time
                    > rclpy.time.Duration(seconds=3)
                ):
                    self.controller.setZeroVelocity()
                    print(f"landing high:{marker_z}")
                    break
                print(
                    f"during last not in range time: {rclpy.clock.Clock().now() - last_not_in_range_time}"
                )
                pass
            else:
                last_not_in_range_time = rclpy.clock.Clock().now()
            # --------------------------- send velocity command -------------------------- #
            self.controller.sendPositionTargetVelocity(move_x, move_y, move_z, move_yaw)
        self.controller.setZeroVelocity()
        print("now I want to land=================================")
        while not self.simpleLanding():
            print("landing")
        # ----------------------------------- 結束任務 ----------------------------------- #
        self.stopMission()
        return True

    def navigateTo(
        self,
        destination_x: float,
        destination_y: float,
        destination_z: float = 2,
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
            bool: True if the drone reaches the location, False otherwise.
        """
        # 檢查先前模式是否為等待模式，定且設定目前模式為導航模式
        if self.mode != self.WAIT_MODE:
            self.stopMission()
            return False
        self.__setMode(self.NAVIGATION_MODE)
        # --------------------------------- variable --------------------------------- #
        LIMIT_SEALING_RANGE = self.LIMIT_SEALING_RANGE
        LOWEST_HEIGHT = self.LOWEST_HEIGHT
        MAX_SPEED = self.MAX_SPEED
        MAX_YAW = self.MAX_YAW
        MAX_VERTICAL_SPEED = self.MAX_VERTICAL_SPEED
        bcn_orient_yaw = (
            self.node.get_parameter("bcn_orient_yaw").get_parameter_value().double_value
        )

        # --------------------------------- function --------------------------------- #
        # 如果距離範圍在threshold內就回傳True    self.spry_pin = gpio.GPIOPin(14, gpio.OUT)
        def around(a, b, threshold=0.2):
            return abs(a - b) < threshold

        def diffZCompute(current_high):
            up_distance = self.flight_info.rangefinder2_range
            down_distance = self.flight_info.rangefinder_alt
            vertical_space = up_distance + down_distance
            if vertical_space < LIMIT_SEALING_RANGE + LOWEST_HEIGHT:
                return 0
            if vertical_space < destination_z + LIMIT_SEALING_RANGE:
                target_high = (
                    vertical_space - LIMIT_SEALING_RANGE - LOWEST_HEIGHT
                ) / 2 + LOWEST_HEIGHT
                return target_high - current_high
            if vertical_space > destination_z + LIMIT_SEALING_RANGE:
                return destination_z - current_high
            return 0

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
            # ----------------------------------- 例外處裡 ----------------------------------- #
            # 設定中斷點，如果不是前往火源模式就直接結束
            if self.mode != self.NAVIGATION_MODE:
                self.stopMission()
                return False
            # 高度空間不足就停止
            if (
                self.flight_info.rangefinder_alt + self.flight_info.rangefinder2_range
                < LIMIT_SEALING_RANGE + LOWEST_HEIGHT
            ):
                self.stopMission()
                self.node.get_logger().error("not enough space to navigate")
                return False
            # 取的目前位置與目標位置的差距
            x_diff = destination_x - self.flight_info.uwb_coordinate.x
            y_diff = destination_y - self.flight_info.uwb_coordinate.y
            z_diff = diffZCompute(self.flight_info.rangefinder_alt)
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
        # ----------------------------------- 結束任務 ----------------------------------- #
        self.stopMission()
        return True

    def fireDistinguish(self):
        # 檢查先前模式是否為等待模式，定且設定目前模式為降落至平台模式
        if self.mode != self.WAIT_MODE:
            self.stopMission()
            return False
        self.__setMode(self.FIRE_DISTINGUISH_MODE)
        # --------------------------------- variable --------------------------------- #
        LIMIT_SEALING_RANGE = self.LIMIT_SEALING_RANGE
        LOWEST_HEIGHT = self.LOWEST_HEIGHT
        MAX_SPEED = self.MAX_SPEED
        MAX_YAW = self.MAX_YAW
        MAX_VERTICAL_SPEED = self.MAX_VERTICAL_SPEED
        TIMEOUT_TIME = 15  # 滅火超時時間
        HOT_SPOT_THRESHOLD = 40  # 熱點溫度閾值
        is_success = True
        # ----------------------------------- 尋找火源 ----------------------------------- #
        # 向前一公尺
        target_distance = 0.32
        current_distance = 0.0
        start_time = rclpy.clock.Clock().now()
        self.node.get_logger().info("start go forward")
        while current_distance < target_distance:
            if self.mode != self.FIRE_DISTINGUISH_MODE:
                self.stopMission()
                return False
            if self.hot_spot.temperature >= HOT_SPOT_THRESHOLD:
                self.node.get_logger().info("hot spot found")
                break
            self.controller.sendPositionTargetVelocity(MAX_SPEED, 0, 0, 0)
            current_distance = (
                MAX_SPEED * (rclpy.clock.Clock().now() - start_time).nanoseconds / 1e9
            )
        self.controller.setZeroVelocity()
        # ----------------------------------- 螺旋尋找火源 ----------------------------------- #
        omega = 0.5
        k = 0.1  # 螺旋擴展速率
        angle = 0
        max_radius = 10.0
        start_time = rclpy.clock.Clock().now()
        self.node.get_logger().info("start spiral")
        while True:
            if self.mode != self.FIRE_DISTINGUISH_MODE:
                self.stopMission()
                return False
            if self.hot_spot.temperature >= HOT_SPOT_THRESHOLD:
                self.node.get_logger().info("hot spot found")
                break
            elapsed = (rclpy.clock.Clock().now() - start_time).nanoseconds / 1e9
            angle = omega * elapsed
            spiral_radius = k * angle
            spiral_radius = min(spiral_radius, max_radius)
            move_x = omega * spiral_radius * math.sin(angle)
            move_y = omega * spiral_radius * math.cos(angle)
            move_distance = math.sqrt(move_x**2 + move_y**2)
            move_speed = min(move_distance, MAX_SPEED)
            move_x = move_x / move_distance * move_speed
            move_y = move_y / move_distance * move_speed
            self.controller.sendPositionTargetVelocity(move_x, move_y, 0, 0)
        self.controller.setZeroVelocity()
        # ----------------------------------- 滅火任務 ----------------------------------- #
        FIRE_EXTINGUISH_START_TIME = rclpy.clock.Clock().now()
        while True:
            # 設定中斷點，如果不是前往火源模式就直接結束
            if self.mode != self.FIRE_DISTINGUISH_MODE:
                self.stopMission()
                return False
            # ----------------------------------- 檢查狀態 ----------------------------------- #
            # 如果時間超過TIMEOUT_TIME就停止
            if (
                rclpy.clock.Clock().now() - FIRE_EXTINGUISH_START_TIME
                > rclpy.time.Duration(seconds=TIMEOUT_TIME)
            ):
                self.node.get_logger().info("fire distinguish time out")
                self.stopMission()
                return False
            # 如果溫度低於60度就停止
            if self.hot_spot.temperature < HOT_SPOT_THRESHOLD:
                self.node.get_logger().info(
                    f"temperature is lower than 60, {self.hot_spot.temperature}"
                )
                self.controller.setZeroVelocity()
                continue
            # 更新時間超過1秒就停止
            during_time = rclpy.clock.Clock().now() - self.last_hot_spot_update_time
            if during_time > rclpy.time.Duration(seconds=1):
                self.node.get_logger().info("hot spot update time out")
                self.controller.setZeroVelocity()
                continue
            # 如果座標都是零就不動
            if (
                self.hot_spot.x == 0
                and self.hot_spot.y == 0
                and self.hot_spot.temperature != 0
            ):
                self.node.get_logger().info("hot spot is zero")
                self.controller.setZeroVelocity()
                continue
            # ----------------------------------- 飛往火源 ----------------------------------- #
            different_move = math.sqrt(self.hot_spot.x**2 + self.hot_spot.y**2)
            if different_move < 0.1:  # 離火原距離小於0.5m就停止，開始滅火
                self.controller.setZeroVelocity()
                break
            # 限制最大速度
            max_speed_temp = min(max(different_move, -MAX_SPEED), MAX_SPEED)
            move_x = -self.hot_spot.y / different_move * max_speed_temp
            move_y = -self.hot_spot.x / different_move * max_speed_temp
            self.node.get_logger().info(f"move_x: {move_x:.2f}, move_y: {move_y:.2f}")
            self.controller.sendPositionTargetVelocity(move_x, move_y, 0, 0)
        # ----------------------------------- 噴灑滅火 ----------------------------------- #
        self.controller.setZeroVelocity()
        self.node.get_logger().info("fire distinguish")
        if not self.node.get_parameter("simulation").get_parameter_value().bool_value:
            self.node.get_logger().info(
                "fire distinguish-----------------------------------------------"
            )
            spry_future = self.fire_extinguisher_spry_client.call_async(Spry.Request())
            self.node.executor.spin_until_future_complete(spry_future, timeout_sec=4)
            if spry_future.result() is None:
                is_success = False
            else:
                is_success = spry_future.result().success
        else:
            time.sleep(2)
            is_success = True
        time.sleep(5)
        self.node.get_logger().info(f"fire distinguish result: {is_success}")
        # ----------------------------------- 結束任務 ----------------------------------- #
        self.stopMission()
        return is_success
