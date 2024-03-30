import math
import rclpy
from rclpy.node import Node
from flight_control_py.tool.PID import PID
from flight_control_py.flight.base_control import BaseControl as FlightControl
from flight_control_py.flight.flight_controller_info import FlightInfo
from flight_control_py.aruco_visual.aruco import Aruco
from flight_control.srv import GetCloestAruco
from aruco_msgs.msg import Marker


class Mission:
    """
    包含都個任務的class，用於導航，降落等功能
    """

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

    def cloest_aruco_callback(self, msg):
        self.cloest_aruco = Aruco(msg.id).fromMsgMarker2Aruco(msg)

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
        max_speed = 0.3  # 速度 單位:公尺/秒
        max_yaw = 15 * 3.14 / 180  # 15度
        downward_speed = -0.2  # the distance to move down
        pid_x = PID(
            0.2,
            0,
            0,
            0,
            time=self.controller.node.get_clock().now().nanoseconds * 1e-9,
        )
        pid_y = PID(
            0.2,
            0,
            0,
            0,
            time=self.controller.node.get_clock().now().nanoseconds * 1e-9,
        )
        pid_yaw = PID(
            0.1,
            0.05,
            0,
            0,
            time=self.controller.node.get_clock().now().nanoseconds * 1e-9,
        )
        last_moveup_time = rclpy.clock.Clock().now()
        while True:
            # rclpy.spin_once(self.flight_info.node)
            # ----------------------- get downward aruco coordinate ---------------------- #
            closest_aruco = self.cloest_aruco
            if closest_aruco is None:
                if rclpy.clock.Clock().now() - last_moveup_time > rclpy.time.Duration(
                    seconds=0.5
                ):
                    if self.flight_info.rangefinder_alt < 3:
                        # print('move up')
                        self.controller.sendPositionTargetVelocity(0, 0, 0.2, 0)
                        last_moveup_time = rclpy.clock.Clock().now()
                # self.controller.setZeroVelocity()
                continue
            x, y, z, yaw, _, _ = closest_aruco.getCoordinate()
            if x is None or y is None or z is None or yaw is None:
                if rclpy.clock.Clock().now() - last_moveup_time > rclpy.time.Duration(
                    seconds=0.5
                ):
                    if self.flight_info.rangefinder_alt < 3:
                        # print('move up')
                        self.controller.sendPositionTargetVelocity(0, 0, 0.2, 0)
                        last_moveup_time = rclpy.clock.Clock().now()
                # self.controller.setZeroVelocity()
                continue
            last_moveup_time = rclpy.clock.Clock().now()
            # -------------------------------- PID control ------------------------------- #
            move_x = pid_x.PID(
                x, self.controller.node.get_clock().now().nanoseconds * 1e-9
            )
            move_y = pid_y.PID(
                y, self.controller.node.get_clock().now().nanoseconds * 1e-9
            )
            move_yaw = pid_yaw.PID(
                yaw, self.controller.node.get_clock().now().nanoseconds * 1e-9
            )  # convert to radians
            # print(f"x:{move_x}, y:{move_y}, yaw:{move_yaw}, high:{self.flight_info.rangefinder_alt}")
            diffrent_distance = math.sqrt(x**2 + y**2)
            # -------------------------- limit move_x and move_y and move_yaw------------------------- #
            move_x = min(max(-x, -max_speed), max_speed)
            move_y = min(max(-y, -max_speed), max_speed)
            if((360-yaw)<yaw):
                yaw = -yaw
            move_yaw = min(max(-yaw * 3.14 / 180, -max_yaw), max_yaw)
            # ----------------------------- send velocity command ----------------------------- #
            if (
                diffrent_distance < 0.03
                and self.flight_info.rangefinder_alt <= lowest_high
                and (0<yaw < 5 or 355<yaw<360)
            ):
                self.controller.setZeroVelocity()
                print(f"landing high:{self.flight_info.rangefinder_alt}")
                print(
                    f"x:{x}, y:{y}, z:{z}, yaw:{yaw}, high:{self.flight_info.rangefinder_alt}"
                )
                break
            self.controller.sendPositionTargetPosition(0, 0, 0, yaw=move_yaw)
            if self.flight_info.rangefinder_alt > lowest_high:
                self.controller.sendPositionTargetVelocity(
                    move_y,
                    move_x,
                    downward_speed,
                    0,
                )
            else:
                # when height is lower than lowest_high, stop moving down
                self.controller.sendPositionTargetVelocity(
                    move_y,
                    move_x,
                    0,
                    0,
                )
            print(
                f"move_x:{move_y}, move_y:{move_x}, move_yaw:{move_yaw}, diffrent_distance:{diffrent_distance}"
            )
            # time.sleep(1)
        self.controller.setZeroVelocity()
        print("now I want to land=================================")
        while not self.controller.land():
            print("landing")
