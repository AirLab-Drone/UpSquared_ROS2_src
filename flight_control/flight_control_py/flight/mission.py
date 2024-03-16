import math
import rclpy
from rclpy.node import Node
from flight_control_py.tool.PID import PID
from flight_control_py.flight.base_control import BaseControl as FlightControl
from flight_control_py.flight.flight_controller_info import FlightInfo
from flight_control_py.aruco_visual.aruco import Aruco
from flight_control.srv import GetCloestAruco


class Mission:
    """
    包含都個任務的class，用於導航，降落等功能
    """

    def __init__(self, controller: FlightControl, flight_info: FlightInfo, node: Node) -> None:
        self.controller = controller
        self.flight_info = flight_info
        self.node = node
        self.getCloestArucoClient = self.node.create_client(GetCloestAruco, 'get_cloest_aruco')
            

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
            get_cloest_aruco_future = self.getCloestArucoClient.call_async(GetCloestAruco.Request())
            rclpy.spin_until_future_complete(self.node, get_cloest_aruco_future)
            closest_aruco = None
            if get_cloest_aruco_future.result().valid:
                closest_aruco = Aruco().fromMsgMarker2Aruco(get_cloest_aruco_future.result().aruco)

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
            move_yaw = -pid_yaw.PID(
                yaw, self.controller.node.get_clock().now().nanoseconds * 1e-9
            )

            diffrent_distance = math.sqrt(x**2 + y**2)
            # -------------------------- limit move_x and move_y and move_yaw------------------------- #
            move_x = min(max(x, -max_speed), max_speed)
            move_y = min(max(y, -max_speed), max_speed)
            move_yaw = min(max(move_yaw * 3.14159 / 180, -max_yaw), max_yaw)
            # ----------------------------- send velocity command ----------------------------- #
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
