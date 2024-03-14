import time
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from sensor_msgs.msg import Range


class FlightControl:
    """透過mavros控制無人機飛行。"""

    def __init__(self, node: Node):
        if not rclpy.ok():
            rclpy.init()
        self.node = node
        self.publisher = self.node.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10
        )
        self.msg = self.setInitPositionTarget()

    def simpleFlight(self, x, y, z, duration):
        """
        @param x: x-axis velocity in m/s
        @param y: y-axis velocity in m/s
        @param z: z-axis velocity in m/s
        @param duration: duration in second
        使無人機飛行x,y,z軸的速度，單位為m/s，持續duration秒。
        """
        start_time = time.time()
        while time.time() - start_time < duration:
            self.sendPositionTargetVelocity(x, y, z)
            time.sleep(0.1)
        self.sendPositionTargetVelocity(0, 0, 0)

    def sendPositionTargetVelocity(self, x, y, z, yaw_rate=0):
        """
        @param x: x-axis velocity in m/s, positive for forward, negative for backward
        @param y: y-axis velocity in m/s, positive for left, negative for right
        @param z: z-axis velocity in m/s, positive for up, negative for down
        @param yaw_rate:CANNOT WORKING, yaw rate in rad/s, positive for clockwise, negative for counterclockwise
        傳送x,y,z軸的速度給飛控，單位為m/s。
        """
        self.msg.velocity.x = float(x)
        self.msg.velocity.y = float(y)
        self.msg.velocity.z = float(z)
        self.msg.yaw_rate = float(yaw_rate)
        # print(
        #     f"x: {self.msg.velocity.x}, y: {self.msg.velocity.y}, z: {self.msg.velocity.z}, yaw_rate: {self.msg.yaw_rate}"
        # )
        self.publisher.publish(self.msg)

    def sendPositionTargetPosition(self, x, y, z, yaw=0):
        """
        @param x: x-axis position in m, positive for forward, negative for backward
        @param y: y-axis position in m, positive for left, negative for right
        @param z: z-axis position in m, positive for up, negative for down
        @param yaw: yaw in rad, positive for counterclockwise, negative for clockwise
        傳送x,y,z軸的位置給飛控，單位為m。
        """
        self.msg.position.x = float(x)
        self.msg.position.y = float(y)
        self.msg.position.z = float(z)
        self.msg.yaw = float(yaw)
        # print(
        #     f"x: {self.msg.position.x}, y: {self.msg.position.y}, z: {self.msg.position.z}, yaw: {self.msg.yaw}"
        # )
        self.publisher.publish(self.msg)

    def setInitPositionTarget(self):
        """
        @return: mavros_msgs.msg.PositionTarget
        設定PositionTarget的初始值，定義飛行加速度與起始速度0。
        """
        msg_obj = PositionTarget()
        msg_obj.coordinate_frame = 8
        msg_obj.type_mask = 128
        msg_obj.position.x = 0.0
        msg_obj.position.y = 0.0
        msg_obj.position.z = 0.0
        msg_obj.velocity.x = 0.0
        msg_obj.velocity.y = 0.0
        msg_obj.velocity.z = 0.0
        msg_obj.acceleration_or_force.x = 0.5
        msg_obj.acceleration_or_force.y = 0.5
        msg_obj.acceleration_or_force.z = 0.5
        msg_obj.yaw = 0.0
        msg_obj.yaw_rate = 0.0
        return msg_obj

    def setZeroVelocity(self):
        """使無人機速度歸零，懸停於空中。"""
        msg = self.setInitPositionTarget()
        self.publisher.publish(msg)

    def armAndTakeoff(self, alt=1.2):
        """
        @param alt: altitude in meter
        啟動馬達並起飛。
        """
        if not self.setMode():
            return False
        if not self.armed():
            return False
        if not self.takeoff(alt):
            return False
        return True

    def setMode(self, mode: str = "4"):
        set_mode_client = self.node.create_client(SetMode, "/mavros/set_mode")
        set_mode_request = SetMode.Request(base_mode=0, custom_mode=mode)
        future_set_mode = set_mode_client.call_async(set_mode_request)
        rclpy.spin_until_future_complete(self.node, future_set_mode, timeout_sec=5)
        print(future_set_mode.result())
        if future_set_mode.result() is None:
            return False
        if not future_set_mode.result().mode_sent:
            return False
        time.sleep(2)
        return True

    def armed(self):
        arming_client = self.node.create_client(CommandBool, "/mavros/cmd/arming")
        arming_request = CommandBool.Request(value=True)
        future_arming = arming_client.call_async(arming_request)
        rclpy.spin_until_future_complete(self.node, future_arming, timeout_sec=5)
        print(future_arming.result())
        if future_arming.result() is None:
            return False
        if not future_arming.result().success:
            return False
        time.sleep(2)
        return True

    def takeoff(self, alt=3.2):
        if type(alt) != float:
            alt = float(alt)
        print('takeoff alt:', alt)
        takeoff_client = self.node.create_client(CommandTOL, "/mavros/cmd/takeoff")
        takeoff_request = CommandTOL.Request(altitude=alt)
        future_takeoff = takeoff_client.call_async(takeoff_request)
        rclpy.spin_until_future_complete(self.node, future_takeoff, timeout_sec=5)
        print(future_takeoff.result())
        if future_takeoff.result() is None:
            return False
        if not future_takeoff.result().success:
            return False
        time.sleep(6)
        return True

    def land(self):
        land_client = self.node.create_client(CommandTOL, "/mavros/cmd/land")
        land_request = CommandTOL.Request()
        future_land = land_client.call_async(land_request)
        rclpy.spin_until_future_complete(self.node, future_land, timeout_sec=5)
        print(future_land.result())
        if future_land.result() is None:
            return False
        if not future_land.result().success:
            return False
        return True

    def destroy(self):
        self.node.destroy_node()
        rclpy.shutdown()


class FlightInfo:
    """
    Class representing flight information.
    """

    def __init__(self, node: Node) -> None:
        if not rclpy.ok():
            rclpy.init()
        self.node = node
        self.subscription_rangefinder_pub = self.node.create_subscription(
            Range,
            "/mavros/rangefinder_pub",
            self.rangefinderCallback,
            rclpy.qos.qos_profile_sensor_data,
        )
        # ------------------------------ set init value ------------------------------ #
        self.rangefinder_alt = 0

    def rangefinderCallback(self, msg):
        """
        Callback function for the rangefinder subscription.
        """
        print(f'rangefinder callback: {msg.range}')
        self.rangefinder_alt = msg.range

    def destroy(self):
        """
        Destroys the node and shuts down the ROS 2 system.
        """
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    rclpy.init()
    node_main = rclpy.create_node("flight_control_test")
    controler = FlightControl(node_main)
    while not controler.armAndTakeoff(alt=2):
        print("armAndTakeoff fail")
    time.sleep(5)
    controler.sendPositionTargetPosition(0, 0, 0, 90*3.14159/180)
    controler.sendPositionTargetPosition(0, 0, 0, 90*3.14159/180)
    controler.sendPositionTargetPosition(0, 0, 0, 90*3.14159/180)
    controler.sendPositionTargetPosition(0, 0, 0, 90*3.14159/180)