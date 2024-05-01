import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, NavSatFix
from mavros_msgs.msg import GPSRAW


class FlightInfo:
    """
    Class representing flight information.
    """

    def __init__(self, node: Node) -> None:
        if not rclpy.ok():
            rclpy.init()
        self.node = node
        self.node.create_subscription(
            Range,
            "/mavros/rangefinder_pub",
            self.rangefinderCallback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.node.create_subscription(
            GPSRAW,
            "/mavros/gpsstatus/gps1/raw",
            self.coordinateCallback,
            rclpy.qos.qos_profile_sensor_data,
        )
        # ------------------------------ set init value ------------------------------ #
        self.rangefinder_alt = 0

    def rangefinderCallback(self, msg):
        """
        Callback function for the rangefinder subscription.
        """
        self.rangefinder_alt = msg.range

    def coordinateCallback(self, msg):
        """
        Callback function for the coordinate subscription.
        """
        LSB_M_TO_LAT_LONG = 8.993216059e-6
        LAT_START = 22.5180977
        LONG_START = 113.9007239
        self.coordinate = msg
        lat = msg.lat * 1e-7
        lon = msg.lon * 1e-7
        alt = msg.alt * 1e-3
        x = (lon - LONG_START) / LSB_M_TO_LAT_LONG
        y = (lat - LAT_START) / LSB_M_TO_LAT_LONG
        # print(f"lat: {lat}, lon: {lon}, alt: {alt}")
        print(f"x: {x}, y: {y}, z: {alt}")

    def destroy(self):
        """
        Destroys the node and shuts down the ROS 2 system.
        """
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("flight_info_node")
    flight_info = FlightInfo(node)
    rclpy.spin(node)
