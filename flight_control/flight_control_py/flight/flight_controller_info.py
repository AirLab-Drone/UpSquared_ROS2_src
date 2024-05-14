import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, NavSatFix
from mavros_msgs.msg import GPSRAW
from std_msgs.msg import Header, Float64


class XYZ:
    __slots__ = ["header", "x", "y", "z"]

    _fields_and_field_types = {
        "header": "std_msgs/Header",
        "x": "float",
        "y": "float",
        "z": "float",
    }

    def __init__(self, header=Header(), x=0, y=0, z=0) -> None:
        self.header = header
        self.x = x
        self.y = y
        self.z = z

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
        self.node.create_subscription(
            Float64,
            "/mavros/global_position/compass_hdg",
            self.compassCallback,
            rclpy.qos.qos_profile_sensor_data,
        )
        # ------------------------------ set init value ------------------------------ #
        self.rangefinder_alt = 0
        self.uwb_coordinate = XYZ()
        self.compass_heading = 0

    def rangefinderCallback(self, msg):
        """
        Callback function for the rangefinder subscription.
        """
        self.rangefinder_alt = msg.range

    def coordinateCallback(self, msg):
        """
        Callback function for the coordinate subscription.
        坐標系:x: 向右為正, y: 向前為正, z: 向上為正
        """
        LSB_M_TO_LAT_LONG = 8.993216059e-6
        # LAT_START = -35.363261         # used in simulation
        # LONG_START = 149.165230        # used in simulation
        LAT_START = 22.5180977
        LONG_START = 113.9007239
        self.coordinate = msg
        lat = msg.lat * 1e-7
        lon = msg.lon * 1e-7
        alt = msg.alt * 1e-3
        x = (lon - LONG_START) / LSB_M_TO_LAT_LONG
        y = (lat - LAT_START) / LSB_M_TO_LAT_LONG
        print(f"x: {x}, y: {y}, z: {alt}")
        # print(f"lat: {lat}, lon: {lon}, alt: {alt}")
        self.uwb_coordinate = XYZ(header=msg.header, x=x, y=y, z=alt)

    def compassCallback(self, msg):
        """
        Callback function for the compass subscription.
        """
        self.compass_heading = msg.data
        # print(f"compass heading: {self.compass_heading}")
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
