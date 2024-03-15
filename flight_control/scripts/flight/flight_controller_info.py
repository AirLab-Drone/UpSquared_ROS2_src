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
