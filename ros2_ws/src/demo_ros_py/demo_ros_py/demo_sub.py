import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class DemoSub(Node):
    def __init__(self):
        """
        Initialize a DemoSub node, which subscribes to the 'demo_topic'.
        """
        super().__init__("demo_sub")
        self.subscription_ = self.create_subscription(
            String,
            "demo_topic",
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: String):
        """
        Callback function when a new message is received from the 'demo_topic'.

        Prints out the received message.

        :param msg: The received message.
        :type msg: std_msgs.msg.String
        """
        self.get_logger().info(f"Received: {msg.data}")


def main(args=None):
    """
    Entry point for the demo_sub application.

    This function parses command line arguments, initializes the rclpy logging
    and other infrastructure, creates a DemoSub node instance, and starts
    processing messages.

    The node will stop when the process receives a SIGINT (Ctrl-C) and clean up
    will be done.
    """
    rclpy.init(args=args)
    demo_sub = DemoSub()
    rclpy.spin(demo_sub)
    demo_sub.destroy_node()
    rclpy.shutdown()
