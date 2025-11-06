import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String


class DemoPub(Node):
    def __init__(self):
        """
        Initialize a DemoPub node, which publishes a message every 1 second.

        Node will be initialized with a publisher to the 'demo_topic' and a
        timer which will trigger the timer_callback every 1 second.
        """
        super().__init__("demo_pub")
        self.i = 1

        # Define QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Reliable communication
            history=HistoryPolicy.KEEP_LAST,  # Keep last message
            depth=10,  # Keep last 10 messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Keep messages until read
        )

        self.publisher_ = self.create_publisher(String, "demo_topic", qos_profile)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """
        Callback function when the timer goes off.

        Publishes a message with the contents 'Hello, ROS 2!' to the 'demo_topic'
        and logs the message at the info level.

        :param self: The instance of the class.
        :type self: rclpy.node.Node
        """
        msg = String()
        msg.data = f"Hello, ROS 2! (#{self.i})"
        self.get_logger().info(f"Publishing: {msg.data}")
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    """
    Entry point for the demo_pub application.

    This function initializes the rclpy library, creates a DemoPub node instance,
    and starts publishing messages at regular intervals.

    The node will continue to publish until the process is interrupted (e.g., via Ctrl-C),
    at which point it will perform cleanup operations and shut down.
    """
    rclpy.init(args=args)
    demo_pub = DemoPub()
    rclpy.spin(demo_pub)
    demo_pub.destroy_node()
    rclpy.shutdown()
