import rclpy
from std_msgs.msg import String


def callback(msg):
    """Callback function for the subscriber"""
    print('[LISTENER] "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2

    node = rclpy.create_node("my_subscriber")  # Create a node

    # Create a subscription that listens to messages of type String on the 'listen_talk' topic with a queue size of 10
    subscriber = node.create_subscription(
        String,
        "listen_talk",
        callback,
        10,  # QoS profile
    )
    rclpy.spin(node)  # Spin the node

    node.destroy_node()  # Destroy the node
    rclpy.shutdown()  # Shutdown ROS2


if __name__ == "__main__":
    main()
