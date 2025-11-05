import rclpy
from std_msgs.msg import String
from time import sleep


def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2

    node = rclpy.create_node("my_publisher")  # Create a node

    # Create a publisher that will publish messages of type String on the 'listen_talk' topic with a queue size of 10
    publisher = node.create_publisher(String, "listen_talk", 10)

    msg = String()

    i = 0
    while rclpy.ok():
        i += 1
        msg.data = f"Hello, ROS 2! (#{str(i)})"
        publisher.publish(msg)  # Publish the message
        node.get_logger().info('[TALKER] "%s"' % msg.data)
        sleep(1)

    rclpy.spin(node)  # Spin the node

    node.destroy_node()  # Destroy the node
    rclpy.shutdown()  # Shutdown ROS2


if __name__ == "__main__":
    main()
