import rclpy
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('my_publisher')
    publisher = node.create_publisher(String, 'listen_talk', 10)

    msg = String()
    msg.data = "Hello, ROS 2!"

    while rclpy.ok():
        publisher.publish(msg)
        node.get_logger().info('[TALKER] "%s"' % msg.data)
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
