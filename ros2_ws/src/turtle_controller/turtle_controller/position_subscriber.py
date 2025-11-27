import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math

class PositionSubscriber(Node):
    def __init__(self):
        super().__init__('position_subscriber')

        # Create subscriber
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.get_logger().info('Position Subscriber Node Started')

    def pose_callback(self, msg):
        self.previous_x = msg.x
        self.previous_y = msg.y

        self.get_logger().info(
            f'Position: x={msg.x:.2f}, y={msg.y:.2f}, '
            f'Orientation: theta={msg.theta:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)

    position_subscriber = PositionSubscriber()

    try:
        rclpy.spin(position_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        position_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
