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

        self.previous_x = 0.0
        self.previous_y = 0.0
        self.total_distance = 0.0

        self.get_logger().info('Position Subscriber Node Started')

    def pose_callback(self, msg):
        # Calculate distance traveled
        if hasattr(self, 'previous_x'):
            dx = msg.x - self.previous_x
            dy = msg.y - self.previous_y
            distance = math.sqrt(dx*dx + dy*dy)
            self.total_distance += distance

        self.previous_x = msg.x
        self.previous_y = msg.y

        self.get_logger().info(
            f'Position: x={msg.x:.2f}, y={msg.y:.2f}, '
            f'theta={msg.theta:.2f}, distance={self.total_distance:.2f}'
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
