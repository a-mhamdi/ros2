import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')

        # Create publisher
        self.publisher_ = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # Timer for periodic publishing
        timer_period = 0.1  # 10 Hz
        self.timer= self.create_timer(timer_period, self.timer_callback)

        # Initialize variables
        self.counter = 0.0

        self.get_logger().info('Velocity Publisher Node Started')

    def timer_callback(self):
        msg= Twist()

        # Create circular motion
        msg.linear.x = 2.0
        msg.angular.z = 1.0 * math.sin(self.counter)

        self.publisher_.publish(msg)
        self.counter += 0.1

        self.get_logger().info(
            f'Publishing: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = VelocityPublisher()

    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
