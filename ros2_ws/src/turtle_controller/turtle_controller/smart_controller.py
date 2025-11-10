import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class SmartController(Node):
    def __init__(self):
        super().__init__('smart_controller')

        # Publisher for velocity commands
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # Subscriber for turtle position
        self.pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        
        # Declare all params
        self.declare_parameter('target_x', 5.)
        self.declare_parameter('target_y', 10.)
        self.declare_parameter('distance_threshold', .2)

        # Control parameters
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.current_pose = None

        self.get_logger().info('Smart Controller Node Started')
        self.get_logger().info(f'Target: ({self.target_x}, {self.target_y})')

    def pose_callback(self, msg):
        self.current_pose = msg
        self.move_to_target()

    def move_to_target(self):
        if self.current_pose is None:
            return

        # Calculate distance to target
        dx = self.target_x - self.current_pose.x
        dy = self.target_y - self.current_pose.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Calculate angle to target
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.current_pose.theta

        # Normalize angle difference
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Create velocity command
        cmd = Twist()

        if distance > self.distance_threshold:
            # Move towards target
            cmd.linear.x = min(2.0 * distance, 2.0)  # Proportional control
            cmd.angular.z = 4.0 * angle_diff  # Proportional angular control
        else:
            # Stop when close to target
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Target reached!')

        self.cmd_publisher.publish(cmd)

        self.get_logger().info(
            f'Distance: {distance:.2f}, Angle diff: {angle_diff:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)

    smart_controller = SmartController()

    try:
        rclpy.spin(smart_controller)
    except KeyboardInterrupt:
        pass
    finally:
        smart_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
