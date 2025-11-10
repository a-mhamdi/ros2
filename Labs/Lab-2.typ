#import "Class.typ": *

#show: ieee.with(
  title: [#text(smallcaps("ROS 2 Lab #2: Motion Control of Turtle"))],

  abstract: [
    This lab establishes the foundation for building complex *ROS 2* applications through custom node development and inter-node communication patterns. It builds upon the fundamental concepts learned in Lab \#1. You will create your own *ROS 2* package, write custom nodes, and implement publisher-subscriber communication patterns. By the end of this lab, you will have a solid understanding of how to build *ROS 2* applications from scratch.
  ],

  authors:
  (
    (
      name: "Abdelbacet Mhamdi",
      department: [Senior-lecturer, Dept. of EE],
      organization: [ISET Bizerte --- Tunisia],
      profile: "a-mhamdi",
    ),

    /*
    (
      name: "Student 1",
      department: [Dept. of EE],
      organization: [ISET Bizerte --- Tunisia],
      profile: "abc",
    ),
    (
      name: "Student 2",
      department: [Dept. of EE],
      organization: [ISET Bizerte --- Tunisia],
      profile: "abc",
    ),
  */

  )
  // index-terms: (""),
  // bibliography-file: "Biblio.bib",
)

= Project Overview

== Workspace Setup

Create a *ROS 2* workspace directory structure:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

The `src` directory will contain all your *ROS 2* packages. This workspace structure is the standard convention for organizing *ROS 2* projects.

== Package Creation

Create a new Python package called `turtle_controller`:

```bash
ros2 pkg create --build-type ament_python turtle_controller --dependencies rclpy geometry_msgs turtlesim
```

/ `--build-type ament_python`: Specifies a Python package
/ `turtle_controller`: Package name
/ `--dependencies`: Lists required packages

This creates the following structure:
```
turtle_controller/
├── package.xml
├── setup.py
├── setup.cfg
├── turtle_controller/
│   └── __init__.py
└── resource/
    └── turtle_controller
```

= Writing Your First Publisher Node

== Simple Velocity Publisher

Navigate to your package directory and create a publisher node:

```bash
cd ~/ros2_ws/src/turtle_controller/turtle_controller
```

Create `velocity_publisher.py`:

```python
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
        self.timer = self.create_timer(timer_period, self.timer_callback)

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
```

/ `Node()`: class inheritance for *ROS 2* functionality
/ `create_publisher()`: method with message type, topic, and queue size
/ `timer_callback()`: Timer-based callbacks for periodic execution
/ `get_logger().info()`: Logging

== Configuring the Package

Edit `setup.py` to include your executable:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'turtle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='A. Mhamdi',
    maintainer_email='a_mhamdi@outlook.com',
    description='Turtle controller package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_publisher = turtle_controller.velocity_publisher:main',
        ],
    },
)
```

#exo[Position Monitor Subscriber][Create `position_subscriber.py` file where you write the code for the position subscriber node. Add the executable to your package executables list. Update `setup.py` to include the new executable.]

/*
```python
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
```

```python
entry_points={
    'console_scripts': [
        'velocity_publisher = turtle_controller.velocity_publisher:main',
        'position_subscriber = turtle_controller.position_subscriber:main',
    ],
},
```
*/

#exo[Intelligent Turtle Controller][You are asked to develop an intelligent controller that both subscribes to position and publishes velocity commands. Call the file `smart_controller.py`.]

/*
```python
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

        # Control parameters
        self.target_x = 9.0
        self.target_y = 9.0
        self.distance_threshold = 0.1
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
```
*/

= Building and Running Your Package

== Build Process

Build your package from the workspace root:

```bash
cd ~/ros2_ws
colcon build --packages-select turtle_controller
```

Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

#info[Open a new terminal for each node and source the workspace again.]

== Running Individual Nodes

Start TurtleSim:
```bash
ros2 run turtlesim turtlesim_node
```

Run your velocity publisher:
```bash
ros2 run turtle_controller velocity_publisher
```

Run your position subscriber (in another terminal):
```bash
ros2 run turtle_controller position_subscriber
```

Run the smart controller:
```bash
ros2 run turtle_controller smart_controller
```

Optional teleop for interactive testing:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

== Launch Files

Create a `launch` directory and launch file:

```bash
mkdir -p ~/ros2_ws/src/turtle_controller/launch
```

Create `turtle_lab.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='turtle_controller',
            executable='position_subscriber',
            name='position_monitor'
        ),
        Node(
            package='turtle_controller',
            executable='smart_controller',
            name='smart_controller'
        )
    ])
```

Update `setup.py` to include launch files:

```python
import os
from glob import glob

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]
```

Run the launch file:

```bash
ros2 launch turtle_controller turtle_lab.launch.py
```

= Summary

#rotate(
  -90deg,
  reflow: true,
  table(
    columns: 2,
    stroke: 1pt,
    [*Concept*], [*Description*],
    [Package Structure], [Organized collection of nodes, launch files, and dependencies],
    [Node Inheritance], [Extending rclpy.node.Node for *ROS 2* functionality],
    [Publishers], [Nodes that send messages to topics],
    [Subscribers], [Nodes that receive messages from topics],
    [Callbacks], [Functions triggered by events (timers, messages)],
    [Launch Files], [Scripts to start multiple nodes simultaneously],
    [Workspace], [Development environment containing multiple packages],
    [Colcon], [Build system for *ROS 2* packages]
  )
)

#rotate(
  -90deg,
  reflow: true,
  table(
    columns: 2,
    stroke: 0.5pt,
    [*Command*], [*Purpose*],
    [`ros2 pkg create`], [Create new *ROS 2* package],
    [`colcon build`], [Build *ROS 2* packages],
    [`ros2 run <pkg> <executable>`], [Run a specific node],
    [`ros2 launch <pkg> <launch_file>`], [Run launch file],
    [`ros2 node list`], [List active nodes],
    [`ros2 node info <node>`], [Show node details],
    [`ros2 topic hz <topic>`], [Show topic frequency],
    [`ros2 topic bw <topic>`], [Show topic bandwidth]
    )
)

/* ---- End of Lab-2.typ ---- */
