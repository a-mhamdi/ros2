#import "../common.typ": *

= Nodes and Communication

---

== What is a ROS2 Node?

A *ROS2* node is a fundamental computational unit in the Robot Operating System 2 that represents an executable process within a robotic system. Unlike *ROS1*, *ROS2* introduces significant improvements in performance, security, and real-time capabilities.

=== Core Components

- Written in C++ or Python
- Uses `rclcpp` (C++) or `rclpy` (Python) client libraries
- Supports multiple communication patterns

=== Types of Nodes

==== Sensor Nodes

- Collect and process sensor data
#example[
  - Camera node
  - LIDAR node
  - IMU sensor node
]

==== Actuator Nodes

- Control robotic actuators
- Manage precise motor movements

==== Processing Nodes

- Implement computational algorithms
- Perform data analysis and decision-making

#align(center)[#image("../imgs/tikz/node.svg", width: 55%)]

```bash
# Launch individual nodes
ros2 run <pkg_name> <node_name>
# Launch multiple nodes with configuration
ros2 launch <pkg_name> <launch_file>
# Display active nodes
ros2 node list
# Show detailed information about a specific node
ros2 node info <node_name>
```

---

== Communication Patterns

*Topics*, *services*, and *actions* are core communication mechanisms used to enable nodes to exchange data, request computations, or manage long-running tasks.

=== Topics

- Provide a publish-subscribe communication model
- Allow nodes to send messages to multiple subscribers
- Useful for broadcasting information

```bash
# Publisher
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello, ROS 2!'"
# Subscriber
ros2 topic echo /chatter
```

---

=== Services

- Provide a request-response communication model
- Allow nodes to call functions on other nodes
- Useful for tasks that require a single response

```bash
# Server
ros2 run demo_nodes_cpp add_two_ints_server
# Client
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

---

=== Actions

- Designed for long-running tasks
- Allow feedback during execution
- Support preemption and cancellation
