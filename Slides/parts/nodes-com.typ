#import "../common.typ": *

= Nodes and Communication

---

== What is a ROS 2 Node?

A *ROS 2* node is a fundamental computational unit in the Robot Operating System 2 that represents an executable process within a robotic system.

=== Core Components

- Written in C++ or Python
- Uses `rclcpp` (C++) or `rclpy` (Python) client libraries
- Supports multiple communication patterns

---

=== Types of Nodes

==== Sensor Nodes

- Collect and process sensor data
#example[
- Camera node
- LIDAR node
- IMU sensor node
]

---

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

---

#exo("Theory", [
    1. What is a *ROS 2* node?
    2. What is the difference between a topic and a service in *ROS 2*?
    3. Give an example use case for a topic and one for a service.
    ])

---

#solution[
    1. A *ROS 2* node is a fundamental process that performs computation in a *ROS 2* system. Each node is an independent executable that can communicate with other nodes using topics, services, and actions.

    2. A topic in *ROS 2* is used for unidirectional, asynchronous, many-to-many communication (publish/subscribe), suitable for streaming data like sensor readings. A service is used for synchronous, two-way communication (request/response), suitable for tasks that require a reply, like resetting a simulation.

    3. / Example use case for a topic: Publishing sensor data from a LIDAR to be consumed by multiple nodes.
    / Example use case for a service: A node requests another node to reset the simulation environment and waits for confirmation.
]

---

#exo("Practical", [
    4. Write the command to list all active nodes in a running *ROS 2* system.
    5. Suppose you have a node publishing to the topic `/chatter`. Write the command to echo messages from this topic.
    6. What is the command to call a service named `/reset_simulation` of type `std_srvs/srv/Empty`?
    ])

---

#solution[
    4. Command to list all active nodes:
    ```bash
    ros2 node list
    ```

    5. Command to echo messages from the `/chatter` topic:
    ```bash
    ros2 topic echo /chatter
    ```

    6. Command to call the `/reset_simulation` service of type `std_srvs/srv/Empty`:
    ```bash
    ros2 service call /reset_simulation std_srvs/srv/Empty "{}"
    ```
]

---

== Build mechanism

```bash
ros2 pkg create --build-type ament_python <package_name>
```

This command creates a new *ROS 2* package with the specified name, using the `ament_python` build type. The generated package structure will look like this:

---

```
<package_name>/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── <package_name>
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── <package_name>/
    └── __init__.py
```

#align(center)[#image("../imgs/project_struct.png", width: 60%)]

---

#info[Root Level Files]

/ `package.xml`: The package manifest file containing metadata about the package _(dependencies, version, description, maintainer info, etc.)_
/ `setup.py`: Python setup script that defines how the package should be built and installed
/ `setup.cfg`: Configuration file for setup tools, typically contains console script entry points

---

The `package.xml` file is a package manifest for *ROS 2* that describes the package. It's written in *XML* and includes key information like:

- *Metadata:* The package's name, version, a description, maintainer information, and license.
- *Dependencies:* Other packages required for the current package to build and run.
- *Build System Info:* Details on the build type, such as `ament_python`.
- *Export Tags:* Extra information for the *ROS 2* build system.

Essentially, this file is how *ROS 2* manages dependencies and compiles our package.

To install the required dependencies, we need to navigate to the package directory and run:
```bash
rosdep install -i --from-path src/<package_name> --rosdistro humble -y
```

#align(center)[#image("../imgs/pkg_xml.png", width: 70%)]

---

The `setup.py` file is a *Python* script that provides instructions for installing a *ROS 2* package. It includes:

- *Metadata:* Package details such as the name, version, and author, which are often sourced from package.xml.
- *Dependencies:* The required Python packages for the project.
- *Entry Points:* Specifies console scripts that define *ROS 2* nodes, allowing them to be run as executable commands.
- *Data Files:* Information on any extra files, like launch files or configurations, that need to be installed.
- *Package Discovery:* Instructions for setuptools on which *Python* packages to include.

This file uses a standard *Python* packaging mechanism to work with the ament build system, making it possible to install and run our *Python* nodes as *ROS 2* executables.

#align(center)[#image("../imgs/setup_py.png", width: 70%)]

---

#info[Directories]
/ `resource/<package_name>`: Contains a marker file _(usually empty)_ that helps *ROS 2* identify this as a package
/ `test/`: Contains basic test files:
/ `test_copyright.py`: Checks for proper copyright headers
/ `test_flake8.py`: Runs `flake8` linting
/ `test_pep257.py`: Checks docstring conventions
/ `<package_name>/`: The main Python module directory where we'll write our actual *Python* code
/ `__init__.py`: Makes this directory a Python package

#align(center)[#image("../imgs/demo_ros_py.png", width: 70%)]

#code("ros2_ws/src/demo_ros_py")

// #align(center)[#image("../../imgs/demo_ros_cpp.png", width: 70%)]

---

#exo("Mini Code (Python, minimal)", [
    7. Fill in the blanks to create a simple *ROS 2* publisher node in Python:
    ])

```python
import rclpy
from rclpy.node import ___
from std_msgs.msg import ___

class MinimalPublisher(___):
    def __init__(___):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, '___', 10)
        timer_period = ___  # seconds
        self.timer = self.create_timer(timer_period, self.___)

    def ___(___):
        msg = ___()
        msg.data = '___'
        self.publisher_.___(___)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    ___ = MinimalPublisher()
    rclpy.spin(___)
    ___.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    ```

    /*
    ```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        ```
         */
