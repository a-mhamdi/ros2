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

== Build mechanism

```bash
ros2 pkg create --build-type ament_python <package_name>
```

#align(center)[#image("../imgs/project_struct.png", width: 60%)]

This command creates a new *ROS2* package with the specified name, using the `ament_python` build type. The generated package structure will look like this:

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

---

#info[Root Level Files]
/ `package.xml`: The package manifest file containing metadata about the package _(dependencies, version, description, maintainer info, etc.)_
/ `setup.py`: Python setup script that defines how the package should be built and installed
/ `setup.cfg`: Configuration file for setup tools, typically contains console script entry points

To install the required dependencies, we need to navigate to the package directory and run:
```bash
rosdep install -i --from-path src/<package_name> --rosdistro humble -y   
```

---

The `package.xml` file is a package manifest for *ROS2* that describes the package. It's written in *XML* and includes key information like:

- *Metadata:* The package's name, version, a description, maintainer information, and license.
- *Dependencies:* Other packages required for the current package to build and run.
- *Build System Info:* Details on the build type, such as `ament_python`.
- *Export Tags:* Extra information for the *ROS2* build system.

Essentially, this file is how *ROS2* manages dependencies and compiles our package.

#align(center)[#image("../imgs/pkg_xml.png", width: 70%)]

---

The `setup.py` file is a *Python* script that provides instructions for installing a *ROS2* package. It includes:

- *Metadata:* Package details such as the name, version, and author, which are often sourced from package.xml.
- *Dependencies:* The required Python packages for the project.
- *Entry Points:* Specifies console scripts that define *ROS2* nodes, allowing them to be run as executable commands.
- *Data Files:* Information on any extra files, like launch files or configurations, that need to be installed.
- *Package Discovery:* Instructions for setuptools on which *Python* packages to include.

This file uses a standard *Python* packaging mechanism to work with the ament build system, making it possible to install and run our *Python* nodes as *ROS2* executables.

#align(center)[#image("../imgs/setup_py.png", width: 70%)]

---

#info[Directories]
/ `resource/<package_name>`: Contains a marker file _(usually empty)_ that helps *ROS2* identify this as a package
/ `test/`: Contains basic test files:
  / `test_copyright.py`: Checks for proper copyright headers
  / `test_flake8.py`: Runs `flake8` linting
  / `test_pep257.py`: Checks docstring conventions
/ `<package_name>/`: The main Python module directory where we'll write our actual *Python* code
  / `__init__.py`: Makes this directory a Python package


#align(center)[#image("../imgs/demo_ros_py.png", width: 70%)]
// #align(center)[#image("../../imgs/demo_ros_cpp.png", width: 70%)]
