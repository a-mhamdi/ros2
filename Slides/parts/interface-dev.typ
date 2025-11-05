#import "../common.typ": *

= Custom Interface Development
---

*ROS 2* supports three primary types of interfaces for inter-node communication: messages, services, and actions. Each serves a unique purpose in the publish-subscribe or client-server paradigms.

== Interface Types Overview

  == Messages (msg)

  - *Publish-Subscribe* communication
  - One-way data transmission
  - Asynchronous communication
  - Best for: sensor data, status updates, continuous streams

  ```yaml
  # Name: "geometry_msgs/msg/Twist.msg"
  geometry_msgs/Vector3 linear
  geometry_msgs/Vector3 angular
  ```

  ---

  == Services (srv)

  - *Request-Response* communication
  - Synchronous, blocking calls
  - Best for: commands, queries, configuration

  ```yaml
  # Name: "turtlesim/srv/Spawn.srv"
  float64 x
  float64 y
  float64 theta
  string name
  ---
  string name
  ```

  ---

  == Actions (action)

  - *Long-running tasks* with feedback
  - Asynchronous with progress updates
  - Can be cancelled
  - Best for: navigation, manipulation, complex behaviors

  ```yaml
  # Name: "turtlesim/action/RotateAbsolute.action"
  float32 theta
  ---
  float32 delta
  ---
  float32 remaining
  ```

== Interface Management Commands

  == Basic Interface Commands

  ```bash
  ros2 interface <command>
  ```

  ```
  Commands:
    list      List all interface types available
    package   Output a list of available interface types within one package
    packages  Output a list of packages that provide interfaces
    proto     Output an interface prototype
    show      Output the interface definition
  ```

  ---

  ```bash
  ros2 interface package turtlesim
  ```

  ```
  turtlesim/msg/Color
  turtlesim/srv/Spawn
  turtlesim/msg/Pose
  turtlesim/srv/SetPen
  turtlesim/srv/Kill
  turtlesim/srv/TeleportAbsolute
  turtlesim/action/RotateAbsolute
  turtlesim/srv/TeleportRelative
  ```

== Creating Custom Interfaces

  == Package Structure for Custom Interfaces

  ```
  my_interfaces/
  ├── CMakeLists.txt
  ├── package.xml
  ├── msg/
  │   └── SensorData.msg
  ├── srv/
  │   └── GetData.srv
  └── action/
      └── NavigateTo.action
  ```

  ---

  *Key Requirements:*
  - Separate directories for each interface type
  - Proper CMakeLists.txt configuration
  - Correct package.xml dependencies

  == Message Definition (.msg)

  ```yaml
  # Name: "my_interfaces/msg/SensorData.msg"
  # Header with timestamp and frame
  std_msgs/Header header
  # Sensor readings
  float64 temperature
  float64 humidity
  float64 pressure
  # Status information
  bool is_valid
  string sensor_id
  ```

  ---

  *Supported Data Types:*
  - Built-in: `bool`, `int8`, `uint8`, `int16`, `uint16`, `int32`, `uint32`, `int64`, `uint64`, `float32`, `float64`, `string`
  - Arrays: `int32[]`, `string[]`
  - Other messages: `geometry_msgs/Point`

  == Service Definition (.srv)

  ```yaml
  # Name: "my_interfaces/srv/GetData.srv"
  # Request
  string sensor_id
  float64 timeout_seconds
  ---
  # Response
  bool success
  string message
  ```

  ---

  *Structure:*
  - Above `---`: Request fields
  - Below `---`: Response fields
  - Can use custom message types

  == Action Definition (.action)

```yaml
# Name: "my_interfaces/action/NavigateTo.action")
# Goal
geometry_msgs/Point target_position
float64 max_velocity


---
# Result
bool success
string message
float64 final_distance

---
# Feedback
geometry_msgs/Point current_position
float64 progress_percentage
float64 remaining_distance
```

  ---

  *Three Parts:*
  - *Goal*: What to accomplish
  - *Result*: Final outcome
  - *Feedback*: Progress updates

== CMakeLists.txt Configuration

  == Essential CMakeLists.txt Setup

  ```cmake
# Find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  "srv/GetData.srv"
  "action/NavigateTo.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
  )
  ```

  == Package.xml Dependencies

  ```xml
<name>my_interfaces</name>
<version>0.1.0</version>
<description>Custom ROS 2 interfaces</description>

<depend>builtin_interfaces</depend>
<depend>geometry_msgs</depend>
<depend>std_msgs</depend>
  ```

== Building and Using Custom Interfaces

  == Building the Package

  ```bash
  # Build the interface package
  colcon build --packages-select my_interfaces

  # Source the workspace
  source install/setup.bash
  ```

  ---

  *Verification:*
  ```bash
  # List available interfaces
  ros2 interface list | grep my_interface_package

  # Show interface definition
  ros2 interface show my_interfaces/msg/SensorData

  # Check package interfaces
  ros2 interface package my_interfaces
  ```

  == Using Custom Interfaces in Python and C++

  ```python
  from my_interfaces.msg import SensorData
  from my_interfaces.srv import GetData
  from my_interfaces.action import NavigateTo
  ```
  ```cpp
  #include "my_interfaces/msg/sensor_data.hpp"
  #include "my_interfaces/srv/get_data.hpp"
  #include "my_interfaces/action/navigate_to.hpp"
  ```
  ---
  ```python
  # Publisher
  publisher = node.create_publisher(SensorData, 'sensor_data', 10)
  ```
  ```cpp
  // Publisher
  auto publisher = node->create_publisher<my_interfaces::msg::SensorData>(
    "sensor_data", 10);
  ```
  ---
  ```python
  # Service client
  client = node.create_client(GetData, 'get_data')
  ```
  ```cpp
  // Service client
  auto client = node->create_client<my_interfaces::srv::GetData>(
    "get_data");
  ```
  ---
  ```python
  # Action client
  action_client = ActionClient(node, NavigateTo, 'navigate_to')
  ```
  ```cpp
  // Action client
  auto action_client = rclcpp_action::create_client<my_interfaces::action::NavigateTo>(
    node, "navigate_to");
  ```

/*
== Best Practices

  == Interface Design Guidelines

  *Message Design*
  /*
  - Use meaningful field names
  - Include header with timestamp and frame_id
  - Keep messages focused and atomic
  - Use appropriate data types
  */

  *Service Design*
  /*
  - Keep requests simple
  - Provide clear success/failure indicators
  - Include descriptive error messages
  */

  *Action Design*
  /*
  - Design for cancellation
  - Provide meaningful feedback
  - Set reasonable timeouts
  */

  == Versioning and Compatibility

  *Backward Compatibility*
  /*
  - Add new fields at the end
  - Use optional fields when possible
  - Maintain semantic meaning of existing fields
  */

  *Breaking Changes*
  /*
  - Rename fields or types
  - Change data types
  - Reorder required fields
  */

  *Migration Strategy*
  /*
  - Plan deprecation periods
  - Provide migration tools
  - Document changes clearly
  */
*/

== Advanced Topics

  == Interface Validation

  ```python
  # Validate message before publishing
  from rclpy.qos import QoSProfile, ReliabilityPolicy

  # Set up validation
  qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=10
  )

  # Use with validation
  publisher = node.create_publisher(
    SensorData, 'sensor_data', qos_profile=qos
  )
  ```

  == Custom Interface Testing

  ```bash
  # Test interface compilation
  colcon build --packages-select my_interfaces

  # Test interface usage
  ros2 run my_interfaces test_interfaces

  # Validate with rosbag
  ros2 bag record /sensor_data
  ros2 bag play recorded_bag
  ```

  ---

  *Testing Strategies:*
  - Unit tests for interface generation
  - Integration tests for message flow
  - Performance tests for large messages

== Summary

  / Interface Types:
    - Messages: One-way, asynchronous
    - Services: Request-response, synchronous
    - Actions: Long-running with feedback

  / Development Process:
    1. Define interface files (.msg, .srv, .action)
    2. Configure CMakeLists.txt and package.xml
    3. Build with colcon

  / Best Practices:
    - Design for reusability
    - Maintain backward compatibility
    - Test thoroughly
    - Document clearly
