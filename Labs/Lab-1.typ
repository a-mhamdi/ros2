#import "Class.typ": *

#show: ieee.with(
  title: [#text(smallcaps("ROS2 Lab #1: Getting Started Guide"))],
 
  abstract: [
    This guide introduces the fundamentals of *ROS2* (Robot Operating System 2) using the TurtleSim simulator. TurtleSim is a simple 2D simulator that helps beginners understand core *ROS2* concepts without the complexity of real hardware.
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

= Environment Setup

== Checking The Shell

First, identify which shell we're currently using:
```bash
which $SHELL
```
This command returns the path to the current shell executable (`bash`, `zsh`, etc.). Different shells may require different setup procedures, so this information is important for proper configuration.

== Sourcing ROS2 Installation

Add ROS2 commands and environment variables to the current terminal session:
```bash
source /opt/ros/humble/setup.bash
```
This step is essential as *ROS2* commands won't be available without sourcing. The sourcing must be done in every new terminal session, or we can add this line to our shell's configuration file (`.bashrc`, `.zshrc`) to make it automatic.

= Running Our First ROS2 Node

A *node* is a fundamental unit of computation. Each node is a separate process that performs a specific task. Nodes communicate with each other through topics, services, and actions. The command structure follows the pattern: 
```bash
ros2 run <package_name> <executable_name>
```
To list all executables in a package:
```bash
ros2 pkg executables <package_name>
```
The following command lists all executables in the `turtlesim` package:
```bash
ros2 pkg executables turtlesim
```
Let’s launch the *TurtleSim* simulator:
```bash
ros2 run turtlesim turtlesim_node
```
This command opens a blue window with a turtle in the center. 

= Understanding Topics and Communication

== Listing Active Topics

*Topics* are named communication channels where nodes can publish _(send)_ or subscribe _(receive)_ messages. 

Display all currently active topics with their message types:
```bash
ros2 topic list -t
```
The `-t` flag shows the message type for each topic. Expected output includes topics such as:
- `/turtle1/cmd_vel` [geometry_msgs/msg/Twist]
- `/turtle1/color_sensor` [turtlesim/msg/Color]  
- `/turtle1/pose` [turtlesim/msg/Pose]

== Inspecting Message Structure

To understand the data structure of messages used in topics, we can inspect a specific message type. This command shows the Twist message structure:
```bash
ros2 interface show geometry_msgs/msg/Twist
```
The Twist message contains linear and angular velocity components for 3D space. 
```bash
geometry_msgs/Vector3 linear
    float64 x
    float64 y  
    float64 z
geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z
```
For the 2D turtle, we primarily use `linear.x` _(forward/backward)_ and `angular.z` _(rotation)_.

= Publishing Messages

== Single Movement Command

If we want to move the turtle forward by 1 unit, we can publish a single message to the `/turtle1/cmd_vel` topic:
```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.}}"
```
Let's breakdown the structure of the previous instruction:
- `ros2 topic pub`: Command to publish to a topic
- `--once`: Publish only one message then exit
- `/turtle1/cmd_vel`: The target topic name
- `geometry_msgs/msg/Twist`: The message type
- `"{linear: {x: 1.}}"`: Message data in YAML format

== Continuous Movement Command

Now, we want to publish velocity commands continuously to create circular motion:
```bash
ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1., y: 0., z: 0.}, angular: {x: 0., y: 0., z: .7}}"
```
The key parameters are:
- `-r 1`: Publish at a 1 Hz _(i.e., 1 message per sec)_
- `linear: {x: 1.}`: Move forward at 1 unit/sec
- `angular: {z: .7}`: Rotate at 0.7 radians/sec

As it can be observed, the combination of forward motion and rotation creates circular movement.

= Monitoring Communication

In *ROS2*, it is crucial to monitor messages being published on a topic in real-time. This can be done using the `ros2 topic echo` command:
```bash
ros2 topic echo /turtle1/cmd_vel
```
This debugging tool displays all messages on the specified topic. When used alongside the continuous movement command, we'll see the velocity messages being printed continuously.

= Interactive Control

== Direct Turtle Teleop

Let's use TurtleSim's built-in keyboard control to teleoperate the turtle:
```bash
ros2 run turtlesim turtle_teleop_key
```
This executable is specifically designed for TurtleSim and publishes directly to `/turtle1/cmd_vel`, eliminating the need for topic remapping.

== Keyboard Teleop with Remapping

In general, it is possible to control the turtle using keyboard input with topic remapping:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/turtle1/cmd_vel
```
The command components are:
- `teleop_twist_keyboard`: Package and executable name
- `--ros-args --remap`: ROS2 argument for topic remapping
- `cmd_vel:=/turtle1/cmd_vel`: Maps the default `cmd_vel` topic to `/turtle1/cmd_vel`

= System Visualization

There is a tool to visualize the relationships between nodes and topics, called `rqt_graph`:
```bash
rqt_graph
```
This opens a graphical tool displaying:
- Nodes as ovals
- Topics as rectangles  
- Arrows showing communication direction

The visualization helps understand data flow in *ROS2* systems and is invaluable for debugging communication issues in complex robotic applications.

#figure(
  image("Images/rqt-graph.png", width: 100%, fit: "contain")
)

= Summary

== Terminology

#rotate(
  -90deg,
  reflow: true,
  table(
  columns: 2,
  stroke: 1pt,
  [*Concept*], [*Description*],
  [Nodes], [Independent processes performing specific tasks],
  [Topics], [Named communication channels for message passing],
  [Messages], [Structured data types (e.g., Twist for velocity)],
  [Publishing], [Sending messages to topics],
  [Subscribing], [Receiving messages from topics],
  [Packages], [Collections of related nodes and resources],
  [Remapping], [Redirecting topic names for system flexibility]
))

== Command Reference

#rotate(
  -90deg,
  reflow: true,
  table(
  columns: 2,
  stroke: 1pt,
  [*Command*], [*Purpose*],
  [`which $SHELL`], [Check current shell],
  [`source /opt/ros/humble/setup.bash`], [Source ROS2 environment],
  [`ros2 run <package> <executable>`], [Run a ROS2 node],
  [`ros2 topic list -t`], [List active topics with types],
  [`ros2 interface show <msg_type>`], [Show message structure],
  [`ros2 topic pub <topic> <type> <data>`], [Publish to topic],
  [`ros2 topic echo <topic>`], [Monitor topic messages],
  [`rqt_graph`], [Visualize system graph]
))

#exo[Fibonacci Spiral][Implement a *ROS2* application using the `turtlesim` package to visualize the Fibonacci spiral. The turtle should trace the golden spiral pattern governed by the mathematical equation:
$ r = a dot Phi ^(theta / (2 * pi)) $
where:
/ $r$: radial distance from the origin,
/ $a$: a constant that controls the size of the spiral,
/ $Phi$: the golden ratio (approximately 1.618),
/ $theta$: the angle in degrees.
]

#test[Use smooth incremental angle steps for continuous curve visualization. You can choose $theta$ ranging from $0$ to $pi/ 4$ with small increments.]

The resulting spiral should smoothly transition as the turtle moves, creating a visually appealing pattern that looks like the image shown below.

#figure(
  image("Images/fibonacci_spiral.png", width:100%)
)

/*
#solution[
```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```
```bash
ros2 service call /reset std_srvs/srv/Empty

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'off': 1}"
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: .5, y: .5, theta: 0.}"
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r': 200, 'width':2, 'off': 0}"

for theta in $(seq 0 .02 .785); do # theta: 0 -> pi/4
	r=$(echo "scale=3; .9 * e($theta / (2 * 3.14159) * l(1.618))" | bc -l)
	ros2 service call /turtle1/teleport_relative turtlesim/srv/TeleportRelative "{'linear': $r, 'angular': $theta}"
	sleep .05
done
```
]
*/

/* ---- End of Lab-1.typ ---- */