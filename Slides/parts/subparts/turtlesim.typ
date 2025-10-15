#import "../../common.typ": *

== A Practical Introduction

*TurtleSim* is a beginner-friendly tool for learning *ROS2* concepts through visual, interactive examples. This lightweight simulator offers a 2D environment where we control virtual turtles that can move around, draw lines, and respond to commands. 

=== Installation

*TurtleSim* comes pre-installed with the *ROS2* desktop installation. We can also install it using the following command:
```bash
sudo apt install ros-humble-turtlesim # For Ubuntu 22: ROS2 Humble
```
The way to verify the installation is by checking the available executables:
```bash
ros2 pkg executables turtlesim
```

---

=== Launching TurtleSim

The first step is launching the _turtlesim\_node_, which creates the simulation window:
```bash
ros2 run turtlesim turtlesim_node
```
This command starts a blue simulation window with a turtle in the center. 

#columns(2)[
This node:
- creates the simulation environment
- manages turtle state (position, orientation, pen status)
- processes incoming commands
- publishes turtle pose information
#colbreak()
#image("../../imgs/turtlesim/turtlesim_window.png", width: 100%)
]

---

#columns(2)[
- The turtle's position and orientation are represented in a 2D coordinate system. 
- The turtle can move forward, backward, and rotate, and its pen can be raised or lowered to draw on the canvas.
#colbreak()
#image("../../imgs/turtlesim/turtlesim_terminal.png", width: 100%)
]

==== List Active Components

The turtle's state is managed by the _TurtleSim_ node, which processes incoming commands and publishes the turtle's pose information. This allows us to control the turtle's movement and visualize its position in real time.
#columns(2)[
```bash
# running nodes
ros2 node list
# active topics              
ros2 topic list 
# available services            
ros2 service list 
# display actions          
ros2 action list    
```
#colbreak()
#image("../../imgs/turtlesim/turtlesim_node_list.png", width: 100%)
]

==== Moving the Turtle

```bash
ros2 run turtlesim turtle_teleop_key
```
#info[
Keep the teleop terminal window focused while sending keyboard commands.
]

#memo[
/ Arrow keys: Move the turtle forward/backward and rotate left/right
/ Space: Stop the turtle
// G: Quit
]

=== Programmatic Control

==== Using Topics

Topics are named communication channels for message passing.

#align(center)[#image("../../imgs/tikz/topic.svg", width: 50%)]

We can explore active topics via:
```bash
# List all active topics
ros2 topic list
```
Key topics include:
/ `/turtle1/cmd_vel`: For sending velocity commands
/ `/turtle1/color_sensor`: Color detected by the turtle
/ `/turtle1/pose`: Published turtle position and orientation

Examine topic types and message structures:
```bash
ros2 topic type /turtle1/cmd_vel # Result: geometry_msgs/msg/Twist
```
```bash
ros2 topic echo /turtle1/pose # Echo messages on a topic
```

1. *Forward*
```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2., y: 0., z: 0.}, angular: {x: 0., y: 0., z: 0.}}"
```

2. *Backward*
```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -2., y: 0., z: 0.}, angular: {x: 0., y: 0., z: 0.}}"
```

---

3. *Rotate Left*
```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0., y: 0., z: 0.}, angular: {x: 0., y: 0., z: 1.}}"
```

4. *Rotate right*
```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0., y: 0., z: 0.}, angular: {x: 0., y: 0., z: -1.}}"
```

---

==== Using Services

Services enable us to request specific actions or information from the turtle's services. 

#align(center)[#image("../../imgs/tikz/service.svg", width: 50%)]

---

To list all active services, we use:
```bash
ros2 service list -t
```

1. *Set Absolute Position*
```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 5., y: 5., theta: 0.}"
```

2. *Set Relative Position*
```bash
ros2 service call /turtle1/teleport_relative turtlesim/srv/TeleportRelative "{linear: 1., angular: 1.}"
```

---

3. *Changing the Turtle's Color* 
```bash
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 2, 'off': 0}"
```

4. *Clearing the Screen*
```bash
ros2 service call /clear std_srvs/srv/Empty "{}"
```

5. *Resetting the Turtle*
```bash
ros2 service call /reset std_srvs/srv/Empty "{}"
```

== Turtle Drawing Challenge

In this exercise, you will use `turtlesim` to create a square pattern using only *ROS2* command-line tools. The goal is to make the turtle draw a perfect square without any programming.

=== What to Observe

- How does the turtle's movement affect the drawing?
- What happens when you change the pen color mid-drawing?
- How precise can you make the square using only command-line tools?

=== Learning Objectives

- Understand *ROS2* topic publishing
- Learn about service calls
- Practice with `geometry_msgs/msg/Twist` messages
- Experience real-time robot control concepts

---

#exo("Pen Control", [
    Before starting the square, set the pen to:
    - Red color (R=255, G=0, B=0)
    - Width of 3 pixels
    - Pen down (off=0)
    ])

---

#solution[
```bash
# Set pen to red color and width 3
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 3, 'off': 0}"
```
]

---

#exo("Basic Movement", [
    Use the `ros2 topic pub` command to make the turtle:
    - Move forward 2 units
    - Rotate 90 degrees to the right
    - Move forward 2 units
    - Rotate 90 degrees to the right
    - Move forward 2 units
    - Rotate 90 degrees to the right
    - Move forward 2 units
    - Rotate 90 degrees to the right
    ])

#tip[Use `geometry_msgs/msg/Twist` messages with:
- `linear.x` for forward/backward movement
- `angular.z` for rotation (positive = left, negative = right)
]


#info[Don't forget to reset the turtle's position if needed:
```bash
ros2 service call /reset std_srvs/srv/Empty "{}"
```
]

---

#solution[
```bash
# Move forward 2 units
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}}"

# Rotate 90 degrees right
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{angular: {z: -1.57}}"
```
]

---

#solution[
We could also create a square using `/turtle1/teleport_absolute` service:
```bash
# Teleport to corners of square
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 7.5, y: 5.5}" && sleep 1

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 7.5, y: 3.5}" && sleep 1

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 5.5, y: 3.5}" && sleep 1

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 5.5, y: 5.5}"
```
]

---

#solution[
Using `/turtle1/teleport_relative` service is also possible:
```bash
ros2 service call /turtle1/teleport_relative turtlesim/srv/TeleportRelative "{linear: 2.}" && sleep 1

ros2 service call /turtle1/teleport_relative turtlesim/srv/TeleportRelative "{linear: -2., angular: 1.57}" && sleep 1

ros2 service call /turtle1/teleport_relative turtlesim/srv/TeleportRelative "{linear: 2., angular: 1.57}" && sleep 1

ros2 service call /turtle1/teleport_relative turtlesim/srv/TeleportRelative "{linear: 2., angular: -1.57}"
```
]

--- 

#exo("Precision Challenge", [
    Try to make the turtle return to its starting position after completing the square. Use the `teleport_absolute` service to verify the final position.
    ])

---

#solution[
To check if the turtle returned to its starting position:
```bash
# Check current position
ros2 topic echo /turtle1/pose

# If needed, teleport to exact starting position
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 5.5, y: 5.5, theta: 0.0}"
```
]

---

=== Summary and Key Takeaways

/ Understanding Twist Messages: 
- `linear.x`: Forward/backward velocity (positive = forward)
- `angular.z`: Rotational velocity (positive = counterclockwise, negative = clockwise)
- Values are in radians per second for rotation

/ Timing Considerations:
- The `sleep` commands are crucial for allowing the turtle to complete each movement
- Without proper timing, movements overlap and the pattern becomes distorted
- Real robots would use feedback control instead of fixed timing

/ Service vs Topic Usage:
- *Topics*: For continuous commands (velocity control)
- *Services*: For discrete actions (teleportation, pen control)

---

#warning[
- If the turtle doesn't move, check that the Turtlesim window is active
- If movements are too fast/slow, adjust the sleep duration
- If the square isn't closed, verify the rotation angles ($90 degree = pi/2 approx 1.57$ radians)
- Use `ros2 topic echo /turtle1/pose` to monitor the turtle's position in real-time
]

#idea[
- Try creating different shapes (triangle, pentagon, hexagon)
- Experiment with different pen colors and widths
- Create patterns by combining multiple shapes
- Use relative teleportation for more complex movements
]
