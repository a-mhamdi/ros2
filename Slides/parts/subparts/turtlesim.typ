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
    #image("../../imgs/turtlesim/turtlesim_window.png", width: 100%) // caption: [TurtleSim window after launching the simulator node],
  ]
  ---
  #columns(2)[
  - The turtle's position and orientation are represented in a 2D coordinate system. // with the origin (0,0) at the center of the window.
  - The turtle can move forward, backward, and rotate, and its pen can be raised or lowered to draw on the canvas.
  #colbreak()
  #image("../../imgs/turtlesim/turtlesim_terminal.png", width: 100%) // caption: [TurtleSim node in the ROS2 ecosystem],
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
      #image("../../imgs/turtlesim/turtlesim_node_list.png", width: 100%) // caption: [List of active nodes, topics, services, and actions],
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
    / `/turtle1/pose`: Published turtle position and orientation
    / `/turtle1/color_sensor`: Color detected by the turtle

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
