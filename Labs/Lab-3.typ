#import "Class.typ": *


#show: ieee.with(
  title: [#text(smallcaps("Lab #3: Gazebo and RViz2"))],

  abstract: [
    This lab demonstrates how to create a basic *ROS 2* package that integrates *Gazebo* simulation, *RViz* visualization, and launch files in *ROS 2* Humble. For that we need the following prerequisites:
    - *ROS 2* Humble installed
    - *Gazebo Garden* (or *Gazebo Classic*)
    - *RViz2*
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
    (
      name: "Student 3",
      department: [Dept. of EE],
      organization: [ISET Bizerte --- Tunisia],
      profile: "abc",
    )
  */

  )
  // index-terms: (""),
  // bibliography-file: "Biblio.bib",
)



== Create a *ROS 2* Package

First, create a new *ROS 2* workspace and package:

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_python basic_gazebo_demo --dependencies rclpy geometry_msgs sensor_msgs nav_msgs tf2_ros gazebo_ros_pkgs

cd basic_gazebo_demo
```

== Package Structure

Your package should have the following structure:

```
basic_gazebo_demo/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── basic_gazebo_demo
├── basic_gazebo_demo/
│   └── __init__.py
├── launch/
│   └── gazebo_rviz.launch.py
├── worlds/
│   └── simple_world.world
├── models/
│   └── simple_robot/
│       ├── model.sdf
│       └── model.config
└── rviz/
    └── robot_config.rviz
```

== Package Configuration

Update `package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>basic_gazebo_demo</name>
  <version>0.0.0</version>
  <description>Basic Gazebo, RViz, and Launch File Demo</description>
  <maintainer email="student@example.com">Student</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>gazebo_ros_pkgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

== Gazebo World File

Create `worlds/simple_world.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="simple_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <light name="point_light" type="point">
      <pose>0 0 2 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>1 1 1 1</specular>
      <attenuation>
        <range>10</range>
        <constant>0.2</constant>
        <linear>0.1</linear>
        <quadratic>0.01</quadratic>
      </attenuation>
    </light>
  </world>
</sdf>
```

== Simple Robot Model

Create `models/simple_robot/model.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="simple_robot">
    <pose>0 0 0.1 0 0 0</pose>

    <link name="base_link">
      <visual>
        <geometry>
          <box>
            <size>0.3 0.3 0.1</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
        </material>
      </visual>

      <collision>
        <geometry>
          <box>
            <size>0.3 0.3 0.1</size>
          </box>
        </geometry>
      </collision>

      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <iyy>0.1</iyy>
          <izz>0.1</izz>
        </inertia>
      </inertial>
    </link>

    <joint name="left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>

    <link name="left_wheel">
      <visual>
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
      </collision>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
    </link>

    <joint name="right_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>right_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>

    <link name="right_wheel">
      <visual>
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
      </collision>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
    </link>

    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/robot</namespace>
      </ros>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <wheel_acceleration>1.0</wheel_acceleration>
      <wheel_torque>20</wheel_torque>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </model>
</sdf>
```

Create `models/simple_robot/model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>simple_robot</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>A simple differential drive robot</description>
</model>
```

== Launch File

Create `launch/gazebo_rviz.launch.py`:

```py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare(package='basic_gazebo_demo').find('basic_gazebo_demo')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_share, 'worlds', 'simple_world.world'),
        description='Full path to world file to load'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_share, 'rviz', 'robot_config.rviz'),
        description='Full path to RVIZ config file to load'
    )

    # Gazebo launch
    gazebo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Spawn robot in Gazebo
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'simple_robot', '-file',
                  os.path.join(pkg_share, 'models', 'simple_robot', 'model.sdf')],
        output='screen'
    )

    # RViz launch
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add the commands to the launch description
    ld.add_action(gazebo_launch_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(rviz_cmd)

    return ld
```

== Build and Run

Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select basic_gazebo_demo
source install/setup.bash
```

Run the complete example:

```bash
# Terminal 1: Launch everything
ros2 launch basic_gazebo_demo gazebo_rviz.launch.py

# Terminal 2: Control the robot
ros2 topic pub /robot/cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.5}"

# Terminal 3: View topics
ros2 topic list
ros2 topic echo /robot/odom
```
