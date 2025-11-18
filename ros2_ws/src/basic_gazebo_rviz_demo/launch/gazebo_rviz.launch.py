import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare(package="basic_gazebo_rviz_demo").find(
        "basic_gazebo_rviz_demo"
    )

    urdf_file = os.path.join(pkg_share, "models", "model.urdf")

    rviz_config_file = os.path.join(pkg_share, "config", "sim.rviz")

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_file = LaunchConfiguration("world_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_world_file_cmd = DeclareLaunchArgument(
        "world_file",
        default_value=os.path.join(pkg_share, "worlds", "simple_world.world"),
        description="Full path to world file to load",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(pkg_share, "rviz", "robot_config.rviz"),
        description="Full path to RVIZ config file to load",
    )

    # Gazebo launch
    gazebo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={"world": world_file}.items(),
    )

    # Spawn robot in Gazebo
    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "simple_robot",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
        output="screen",
    )

    # Robot state publisher
    robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[urdf_file],
    )

    # Joint State Publisher (for non-simulated joints)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # RViz launch
    rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
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
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz_cmd)

    return ld
