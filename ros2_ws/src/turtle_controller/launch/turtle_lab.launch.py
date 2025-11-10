import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('turtle_controller'),
        'config',
        'params.yaml'
    )

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
            name='smart_controller',
            parameters=[config]
        )
    ])
