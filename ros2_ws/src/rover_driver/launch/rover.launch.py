"""Start the rover driver.

    ros2 launch rover_driver rover.launch.py
    ros2 launch rover_driver rover.launch.py rover_host:=192.168.1.50
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('rover_driver'), 'config', 'driver.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('rover_host', default_value='roverc.local',
                              description='Rover hostname or IP address'),
        Node(
            package='rover_driver',
            executable='rover_driver',
            name='rover_driver',
            output='screen',
            parameters=[config, {'rover_host': LaunchConfiguration('rover_host')}],
        ),
    ])
