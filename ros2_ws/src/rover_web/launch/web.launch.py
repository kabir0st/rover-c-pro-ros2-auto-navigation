"""Start the rover driver, dead-reckoning SLAM and the browser control page.

    ros2 launch rover_web web.launch.py
    ros2 launch rover_web web.launch.py rover_host:=192.168.1.50 port:=8080
    ros2 launch rover_web web.launch.py driver:=false slam:=false   # already running
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    driver_launch = os.path.join(
        get_package_share_directory('rover_driver'), 'launch', 'rover.launch.py')
    slam_launch = os.path.join(
        get_package_share_directory('rover_slam'), 'launch', 'slam.launch.py')
    config = os.path.join(get_package_share_directory('rover_web'), 'config', 'web.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('rover_host', default_value='roverc.local',
                              description='Rover hostname or IP address'),
        DeclareLaunchArgument('port', default_value='8080',
                              description='Port for the control page'),
        DeclareLaunchArgument('driver', default_value='true',
                              description='Also start rover_driver'),
        DeclareLaunchArgument('slam', default_value='true',
                              description='Also start rover_slam'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch),
            launch_arguments={'rover_host': LaunchConfiguration('rover_host')}.items(),
            condition=IfCondition(LaunchConfiguration('driver')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            condition=IfCondition(LaunchConfiguration('slam')),
        ),
        Node(
            package='rover_web',
            executable='rover_web',
            name='rover_web',
            output='screen',
            parameters=[config,
                        {'port': ParameterValue(LaunchConfiguration('port'), value_type=int)}],
        ),
    ])
