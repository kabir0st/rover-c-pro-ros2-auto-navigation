"""Start dead-reckoning SLAM: odometry, map and the base_link -> imu_link transform.

    ros2 launch rover_slam slam.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('rover_slam'), 'config', 'slam.yaml')
    return LaunchDescription([
        Node(
            package='rover_slam',
            executable='slam_node',
            name='rover_slam',
            output='screen',
            parameters=[config],
        ),
        # The stick sits roughly at the rover's centre. Its axes don't matter for
        # heading (the filter finds "up" from gravity), so identity is fine here.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf',
            arguments=['--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        ),
    ])
