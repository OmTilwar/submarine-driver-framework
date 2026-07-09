"""
ROS2 launch file for the submarine driver framework.

Launches the sensor driver node and state estimator node with vehicle
configuration loaded from YAML.

Usage:
    ros2 launch submarine_drivers drivers.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('submarine_drivers')
    config_file = os.path.join(pkg_dir, 'config', 'vehicle_params.yaml')

    return LaunchDescription([
        Node(
            package='submarine_drivers',
            executable='sensor_driver_node',
            name='sensor_driver_node',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='submarine_drivers',
            executable='state_estimator_node',
            name='state_estimator_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
