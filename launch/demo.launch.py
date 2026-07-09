"""
ROS2 launch file for the submarine driver framework demo.

Launches:
  1. Trajectory simulation node (generates ground-truth motion)
  2. State estimator node (fuses sensor data into 6-DOF pose)
  3. Robot state publisher (publishes URDF for RViz visualization)
  4. RViz2 (visualization)

Usage:
    # Full demo with visualization
    ros2 launch submarine_drivers demo.launch.py

    # Change trajectory pattern
    ros2 launch submarine_drivers demo.launch.py pattern:=figure8

    # Without RViz (headless)
    ros2 launch submarine_drivers demo.launch.py launch_rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('submarine_drivers')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'submarine.urdf')
    rviz_config = os.path.join(pkg_dir, 'config', 'submarine_rviz.rviz')
    config_file = os.path.join(pkg_dir, 'config', 'vehicle_params.yaml')

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('pattern', default_value='circle',
            description='Trajectory pattern: circle, figure8, straight'),
        DeclareLaunchArgument('speed', default_value='2.0',
            description='Vehicle speed in m/s'),
        DeclareLaunchArgument('launch_rviz', default_value='true',
            description='Launch RViz2 visualization'),

        # Trajectory simulation (generates sensor data)
        Node(
            package='submarine_drivers',
            executable='trajectory_sim_node',
            name='trajectory_sim',
            output='screen',
            parameters=[{
                'pattern': LaunchConfiguration('pattern'),
                'speed': LaunchConfiguration('speed'),
                'radius': 10.0,
                'depth': 20.0,
                'dive_amplitude': 5.0,
                'rate_hz': 50.0,
            }],
        ),

        # State estimator (fuses sensor data)
        Node(
            package='submarine_drivers',
            executable='state_estimator_node',
            name='state_estimator',
            output='screen',
            parameters=[config_file],
        ),

        # Robot state publisher (URDF for RViz)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # RViz2 (visualization)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
        ),
    ])
