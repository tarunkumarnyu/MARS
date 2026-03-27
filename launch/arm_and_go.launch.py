"""
Utility launch that arms the rover and sets GUIDED mode.

Run AFTER rover_bringup.launch.py is up and MAVROS shows "Connected".
Sends waypoints defined in the 'waypoints' parameter.

Usage:
  ros2 launch rover_nav arm_and_go.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    arm_guided_node = Node(
        package='rover_nav',
        executable='arm_and_go',
        name='arm_and_go',
        output='screen',
    )

    return LaunchDescription([arm_guided_node])
