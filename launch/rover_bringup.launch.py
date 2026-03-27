"""
Full bringup launch for indoor waypoint navigation.

Launches:
  1. MAVROS (ArduPilot <-> ROS2)
  2. ZED 2i camera (depth + VIO)
  3. Static TF: base_link -> zed2i_base_link
  4. Static TF: map -> odom (identity, no separate mapping)
  5. ZED-to-MAVROS vision pose bridge
  6. Nav2 (waypoint following + obstacle avoidance)
  7. cmd_vel relay to MAVROS
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # -- Args --
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='serial:///dev/ttyACM0:921600',
        description='MAVLink connection string to Pixracer Pro'
    )

    cam_x = DeclareLaunchArgument('cam_x', default_value='0.10',
                                   description='Camera X offset from base_link (meters)')
    cam_y = DeclareLaunchArgument('cam_y', default_value='0.0',
                                   description='Camera Y offset from base_link')
    cam_z = DeclareLaunchArgument('cam_z', default_value='0.15',
                                   description='Camera Z offset from base_link')

    rover_nav_dir = get_package_share_directory('rover_nav')
    nav2_params_file = os.path.join(rover_nav_dir, 'config', 'nav2_params.yaml')

    # ---- 1. MAVROS (wrapped in GroupAction to contain namespace) ----
    mavros_dir = get_package_share_directory('mavros')
    mavros_launch = GroupAction([
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(mavros_dir, 'launch', 'apm.launch')
            ),
            launch_arguments={
                'fcu_url': LaunchConfiguration('fcu_url'),
            }.items(),
        ),
    ])

    # ---- 2. ZED 2i ----
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2i',
        }.items(),
    )

    # ---- 3. Static TF: zed_camera_link -> base_link ----
    # ZED publishes odom -> zed_camera_link, so base_link must be a child
    # Offsets are inverted (camera is +x,+z from base, so base is -x,-z from camera)
    zed_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_to_base_tf',
        arguments=[
            '--x', '-0.10',
            '--y', '0.0',
            '--z', '-0.15',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'zed_camera_link',
            '--child-frame-id', 'base_link',
        ],
    )

    # ---- 4. Static TF: map -> odom (identity) ----
    # Without a mapping system, map and odom are the same frame.
    # Waypoints are relative to where the rover starts.
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
    )

    # ---- 5. ZED odom -> MAVROS vision pose bridge ----
    zed_mavros_bridge = Node(
        package='rover_nav',
        executable='zed_mavros_bridge',
        name='zed_mavros_bridge',
        output='screen',
    )

    # ---- 6. Nav2 nodes (launched directly, no collision_monitor/docking) ----
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': False}],
        remappings=remappings,
    )

    nav2_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': False}],
        remappings=remappings,
    )

    nav2_behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': False}],
        remappings=remappings,
    )

    nav2_bt = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': False}],
        remappings=remappings,
    )

    nav2_waypoint = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': False}],
        remappings=remappings,
    )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ],
        }],
    )

    # ---- 7. cmd_vel -> MAVROS (Twist to TwistStamped) ----
    # Nav2 publishes Twist on /cmd_vel, MAVROS needs TwistStamped on /mavros/setpoint_velocity/cmd_vel
    cmd_vel_relay = Node(
        package='rover_nav',
        executable='cmd_vel_stamper',
        name='cmd_vel_stamper',
        output='screen',
    )

    # Delay Nav2 a few seconds so ZED + MAVROS TF are ready
    delayed_nav2 = TimerAction(period=5.0, actions=[
        nav2_controller,
        nav2_planner,
        nav2_behaviors,
        nav2_bt,
        nav2_waypoint,
        nav2_lifecycle_manager,
    ])

    return LaunchDescription([
        fcu_url_arg,
        cam_x, cam_y, cam_z,
        mavros_launch,
        zed_launch,
        zed_to_base_tf,
        map_to_odom_tf,
        zed_mavros_bridge,
        cmd_vel_relay,
        delayed_nav2,
    ])
