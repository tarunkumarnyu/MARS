#!/usr/bin/env python3
"""
Arms ArduPilot Rover via MAVROS, sets GUIDED mode,
then sends waypoints through Nav2's waypoint follower.

Edit the WAYPOINTS list below with your desired (x, y) positions.
Coordinates are in meters relative to where the rover powered on.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from mavros_msgs.srv import SetMode, CommandBool
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
import time
import math

# ===== EDIT YOUR WAYPOINTS HERE =====
# (x, y, yaw_degrees) relative to start position
WAYPOINTS = [
    (2.0, 0.0, 0),
    (2.0, 2.0, 90),
    (0.0, 2.0, 180),
    (0.0, 0.0, 270),
]


class ArmAndGo(Node):
    def __init__(self):
        super().__init__('arm_and_go')

        # MAVROS service clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Nav2 waypoint follower action client
        self.waypoint_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        self.get_logger().info('Waiting for MAVROS services...')
        self.set_mode_client.wait_for_service(timeout_sec=30.0)
        self.arming_client.wait_for_service(timeout_sec=30.0)
        self.get_logger().info('MAVROS services available.')

        self.run()

    def set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f'Mode set to {mode}')
            return True
        self.get_logger().error(f'Failed to set mode {mode}')
        return False

    def arm(self) -> bool:
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None and future.result().success:
            self.get_logger().info('Vehicle ARMED')
            return True
        self.get_logger().error('Failed to arm')
        return False

    def send_waypoints(self):
        self.get_logger().info('Waiting for Nav2 waypoint follower...')
        if not self.waypoint_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Nav2 waypoint follower not available!')
            return

        poses = []
        for x, y, yaw_deg in WAYPOINTS:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            # Convert yaw to quaternion (rotation about Z)
            yaw = math.radians(yaw_deg)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(pose)

        goal = FollowWaypoints.Goal()
        goal.poses = poses

        self.get_logger().info(f'Sending {len(poses)} waypoints...')
        future = self.waypoint_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Waypoints rejected by Nav2!')
            return

        self.get_logger().info('Waypoints accepted — rover is moving!')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Waypoint mission complete!')

    def run(self):
        # Step 1: Set GUIDED mode
        if not self.set_mode('GUIDED'):
            return

        time.sleep(1.0)

        # Step 2: Arm
        if not self.arm():
            return

        time.sleep(2.0)

        # Step 3: Send waypoints via Nav2
        self.send_waypoints()


def main(args=None):
    rclpy.init(args=args)
    node = ArmAndGo()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
