#!/usr/bin/env python3
"""
Bridge ZED 2i visual-inertial odometry to MAVROS vision_pose
so ArduPilot's EKF3 can use it as an external navigation source.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped


class ZedMavrosBridge(Node):
    def __init__(self):
        super().__init__('zed_mavros_bridge')

        # Parameters
        self.declare_parameter('zed_odom_topic', '/zed/zed_node/odom')
        self.declare_parameter('vision_pose_topic', '/mavros/vision_pose/pose')
        self.declare_parameter('vision_speed_topic', '/mavros/vision_speed/speed_twist')

        zed_odom_topic = self.get_parameter('zed_odom_topic').value
        vision_pose_topic = self.get_parameter('vision_pose_topic').value
        vision_speed_topic = self.get_parameter('vision_speed_topic').value

        # QoS for ZED topics (best effort to match ZED publisher)
        zed_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, zed_odom_topic, self.odom_callback, zed_qos
        )

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, vision_pose_topic, 10)
        self.speed_pub = self.create_publisher(TwistStamped, vision_speed_topic, 10)

        self.get_logger().info(
            f'Bridge started: {zed_odom_topic} -> {vision_pose_topic}'
        )

    def odom_callback(self, msg: Odometry):
        # Publish pose to MAVROS vision_pose
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'map'
        pose.pose = msg.pose.pose
        self.pose_pub.publish(pose)

        # Publish velocity to MAVROS vision_speed
        twist = TwistStamped()
        twist.header.stamp = msg.header.stamp
        twist.header.frame_id = msg.child_frame_id
        twist.twist = msg.twist.twist
        self.speed_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ZedMavrosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
