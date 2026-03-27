#!/usr/bin/env python3
"""Converts Twist on /cmd_vel to TwistStamped on /mavros/setpoint_velocity/cmd_vel."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelStamper(Node):
    def __init__(self):
        super().__init__('cmd_vel_stamper')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10
        )

    def cb(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist = msg
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
