#!/usr/bin/env python3
"""
Sets EKF origin, waits for convergence, arms, and sets GUIDED mode.

Run this after rover_bringup.launch.py is running and MAVROS shows connected.
This is the correct startup sequence to avoid EKF issues.

Usage:
    source ~/rover_ws/install/setup.bash
    python3 scripts/set_origin_and_arm.py
"""

import rclpy
import time
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import StatusText, State
from geographic_msgs.msg import GeoPointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def main():
    rclpy.init()
    node = Node('startup')

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        depth=10,
    )

    fcu_msgs = []

    def status_cb(msg):
        fcu_msgs.append(msg.text)
        print(f'  FCU: {msg.text}')

    node.create_subscription(StatusText, '/mavros/statustext/recv', status_cb, qos)

    # Step 1: Set EKF origin
    print('[1/4] Setting EKF origin...')
    origin_pub = node.create_publisher(
        GeoPointStamped, '/mavros/global_position/set_gp_origin', 10
    )
    time.sleep(1.0)

    msg = GeoPointStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'map'
    msg.position.latitude = 40.6943
    msg.position.longitude = -73.9867
    msg.position.altitude = 0.0

    for i in range(10):
        origin_pub.publish(msg)
        time.sleep(0.3)
        rclpy.spin_once(node, timeout_sec=0.1)

    # Step 2: Wait for EKF convergence
    print('[2/4] Waiting 15 seconds for EKF to converge...')
    for i in range(75):
        rclpy.spin_once(node, timeout_sec=0.2)

    # Step 3: Arm
    print('[3/4] Arming...')
    arm_cli = node.create_client(CommandBool, '/mavros/cmd/arming')
    arm_cli.wait_for_service(timeout_sec=10.0)

    req = CommandBool.Request()
    req.value = True
    fut = arm_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)

    if fut.result() and fut.result().success:
        print('  Armed successfully')
    else:
        print('  ERROR: Arming failed. Check FCU messages above.')
        node.destroy_node()
        rclpy.shutdown()
        return

    time.sleep(2)
    for i in range(10):
        rclpy.spin_once(node, timeout_sec=0.2)

    # Step 4: Set GUIDED
    print('[4/4] Setting GUIDED mode...')
    mode_cli = node.create_client(SetMode, '/mavros/set_mode')
    mode_cli.wait_for_service(timeout_sec=5.0)

    req = SetMode.Request()
    req.custom_mode = 'GUIDED'
    fut = mode_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)

    time.sleep(1)
    state_val = [None]

    def state_cb(m):
        state_val[0] = m

    node.create_subscription(State, '/mavros/state', state_cb, 10)
    rclpy.spin_once(node, timeout_sec=2.0)

    if state_val[0]:
        print(f'  Mode: {state_val[0].mode}  Armed: {state_val[0].armed}')
        if state_val[0].mode == 'GUIDED' and state_val[0].armed:
            print('\nREADY! Send Nav2 goals now.')
        else:
            print('\nWARNING: Not in GUIDED+armed state. Check troubleshooting in README.')
    else:
        print('  Could not read state')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
