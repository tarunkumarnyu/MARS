#!/usr/bin/env python3
"""
Sets all required ArduPilot parameters for MARS indoor navigation.

Run this while rover_bringup.launch.py is active and MAVROS is connected.
After running, power-cycle the Pixracer (unplug USB, wait 3s, replug)
for GPS1_TYPE change to take effect.

Usage:
    source ~/rover_ws/install/setup.bash
    python3 scripts/set_ardupilot_params.py
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import ParamSetV2
from rcl_interfaces.msg import ParameterValue, ParameterType

PARAMS = {
    # EKF3 sources — all from ExternalNav (ZED VIO via MAVROS vision_pose)
    'EK3_SRC1_POSXY': 6,       # ExternalNav
    'EK3_SRC1_VELXY': 6,       # ExternalNav
    'EK3_SRC1_YAW': 6,         # ExternalNav
    'EK3_SRC1_POSZ': 1,        # Barometer (stable indoors)
    'EK3_SRC1_VELZ': 6,        # ExternalNav

    # GPS disabled (indoor, no signal)
    'GPS1_TYPE': 0,             # REQUIRES POWER CYCLE after setting

    # Visual odometry enabled
    'VISO_TYPE': 1,

    # Compass disabled (unreliable indoors due to magnetic interference)
    'COMPASS_USE': 0,
    'COMPASS_USE2': 0,
    'COMPASS_USE3': 0,

    # Failsafes disabled for indoor use without RC
    'FS_THR_ENABLE': 0,         # No RC transmitter
    'FS_EKF_ACTION': 0,         # Don't switch to HOLD on EKF issues

    # Mode control — disable RC mode channel so MAVROS can set GUIDED
    'MODE_CH': 0,

    # Arming checks disabled for testing (re-enable for production: set to 1)
    'ARMING_CHECK': 0,

    # Safety switch defaults to off
    'BRD_SAFETY_DEFLT': 0,
}


def main():
    rclpy.init()
    node = Node('param_setter')
    cli = node.create_client(ParamSetV2, '/mavros/param/set')

    print('Waiting for MAVROS param service...')
    if not cli.wait_for_service(timeout_sec=15.0):
        print('ERROR: MAVROS param service not available. Is the bringup running?')
        return

    print(f'Setting {len(PARAMS)} parameters...\n')
    failed = []
    for name, val in PARAMS.items():
        req = ParamSetV2.Request()
        req.param_id = name
        req.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=val)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        if fut.done() and fut.result() and fut.result().success:
            print(f'  {name} = {val}  [OK]')
        else:
            print(f'  {name} = {val}  [FAILED]')
            failed.append(name)

    print()
    if failed:
        print(f'WARNING: {len(failed)} params failed: {failed}')
    else:
        print('All parameters set successfully!')

    print('\n*** IMPORTANT: Power-cycle the Pixracer now ***')
    print('    (unplug USB, wait 3 seconds, replug)')
    print('    GPS1_TYPE change requires a reboot to take effect.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
