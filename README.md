# MARS - Mobile Autonomous Rover System

Indoor GPS-denied waypoint navigation for a Traxxas Ackermann-steering rover using ArduPilot, ZED 2i visual-inertial odometry, and ROS2 Nav2.

## Hardware

| Component | Model | Purpose |
|-----------|-------|---------|
| Chassis | Traxxas RC car (Ackermann steering) | Base platform |
| Flight Controller | MRO Pixracer Pro | ArduPilot Rover firmware (v4.6.3) |
| Camera | ZED 2i (Stereolabs) | Stereo depth + Visual-Inertial Odometry (VIO) |
| Companion Computer | Laptop (Ubuntu 24.04) | Runs ROS2, Nav2, MAVROS |
| Connection | USB (Pixracer) + USB 3.0 (ZED) | Both connected to laptop |

### Wiring

- **Pixracer Output 1** -> Traxxas steering servo (SERVO1_FUNCTION=26, GroundSteering)
- **Pixracer Output 3** -> Traxxas ESC (SERVO3_FUNCTION=70, Throttle)
- **Pixracer USB** -> Laptop (`/dev/ttyACM0` at 921600 baud)
- **ZED 2i USB 3.0** -> Laptop (must be USB 3.0 blue port)

## Software Stack

| Component | Version |
|-----------|---------|
| Ubuntu | 24.04 LTS |
| ROS2 | Jazzy Jalisco |
| ArduPilot | Rover v4.6.3 |
| MAVROS | ros-jazzy-mavros (apt) |
| Nav2 | ros-jazzy-nav2-bringup (apt) |
| ZED SDK + ROS2 Wrapper | v5.2.x (built from source in `~/zed_ws`) |

## Architecture

```
                    ZED 2i Camera
                         |
                    ZED VIO (60Hz)
                    /zed/zed_node/odom
                         |
              +----------+----------+
              |                     |
     zed_mavros_bridge         Nav2 Costmap
     (pose + velocity)     /zed/zed_node/point_cloud
              |                     |
    /mavros/vision_pose        Nav2 Stack
    /mavros/vision_speed    (planner + controller)
              |                     |
         ArduPilot EKF3        /cmd_vel (Twist)
              |                     |
         Pixracer Pro       cmd_vel_stamper
              |             (Twist -> TwistStamped)
         Servo Outputs              |
          |        |     /mavros/setpoint_velocity/cmd_vel
       Steering  Throttle           |
                              ArduPilot GUIDED
                                    |
                              Servo Outputs
```

### TF Tree

```
map -> odom -> zed_camera_link -> base_link
 (static)  (ZED VIO publishes)   (static, inverted camera mount offset)
```

**Important:** The ZED publishes `odom -> zed_camera_link`. Since TF is a tree (one parent per frame), `base_link` must be a **child** of `zed_camera_link`, not the other way around. The static TF uses inverted offsets (if camera is at x=+0.10, z=+0.15 from base, then base is at x=-0.10, z=-0.15 from camera).

## Installation

### Prerequisites

```bash
# ROS2 Jazzy
sudo apt install ros-jazzy-desktop

# MAVROS
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras
# Install geographic datasets (required by MAVROS)
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# Nav2
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-nav2-regulated-pure-pursuit-controller ros-jazzy-nav2-rviz-plugins

# Topic tools
sudo apt install ros-jazzy-topic-tools
```

### ZED ROS2 Wrapper (build from source)

```bash
mkdir -p ~/zed_ws/src && cd ~/zed_ws/src
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
git clone https://github.com/stereolabs/zed-ros2-interfaces.git
cd ~/zed_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

### MARS Package

```bash
mkdir -p ~/rover_ws/src && cd ~/rover_ws/src
git clone https://github.com/tarunkumarnyu/MARS.git rover_nav
cd ~/rover_ws
source /opt/ros/jazzy/setup.bash
source ~/zed_ws/install/setup.bash
colcon build --packages-select rover_nav
```

## ArduPilot Parameters

These parameters MUST be set on the Pixracer. They persist across reboots once set.

### One-time Setup

Run the provided script while the bringup is active:

```bash
source ~/rover_ws/install/setup.bash
python3 ~/rover_ws/src/rover_nav/scripts/set_ardupilot_params.py
```

Then **power-cycle the Pixracer** (unplug USB, wait 3s, replug) for `GPS1_TYPE` to take effect.

### Parameter Reference

#### EKF3 Sources

| Parameter | Value | Description |
|-----------|-------|-------------|
| `GPS1_TYPE` | `0` | Disable GPS (indoor, no GPS signal). **Requires reboot.** |
| `VISO_TYPE` | `1` | Enable visual odometry from MAVLink |
| `EK3_SRC1_POSXY` | `6` | Position XY from ExternalNav (vision) |
| `EK3_SRC1_VELXY` | `6` | Velocity XY from ExternalNav (vision) |
| `EK3_SRC1_POSZ` | `1` | Position Z from Barometer |
| `EK3_SRC1_VELZ` | `6` | Velocity Z from ExternalNav (vision) |
| `EK3_SRC1_YAW` | `6` | Yaw from ExternalNav (vision) |
| `COMPASS_USE` | `0` | Disable compass (unreliable indoors) |
| `COMPASS_USE2` | `0` | Disable compass 2 |
| `COMPASS_USE3` | `0` | Disable compass 3 |

#### Failsafe / Mode

| Parameter | Value | Description |
|-----------|-------|-------------|
| `FS_THR_ENABLE` | `0` | Disable throttle failsafe (no RC transmitter) |
| `FS_EKF_ACTION` | `0` | Disable EKF failsafe action (prevents auto-HOLD) |
| `MODE_CH` | `0` | Disable RC mode switching (allows MAVROS to control mode) |
| `ARMING_CHECK` | `0` | Disable arming checks (for testing; re-enable = `1`) |
| `BRD_SAFETY_DEFLT` | `0` | Safety switch defaults to off |

#### Servo

| Parameter | Value | Description |
|-----------|-------|-------------|
| `SERVO1_FUNCTION` | `26` | GroundSteering on output 1 |
| `SERVO3_FUNCTION` | `70` | Throttle on output 3 |
| `SERVO1_MIN/MAX/TRIM` | `1100/1900/1500` | Steering servo PWM range |
| `SERVO3_MIN/MAX/TRIM` | `1100/1900/1500` | ESC PWM range |
| `TURN_RADIUS` | `0.9` | Minimum turn radius in meters |

## Usage

### Step 1: Launch the Full Stack

```bash
source ~/zed_ws/install/setup.bash
source ~/rover_ws/install/setup.bash
ros2 launch rover_nav rover_bringup.launch.py
```

Wait until you see:
- `FCU: ArduRover V4.6.3` (MAVROS connected)
- ZED camera output (depth + odom publishing)

### Step 2: Set Origin, Arm, and Set GUIDED

In a new terminal, run the startup script:

```bash
source ~/rover_ws/install/setup.bash
python3 ~/rover_ws/src/rover_nav/scripts/set_origin_and_arm.py
```

This does everything in the correct order:
1. Sets EKF origin (required after every Pixracer boot)
2. Waits 15 seconds for EKF convergence
3. Arms the rover
4. Sets GUIDED mode

You should see `READY! Send Nav2 goals now.`

### Step 3: Send Navigation Goals

**Option A: Command line**

```bash
# 2 meters forward
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}"

# 2 meters left
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 2.0}, orientation: {z: 0.707, w: 0.707}}}"
```

**Option B: RViz2**

```bash
rviz2 -d ~/rover_ws/install/rover_nav/share/rover_nav/config/rover_nav.rviz
```

Use the Nav2 Goal button in the toolbar to click and drag goals on the map.

**Option C: Automated waypoint mission**

```bash
ros2 launch rover_nav arm_and_go.launch.py
```

This arms, sets GUIDED, and sends a predefined square waypoint path.

## Verification Checklist

```bash
# 1. MAVROS connected?
ros2 topic echo /mavros/state --once
# Expect: connected: true

# 2. ZED odom flowing?
ros2 topic hz /zed/zed_node/odom
# Expect: ~60 Hz

# 3. Vision pose reaching ArduPilot?
ros2 topic hz /mavros/vision_pose/pose
# Expect: ~60 Hz

# 4. TF tree connected?
ros2 run tf2_ros tf2_echo odom base_link
# Expect: valid transform, no errors

# 5. Nav2 active?
for n in controller_server planner_server behavior_server bt_navigator waypoint_follower; do
  echo -n "$n: "; ros2 lifecycle get /$n
done
# Expect: all "active [3]"

# 6. cmd_vel flowing during navigation?
ros2 topic hz /cmd_vel
# Expect: ~10 Hz when a goal is active
```

## Troubleshooting

### EKF variance / EKF failsafe

The EKF is not converging on the vision data. This is the most common issue.

**Fix:**
1. Power-cycle the Pixracer (unplug USB, wait 3s, replug)
2. Restart the launch
3. Set the EKF origin using the startup script
4. **Wait 15 seconds** before arming
5. Verify `COMPASS_USE=0` (compass interferes indoors)
6. Verify `FS_EKF_ACTION=0`

### AHRS: waiting for home

ArduPilot doesn't have a home position.

**Fix:** Set the EKF origin (Step 2 above). You must do this after every Pixracer power cycle.

### Flight mode change failed

ArduPilot won't switch to GUIDED.

**Fix:**
- Verify `MODE_CH=0` (RC mode switch overrides MAVROS if non-zero)
- Check EKF is healthy (no "EKF variance" messages)
- Power-cycle and re-initialize if needed

### Rover not moving / servos stuck at 1500

| Cause | Fix |
|-------|-----|
| EKF unhealthy | Power-cycle Pixracer, wait for convergence |
| Not in GUIDED mode | `ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"` |
| Not armed | `ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"` |
| cmd_vel not flowing | Check `ros2 topic hz /cmd_vel` — goal may have completed or failed |

### Steering doesn't work but throttle does

| Cause | Fix |
|-------|-----|
| Steering servo on wrong Pixracer output | Must be on Output 1 |
| Servo not powered | Pixracer servo rail needs power from ESC BEC |
| `use_rotate_to_heading: true` | Ackermann can't rotate in place. Must be `false` in nav2_params.yaml |

### Nav2 nodes stuck in "inactive"

The lifecycle manager couldn't activate them because `odom -> base_link` TF wasn't available when Nav2 started.

**Fix:** Restart the launch. The 5-second delay before Nav2 should handle this.

### MAVROS: `/dev/ttyUSB0: No such file or directory`

The Pixracer is on a different serial port.

```bash
ls /dev/ttyACM* /dev/ttyUSB*
ros2 launch rover_nav rover_bringup.launch.py fcu_url:=serial:///dev/ttyACM0:921600
```

### ZED topics at wrong namespace (`/mavros/zed/` instead of `/zed/zed_node/`)

The MAVROS XML launch leaked its namespace to the ZED launch.

**Fix:** MAVROS launch must be wrapped in a `GroupAction` in the launch file (already done).

### Nav2 loads wrong controller (DWBLocalPlanner instead of RegulatedPurePursuit)

Nav2's `navigation_launch.py` in composition mode doesn't load custom params correctly.

**Fix:** Launch Nav2 nodes directly instead of including `navigation_launch.py` (already done).

### Nav2 plugin class not found

Nav2 Jazzy uses `::` separator for plugin class names, not `/`.

| Wrong | Correct |
|-------|---------|
| `nav2_behaviors/Spin` | `nav2_behaviors::Spin` |
| `nav2_navfn_planner/NavfnPlanner` | `nav2_navfn_planner::NavfnPlanner` |
| `nav2_bt_navigator/NavigateToPoseNavigator` | `nav2_bt_navigator::NavigateToPoseNavigator` |
| `nav2_costmap_2d::ObstacleCostMapLayer` | `nav2_costmap_2d::ObstacleLayer` |
| `nav2_costmap_2d::InflationCostMapLayer` | `nav2_costmap_2d::InflationLayer` |

### cmd_vel published but servos don't respond

ArduPilot Rover requires `TwistStamped` (with `frame_id`), not `Twist`.

**Fix:** Use `cmd_vel_stamper` node (included) instead of `topic_tools relay`. It converts `Twist` on `/cmd_vel` to `TwistStamped` on `/mavros/setpoint_velocity/cmd_vel`.

## Complete Startup Sequence

```bash
# Terminal 1: Launch everything
source ~/zed_ws/install/setup.bash && source ~/rover_ws/install/setup.bash
ros2 launch rover_nav rover_bringup.launch.py

# Terminal 2: After "FCU: ArduRover" appears, run startup script
source ~/rover_ws/install/setup.bash
python3 ~/rover_ws/src/rover_nav/scripts/set_origin_and_arm.py

# Terminal 3: Send goals (or use RViz)
source ~/rover_ws/install/setup.bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}"

# Terminal 4 (optional): RViz visualization
source ~/zed_ws/install/setup.bash && source ~/rover_ws/install/setup.bash
rviz2 -d ~/rover_ws/install/rover_nav/share/rover_nav/config/rover_nav.rviz
```

## Package Structure

```
MARS/
├── config/
│   ├── nav2_params.yaml          # Nav2 config (RPP controller, costmaps, planner)
│   └── rover_nav.rviz            # RViz2 config with Nav2 Goal tool
├── launch/
│   ├── rover_bringup.launch.py   # Full system bringup (MAVROS + ZED + Nav2 + bridge)
│   └── arm_and_go.launch.py      # Arm + GUIDED + send waypoints
├── rover_nav/
│   ├── __init__.py
│   ├── zed_mavros_bridge.py      # ZED VIO odom -> MAVROS vision_pose/speed
│   ├── cmd_vel_stamper.py        # Twist -> TwistStamped for MAVROS
│   └── arm_and_go.py             # Arm, set GUIDED, send Nav2 waypoints
├── scripts/
│   ├── set_ardupilot_params.py   # One-time ArduPilot parameter setup
│   └── set_origin_and_arm.py     # Per-session startup (origin + arm + GUIDED)
├── resource/rover_nav
├── package.xml
├── setup.py
└── setup.cfg
```

## Key Design Decisions

1. **Nav2 nodes launched directly** instead of `navigation_launch.py` — avoids collision_monitor, docking_server, and composition mode issues that break param loading.

2. **`cmd_vel_stamper` instead of `topic_tools relay`** — ArduPilot Rover only responds to `TwistStamped` with a `frame_id`, not plain `Twist`.

3. **`use_rotate_to_heading: false`** — Ackermann vehicles cannot rotate in place. The controller must always command forward velocity with steering.

4. **`zed_camera_link -> base_link` (child)** not `base_link -> zed_camera_link` — because ZED publishes `odom -> zed_camera_link`, and TF requires a single-parent tree.

5. **MAVROS wrapped in `GroupAction`** — prevents the MAVROS XML launch namespace from leaking to other nodes.

6. **Compass disabled indoors** — magnetic interference causes EKF divergence when compass conflicts with vision yaw.

7. **`MODE_CH=0`** — disables RC flight mode channel so MAVROS can control the mode without RC override.

## License

MIT
