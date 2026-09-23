# Ridgeback AMR Simulation with ROS2 Humble & Ignition Fortress

Complete simulation framework for the **Clearpath Ridgeback** Autonomous Mobile Robot in **ROS2 Humble** with **Gazebo Ignition Fortress**.

## 🎯 Overview

This project provides:
- ✅ Full Ridgeback AMR simulation in Ignition Fortress
- ✅ ROS2 bridge for topic communication
- ✅ 4 Python control nodes for different behaviors
- ✅ LiDAR-based obstacle avoidance
- ✅ Autonomous goal-seeking navigation
- ✅ Sensor monitoring (odometry, IMU, LiDAR)
- ✅ Interactive velocity control

## 🤖 Robot Specifications

**Ridgeback AMR** (Clearpath):
- **Type**: Differential-drive Mobile Manipulator Platform
- **Dimensions**: ~1.16m L × 0.78m W × 0.40m H
- **Payload**: Up to 50 kg
- **Sensors**:
  - GPU LiDAR (2D scanning, 20 Hz, ±270°, 40m range)
  - 9-DOF IMU (accel, gyro, magnetometer)
  - Wheel encoders for odometry
  - Optional RGB-D camera
- **Actuation**: Differential wheels + caster wheels

## 📦 Packages

### 1. `ridgeback_sim_control` (Python)
Control and sensing nodes for the Ridgeback.

**Executables:**
- `velocity_commander` - Publish geometry_msgs/Twist commands
- `sensor_listener` - Monitor odometry, IMU, LiDAR
- `obstacle_avoider` - Reactive obstacle avoidance using LiDAR
- `autonomous_navigator` - Goal-seeking with avoidance

### 2. `ridgeback_sim_launch` (Launch Files)
Ignition Fortress configuration and launch files.

**Files:**
- `ridgeback_sim.launch.py` - Main simulation launcher
- `worlds/empty.sdf` - Empty test environment
- `worlds/obstacles.sdf` - Environment with static obstacles

## 🚀 Quick Start

### Installation

```bash
# Install dependencies (Ubuntu 22.04 with ROS2 Humble)
sudo apt install -y \
    ignition-fortress \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gzbridge \
    ros-humble-topic-tools

# Clone/setup this repo (already done at ~/ridgeback_sim)
cd ~/ridgeback_sim

# Build
./setup.sh
# or manually:
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Run Simulation

```bash
# Source workspace
source ~/ridgeback_sim/install/setup.bash

# Launch Ridgeback in Ignition Fortress
ros2 launch ridgeback_sim_launch ridgeback_sim.launch.py
```

### Control the Robot

**Terminal 2** - Velocity commands:
```bash
ros2 run ridgeback_sim_control velocity_commander interactive
# Commands: f(forward) b(back) l(left) r(right) s(stop) q(quit)
```

**Terminal 3** - Monitor sensors:
```bash
ros2 run ridgeback_sim_control sensor_listener
```

**Terminal 4** - Enable obstacle avoidance:
```bash
ros2 run ridgeback_sim_control obstacle_avoider
```

## 🎮 Node Details

### `velocity_commander`
Publishes `geometry_msgs/Twist` to `/cmd_vel`.

```bash
# Interactive mode (manual control)
ros2 run ridgeback_sim_control velocity_commander interactive

# Preset behaviors
ros2 run ridgeback_sim_control velocity_commander forward
ros2 run ridgeback_sim_control velocity_commander rotate
```

**Parameters:**
- None (uses hardcoded speeds in functions)

**Topics:**
- Publishes: `/cmd_vel` (geometry_msgs/Twist)

---

### `sensor_listener`
Subscribes to and logs sensor data.

```bash
ros2 run ridgeback_sim_control sensor_listener
```

**Topics:**
- Subscribes:
  - `/odom` (nav_msgs/Odometry)
  - `/imu/data` (sensor_msgs/Imu)
  - `/scan` (sensor_msgs/LaserScan)
- Provides status via logging

---

### `obstacle_avoider`
Reactive obstacle avoidance using LiDAR.

```bash
ros2 run ridgeback_sim_control obstacle_avoider

# Custom parameters
ros2 run ridgeback_sim_control obstacle_avoider --ros-args \
    -p safe_distance:=0.6 \
    -p linear_speed:=0.4
```

**Parameters:**
- `safe_distance` (float): Minimum safe distance to obstacles (default: 0.5m)
- `linear_speed` (float): Forward speed when clear (default: 0.3 m/s)
- `angular_speed` (float): Rotation speed when turning (default: 0.5 rad/s)

**Topics:**
- Subscribes: `/scan` (sensor_msgs/LaserScan)
- Publishes: `/cmd_vel` (geometry_msgs/Twist)

**Behavior:**
- Scans LiDAR into 3 sectors (front, left, right)
- If front clear: move forward with proportional steering
- If front blocked but sides clear: turn toward clear side
- If all sides blocked: spin in place
- Emergency stop if obstacle < 0.3m

---

### `autonomous_navigator`
Goal-seeking navigation with obstacle avoidance.

```bash
ros2 run ridgeback_sim_control autonomous_navigator

# Custom parameters
ros2 run ridgeback_sim_control autonomous_navigator --ros-args \
    -p max_linear_speed:=0.6 \
    -p goal_tolerance:=0.3
```

**Parameters:**
- `max_linear_speed` (float): Maximum forward velocity (default: 0.5 m/s)
- `max_angular_speed` (float): Maximum rotation velocity (default: 1.0 rad/s)
- `goal_tolerance` (float): Distance threshold to consider goal reached (default: 0.2m)
- `obstacle_distance` (float): Minimum safe distance (default: 0.6m)

**Topics:**
- Subscribes:
  - `/odom` (nav_msgs/Odometry)
  - `/scan` (sensor_msgs/LaserScan)
- Publishes: `/cmd_vel` (geometry_msgs/Twist)

**Behavior:**
- Set goal via `set_goal(x, y)` method
- Calculates angle and distance to goal
- If far from goal angle: turn first
- Once aligned: move forward while steering toward goal
- Stops if obstacles detected or goal reached

**Modified Example:**
```python
# Edit autonomous_navigator.py to change goal
node.set_goal(5.0, 5.0)  # Navigate to (5, 5)
```

## 📊 Topics & Messages

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | → | Robot velocity commands |
| `/odom` | nav_msgs/Odometry | ← | Robot pose & velocity |
| `/imu/data` | sensor_msgs/Imu | ← | IMU acceleration & rotation |
| `/scan` | sensor_msgs/LaserScan | ← | 2D LiDAR scan data |

## 🌍 Simulation Environments

### Empty World
```bash
ros2 launch ridgeback_sim_launch ridgeback_sim.launch.py world:=empty
```
- Flat 100×100m plane
- Single sun light
- No obstacles

### Obstacles World
```bash
ros2 launch ridgeback_sim_launch ridgeback_sim.launch.py world:=obstacles
```
- 4 static box obstacles
- 1 wall
- Good for testing avoidance

## 🛠️ Architecture

```
ROS2 Humble
    ↓
[velocity_commander] → /cmd_vel
[sensor_listener] ← /odom, /imu/data, /scan
[obstacle_avoider] ← /scan → /cmd_vel
[autonomous_navigator] ← /odom, /scan → /cmd_vel
    ↓
ROS ↔ Gz Bridge
    ↓
Ignition Fortress
    ↓
Ridgeback Robot Model
    ├─ Differential Drive
    ├─ GPU LiDAR Plugin
    ├─ IMU Plugin
    └─ Odometry Plugin
```

## 🔧 Customization

### Change Robot Parameters
Edit the launch file to modify robot initial pose:
```python
# ridgeback_sim_launch/launch/ridgeback_sim.launch.py
initial_pose = Pose(position=Point(x=0, y=0, z=0.1))
```

### Add Custom Behaviors
Create new node in `ridgeback_sim_control/`:
```python
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MyBehavior(Node):
    def __init__(self):
        super().__init__('my_behavior')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def custom_control(self):
        # Your logic here
        msg = Twist()
        self.cmd_pub.publish(msg)
```

Add to `setup.py`:
```python
'my_behavior = ridgeback_sim_control.my_behavior:main',
```

### Extend Worlds
Edit `.sdf` files in `worlds/` to add more obstacles or change lighting.

## 📚 Key Files

```
ridgeback_sim/
├── ridgeback_sim_control/
│   ├── ridgeback_sim_control/
│   │   ├── __init__.py
│   │   ├── velocity_commander.py       (140 lines)
│   │   ├── sensor_listener.py          (130 lines)
│   │   ├── obstacle_avoider.py         (130 lines)
│   │   └── autonomous_navigator.py     (180 lines)
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
├── ridgeback_sim_launch/
│   ├── launch/
│   │   └── ridgeback_sim.launch.py     (100 lines)
│   ├── worlds/
│   │   ├── empty.sdf
│   │   └── obstacles.sdf
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── cmake/
└── README.md (this file)
```

## 🐛 Troubleshooting

### Simulation won't start
```bash
# Check Ignition Fortress
ignition gazebo --version

# Verify ROS2 Humble
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO  # Should print "humble"

# Rebuild
colcon build --symlink-install
source install/setup.bash
```

### Ridgeback model not spawning
```bash
# Ensure Clearpath packages installed
sudo apt install ros-humble-ridgeback-description

# Or manually add to launch file
```

### Topics not connecting
```bash
# Check active nodes
ros2 node list

# Check topics
ros2 topic list -t

# Check ros_gz_bridge connection
ps aux | grep ros_gz_bridge
```

### Low performance
- Reduce physics update rate (edit .sdf)
- Disable GUI: `-r` flag instead of `-g`
- Run in headless mode on weak hardware

## 📖 References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Ignition Gazebo Docs](https://gazebosim.org/docs/fortress/)
- [Clearpath Ridgeback](https://www.clearpathrobotics.com/ridgeback-ug/)
- [ROS GZ Bridge](https://github.com/gazebosim/ros_gz)

## 📄 License

Apache License 2.0

## 👤 Author

Nithish - Robotics Engineering

---

**Last Updated**: September 2026  
**Status**: ✅ Production Ready
