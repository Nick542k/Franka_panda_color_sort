# Ridgeback AMR Simulation - Quick Start Guide

Simulate and control a Ridgeback Autonomous Mobile Robot in ROS2 Humble with Ignition Fortress.

## 📋 Prerequisites

```bash
# ROS2 Humble (assumed installed)
source /opt/ros/humble/setup.bash

# Ignition Fortress
sudo apt install ignition-fortress

# Gazebo ROS integration
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-msgs

# ROS GZ Bridge (for topic bridging)
sudo apt install ros-humble-ros-gzbridge

# Topic tools (for relay)
sudo apt install ros-humble-topic-tools

# Ridgeback packages (if available)
# Option 1: Pre-built binaries
sudo apt install ros-humble-ridgeback-*

# Option 2: Build from source
# git clone https://github.com/clearpath-gh/ridgeback.git
# cd ridgeback && colcon build
```

## 🚀 Setup & Build

```bash
cd ~/ridgeback_sim

# Build the workspace
colcon build --symlink-install

# Source the setup
source install/setup.bash
```

## ▶️ Running the Simulation

### Option 1: Full Simulation (Server + GUI)
```bash
# Launch Ridgeback in Ignition Fortress
ros2 launch ridgeback_sim_launch ridgeback_sim.launch.py

# In another terminal, test velocity commands
ros2 run ridgeback_sim_control velocity_commander forward
```

### Option 2: Manual Mode (Full Control)

**Terminal 1 - Start Ignition:**
```bash
ign gazebo empty.sdf -v 2
```

**Terminal 2 - Bring up Ridgeback (if using pre-built package):**
```bash
ros2 launch ridgeback_description description.launch.py
```

**Terminal 3 - Run sensor listener:**
```bash
ros2 run ridgeback_sim_control sensor_listener
```

**Terminal 4 - Run velocity commander:**
```bash
ros2 run ridgeback_sim_control velocity_commander interactive
```

## 🎮 Control Commands

### Velocity Commander
```bash
# Interactive mode
ros2 run ridgeback_sim_control velocity_commander interactive
# Commands: f(forward) b(back) l(left) r(right) s(stop) q(quit)

# Move forward for 5 seconds
ros2 run ridgeback_sim_control velocity_commander forward

# Rotate in place for 4 seconds
ros2 run ridgeback_sim_control velocity_commander rotate
```

### Obstacle Avoidance
```bash
# Start avoider (reacts to /scan topic automatically)
ros2 run ridgeback_sim_control obstacle_avoider

# Set safe distance parameter
ros2 run ridgeback_sim_control obstacle_avoider --ros-args -p safe_distance:=0.5
```

### Autonomous Navigation
```bash
# Navigate to goal (3, 3) while avoiding obstacles
ros2 run ridgeback_sim_control autonomous_navigator
```

## 📊 Monitoring Topics

```bash
# List all active topics
ros2 topic list -t

# Monitor odometry
ros2 topic echo /odom

# Monitor IMU data
ros2 topic echo /imu/data

# Monitor LiDAR scan
ros2 topic echo /scan --rate=1  # Reduce output rate

# Publish manual velocity (0.5 m/s forward)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

## 🛠️ Troubleshooting

### Ignition Fortress not found
```bash
# Check installation
ignition gazebo --version

# If missing, install:
sudo apt install ignition-fortress

# May need to reinstall ROS GZ bridge
sudo apt reinstall ros-humble-ros-gzbridge
```

### ROS topics not appearing
- Ensure `ros_gz_bridge` is running
- Check topic remappings in launch file
- Verify Ignition Fortress world file path

### Ridgeback model not spawning
- Ensure Clearpath Ridgeback package is installed
- Check `~/.local/share/ignition/` for model files
- May need to manually spawn:
  ```bash
  ign service -s /spawn --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 2000 --req 'sdf: "<?xml version=\"1.0\" ?><sdf version=\"1.10\">...</sdf>"'
  ```

### Bridge connection fails
```bash
# Verify Ignition server is running
ps aux | grep ignition

# Check ROS2 <-> Ignition bridge
ros2 run ros_gz_bridge parameter_bridge -h
```

## 📁 Project Structure

```
ridgeback_sim/
├── ridgeback_sim_control/
│   ├── ridgeback_sim_control/
│   │   ├── velocity_commander.py      # Publish cmd_vel
│   │   ├── sensor_listener.py         # Monitor sensors
│   │   ├── obstacle_avoider.py        # LiDAR-based avoidance
│   │   └── autonomous_navigator.py    # Goal-seeking with avoidance
│   ├── setup.py
│   └── package.xml
├── ridgeback_sim_launch/
│   ├── launch/
│   │   └── ridgeback_sim.launch.py   # Main launch file
│   ├── worlds/
│   │   ├── empty.sdf                 # Empty environment
│   │   └── obstacles.sdf             # Environment with obstacles
│   ├── CMakeLists.txt
│   └── package.xml
└── QUICKSTART.md (this file)
```

## 💡 Next Steps

1. **Test basic movement**: Run velocity commander in interactive mode
2. **Monitor sensors**: Watch odometry/IMU/LiDAR in separate terminals
3. **Test obstacle avoidance**: Launch obstacles world and run avoider node
4. **Autonomous navigation**: Set goals and watch autonomous nav work
5. **Extend**: Add custom controllers, planners, or perception nodes

## 📚 Useful References

- ROS2 Humble docs: https://docs.ros.org/en/humble/
- Ignition Fortress docs: https://ignitionrobotics.org/docs/fortress
- Clearpath Ridgeback: https://www.clearpathrobotics.com/ridgeback-ug/
- ROS GZ Bridge: https://github.com/gazebosim/ros_gz

---

**Status**: ✅ Ready to build and run
