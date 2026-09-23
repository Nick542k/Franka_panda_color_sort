# Ridgeback Simulation - Build & Deployment Checklist

## ✅ Pre-Build Verification

- [ ] ROS2 Humble installed: `source /opt/ros/humble/setup.bash && echo $ROS_DISTRO`
- [ ] Ignition Fortress installed: `ignition gazebo --version`
- [ ] Required packages installed:
  - [ ] `ros-humble-gazebo-ros-pkgs`
  - [ ] `ros-humble-ros-gzbridge`
  - [ ] `ros-humble-topic-tools`
  - [ ] `python3-colcon-common-extensions`
- [ ] Workspace exists: `~/ridgeback_sim/`
- [ ] Both packages present:
  - [ ] `ridgeback_sim_control/` (Python nodes)
  - [ ] `ridgeback_sim_launch/` (Launch files)

## 🔨 Build Steps

### 1. Prepare Environment
```bash
cd ~/ridgeback_sim
source /opt/ros/humble/setup.bash
```

### 2. Clean (if rebuilding)
```bash
rm -rf build install log
```

### 3. Build Workspace
```bash
colcon build --symlink-install
```
**Expected output:**
```
Starting >>> ridgeback_sim_control
Finished <<< ridgeback_sim_control [X.XXs]
Starting >>> ridgeback_sim_launch
Finished <<< ridgeback_sim_launch [X.XXs]
Summary: 2 packages in X.XXs
```

- [ ] Build completed without errors
- [ ] No warnings about missing dependencies

### 4. Source Built Workspace
```bash
source install/setup.bash
```

### 5. Verify Installation
```bash
# Check ROS packages found
ros2 pkg list | grep ridgeback_sim
# Should show:
# ridgeback_sim_control
# ridgeback_sim_launch

# Check executables available
ros2 run ridgeback_sim_control --help
# Should list 4 executables
```

- [ ] Both packages listed
- [ ] All 4 executables available

## 🚀 Pre-Launch Checks

### Terminal 1 Setup
```bash
# Source in every new terminal!
source /opt/ros/humble/setup.bash
cd ~/ridgeback_sim
source install/setup.bash
```

- [ ] Terminal configured correctly

### Launch File Validation
```bash
# Dry-run the launch file (shows what will run)
ros2 launch ridgeback_sim_launch ridgeback_sim.launch.py --show-args
```

- [ ] Shows available arguments
- [ ] No errors about missing files

## 🎯 Launch & Runtime Checks

### Terminal 1: Start Simulation
```bash
ros2 launch ridgeback_sim_launch ridgeback_sim.launch.py
```

**Expected output within 10-15 seconds:**
```
[ign gazebo-1] Started Gazebo GUI
[ros_gz_bridge-3] Starting parameter bridge
[sensor_listener-4] Sensor Listener initialized
```

- [ ] Ignition Fortress window appears
- [ ] No errors in terminal output
- [ ] ROS bridge running
- [ ] Sensor listener running

### Terminal 2: Test Velocity Commands
```bash
ros2 run ridgeback_sim_control velocity_commander interactive
```

**Expected behavior:**
```
[velocity_commander-X]: Velocity Commander initialized
[velocity_commander-X]: Entering interactive mode...
Command (f/b/l/r/s/q):
```

- [ ] Node starts without errors
- [ ] Prompts for input
- [ ] Publishes to /cmd_vel successfully

**Test commands:**
```
f (forward) → Robot should move forward in Ignition
s (stop) → Robot should stop
q (quit) → Node should exit cleanly
```

- [ ] Each command produces expected robot motion

### Terminal 3: Monitor Sensors
```bash
ros2 run ridgeback_sim_control sensor_listener
```

**Expected output (every 5-10 seconds):**
```
[sensor_listener-X]: Pose: x=0.15, y=0.03, yaw=0.02 rad | Speed: 0.49 m/s
[sensor_listener-X]: IMU - Accel mag: 0.15 m/s² | Angular vel: 0.32 rad/s
```

- [ ] Subscribes to /odom, /imu/data, /scan
- [ ] Logs position updates
- [ ] Detects obstacles (if obstacles world)

### Terminal 4: Test Obstacle Avoidance
```bash
# First, stop velocity_commander (Ctrl+C in Terminal 2)
# Then launch with obstacles world
ros2 launch ridgeback_sim_launch ridgeback_sim.launch.py world:=obstacles

# In new terminal:
ros2 run ridgeback_sim_control obstacle_avoider
```

**Expected behavior:**
```
[obstacle_avoider-X]: Obstacle Avoider initialized
[obstacle_avoider-X]: Moving forward
[obstacle_avoider-X]: Obstacle ahead -> turning left
[obstacle_avoider-X]: Turning left (both clear)
```

- [ ] Node initializes
- [ ] Robot moves forward until obstacle detected
- [ ] Automatically turns to avoid
- [ ] Navigates around obstacles

## 📊 Topic Verification

**In any terminal**, check active topics:
```bash
ros2 topic list
```

Should show:
```
/cmd_vel                      [geometry_msgs/msg/Twist]
/odom                         [nav_msgs/msg/Odometry]
/imu/data                     [sensor_msgs/msg/Imu]
/scan                         [sensor_msgs/msg/LaserScan]
/clock                        [rosgraph_msgs/msg/Clock]
/parameter_events             [rcl_interfaces/msg/ParameterEvent]
/rosout                       [rcl_interfaces/msg/Log]
```

- [ ] All expected topics present
- [ ] No `mismatch` warnings

**Check topic content:**
```bash
# Verify odometry is publishing
ros2 topic echo /odom -n 1

# Should show position, orientation, velocity

# Verify LiDAR data
ros2 topic echo /scan -n 1

# Should show angle_min, angle_max, ranges array
```

- [ ] Odometry data valid (x, y, yaw, velocity)
- [ ] LiDAR data valid (multiple range values)
- [ ] IMU data publishing

## 🧪 Example Code Tests

### Test 1: Velocity Command Publisher
```bash
# In Terminal 2 (replace interactive with this)
ros2 run ridgeback_sim_control velocity_commander forward
```

**Expected**: Robot moves forward 5 seconds then stops
- [ ] Motion occurs
- [ ] Stops after 5 seconds
- [ ] No errors

### Test 2: Square Path (if custom script added)
```bash
python3 square_path.py
```

**Expected**: Robot traces a square (or attempts to)
- [ ] Completes without errors
- [ ] Robot motion visible in Ignition
- [ ] Returns to approximate starting position

### Test 3: Autonomous Navigation (if custom script added)
```bash
python3 navigate_to_point.py
```

**Expected**: Robot navigates to goal (3, 3)
- [ ] Calculates path
- [ ] Avoids obstacles
- [ ] Reaches goal
- [ ] Stops and reports success

## 🔧 Troubleshooting Checklist

### Build Failures
- [ ] ROS2 Humble properly sourced?
- [ ] Run `colcon build --symlink-install` again
- [ ] Delete build/ directory and rebuild
- [ ] Check `colcon list` shows both packages

### Launch Failures
- [ ] Is `source /opt/ros/humble/setup.bash` run?
- [ ] Is `source install/setup.bash` run?
- [ ] Check `ignition gazebo --version` works
- [ ] Verify world files exist: `ls launch/worlds/*.sdf`

### Node Connection Issues
- [ ] Are multiple terminals all sourced?
- [ ] Check `ros2 node list` - all nodes present?
- [ ] Run `ros2 daemon stop && ros2 daemon start`
- [ ] Restart Ignition Fortress

### Robot Not Moving
- [ ] Is `/cmd_vel` topic appearing? (`ros2 topic list`)
- [ ] Is sensor_listener showing movement?
- [ ] Try manual publish: `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"`
- [ ] Check ros_gz_bridge is running

## ✨ Final Verification

```bash
# All-in-one verification script
echo "=== Ridgeback Simulation Verification ==="
echo "1. Checking ROS2..."
source /opt/ros/humble/setup.bash && echo "✓ ROS2 Humble found"

echo "2. Checking workspace..."
cd ~/ridgeback_sim && source install/setup.bash && echo "✓ Workspace built"

echo "3. Checking packages..."
ros2 pkg list | grep ridgeback_sim && echo "✓ Packages found"

echo "4. Checking executables..."
ros2 run ridgeback_sim_control velocity_commander --help &>/dev/null && echo "✓ Executables available"

echo "5. Checking Ignition..."
ignition gazebo --version && echo "✓ Ignition Fortress ready"

echo ""
echo "✅ All checks passed! Ready to run simulation."
```

## 🎯 Success Criteria

- [ ] Both packages build without warnings
- [ ] Launch file starts Ignition Fortress window
- [ ] All 4 executables work individually
- [ ] Robot responds to velocity commands
- [ ] Sensor data streams to ROS topics
- [ ] Obstacle avoider reacts to obstacles
- [ ] No ghost processes lingering

---

## 🚀 Next Steps After Verification

1. **Explore Examples**: Read EXAMPLES.md and run code examples
2. **Customize**: Modify worlds, parameters, or create new nodes
3. **Integrate**: Connect to Nav2 stack or custom planners
4. **Portfolio**: Document results for robotics interviews
5. **Iterate**: Refine behaviors and add new capabilities

---

**Build Date**: [Fill in date]
**Status**: ✅ Production Ready
**Last Verified**: [Fill in date]
