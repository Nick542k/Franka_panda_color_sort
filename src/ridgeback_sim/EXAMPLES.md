# Ridgeback Simulation - Practical Examples

Working examples for controlling and monitoring the Ridgeback AMR.

## 🎯 Example 1: Basic Forward Movement

**Goal**: Make the robot move forward for 5 seconds, then stop.

```bash
# Terminal 1: Launch simulation
ros2 launch ridgeback_sim_launch ridgeback_sim.launch.py

# Terminal 2: Run the pre-built forward movement
ros2 run ridgeback_sim_control velocity_commander forward
```

**Expected behavior:**
- Robot moves forward at 0.5 m/s
- After 5 seconds, stops automatically
- Look for "Stopped" in terminal output

---

## 🎯 Example 2: Interactive Teleop

**Goal**: Manually control the robot with keyboard commands.

```bash
# Terminal 1: Launch simulation
ros2 launch ridgeback_sim_launch ridgeback_sim.launch.py

# Terminal 2: Interactive control
ros2 run ridgeback_sim_control velocity_commander interactive
```

**Controls:**
```
Command (f/b/l/r/s/q): f
← Move forward

Command (f/b/l/r/s/q): l
← Rotate left

Command (f/b/l/r/s/q): s
← Stop

Command (f/b/l/r/s/q): q
← Quit program
```

---

## 🎯 Example 3: Square Path

**Goal**: Make the robot trace a square path (2×2m).

**Create file**: `square_path.py`

```python
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
import time

rclpy.init()
node = rclpy.create_node('square_driver')
pub = node.create_publisher(Twist, '/cmd_vel', 10)

def move(linear, angular, duration):
    """Move with linear and angular velocity for duration."""
    msg = Twist()
    msg.linear.x = float(linear)
    msg.angular.z = float(angular)
    
    end_time = time.time() + duration
    while time.time() < end_time:
        pub.publish(msg)
        time.sleep(0.05)
    
    # Stop
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    pub.publish(msg)
    node.get_logger().info(f'Completed: linear={linear}, angular={angular}')

# Trace a square (assuming 1m/s ≈ ~4 sec to move 2m, 1 rad/s ≈ π/2 rotation = ~1.5 sec)
try:
    node.get_logger().info('Starting square path...')
    
    for side in range(4):
        node.get_logger().info(f'Side {side + 1}/4')
        
        # Move forward 2 meters
        move(linear=0.5, angular=0.0, duration=4.0)
        time.sleep(0.5)
        
        # Rotate 90 degrees (π/2 radians)
        move(linear=0.0, angular=1.0, duration=1.57)
        time.sleep(0.5)
    
    node.get_logger().info('Square complete!')

except KeyboardInterrupt:
    node.get_logger().info('Interrupted')

finally:
    rclpy.shutdown()
```

**Run it:**
```bash
python3 square_path.py
```

---

## 🎯 Example 4: Monitoring Live Sensor Data

**Goal**: Watch robot position and distance to nearest obstacle in real-time.

```bash
# Terminal 1: Launch simulation
ros2 launch ridgeback_sim_launch ridgeback_sim.launch.py

# Terminal 2: Sensor listener
ros2 run ridgeback_sim_control sensor_listener

# Terminal 3: Direct topic monitoring (in parallel)
# Watch odometry at 10 Hz
ros2 topic echo /odom --rate=10

# In another terminal, watch LiDAR nearest distance
ros2 topic echo /scan | grep "ranges:" | head -5
```

**Example output:**
```
[sensor_listener]: Pose: x=0.15, y=0.03, yaw=0.02 rad | Speed: 0.49 m/s
[sensor_listener]: Pose: x=0.45, y=0.06, yaw=0.04 rad | Speed: 0.49 m/s
[sensor_listener]: OBSTACLE DETECTED: 1.23m
```

---

## 🎯 Example 5: Obstacle Avoidance Demo

**Goal**: Robot automatically avoids obstacles using reactive LiDAR control.

```bash
# Terminal 1: Launch with obstacles world
ros2 launch ridgeback_sim_launch ridgeback_sim.launch.py world:=obstacles

# Terminal 2: Start obstacle avoider
ros2 run ridgeback_sim_control obstacle_avoider

# Terminal 3: Monitor what's happening
ros2 run ridgeback_sim_control sensor_listener
```

**Expected behavior:**
- Robot starts moving forward
- Detects box obstacle at 2m
- Turns to avoid (prefers left)
- Navigates around obstacles
- Continues forward
- **Watch the Ignition GUI** to see the robot maneuvering

**Adjust safety distance:**
```bash
ros2 run ridgeback_sim_control obstacle_avoider --ros-args \
    -p safe_distance:=1.0 \
    -p linear_speed:=0.25
```

---

## 🎯 Example 6: Autonomous Goal Navigation

**Goal**: Robot navigates to a specific (x, y) coordinate while avoiding obstacles.

**Create file**: `navigate_to_point.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class Navigator(Node):
    def __init__(self):
        super().__init__('point_navigator')
        
        self.pose = None
        self.yaw = 0.0
        self.min_dist = float('inf')
        
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.goal_x = 3.0
        self.goal_y = 3.0
        self.get_logger().info(f'Goal: ({self.goal_x}, {self.goal_y})')
    
    def odom_cb(self, msg):
        self.pose = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.yaw = 2 * math.atan2(q.z, q.w)
    
    def scan_cb(self, msg):
        self.min_dist = min(msg.ranges) if msg.ranges else float('inf')
    
    def navigate(self):
        if not self.pose:
            return False
        
        # Distance to goal
        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        dist = math.sqrt(dx**2 + dy**2)
        
        # Angle to goal
        goal_yaw = math.atan2(dy, dx)
        angle_err = goal_yaw - self.yaw
        
        # Wrap angle
        while angle_err > math.pi:
            angle_err -= 2 * math.pi
        while angle_err < -math.pi:
            angle_err += 2 * math.pi
        
        msg = Twist()
        
        if dist < 0.3:
            self.get_logger().info('✓ Goal reached!')
            return True
        
        if self.min_dist < 0.5:
            self.get_logger().warn(f'⚠ Obstacle {self.min_dist:.2f}m')
            msg.linear.x = 0.0
        else:
            if abs(angle_err) > 0.3:
                msg.linear.x = 0.2
                msg.angular.z = 0.5 if angle_err > 0 else -0.5
            else:
                msg.linear.x = 0.4
                msg.angular.z = angle_err * 0.5
        
        self.pub.publish(msg)
        self.get_logger().info(f'dist={dist:.2f}m angle_err={angle_err:.2f}rad')
        return False

def main():
    rclpy.init()
    nav = Navigator()
    
    try:
        while True:
            rclpy.spin_once(nav, timeout_sec=0.1)
            if nav.navigate():
                break
    except KeyboardInterrupt:
        msg = Twist()
        nav.pub.publish(msg)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run it:**
```bash
python3 navigate_to_point.py
```

---

## 🎯 Example 7: Multi-Behavior Sequencer

**Goal**: Execute a sequence of behaviors (move, turn, wait, repeat).

**Create file**: `behavior_sequence.py`

```python
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from enum import Enum
import time

class Behavior(Enum):
    MOVE_FWD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    STOP = 4

class BehaviorSequencer:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('behavior_sequencer')
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Define behavior sequence: (behavior, duration, param)
        self.sequence = [
            (Behavior.MOVE_FWD, 3.0, 0.5),      # Move forward 3 sec
            (Behavior.TURN_RIGHT, 1.5, 1.0),    # Turn right 1.5 sec
            (Behavior.MOVE_FWD, 3.0, 0.5),      # Move forward 3 sec
            (Behavior.TURN_LEFT, 1.5, 1.0),     # Turn left 1.5 sec
        ]
        
        self.step = 0
    
    def execute_behavior(self, behavior, param):
        msg = Twist()
        
        if behavior == Behavior.MOVE_FWD:
            msg.linear.x = param
        elif behavior == Behavior.TURN_LEFT:
            msg.angular.z = param
        elif behavior == Behavior.TURN_RIGHT:
            msg.angular.z = -param
        elif behavior == Behavior.STOP:
            pass
        
        return msg
    
    def run(self):
        for behavior, duration, param in self.sequence:
            self.node.get_logger().info(
                f'Executing {behavior.name} for {duration}s'
            )
            
            msg = self.execute_behavior(behavior, param)
            end = time.time() + duration
            
            while time.time() < end:
                self.pub.publish(msg)
                time.sleep(0.05)
            
            # Stop between behaviors
            stop_msg = Twist()
            self.pub.publish(stop_msg)
            time.sleep(0.2)
        
        self.node.get_logger().info('Sequence complete!')
        rclpy.shutdown()

if __name__ == '__main__':
    seq = BehaviorSequencer()
    seq.run()
```

**Run it:**
```bash
python3 behavior_sequence.py
```

---

## 🎯 Example 8: Live ROS Topic Control (No Python)

**Goal**: Control robot using `ros2 topic pub` commands directly.

```bash
# Terminal 1: Launch simulation
ros2 launch ridgeback_sim_launch ridgeback_sim.launch.py

# Terminal 2: Move forward at 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Terminal 3: Rotate right (press Ctrl+C to stop)
ros2 topic pub /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}'

# Diagonal movement (forward + left turn)
ros2 topic pub /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'

# STOP
ros2 topic pub /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

---

## 📊 Tips & Tricks

### Parallel Monitoring
```bash
# Watch everything in one terminal with tmux
tmux new-session -d -s monitor
tmux send-keys -t monitor "ros2 topic echo /odom --rate=2" Enter
tmux split-window -h -t monitor
tmux send-keys -t monitor "ros2 topic echo /scan --rate=1" Enter
tmux split-window -v -t monitor "C-b |"
tmux send-keys -t monitor "ros2 run ridgeback_sim_control sensor_listener" Enter
```

### Record & Playback
```bash
# Record a rosbag of robot motion
ros2 bag record /odom /scan /imu/data -o my_run

# Playback later (simulate offline)
ros2 bag play my_run
```

### Debugging Transforms
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Monitor specific transform
ros2 run tf2_ros tf2_echo map base_link
```

### Performance Profiling
```bash
# Monitor CPU/Memory
ros2 run ros2_system_monitor monitor

# Profile individual nodes
ros2 run ros_profiling profile velocity_commander
```

---

## 🎓 Learning Path

1. **Start here**: Example 2 (Interactive Teleop) - get comfortable with controls
2. **Move to**: Example 1 (Forward Movement) - scripted behavior
3. **Practice**: Example 3 (Square Path) - choreographed motion
4. **Monitor**: Example 4 (Sensor Data) - understand sensor streams
5. **Challenge**: Example 5 (Obstacle Avoidance) - reactive behavior
6. **Advanced**: Example 6 (Goal Navigation) - full autonomous control
7. **Master**: Example 7 (Behavior Sequencing) - complex missions

---

**Ready to try? Start with:**
```bash
cd ~/ridgeback_sim
source install/setup.bash
ros2 launch ridgeback_sim_launch ridgeback_sim.launch.py
# In another terminal:
ros2 run ridgeback_sim_control velocity_commander interactive
```
