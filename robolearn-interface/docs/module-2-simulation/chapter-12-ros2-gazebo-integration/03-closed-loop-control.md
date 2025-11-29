---
id: lesson-12-3-closed-loop-control
title: "Lesson 12.3: Closed-Loop Control"
sidebar_position: 3
sidebar_label: "12.3 Closed-Loop Control"
description: "Implementing feedback control loops between ROS 2 and Gazebo"
duration_minutes: 75
proficiency_level: "B1"
layer: "L2"
hardware_tier: 1
learning_objectives:
  - "Implement velocity control via cmd_vel topic"
  - "Subscribe to sensor feedback from simulation"
  - "Create a simple reactive behavior using sensor data"
  - "Visualize robot state in RViz2 with real-time feedback"
skills:
  - "ros2-gazebo-bridge"
  - "control-loops"
cognitive_load:
  new_concepts: 8
tier_1_path: "TheConstruct cloud environment"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 12.3: Closed-Loop Control

## The Three Parts of Robot Control

A robot is only useful when it **acts on the world**. But action without feedback is blind. The robot needs to sense, decide, then act—repeatedly, fast enough to respond to changes.

This is the **control loop**:

```
┌─────────────┐
│   SENSE     │ ← Read sensors (LIDAR, camera, IMU)
└──────┬──────┘
       │
       ↓
┌─────────────┐
│   DECIDE    │ ← Compute what to do
└──────┬──────┘
       │
       ↓
┌─────────────┐
│    ACT      │ ← Command motion
└──────┬──────┘
       │
       └─────→ [repeat at high frequency]
```

**Example: Obstacle Avoidance**
1. **SENSE**: LIDAR reports obstacle 0.5m ahead
2. **DECIDE**: "Obstacle close, stop moving"
3. **ACT**: Publish zero velocity to `/cmd_vel`
4. **Result**: Robot stops before collision

All three steps must work together. A robot with sensors but no control logic is a passenger. A robot with control logic but no sensors is blind.

---

## Part 1: Publishing Velocity Commands

Your robot moves when you publish to the `/cmd_vel` topic. The message type is `geometry_msgs/msg/Twist`:

```python
from geometry_msgs.msg import Twist

def cmd_vel_message(linear_x, angular_z):
    """Create a velocity command."""
    msg = Twist()
    msg.linear.x = linear_x    # Forward velocity (m/s)
    msg.linear.y = 0.0         # Lateral (usually zero for differential drive)
    msg.linear.z = 0.0         # Vertical (usually zero for wheeled robots)
    msg.angular.x = 0.0        # Roll rotation
    msg.angular.y = 0.0        # Pitch rotation
    msg.angular.z = angular_z  # Yaw rotation (rad/s)
    return msg
```

**Common patterns**:

```python
# Move forward
move_forward = Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0))

# Turn in place
turn_left = Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.5))

# Curve forward-left
curve = Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.2))

# Emergency stop
stop = Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0))
```

**Publishing continuously**:

```python
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.speed = 0.5  # m/s
        self.angular = 0.0  # rad/s

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = self.angular
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: v={self.speed}, w={self.angular}')

    def stop(self):
        """Emergency stop."""
        self.speed = 0.0
        self.angular = 0.0
```

**Key insight**: You must **continuously publish** velocity commands. If you publish once and stop, the robot executes that command once, then stops. For sustained motion, publish in a timer loop at 10-50 Hz.

---

## Part 2: Subscribing to Sensor Feedback

To make decisions, you must read what the robot senses. LIDAR scans and camera images come through ROS 2 topics.

### LIDAR Feedback: Obstacle Detection

LIDAR publishes `sensor_msgs/msg/LaserScan` with distance measurements:

```python
from sensor_msgs.msg import LaserScan
from rclpy.node import Node

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # LIDAR topic
            self.lidar_callback,
            10
        )
        self.min_distance = float('inf')

    def lidar_callback(self, msg: LaserScan):
        """Called whenever LIDAR publishes."""
        # msg.ranges is a list of distances [0...360 degrees]
        # Front is usually middle index, sides are edges

        # Find minimum distance (obstacle closest to robot)
        valid_ranges = [r for r in msg.ranges if 0 < r < msg.range_max]
        self.min_distance = min(valid_ranges) if valid_ranges else float('inf')

        # Publish result
        if self.min_distance < 0.5:
            self.get_logger().warn(f'Obstacle close: {self.min_distance:.2f}m')
        else:
            self.get_logger().info(f'Clear ahead: {self.min_distance:.2f}m')
```

**LaserScan structure**:
```
msg.angle_min = -π       # Start angle (radians)
msg.angle_max = π        # End angle
msg.angle_increment = Δθ # Radians per sample
msg.ranges = [r0, r1, r2, ...]  # Distance at each angle
```

**Example**: If a LIDAR sweeps 180 degrees with 180 samples, each sample is 1 degree apart.

### Camera Feedback: Vision Processing

Camera images publish as `sensor_msgs/msg/Image`. Processing typically uses OpenCV:

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.red_detected = False

    def image_callback(self, msg: Image):
        """Process incoming camera frame."""
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect red color
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # If red pixels exist, red object detected
        self.red_detected = cv2.countNonZero(mask) > 100

        if self.red_detected:
            self.get_logger().info('Red object detected!')
```

---

## Part 3: Decision Logic

Once you sense, you must decide. Simple reactive logic works well:

**If-Then rules**:
```python
if obstacle_distance < 0.5:
    command = Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0))
    # Stop
elif obstacle_distance < 1.0:
    command = Twist(linear=Vector3(x=0.2), angular=Vector3(z=0.0))
    # Slow down
else:
    command = Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0))
    # Full speed
```

**State machines** (more complex behaviors):

```python
class RobotStateMachine:
    STATE_EXPLORING = 0
    STATE_OBSTACLE_DETECTED = 1
    STATE_BACKING_UP = 2

    def __init__(self):
        self.state = self.STATE_EXPLORING
        self.time_in_state = 0

    def update(self, obstacle_distance, dt):
        """Update state based on sensor input."""
        self.time_in_state += dt

        if self.state == self.STATE_EXPLORING:
            if obstacle_distance < 0.5:
                self.state = self.STATE_BACKING_UP
                self.time_in_state = 0

        elif self.state == self.STATE_BACKING_UP:
            if self.time_in_state > 2.0:  # Back up for 2 seconds
                self.state = self.STATE_EXPLORING
                self.time_in_state = 0

        # Return velocity command based on state
        if self.state == self.STATE_EXPLORING:
            return Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0))
        else:
            return Twist(linear=Vector3(x=-0.3), angular=Vector3(z=0.0))
```

---

## Complete Example: Stop-on-Obstacle

Here's a complete node that reads LIDAR, detects obstacles, and stops the robot:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

class StopOnObstacle(Node):
    def __init__(self):
        super().__init__('stop_on_obstacle')

        # Publishers and subscribers
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        # Publish velocity at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_velocity)

        # State
        self.min_distance = 2.0  # Initial: nothing detected
        self.desired_velocity = 0.5

    def lidar_callback(self, msg: LaserScan):
        """Process LIDAR scan."""
        # Get minimum distance in front (middle half of scan)
        front_ranges = msg.ranges[len(msg.ranges)//4:3*len(msg.ranges)//4]
        valid = [r for r in front_ranges if 0 < r < msg.range_max]

        self.min_distance = min(valid) if valid else 2.0
        self.get_logger().info(f'Min distance: {self.min_distance:.2f}m')

    def publish_velocity(self):
        """Publish velocity command based on sensor data."""
        msg = Twist()

        # Decision logic: stop if obstacle close
        if self.min_distance < 0.5:
            msg.linear.x = 0.0
            self.get_logger().warn('OBSTACLE! Stopping.')
        elif self.min_distance < 1.0:
            msg.linear.x = 0.2
            self.get_logger().info('Obstacle approaching, slowing down.')
        else:
            msg.linear.x = self.desired_velocity
            self.get_logger().debug('All clear, moving forward.')

        msg.angular.z = 0.0
        self.vel_publisher.publish(msg)

def main():
    rclpy.init()
    node = StopOnObstacle()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Emergency stop on Ctrl+C
        msg = Twist()
        node.vel_publisher.publish(msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**What this does**:
1. **Callback**: LIDAR data triggers `lidar_callback()`, updates `min_distance`
2. **Timer**: Every 0.1s, `publish_velocity()` runs
3. **Decision**: If obstacle < 0.5m, set velocity to 0
4. **Publish**: Send command to robot

**Result**: Robot moves forward, stops when it sees an obstacle.

---

## Visualizing in RViz2

To debug your control loop, visualize what's happening:

**Launch RViz2**:
```bash
ros2 launch robot_launcher display.launch.py
```

**Add visualizations**:
1. **TF** tab: Robot structure and transforms
2. **LaserScan** tab: Point cloud from LIDAR
3. **Image** tab: Raw camera feed
4. **Velocity Vector** (custom marker): Show published cmd_vel

**Debug workflow**:
- Watch LIDAR visualization while robot moves
- Verify obstacle appears in point cloud before robot stops
- Check cmd_vel topic with `ros2 topic echo` to see published commands

---

## Exercise: Implement Obstacle Avoidance

**Goal**: Build a robot that moves forward, stops when obstacles appear.

**Setup**:
1. Gazebo with spawned robot (from Lesson 12.2)
2. LIDAR sensor configured (from Chapter 11)
3. ros_gz_bridge mapping /cmd_vel

**Step 1: Create control node**
```python
# save as nodes/stop_on_obstacle.py
# [Use complete code from above]
```

**Step 2: Add to launch file**
```python
# In spawn_robot.launch.py, add:
control_node = Node(
    package='robot_controller',
    executable='stop_on_obstacle.py'
)
```

**Step 3: Launch everything**
```bash
ros2 launch robot_launcher spawn_robot.launch.py
```

**Step 4: Test**
- Robot moves forward
- Add an obstacle in Gazebo (wall, box)
- Robot detects obstacle and stops

**Success**: Robot demonstrates sense-decide-act loop in real-time.

---

## Try With AI

**Setup**: Open ChatGPT or Claude and ask about refining your control behavior.

**Prompt 1** (Advanced obstacle avoidance):
```
My current obstacle avoidance just stops. I want the robot to:
1. Detect obstacle on the left
2. Turn right and navigate around it
3. Resume original direction

What decision logic handles this? Should I use a state machine?
Show me the code structure.
```

**Prompt 2** (Tuning thresholds):
```
My robot's obstacle detection threshold is 0.5m. It works in simulation
but feels either too aggressive or too timid. How do I systematically
tune this threshold? What's a good starting point?
```

**Prompt 3** (Multi-sensor fusion):
```
I want to fuse LIDAR and camera data: LIDAR detects obstacles,
camera identifies if it's a moving object (person) vs static wall.
React differently for each. How do I combine two sensor streams
in one decision node?
```

**Expected Outcomes**:
- AI suggests control patterns you haven't considered (state machines, hysteresis)
- AI helps tune parameters for your specific environment
- AI explains sensor fusion techniques

**Safety Note**: In simulation, testing aggressive behaviors is safe. Before deploying to real robots, validate all thresholds thoroughly.

---

## Next Steps

You've closed the control loop: sense, decide, act. In the next lesson, you'll crystallize these patterns into reusable skills that compound across projects.

**What emerged from this lesson**: Control loops are the heart of autonomous systems. Sensors without decision logic are data streams. Decision logic without sensors is guessing. Together, they enable robots to react intelligently to their world.

---

## Key Concepts Checkpoint

Before moving on, verify you understand:

- **Twist message**: How to publish velocity commands
- **Sensor subscriptions**: How to read LIDAR scans and camera images
- **Decision logic**: If-then rules and state machines
- **Control loop frequency**: Why 10-50 Hz is typical
- **Visualization**: Using RViz2 to debug behavior

If these are clear, you're ready for Lesson 12.4.
