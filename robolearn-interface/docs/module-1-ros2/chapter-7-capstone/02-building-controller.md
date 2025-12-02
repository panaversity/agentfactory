---
id: lesson-7-2-building-controller
title: "Lesson 7.2: Building the Controller"
sidebar_position: 2
sidebar_label: "7.2 Building the Controller"
description: "Implement multi-node system from specification using skills from Chapters 4-6."
duration_minutes: 90
proficiency_level: "B1"
layer: "L4"
hardware_tier: 1
learning_objectives:
  - "Compose multi-node systems from existing patterns"
  - "Create custom messages and service definitions"
  - "Implement stateful service servers"
  - "Coordinate multiple publishers in a launch system"
  - "Debug integration issues systematically"
---

# Lesson 7.2: Building the Controller

## Now Implement

You have your specification. It's clear, unambiguous, testable. Now you implement it.

Here's the good news: **You've built every piece before.** This isn't new concepts. This is composition—taking what you learned in Chapters 4-6 and assembling them into a complete system.

- **Chapter 4**: Publisher/subscriber nodes ✅
- **Chapter 5**: Service servers and custom messages ✅
- **Chapter 6**: Parameters and launch files ✅

You're orchestrating existing skills into a new system. That's professional robotics development.

---

## Implementation Phases

### Phase 1: Create Package & Interfaces

**Your job**: Create the package structure and define custom messages.

From **Chapter 5**, remember:
- Custom messages go in `robot_controller/msg/TurtleStatus.msg`
- Services go in `robot_controller/srv/NavigateTo.srv`

```bash
# Create workspace and package
mkdir -p turtle_ws/src
cd turtle_ws/src
ros2 pkg create robot_controller --build-type ament_cmake

# Create message and service directories
mkdir -p robot_controller/msg robot_controller/srv
```

**TurtleStatus.msg** (custom message for status publishing):
```
float32 x
float32 y
float32 theta
float32 vel_x
float32 vel_y
float32 battery
```

**NavigateTo.srv** (service for navigation requests):
```
float32 x
float32 y
float32 theta
---
bool success
float32 time_taken
```

Update `CMakeLists.txt` to compile messages:
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TurtleStatus.msg"
  "srv/NavigateTo.srv"
)

ament_package()
```

### Phase 2: Implement Navigator Node

**Your job**: Create a Python node that accepts goals via service and commands the turtle.

From **Chapter 4 & 5**, remember:
- Nodes use `Node` base class from `rclpy`
- Services use `Service` objects
- Publishers send messages

**robot_controller/navigator_node.py**:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from your_package.srv import NavigateTo  # Use correct package name
import math
import time

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator')

        # Publisher to control turtle
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )

        # Service server for navigation goals
        self.navigate_srv = self.create_service(
            NavigateTo,
            '/navigate_to',
            self.navigate_callback
        )

        # Current state tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.get_logger().info('Navigator node started')

    def navigate_callback(self, request, response):
        """Handle navigation goal requests"""
        self.get_logger().info(
            f'Received goal: x={request.x}, y={request.y}, theta={request.theta}'
        )

        start_time = time.time()

        # Compute distance to goal (simplified)
        dx = request.x - self.current_x
        dy = request.y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        # Move toward goal (simplified linear movement)
        # In real implementation, this would be more sophisticated
        msg = Twist()
        msg.linear.x = 0.2  # Move forward at 0.2 m/s

        # Move for distance/speed seconds (approximately)
        move_time = distance / 0.2 if distance > 0 else 0.0
        end_time = time.time() + move_time

        while time.time() < end_time:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.1)

        # Stop
        msg.linear.x = 0.0
        self.cmd_vel_pub.publish(msg)

        # Update state (in real system, would come from odometry)
        self.current_x = request.x
        self.current_y = request.y
        self.current_theta = request.theta

        # Response
        elapsed = time.time() - start_time
        response.success = distance < 0.5  # Within 0.5m is "success"
        response.time_taken = elapsed

        self.get_logger().info(f'Goal completed in {elapsed:.2f}s')
        return response

def main(args=None):
    rclpy.init(args=args)
    navigator = NavigatorNode()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Points**:
- Service callback handles goal requests
- Publisher sends velocity commands (from Chapter 4)
- Response returns success and time_taken (from Chapter 5)
- Real implementation would read odometry; this simplified version tracks state

### Phase 3: Implement Status Monitor Node

**Your job**: Create a node that publishes robot status continuously.

From **Chapter 4**, remember publisher patterns.

**robot_controller/status_monitor_node.py**:
```python
import rclpy
from rclpy.node import Node
from your_package.msg import TurtleStatus  # Custom message
import random

class StatusMonitorNode(Node):
    def __init__(self):
        super().__init__('status_monitor')

        # Publisher for status
        self.status_pub = self.create_publisher(
            TurtleStatus,
            '/robot/status',
            10
        )

        # Timer to publish every 100ms (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_status)

        # Simulated state (in real system, comes from odometry)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.battery = 100.0

        self.get_logger().info('Status Monitor started')

    def publish_status(self):
        """Publish status message every 100ms"""
        msg = TurtleStatus()
        msg.x = self.x
        msg.y = self.y
        msg.theta = self.theta
        msg.vel_x = self.vel_x
        msg.vel_y = self.vel_y
        msg.battery = self.battery

        self.status_pub.publish(msg)
        self.get_logger().debug(f'Status: ({self.x:.2f}, {self.y:.2f})')

        # Simulate battery drain
        self.battery = max(0.0, self.battery - 0.1)

def main(args=None):
    rclpy.init(args=args)
    monitor = StatusMonitorNode()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Points**:
- Custom message `TurtleStatus` from Chapter 5
- Timer ensures 10 Hz publication (every 100ms from Chapter 6 patterns)
- State management (tracks position, velocity, battery)

### Phase 4: Implement Obstacle Detector Node

**Your job**: Create a node that simulates obstacle detection.

**robot_controller/obstacle_detector_node.py**:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import random

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # Publisher for obstacle data (mock sensor)
        self.obstacle_pub = self.create_publisher(
            LaserScan,
            '/obstacles',
            10
        )

        # Timer to publish every 200ms (5 Hz)
        self.timer = self.create_timer(0.2, self.publish_obstacles)

        self.scan_counter = 0
        self.get_logger().info('Obstacle Detector started')

    def publish_obstacles(self):
        """Publish mock obstacle scan"""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.angle_min = -1.5708  # -90 degrees
        msg.angle_max = 1.5708   # +90 degrees
        msg.angle_increment = 0.0174  # ~1 degree
        msg.time_increment = 0.0
        msg.range_min = 0.0
        msg.range_max = 10.0

        # Simulate some "detections"
        ranges = []
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        for i in range(num_readings):
            # Random distances (simulate detecting nothing mostly,
            # occasional obstacles)
            if random.random() > 0.95:  # 5% chance of obstacle
                ranges.append(0.5)  # Obstacle at 0.5m
            else:
                ranges.append(10.0)  # No obstacle (max range)

        msg.ranges = ranges
        self.obstacle_pub.publish(msg)

        self.scan_counter += 1
        if self.scan_counter % 5 == 0:  # Log every 5 scans
            self.get_logger().debug(f'Published scan #{self.scan_counter}')

def main(args=None):
    rclpy.init(args=args)
    detector = ObstacleDetectorNode()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Points**:
- Uses standard ROS 2 message `LaserScan` (from Chapter 5 knowledge)
- Publishes at 5 Hz (200ms intervals from Chapter 6)
- Simulates occasional obstacles (5% chance) for system testing

### Phase 5: Create Launch File

**Your job**: Create a launch file that starts all nodes with parameters.

From **Chapter 6**, remember launch file structure.

**robot_controller/launch/start_system.launch.py**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Navigator node
        Node(
            package='robot_controller',
            executable='navigator',
            name='navigator',
            output='screen',
            parameters=[
                {'goal_timeout': 5.0},
            ]
        ),

        # Status Monitor node
        Node(
            package='robot_controller',
            executable='status_monitor',
            name='status_monitor',
            output='screen',
            parameters=[
                {'publish_rate': 10},
            ]
        ),

        # Obstacle Detector node
        Node(
            package='robot_controller',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
            parameters=[
                {'scan_rate': 5},
            ]
        ),
    ])
```

**setup.py** entry points (so `ros2 launch` can find executables):
```python
entry_points={
    'console_scripts': [
        'navigator=robot_controller.navigator_node:main',
        'status_monitor=robot_controller.status_monitor_node:main',
        'obstacle_detector=robot_controller.obstacle_detector_node:main',
    ],
},
```

---

## Building and Testing Nodes

### Build the Package

```bash
cd ~/turtle_ws
colcon build --packages-select robot_controller
source install/setup.bash
```

### Test Individual Nodes

**Terminal 1: Start turtlesim**
```bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2: Start launch file**
```bash
cd ~/turtle_ws
ros2 launch robot_controller start_system.launch.py
```

**Terminal 3: Call navigation service**
```bash
ros2 service call /navigate_to robot_controller/srv/NavigateTo "{x: 5.0, y: 5.0, theta: 0.0}"
```

**Terminal 4: Monitor status topic**
```bash
ros2 topic echo /robot/status
```

**Terminal 5: Monitor obstacles**
```bash
ros2 topic echo /obstacles
```

---

## Common Integration Issues & Debugging

### Issue 1: "Cannot find custom message"
**Symptom**: Import error `robot_controller.srv.NavigateTo not found`
**Fix**:
1. Rebuild: `colcon build --packages-select robot_controller`
2. Source setup: `source install/setup.bash`
3. Check CMakeLists.txt includes rosidl_generate_interfaces

### Issue 2: "Service call times out"
**Symptom**: `ros2 service call` hangs
**Fix**:
1. Check node is running: `ros2 node list`
2. Check service exists: `ros2 service list`
3. Add logging to service callback to see if it's being called
4. Increase timeout: `ros2 service call /navigate_to ... --timeout 10.0`

### Issue 3: "Topic not published"
**Symptom**: `ros2 topic echo /robot/status` shows no output
**Fix**:
1. Check publisher node is running: `ros2 node list`
2. Check topic name is correct: `ros2 topic list`
3. Check frequency isn't too slow (maybe publisher is running but data is old)
4. Add logging to publish callback

### Issue 4: "Turtle doesn't move"
**Symptom**: Service returns success but turtle stays still
**Fix**:
1. Verify `turtlesim_node` is running and listening on `/turtle1/cmd_vel`
2. Test manually: `ros2 topic pub /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"`
3. Check velocity magnitude (0.2 m/s is standard for testing)
4. Add delays between velocity commands

---

## Composition vs Creation

Notice: **You didn't create new concepts.** You composed existing ones:

| From Chapter | Concept | Used In |
|--------------|---------|---------|
| 4 | Publisher/Subscriber | Navigator (pub), Status Monitor (pub), Obstacle Detector (pub) |
| 5 | Service Pattern | Navigator service server (/navigate_to) |
| 5 | Custom Messages | TurtleStatus message |
| 6 | Launch Files | start_system.launch.py with all 3 nodes |
| 6 | Parameters | goal_timeout, publish_rate, scan_rate |

This is **Layer 3 Intelligence Design**: You're taking reusable components and assembling them.

---

## Try With AI

**Setup**: Use a chat AI to help debug issues during implementation.

**Debugging Prompt**:
```
I'm building a multi-node ROS 2 system. Here's my specification:
[paste your spec from Lesson 7.1]

When I run the nodes, I get this error:
[paste error message]

What's wrong? How do I fix it?
```

**Code Review Prompt**:
```
Here's my navigator_node.py:
[paste code]

Does this correctly implement:
1. The /navigate_to service from my spec?
2. Publishing velocity commands to the turtle?
3. Returning success and time_taken in the response?

Any bugs or improvements?
```

**Expected Outcome**:
- All 3 nodes compile without errors
- Launch file starts all nodes simultaneously
- Services and topics are visible with `ros2 service list` and `ros2 topic list`
- You can call the navigation service and see the turtle respond

**Safety Note**: Your AI might suggest features beyond the spec (real path planning, obstacle avoidance, kinematic constraints). Remember your non-goals from Lesson 7.1. Keep it simple—focus on the core pub/sub/service pattern from Chapters 4-6.

---

**Next Step**: [Lesson 7.3: Testing & Validation →](./03-testing-validation.md)
