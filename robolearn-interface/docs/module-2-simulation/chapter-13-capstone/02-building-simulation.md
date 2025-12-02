---
id: lesson-13-2-building-simulation
title: "Lesson 13.2: Building the Simulation"
sidebar_position: 2
sidebar_label: "13.2 Building the Simulation"
description: "Implement your capstone using skills accumulated from Chapters 9-12"
duration_minutes: 90
proficiency_level: "B1"
layer: "L4"
hardware_tier: 1
learning_objectives:
  - "Compose skills from prior chapters into a complete simulation"
  - "Create robot URDF matching your specification"
  - "Create world SDF matching your specification"
  - "Configure sensors according to your specification"
  - "Bridge Gazebo to ROS 2 using accumulated skills"
  - "Test incrementally: robot, world, sensors, bridge, behavior"
  - "Debug and fix integration issues systematically"
skills:
  - "urdf-robot-model"
  - "gazebo-world-builder"
  - "sensor-simulation"
  - "ros2-gazebo-bridge"
cognitive_load:
  new_concepts: 8
tier_1_path: "TheConstruct cloud environment"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 13.2: Building the Simulation

Now implement the simulation you specified in Lesson 13.1. You won't build from scratch—you'll use skills accumulated from Chapters 9-12, composing them into a working system.

The key to successful capstone implementation is **incremental testing**: verify each component works before moving to the next. Don't build everything, then discover it doesn't work. Build incrementally, validate each step, then continue.

---

## Implementation Workflow

### Phase 1: Prepare Your Specification

**Start here**: Open your capstone specification from Lesson 13.1.

**Checklist**:
- ✅ Intent statement is clear
- ✅ Robot requirements specify links, joints, sensors
- ✅ World requirements specify layout, obstacles, initial conditions
- ✅ Sensor requirements specify topics and message types
- ✅ Behavior requirements describe perception, decision, execution
- ✅ Success criteria are measurable (5+ criteria)

If anything is unclear or missing, go back to Lesson 13.1 and refine before proceeding.

---

### Phase 2: Create Robot URDF

Use the `urdf-robot-model` skill from Chapter 9. Your URDF should include:
- All links from your specification (chassis, wheels, sensors, etc.)
- All joints (wheel motors, arm joints, gripper articulation)
- Visual geometry (boxes, cylinders for simple shape)
- Collision geometry (match visual for accurate physics)
- Physical properties (mass, inertia—estimate based on size)
- Sensor plugins (camera, LIDAR, IMU) matching your specification

**Reference**: [Chapter 9.4 URDF with AI](../chapter-9-robot-description/04-urdf-with-ai.md) for how to work with AI on URDF generation.

**Your URDF should**:
- Load without errors in Gazebo (`gz sim robot.urdf`)
- Have correct joint limits matching robot capabilities
- Include all sensors from your specification
- Have proper mass distribution (shouldn't flip over)

**Test this phase**:
```bash
# Launch Gazebo and load your URDF
gz sim -r my_robot.urdf

# Verify no errors appear
# Verify robot appears in the 3D view
# Verify you can move the camera to see the robot from all angles
```

**Output**:
- `my_robot.urdf` file in your workspace

---

### Phase 3: Create World SDF

Use the `gazebo-world-builder` skill from Chapter 10. Your world should include:
- Ground plane (physics-enabled)
- Obstacles specified in your specification (shelves, walls, objects)
- Lighting configuration (uniform indoor lighting suggested)
- Physics engine settings (gravity, friction, contact)
- Initial spawn location for your robot
- Goal locations (marked with visual indicators)

**Reference**: [Chapter 10.4 World Building with AI](../chapter-10-simulation-worlds/04-world-building-with-ai.md) for how to work with AI on SDF generation.

**Your world should**:
- Load without physics errors (`gz sim world.sdf`)
- Have obstacles in correct positions (matching your specification)
- Have stable physics (objects don't shake, fall through ground, etc.)
- Have your robot spawn at the correct starting position

**Test this phase**:
```bash
# Launch Gazebo with your world (robot will spawn in it)
gz sim world.sdf

# Verify robot appears at starting position
# Verify obstacles are in expected places
# Drop an object to verify gravity and friction work
```

**Output**:
- `world.sdf` file with all obstacles, ground, physics configuration
- Updated URDF with proper `<inertial>` properties

---

### Phase 4: Configure Sensors

Use the `sensor-simulation` skill from Chapter 11. Your sensors should include:
- Camera (if required by specification)
- LIDAR (if required for obstacle detection)
- IMU (if required for odometry/motion tracking)
- Contact sensors/bumpers (if required for collision detection)

**Reference**: [Chapter 11.1-11.3](../chapter-11-sensors-simulation/) for sensor configuration patterns.

**Each sensor should**:
- Be attached to the correct robot link
- Publish to the correct topic (matching your specification)
- Publish the correct message type (sensor_msgs/Image, LaserScan, etc.)
- Have realistic configuration (noise, update rate, field of view)

**Test this phase**:
```bash
# Launch Gazebo with your world and robot
gz sim world.sdf

# In a separate terminal, check which topics exist
gz topic --list

# For camera, verify image publishes
gz topic -e /camera/image_raw | head -5

# For LIDAR, verify scan publishes
gz topic -e /scan | head -5

# For IMU, verify data publishes
gz topic -e /imu/data | head -5
```

**Output**:
- Updated URDF with all sensor plugins configured
- Verification that all sensor topics publish data in Gazebo

---

### Phase 5: Configure ros_gz Bridge

Use the `ros2-gazebo-bridge` skill from Chapter 12. Your bridge should:
- Map all sensor topics (Gazebo → ROS 2)
- Map all command topics (ROS 2 → Gazebo)
- Use correct message type mappings (Chapter 12.1 reference)
- Support the behavior your robot will execute

**Reference**: [Chapter 12.1 ros_gz Bridge](../chapter-12-ros2-gazebo-integration/01-ros-gz-bridge.md) for bridge syntax and configuration.

**Create bridge configuration** (`bridge.yaml`):
```yaml
# Sensor bridges (Gazebo → ROS 2)
- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/scan"
  gz_topic_name: "/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/imu/data"
  gz_topic_name: "/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

# Command bridges (ROS 2 → Gazebo)
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
```

**Test this phase**:
```bash
# Terminal 1: Launch Gazebo with world
gz sim world.sdf

# Terminal 2: Start the bridge
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge.yaml

# Terminal 3: Verify topics appear in ROS 2
ros2 topic list

# You should see:
# /camera/image_raw
# /scan
# /imu/data
# /cmd_vel

# Verify sensor data flows
ros2 topic echo /scan
```

**Output**:
- `bridge.yaml` configuration file
- Verification that all bridged topics appear in `ros2 topic list`
- Verification that sensor data publishes to ROS 2 topics

---

### Phase 6: Implement Behavior

Now your robot is in Gazebo, sensors are running, and ROS 2 can command the robot. Implement the behavior specified in Lesson 13.1.

**Behavior can be**:
- **Simple rule-based**: If obstacle near, stop; else move forward
- **Path-following**: Navigate from waypoint to waypoint
- **Sensor-based**: React to camera/LIDAR data with logic
- **Search-based**: Explore systematically to find targets

**Reference**: [Chapter 12.3 Closed-Loop Control](../chapter-12-ros2-gazebo-integration/03-closed-loop-control.md) for behavior implementation patterns.

**Create ROS 2 node** (`behavior_node.py`):
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class DeliveryRobot(Node):
    def __init__(self):
        super().__init__('delivery_robot')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan',
                                                 self.scan_callback, 10)

        # Simple navigation state
        self.goal_x = 30.0
        self.goal_y = 5.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.min_obstacle_distance = 3.0

    def scan_callback(self, msg):
        """React to LIDAR scan: detect obstacles, command motion"""

        # Find minimum distance reading
        if msg.ranges:
            min_range = min([r for r in msg.ranges if r < msg.range_max])
        else:
            min_range = float('inf')

        # Decision logic
        cmd = Twist()
        if min_range > self.min_obstacle_distance:
            # No obstacle detected, move toward goal
            cmd.linear.x = 0.5  # 0.5 m/s forward
            cmd.angular.z = 0.0  # No rotation
        else:
            # Obstacle detected, stop
            cmd.linear.x = 0.0  # Stop
            cmd.angular.z = 0.0

        # Execute command
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f"Min range: {min_range:.2f}m, cmd_vel: {cmd.linear.x:.2f} m/s")

def main(args=None):
    rclpy.init(args=args)
    robot = DeliveryRobot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Launch everything together**:
```bash
# Terminal 1: Gazebo
gz sim world.sdf

# Terminal 2: ros_gz_bridge
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge.yaml

# Terminal 3: Your behavior node
python3 behavior_node.py

# Watch the robot move in Gazebo!
```

**Output**:
- `behavior_node.py` implementing your specification's required behavior
- Robot responding to sensor data and executing motion commands
- Console output showing decision logic in action

---

## Testing Incrementally

Don't wait until the end to verify everything works. Test as you go:

**After Phase 2 (URDF)**:
- ✅ Robot loads in Gazebo without errors
- ✅ Visual geometry looks reasonable
- ✅ Joints have correct limits

**After Phase 3 (World)**:
- ✅ World loads in Gazebo without physics errors
- ✅ Obstacles are in correct positions
- ✅ Robot spawns at starting location

**After Phase 4 (Sensors)**:
- ✅ Gazebo topics include all sensors
- ✅ Sensor data publishes to Gazebo topics
- ✅ Data looks realistic (LIDAR range, camera resolution, etc.)

**After Phase 5 (Bridge)**:
- ✅ ros2 topic list shows all bridged topics
- ✅ Sensor data flows to ROS 2 topics
- ✅ Can publish to `/cmd_vel` and robot responds

**After Phase 6 (Behavior)**:
- ✅ Behavior node starts without errors
- ✅ Robot responds to sensor data
- ✅ Robot executes motion commands
- ✅ Behavior implements specification requirements

---

## Debugging Incremental Failures

| Issue | How to Debug | Solution |
|-------|------|----------|
| URDF doesn't load | Check console: `[ERROR] XML parse error` | Fix XML syntax in URDF file |
| Geometry overlaps (collision in place) | Robot shakes/flies in Gazebo | Adjust geometry positions, reduce mass |
| Physics unstable (objects fall through ground) | Ground plane doesn't hold physics objects | Enable collision shapes, verify friction config |
| Sensors don't output data | `gz topic --list` doesn't show sensor topics | Add sensor plugins to URDF with correct names |
| ros_gz_bridge fails | Bridge won't start, syntax error | Check YAML format, verify message types exist |
| ros2 topics don't appear | ros_gz_bridge runs but topics missing | Check direction (GZ_TO_ROS vs ROS_TO_GZ) |
| cmd_vel published but robot doesn't move | Commands publish but no motion | Check joint names in URDF match bridge topics |
| Behavior node starts but robot doesn't move | Console shows node running but no motion | Verify cmd_vel topic name, check subscriber working |

---

## Try With AI

**Setup**: Open ChatGPT or Claude and ask for help implementing your capstone.

**Prompt Set:**

**Prompt 1** (Generating URDF from specification):
```
I'm building a robot for my capstone simulation. Here's my specification:

[Paste relevant parts of your specification: robot requirements, sensors]

Generate a URDF file for this robot with:
- All links and joints specified
- Correct collision/visual geometry
- All sensors configured with proper plugins
- Realistic mass and inertia estimates

Make it load successfully in Gazebo Harmonic.
```

**Prompt 2** (Generating world SDF):
```
I'm building a simulation world for my capstone. Here are the world requirements:

[Paste: world layout, obstacles, physics requirements]

Generate an SDF file that includes:
- Ground plane with physics
- All obstacles at specified positions
- Correct physics parameters (gravity, friction, contact)
- Starting position and goal locations marked

Make it work with Gazebo Harmonic.
```

**Prompt 3** (Debugging bridge configuration):
```
My robot is in Gazebo and sensors are publishing. I want to bridge to ROS 2.
Here are my sensors:

[List sensor topics from Gazebo, message types, and directions needed]

Generate the bridge.yaml configuration that:
- Maps all sensor topics (Gazebo → ROS 2)
- Maps command topics (ROS 2 → Gazebo)
- Uses correct message type mappings

Include explanation of why each direction is chosen.
```

**Prompt 4** (Creating behavior node):
```
My robot needs to [describe behavior from your specification].

Here's my sensor and command setup:
- Sensors: [list topics]
- Commands: [list topics]

Create a ROS 2 Python node that:
- Subscribes to sensor data
- Implements the behavior logic
- Publishes motion commands

Show the complete code with comments.
```

**Expected Outcomes**:
- AI generates working URDF matching your specification
- AI generates SDF world matching your specification
- AI helps debug bridge configuration
- AI provides behavior implementation code you can adapt

**Safety Note**: When AI generates code, review it before running. Especially:
- Verify motor speeds (don't command dangerous velocities)
- Check joint limits (motors should stay within mechanical limits)
- Review sensor configurations (filters, noise levels appropriate)

---
