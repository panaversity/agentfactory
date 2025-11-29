---
id: lesson-12-1-ros-gz-bridge
title: "Lesson 12.1: The ros_gz Bridge"
sidebar_position: 1
sidebar_label: "12.1 ros_gz Bridge"
description: "Connecting Gazebo topics to ROS 2 using ros_gz_bridge"
duration_minutes: 75
proficiency_level: "B1"
layer: "L2"
hardware_tier: 1
learning_objectives:
  - "Explain the ros_gz_bridge architecture and purpose"
  - "Configure topic bridging using command-line syntax"
  - "Create YAML configuration files for multiple topic bridges"
  - "Verify bridge connectivity and debug common issues"
skills:
  - "ros2-gazebo-bridge"
cognitive_load:
  new_concepts: 8
tier_1_path: "TheConstruct cloud environment"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 12.1: The ros_gz Bridge

## The Connection Problem

Gazebo runs a physics simulation. ROS 2 is a middleware for robotics. They speak different languages:

**Gazebo** thinks in terms of entities, models, and physics properties. It publishes sensor data in Gazebo-native message types (`gz.msgs.Image`, `gz.msgs.LaserScan`).

**ROS 2** thinks in terms of nodes and topics. It publishes data in ROS message types (`sensor_msgs/msg/Image`, `sensor_msgs/msg/LaserScan`).

For a ROS 2 application to control a robot in Gazebo, something must translate between them. That something is `ros_gz_bridge`—a tool that bidirectionally maps ROS 2 topics to Gazebo topics, converting messages as needed.

Without the bridge: Your ROS 2 node publishes `geometry_msgs/msg/Twist` to `/cmd_vel`, but Gazebo has no idea what this message is. Your robot doesn't move.

With the bridge: The bridge translates `/cmd_vel (Twist)` to Gazebo's `/cmd_vel (gz.msgs.Twist)`, and your robot responds.

---

## Understanding Bridge Architecture

Think of ros_gz_bridge as a translator in a telephone conversation:

```
ROS 2 Node (English)
    ↓
    "Set velocity to 1.0 m/s forward"
    ↓
ros_gz_bridge (Translator)
    ↓
    [Interprets intent, converts to Gazebo format]
    ↓
Gazebo (Gazebo language)
    "Apply velocity vector [1.0, 0, 0]"
    ↓
Physics Simulation
    Robot moves
```

The bridge handles three operations:

**1. ROS 2 → Gazebo (ROS_TO_GZ)**
- ROS node publishes message on topic `/cmd_vel`
- Bridge receives, converts to Gazebo format
- Bridge writes to Gazebo topic `/cmd_vel`
- Gazebo entity executes command

**2. Gazebo → ROS 2 (GZ_TO_ROS)**
- Gazebo publishes sensor data on internal topic (e.g., camera image)
- Bridge receives, converts to ROS message format
- Bridge publishes on ROS 2 topic (e.g., `/camera/image_raw`)
- ROS 2 nodes subscribe and process

**3. Bidirectional (BIDIRECTIONAL)**
- Same topic, messages flow both directions
- Less common, mainly for synchronized state

---

## Bridge Syntax: The Three Parts

A bridge configuration has three key components:

```
/topic_name @ ROS_MESSAGE_TYPE @ GAZEBO_MESSAGE_TYPE
```

**Example**:
```
/cmd_vel @ geometry_msgs/msg/Twist @ gz.msgs.Twist
```

Breaking this down:

### Part 1: Topic Name
```
/cmd_vel
```
The topic both sides use (or Gazebo topic if different). In practice, usually the same.

### Part 2: ROS Message Type
```
geometry_msgs/msg/Twist
```
The message type ROS 2 uses. Full qualified name: `package/msg/MessageName`.

**Common ROS types**:
- `geometry_msgs/msg/Twist` — velocity commands (linear + angular)
- `sensor_msgs/msg/Image` — camera images
- `sensor_msgs/msg/LaserScan` — LIDAR data (2D)
- `sensor_msgs/msg/PointCloud2` — point clouds (3D)
- `std_msgs/msg/Float64` — scalar numeric values

### Part 3: Gazebo Message Type
```
gz.msgs.Twist
```
The message type Gazebo uses. Namespace is always `gz.msgs`.

**Common Gazebo types**:
- `gz.msgs.Twist` — velocity commands
- `gz.msgs.Image` — camera images
- `gz.msgs.LaserScan` — LIDAR data
- `gz.msgs.PointCloudPacked` — point clouds (packed binary)
- `gz.msgs.Double` — scalar values
- `gz.msgs.Boolean` — true/false values

---

## Command-Line Bridge: Quick Testing

The simplest way to test a bridge is via command line:

```bash
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

This launches a single bridge for the `/cmd_vel` topic, converting between ROS Twist and Gazebo Twist.

**To add more topics**, run additional bridge commands in separate terminals:

```bash
# Terminal 1: Velocity commands
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist

# Terminal 2: Camera image
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image

# Terminal 3: LIDAR scan
ros2 run ros_gz_bridge parameter_bridge /lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

**Testing the bridge:**

In one terminal, echo a topic to verify data flows:

```bash
ros2 topic echo /cmd_vel
```

In another terminal, publish a test message:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{"linear": {"x": 0.5}, "angular": {"z": 0.0}}'
```

If the robot moves, the bridge is working.

---

## YAML Configuration: Production Setup

Command-line works for testing. For real applications, use YAML configuration to define all bridges at once:

**File**: `bridge.yaml`
```yaml
# Single topic bridge (ROS to Gazebo)
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

# Single topic bridge (Gazebo to ROS)
- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

# Bidirectional bridge
- ros_topic_name: "/odom"
  gz_topic_name: "/odom"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: BIDIRECTIONAL
```

**Launch the bridge with YAML**:
```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge.yaml
```

**Advantages of YAML**:
- All bridges in one place
- Version control friendly (commit to git)
- Reusable across multiple robots
- Clear documentation of message mappings

---

## Common Message Type Mappings

Here are the most useful ROS 2 ↔ Gazebo message mappings:

### Velocity and Motion
```
ROS 2                           Gazebo
geometry_msgs/msg/Twist      ↔ gz.msgs.Twist
geometry_msgs/msg/Pose       ↔ gz.msgs.Pose
geometry_msgs/msg/Transform  ↔ gz.msgs.Transform
```

**Use for**: Robot velocity commands, position commands, coordinate transforms

### Sensors
```
ROS 2                           Gazebo
sensor_msgs/msg/Image        ↔ gz.msgs.Image
sensor_msgs/msg/LaserScan    ↔ gz.msgs.LaserScan
sensor_msgs/msg/PointCloud2  ↔ gz.msgs.PointCloudPacked
sensor_msgs/msg/Imu          ↔ gz.msgs.IMU
```

**Use for**: Camera output, LIDAR scans, 3D point clouds, inertial data

### Simple Values
```
ROS 2                         Gazebo
std_msgs/msg/Float64      ↔ gz.msgs.Double
std_msgs/msg/Int32        ↔ gz.msgs.Int32
std_msgs/msg/Bool         ↔ gz.msgs.Boolean
std_msgs/msg/String       ↔ gz.msgs.StringMsg
```

**Use for**: Scalar sensor readings, configuration values, simple signals

---

## Verifying Bridge Connectivity

Once your bridge is running, verify it's actually connected:

**Step 1: Check ROS 2 topics**
```bash
ros2 topic list
```
You should see your bridge topics listed.

**Step 2: Check Gazebo topics**
```bash
gz topic --list
```
You should see Gazebo's internal topics.

**Step 3: Echo topic data**
```bash
# Check if data is flowing
ros2 topic echo /cmd_vel
```
If you see messages, the bridge is publishing.

**Step 4: Test with a published message**
```bash
# Send a test command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{"linear": {"x": 1.0, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}'
```
Watch the simulated robot. If it moves, the bridge works end-to-end.

---

## Debugging Bridge Problems

**Problem 1: Bridge won't start**
```
[ERROR] ros_gz_bridge: Invalid bridge syntax: /cmd_vel@...
```
Check:
- Syntax is `/topic@ROS_TYPE@GZ_TYPE`
- No extra spaces
- ROS type includes package (`geometry_msgs/msg/Twist`, not just `Twist`)
- Gazebo type starts with `gz.msgs`

**Problem 2: Data not flowing (bridge runs but no messages)**
```bash
# Check if bridge is listening
ros2 topic info /cmd_vel
# Should show "1 publisher, 0 subscribers" or similar

# Manually publish a test message
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{"linear": {"x": 1.0}}'
```
If manual publishing works but nothing happens automatically:
- Check that your ROS 2 node is actually publishing (use `ros2 topic echo`)
- Verify message format matches (wrong fields = message rejected)

**Problem 3: Type mismatch errors**
```
[WARN] Cannot find ROS type: geometry_msgs/msg/Twist
```
The type doesn't exist or isn't installed. Install the message package:
```bash
sudo apt install ros-humble-geometry-msgs
```

---

## Exercise: Configure Your First Bridge

**Goal**: Bridge velocity commands from ROS 2 to a simulated robot in Gazebo.

**Setup**:
1. Launch Gazebo with a robot model (from Chapter 10/11)
2. In a terminal, start the bridge:
   ```bash
   ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
   ```
3. In another terminal, publish a velocity command:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{"linear": {"x": 0.5, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}'
   ```

**Success**: The robot moves forward in Gazebo.

**Verify**:
- `ros2 topic list` includes `/cmd_vel`
- `ros2 topic echo /cmd_vel` shows your velocity messages
- Robot in Gazebo responds to commands

---

## Try With AI

**Setup**: Open ChatGPT or Claude and ask about ros_gz_bridge configuration for your specific robot.

**Prompt Set:**

**Prompt 1** (Understanding message types):
```
I'm working with ros_gz_bridge and my robot has these sensors:
- A forward-facing camera
- A 2D LIDAR scanner
- An IMU sensor

Show me the YAML configuration for bridging these three sensors
from Gazebo to ROS 2. Include topic names and message types.
```

**Prompt 2** (Troubleshooting):
```
My ros_gz_bridge is running but ros2 topic echo /cmd_vel shows no messages
even though my node is publishing. What could be wrong? Walk me through
the debugging checklist.
```

**Prompt 3** (Advanced configuration):
```
I want to bridge the robot's odometry (position estimate) bidirectionally
between Gazebo and ROS 2. What message type should I use, and should
the direction be BIDIRECTIONAL or ROS_TO_GZ? Explain the tradeoff.
```

**Expected Outcomes**:
- AI provides working YAML configuration you can adapt
- AI suggests debugging steps you might not have considered
- AI explains why certain message types match better than others

**Safety Note**: Test all bridge configurations in simulation first. A misconfigured bridge might command unexpected motion once deployed.

---

## Next Steps

You've configured the bridge. In the next lesson, you'll use this bridge to spawn robots dynamically into Gazebo worlds, enabling modular testing and multi-robot scenarios.

**What emerged from this lesson**: Understanding how ROS 2 and Gazebo communicate enables debugging when things go wrong. The bridge is the foundation for all ROS 2-Gazebo integration work.

---

## Key Concepts Checkpoint

Before moving on, verify you understand:

- **Bridge purpose**: Translate between ROS 2 and Gazebo message formats
- **Bridge syntax**: `/topic@ROS_TYPE@GZ_TYPE` with direction
- **Configuration methods**: Command-line for testing, YAML for production
- **Message mappings**: Knowing which ROS type corresponds to which Gazebo type
- **Verification**: Using `ros2 topic list/echo` to confirm bridge functionality

If these are clear, you're ready for Lesson 12.2.
