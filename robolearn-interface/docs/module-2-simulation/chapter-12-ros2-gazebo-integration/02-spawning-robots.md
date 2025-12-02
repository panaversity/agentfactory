---
id: lesson-12-2-spawning-robots
title: "Lesson 12.2: Spawning Robots from ROS 2"
sidebar_position: 2
sidebar_label: "12.2 Spawning Robots"
description: "Spawning robot models into Gazebo from ROS 2 launch files"
duration_minutes: 60
proficiency_level: "B1"
layer: "L2"
hardware_tier: 1
learning_objectives:
  - "Use ros_gz spawn service to add robots to simulation"
  - "Convert URDF to SDF for Gazebo spawning"
  - "Integrate robot spawning into ROS 2 launch files"
  - "Manage robot_description parameter and joint state publishing"
skills:
  - "ros2-gazebo-bridge"
  - "launch-files"
cognitive_load:
  new_concepts: 7
tier_1_path: "TheConstruct cloud environment"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 12.2: Spawning Robots from ROS 2

## The Static vs Dynamic Problem

In Chapter 10, you designed Gazebo worlds with robots pre-placed using the GUI. This works for single-robot scenarios, but creates problems as complexity grows:

- **Testing different robots**: Launch different world file for each robot variant
- **Multi-robot scenarios**: Manually manage multiple copies in world file
- **Automated testing**: Can't easily spin up 10 robots for batch testing
- **Distribution**: World files include robot details, making them large and coupled

**Better approach**: Define the robot once (in URDF), then spawn it dynamically via a ROS 2 service. This separates concerns: worlds describe environments, robots are spawned on demand.

The `ros_gz spawn_entity` service is your tool for dynamic spawning.

---

## How Dynamic Spawning Works

When you spawn a robot via ROS 2:

```
ROS 2 Launch File
    ↓
Load URDF from parameter
    ↓
Convert URDF to SDF (if needed)
    ↓
Call spawn_entity ROS 2 service
    ↓
Gazebo creates robot in world
    ↓
Bridge connects ROS 2 topics
    ↓
Robot accepts commands and publishes sensor data
```

This happens all from a single ROS 2 launch command. No manual GUI actions.

---

## Step 1: Load Robot Description

Your robot must be available as a ROS 2 parameter called `robot_description`. This parameter contains the robot's URDF (or SDF) as a string.

### Option A: Load from File in Launch

**Python Launch File** (`launch/spawn_robot.launch.py`):
```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the robot_description package and URDF file
    robot_description_path = PathJoinSubstitution([
        FindPackageShare("robot_description"),
        "urdf",
        "robot.urdf"
    ])

    # Read URDF file and expose as parameter
    with open(robot_description_path, 'r') as infp:
        robot_desc = infp.read()

    # Create launch description
    ld = LaunchDescription()

    # Set robot_description parameter (published to all nodes)
    ld.add_action(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': robot_desc
        }]
    ))

    return ld
```

### Option B: Use robot_state_publisher (Recommended)

The `robot_state_publisher` node does this for you:

```python
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                'robot_description': open(
                    PathJoinSubstitution([
                        FindPackageShare("robot_description"),
                        "urdf",
                        "robot.urdf"
                    ])
                ).read()
            }]
        )
    ])
```

**What `robot_state_publisher` does**:
1. Reads URDF from `robot_description` parameter
2. Publishes to `/tf` topic (coordinate transforms)
3. Subscribes to `/joint_states` (joint positions)
4. Broadcasts robot structure to RViz and other tools

---

## Step 2: Call Spawn Service

Once `robot_description` is available, call the spawn service:

**Service name**: `/spawn_entity`

**Service definition** (`ros_gz_msgs/srv/SpawnEntity.srv`):
```
# Request
string name                  # Unique robot name
string xml                   # URDF/SDF as string
string world_name            # World to spawn into
geometry_msgs/Pose initial_pose  # Position and orientation
---
# Response
bool success
string statusMessage
```

**In Python launch file**:
```python
from ros_gz_sim.srv import SpawnEntity
from geometry_msgs.msg import Pose

# Create service call
def spawn_robot():
    client = rclpy.client.Client(SpawnEntity)
    request = SpawnEntity.Request()
    request.name = "my_robot"
    request.xml = robot_desc_string
    request.world_name = "default"
    request.initial_pose = Pose(position=Point(x=0.0, y=0.0, z=1.0))

    future = client.call_async(request)
    rclpy.spin_until_future_complete(client, future)

    if future.result().success:
        print("Robot spawned successfully!")
    else:
        print(f"Spawn failed: {future.result().statusMessage}")
```

---

## Step 3: Integrate into Launch File

**Complete launch file** (`launch/spawn_robot.launch.py`):

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get robot description package
    robot_pkg = FindPackageShare("robot_description")
    urdf_file = PathJoinSubstitution([robot_pkg, "urdf", "robot.urdf"])

    # Read URDF content
    urdf_file_path = os.path.join(
        get_package_share_directory("robot_description"),
        "urdf",
        "robot.urdf"
    )

    with open(urdf_file_path, 'r') as f:
        robot_description = f.read()

    # Node 1: robot_state_publisher (publishes TF frames)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': ''
        }]
    )

    # Node 2: ros_gz_bridge (bridges ROS 2 topics to Gazebo)
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry"
        ],
        output="screen"
    )

    # Node 3: Spawn robot into Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="spawn",
        arguments=[
            "-name", "my_robot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5",
            "-file", urdf_file_path
        ],
        output="screen"
    )

    return LaunchDescription([
        rsp_node,
        bridge_node,
        spawn_robot
    ])
```

**Breaking this down**:
1. **robot_state_publisher**: Reads URDF, publishes transforms
2. **ros_gz_bridge**: Translates ROS 2 topics to Gazebo
3. **spawn** node: Calls spawn service to add robot to world

---

## URDF to SDF Conversion

Gazebo prefers SDF format, but `robot_state_publisher` automatically converts URDF for you. Here's what happens:

**Input (URDF)**:
```xml
<robot name="robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
  </link>
  <joint name="wheel_joint" type="revolute">
    <parent link="base_link"/>
    <child link="wheel"/>
    <limit lower="0" upper="0" effort="100" velocity="1"/>
  </joint>
</robot>
```

**Automatic conversion** (internal, you don't write SDF):
- URDF links become SDF links
- URDF joints become SDF joints
- Inertia parameters convert to physics properties
- Collision and visual geometry preserved

You write URDF once, Gazebo handles the conversion.

---

## Position and Orientation Parameters

When spawning, specify where the robot appears:

**Position** (x, y, z in meters):
```python
request.initial_pose.position.x = 0.0
request.initial_pose.position.y = 0.0
request.initial_pose.position.z = 0.5  # 0.5m above ground
```

**Orientation** (quaternion):
```python
from math import sin, cos, pi

# Rotate 45 degrees around z-axis
theta = pi / 4
request.initial_pose.orientation.x = 0
request.initial_pose.orientation.y = 0
request.initial_pose.orientation.z = sin(theta / 2)
request.initial_pose.orientation.w = cos(theta / 2)
```

**Common orientations**:
- Facing +X: quat [0, 0, 0, 1]
- Facing +Y: quat [0, 0, 0.707, 0.707]
- Facing -X: quat [0, 0, 1, 0]
- Facing -Y: quat [0, 0, 0.707, -0.707]

Or use a quaternion library:
```python
from tf_transformations import quaternion_from_euler

quat = quaternion_from_euler(0, 0, pi/4)  # Roll, pitch, yaw
request.initial_pose.orientation.x = quat[0]
request.initial_pose.orientation.y = quat[1]
request.initial_pose.orientation.z = quat[2]
request.initial_pose.orientation.w = quat[3]
```

---

## Joint State Publishing

When you spawn a robot, it starts with all joints at zero position. To visualize the robot correctly in RViz, you need to publish joint states.

**Option A: From Gazebo**
The robot in Gazebo automatically publishes its joint states. Subscribe to `/joint_states` topic in RViz.

**Option B: Manual Publishing**
If you're controlling joints manually, publish joint states:

```python
from sensor_msgs.msg import JointState
from rclpy.node import Node
import time

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joints)

    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['wheel_left', 'wheel_right', 'arm_joint']
        msg.position = [0.0, 0.0, 0.5]  # Current positions
        msg.velocity = [0.0, 0.0, 0.0]  # Current velocities
        msg.effort = [0.0, 0.0, 0.0]    # Applied forces

        self.publisher.publish(msg)
```

---

## Exercise: Spawn a Robot

**Goal**: Spawn a robot into Gazebo using ros_gz spawn service.

**Setup**:
1. Have a robot URDF ready (from Chapter 9)
2. Have Gazebo running with an empty world
3. Create launch file with robot_state_publisher + bridge + spawn nodes

**Step 1: Create launch file**
```python
# launch/spawn_robot.launch.py
# [Use complete launch file from above]
```

**Step 2: Launch**
```bash
ros2 launch robot_launcher spawn_robot.launch.py
```

**Expected output**:
```
[robot_state_publisher-1]: Publishing robot state
[parameter_bridge-2]: Starting ros_gz_bridge
[spawn-3]: Spawning robot 'my_robot' at [0.0, 0.0, 0.5]
[spawn-3]: Robot spawned successfully!
```

**Step 3: Verify in Gazebo**
- Robot appears in world at specified position
- Gazebo simulation shows robot model

**Step 4: Verify in RViz2**
```bash
ros2 launch robot_launcher display.launch.py
```
- TF frames appear for robot links
- Robot model shows in RViz

**Success**: Robot appears in both Gazebo (physics) and RViz (visualization) with synchronized state.

---

## Try With AI

**Setup**: Open ChatGPT or Claude and ask about spawning your specific robot.

**Prompt 1** (Debugging spawn failures):
```
My robot won't spawn in Gazebo. The error message is:
"Spawn failed: Could not find model"

The URDF loads fine when I view it in RViz. What could be wrong
with the spawn call? Walk me through the checklist.
```

**Prompt 2** (Multiple robot spawning):
```
I want to spawn 5 instances of the same robot in different positions:
(0, 0), (1, 1), (2, 0), (1, 2), (0, 2)

Show me how to modify the spawn launch file to spawn multiple robots
with unique names at each position.
```

**Prompt 3** (URDF to SDF conversion):
```
I'm getting warnings about inertia values during spawn. My URDF has
inertia properties but Gazebo says some values are near zero.
What inertia properties are critical for spawning, and what are
the minimum reasonable values?
```

**Expected Outcomes**:
- AI helps debug spawn failures (missing parameters, syntax errors)
- AI shows patterns for spawning multiple robots
- AI explains which URDF properties affect Gazebo spawning

**Safety Note**: Start with zero velocity (don't command motion immediately after spawning). Give Gazebo a moment to simulate gravity and settle the robot.

---

## Next Steps

You now spawn robots dynamically from launch files. In the next lesson, you'll command these robots and read their sensors, closing the control loop.

**What emerged from this lesson**: Separating robot definition (URDF) from robot instantiation (spawn service) enables flexible, reusable simulation scenarios.

---

## Key Concepts Checkpoint

Before moving on, verify you understand:

- **robot_description parameter**: How to load and publish robot URDF
- **robot_state_publisher**: What it publishes and why it's needed
- **Spawn service**: How to call it and what parameters it needs
- **Position/orientation**: How to specify where robot spawns
- **Launch file integration**: Bringing all components together

If these are clear, you're ready for Lesson 12.3.
