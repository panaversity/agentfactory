---
id: lesson-6-2-launch-files
title: "Lesson 6.2: Launch Files (Multi-Node Startup)"
sidebar_position: 2
sidebar_label: "6.2 Launch Files"
description: "Learn how to start multiple ROS 2 nodes together and pass parameters using Python launch files."
duration_minutes: 60
proficiency_level: "B1"
layer: "L3"
hardware_tier: 1
learning_objectives:
  - "Write Python launch files that start multiple nodes"
  - "Pass parameters to nodes via launch files"
  - "Use YAML configuration files for complex systems"
  - "Implement launch arguments for flexible deployment"
  - "Understand launch file execution order and dependencies"
---

# Lesson 6.2: Launch Files — Starting Multi-Node Systems

Imagine you built a robot system with 10 nodes:
- 3 sensor publishers (camera, LIDAR, IMU)
- 2 controllers (motion, arm)
- 3 monitors (battery, thermal, diagnostics)
- 2 safety nodes (emergency stop, collision avoidance)

Launching each node individually in separate terminals would be chaotic:
```bash
# Terminal 1
ros2 run sensor_pkg camera_node --ros-args --log-level debug

# Terminal 2
ros2 run sensor_pkg lidar_node --ros-args --log-level debug

# Terminal 3
ros2 run sensor_pkg imu_node

# ... etc
```

**Launch files** solve this problem. A single launch file can start all nodes with one command, pass parameters to each node, and coordinate initialization.

## What is a Launch File?

A ROS 2 launch file is a Python script that:
1. Defines a set of nodes to start
2. Specifies parameters for each node
3. Optionally remaps topic names
4. Coordinates node startup and shutdown

Launch files use the `launch` and `launch_ros` packages to describe the system declaratively.

## Basic Launch File Structure

### Minimal Example: Two Nodes

Create a file: `my_robot_pkg/launch/simple_system.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node 1: Publisher
        Node(
            package='my_first_package',
            executable='minimal_publisher',
            name='sensor_publisher',
            output='screen',
        ),
        # Node 2: Subscriber
        Node(
            package='my_first_package',
            executable='minimal_subscriber',
            name='data_processor',
            output='screen',
        ),
    ])
```

**What this does:**
- `package='my_first_package'`: Which ROS 2 package contains the executable
- `executable='minimal_publisher'`: The entry point name (from package's setup.py)
- `name='sensor_publisher'`: The node name when it runs (can differ from executable name)
- `output='screen'`: Print logs to terminal (instead of silent mode)

**Run it:**
```bash
ros2 launch my_first_package simple_system.launch.py
```

**Expected output:**
```
[minimal_publisher-1] [INFO] [minimal_publisher]: Starting publisher
[data_processor-1] [INFO] [minimal_subscriber]: I heard: "Hello World: 0"
[data_processor-1] [INFO] [minimal_subscriber]: I heard: "Hello World: 1"
[minimal_publisher-1] [INFO] [minimal_publisher]: Publishing: Hello World: 2
[data_processor-1] [INFO] [minimal_subscriber]: I heard: "Hello World: 2"
```

Both nodes start together (prefixed with their index), communicate immediately, and both log to the same terminal. The publisher keeps publishing, the subscriber keeps receiving.

## Launch Files with Parameters

The real power of launch files is passing parameters to nodes at startup.

### Multi-Node System with Different Parameters

Create: `my_robot_pkg/launch/robot_system.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Fast sensor: High publication rate
        Node(
            package='my_first_package',
            executable='configurable_node',
            name='camera_publisher',
            output='screen',
            parameters=[{
                'publish_rate': 30.0,  # 30 Hz for camera
                'robot_name': 'my_robot',
                'sensor_type': 'camera',
            }],
        ),

        # Slow sensor: Lower publication rate
        Node(
            package='my_first_package',
            executable='configurable_node',
            name='lidar_publisher',
            output='screen',
            parameters=[{
                'publish_rate': 10.0,  # 10 Hz for LIDAR
                'robot_name': 'my_robot',
                'sensor_type': 'lidar',
            }],
        ),

        # Data processor: Consumes both sensors
        Node(
            package='my_first_package',
            executable='data_processor',
            name='sensor_fusion',
            output='screen',
            parameters=[{
                'timeout': 5.0,
            }],
        ),
    ])
```

**Key observations:**
- Same executable (`configurable_node`) runs **twice** with different names and parameters
- Each node instance has its own parameter set
- Parameters are passed as a list of dictionaries to the `parameters` argument

**Run it:**
```bash
ros2 launch my_first_package robot_system.launch.py
```

**Verify parameter assignment:**
```bash
ros2 param list /camera_publisher
# Output: /camera_publisher:
#   publish_rate
#   robot_name
#   sensor_type

ros2 param get /camera_publisher publish_rate
# Output: publish_rate: 30.0

ros2 param get /lidar_publisher publish_rate
# Output: publish_rate: 10.0
```

## Launch Files with Configuration Files (YAML)

For complex systems, hardcoding parameters in the launch file is messy. Instead, use YAML configuration files.

### YAML Configuration File

Create: `my_robot_pkg/config/robot_params.yaml`

```yaml
# Robot-wide configuration
robot_name: "physical_ai_bot"
robot_id: 001

# Camera node configuration
camera_publisher:
  ros__parameters:
    publish_rate: 30.0
    resolution: "1920x1080"
    encoding: "RGB8"
    sensor_type: "camera"

# LIDAR node configuration
lidar_publisher:
  ros__parameters:
    publish_rate: 10.0
    angle_range: 360.0
    max_distance: 50.0
    sensor_type: "lidar"

# Sensor fusion configuration
sensor_fusion:
  ros__parameters:
    timeout: 5.0
    fusion_mode: "weighted"
```

### Launch File Using YAML

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Locate the config file
    config_dir = os.path.join(
        get_package_share_directory('my_first_package'),
        'config'
    )
    config_file = os.path.join(config_dir, 'robot_params.yaml')

    return LaunchDescription([
        Node(
            package='my_first_package',
            executable='configurable_node',
            name='camera_publisher',
            output='screen',
            parameters=[config_file],  # Load from YAML
        ),
        Node(
            package='my_first_package',
            executable='configurable_node',
            name='lidar_publisher',
            output='screen',
            parameters=[config_file],  # Load from YAML
        ),
        Node(
            package='my_first_package',
            executable='data_processor',
            name='sensor_fusion',
            output='screen',
            parameters=[config_file],  # Load from YAML
        ),
    ])
```

**Advantage:** Change parameters without modifying the Python launch file. Just edit the YAML and relaunch.

## Making Launch Files Flexible with Arguments

What if you want the same launch file to work in different modes (simulation vs reality, debug vs production)?

### Launch File with Arguments

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='default_robot',
        description='Name of the robot'
    )

    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='true',
        description='Run in simulation (true) or real (false)'
    )

    return LaunchDescription([
        robot_name_arg,
        sim_mode_arg,

        Node(
            package='my_first_package',
            executable='robot_controller',
            name='controller',
            output='screen',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name'),
                'simulation_mode': LaunchConfiguration('sim_mode'),
                'safety_enabled': True,
            }],
        ),
    ])
```

**Run with default arguments:**
```bash
ros2 launch my_first_package robot_system.launch.py
# Uses: robot_name=default_robot, sim_mode=true
```

**Run with custom arguments:**
```bash
ros2 launch my_first_package robot_system.launch.py robot_name:=my_bot sim_mode:=false
# Uses: robot_name=my_bot, sim_mode=false
```

## Setting Up Your Package for Launch Files

For launch files to work, you must tell ROS 2 where to find them. This is done in `setup.py`.

### Minimal setup.py

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_first_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Standard ROS 2 files
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'minimal_publisher = my_first_package.minimal_publisher:main',
            'minimal_subscriber = my_first_package.minimal_subscriber:main',
            'configurable_node = my_first_package.configurable_node:main',
            'data_processor = my_first_package.data_processor:main',
        ],
    },
)
```

**Key parts:**
- `data_files` with glob patterns tells ROS 2 to include all `.launch.py` files from the `launch/` directory
- Same for config files in the `config/` directory

**After adding launch files:**
```bash
cd ~/ros2_ws
colcon build --packages-select my_first_package
source install/setup.bash
```

## Guided Practice: Complete Multi-Node System

Let's build a small robot system launch configuration with:
- 2 sensor publishers (different rates)
- 1 data processor
- Configurable parameters

### Step 1: Create Package Structure

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_demo
cd robot_demo
mkdir -p launch config
```

### Step 2: Create Nodes

Create three simple Python files in `robot_demo/robot_demo/`:

**sensor_node.py** (publishes fake sensor data):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('sensor_name', 'sensor1')

        rate = self.get_parameter('publish_rate').value
        self.publisher_ = self.create_publisher(Float32, 'sensor_data', 10)
        self.timer = self.create_timer(1.0/rate, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        msg.data = 42.0
        sensor_name = self.get_parameter('sensor_name').value
        self.get_logger().info(f'{sensor_name} published: {msg.data}')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SensorNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**processor_node.py** (subscribes to sensor data):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ProcessorNode(Node):
    def __init__(self):
        super().__init__('processor_node')
        self.subscription = self.create_subscription(Float32, 'sensor_data', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'Processed: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ProcessorNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Create Launch File

Create `robot_demo/launch/robot_demo.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_demo',
            executable='sensor_node',
            name='temperature_sensor',
            output='screen',
            parameters=[{
                'publish_rate': 5.0,
                'sensor_name': 'Temperature',
            }],
        ),
        Node(
            package='robot_demo',
            executable='sensor_node',
            name='humidity_sensor',
            output='screen',
            parameters=[{
                'publish_rate': 2.0,
                'sensor_name': 'Humidity',
            }],
        ),
        Node(
            package='robot_demo',
            executable='processor_node',
            name='data_processor',
            output='screen',
        ),
    ])
```

### Step 4: Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select robot_demo
source install/setup.bash
ros2 launch robot_demo robot_demo.launch.py
```

**Expected output:**
```
[sensor_node-1] [INFO] [temperature_sensor]: Temperature published: 42.0
[sensor_node-2] [INFO] [humidity_sensor]: Humidity published: 42.0
[processor_node-1] [INFO] [processor_node]: Processed: 42.0
[processor_node-1] [INFO] [processor_node]: Processed: 42.0
```

## Independent Practice

### Exercise 1: Parameter Configuration

Add a `config/sensors.yaml` file with sensor-specific parameters, and modify the launch file to load it.

### Exercise 2: Launch with Arguments

Modify the launch file to accept a `robot_mode` argument (simulation/real) and pass it to nodes.

### Exercise 3: Complex System

Create a system with:
- 3 sensor nodes (different rates)
- 2 processors (each subscribes to different sensors)
- 1 aggregator (subscribes to both processors)

Document each node's role in a comment.

## Try With AI

**Setup:** Work with your AI to design and debug a multi-node launch system.

**Prompt Set:**

```
Prompt 1: "I have 5 nodes that need to start together, but they depend on each other (Node A must start before Node B). How do I structure this in a launch file? Can I specify startup order?"

Prompt 2: "I want to run the same node executable multiple times with different parameters. What's the best way to set up the launch file to avoid duplication?"

Prompt 3: "When I launch my system, one node fails to start. How would I debug this? What tools help identify which node crashed and why?"
```

**Expected Outcomes:**
- AI explains launch file execution order (usually parallel, but dependencies can be specified)
- AI shows parameter templating patterns to reduce duplication
- AI demonstrates debugging workflows (ros2 launch verbosity, ros2 node list, logs)

**Safety Note:** When launching robot systems, verify all nodes start successfully and are communicating before operating physical hardware. Use `ros2 node list` and `rqt_graph` to confirm connectivity.

**Optional Stretch:**
Create a launch file that:
- Starts a system with 3+ nodes
- Has 3+ configurable parameters
- Accepts command-line arguments to switch between modes
- Includes logging configuration for debugging
