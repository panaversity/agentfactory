---
id: lesson-11-2-lidar-simulation
title: "Lesson 11.2: LIDAR Simulation"
sidebar_position: 2
sidebar_label: "11.2 LIDAR Simulation"
description: "Configuring LIDAR sensors for distance measurement and mapping"
duration_minutes: 60
proficiency_level: "B1"
layer: "L1"
hardware_tier: 1
learning_objectives:
  - "Add a LIDAR sensor to a robot model in SDF"
  - "Configure LIDAR parameters (range, samples, angle)"
  - "Visualize LIDAR point cloud in Gazebo"
  - "Understand 2D vs 3D LIDAR"
skills:
  - "sensor-simulation"
  - "lidar-config"
cognitive_load:
  new_concepts: 7
tier_1_path: "TheConstruct cloud environment"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 11.2: LIDAR Simulation

## Why LIDAR Powers Robot Perception

LIDAR (Light Detection and Ranging) measures distance by bouncing light beams off surfaces and measuring the return time. A 360-degree LIDAR scanner sees the entire environment around a robot, point by point. This data enables:

- **Obstacle avoidance**: Detect walls, furniture, people
- **SLAM** (Simultaneous Localization and Mapping): Build a map while navigating
- **Autonomous navigation**: Plan paths around obstacles
- **Object detection**: Recognize shapes in 3D space

In simulation, LIDAR produces dense 3D point clouds. Your robot publishes thousands of points per second, and your navigation nodes process them to avoid collisions.

This lesson teaches you how to configure LIDAR sensors and interpret the data flow.

---

## Understanding LIDAR Parameters

### Scanning Geometry

LIDAR works by sweeping laser beams across an angle range and measuring distance to each point.

**Horizontal scan** (2D LIDAR, planar):
```
Robot with 2D LIDAR (side view):

        Laser beam (pointing right)
        ↓
    ┌───────┐
    │ LIDAR │ ─ ─ ─ ─ Wall (detects 2 meters)
    └───────┘
    │ LIDAR │ ─ ─ ─ Ground (detects 0.5 meters)
        ↓
    Sweep angle range: from -180° to +180°
```

**Vertical array** (3D LIDAR, Velodyne-style):
```
Multiple horizontal planes stacked vertically.
Each plane independently scans 360 degrees.
Result: 3D point cloud (millions of points per second).
```

### Critical Parameters

**Angle Range (in radians):**
- Min angle: start of sweep (often -π, pointing backward)
- Max angle: end of sweep (often +π, after full rotation)
- Formula: `max_angle - min_angle` = total sweep angle

**Typical values:**
- 360-degree scanner: min = -3.14159, max = 3.14159 (covers full circle)
- 180-degree scanner: min = -1.571, max = 1.571 (front half only)
- 120-degree scanner: min = -1.047, max = 1.047 (narrow sweep)

**Sample Count (horizontal samples):**
- How many distance measurements in the sweep angle range
- **360 samples** for 360-degree scan: one measurement per degree
- **720 samples** for higher resolution (0.5 degrees per sample)
- **180 samples** for lower resolution (2 degrees per sample)

**Why it matters**: More samples mean finer detail but more processing load.

**Range (distance limits):**
- **Min range**: Closest distance the LIDAR can measure (typically 0.1 to 0.5 meters)
- **Max range**: Farthest distance (typically 10 to 100 meters depending on hardware)

**Update rate (frames per second):**
- How often LIDAR completes a full scan
- Typical: 10 Hz (10 scans per second)
- Faster hardware: 20 Hz

---

## Adding a LIDAR to Your Robot

### Step 1: Define the LIDAR Sensor in SDF

Add a `<sensor>` element of type `gpu_lidar` to your robot's SDF.

**Example: 360-degree LIDAR on robot top**

```xml
<model name="my_robot">
  <link name="base_link">
    <!-- Robot body... -->
  </link>

  <link name="lidar_link">
    <!-- LIDAR physical representation (small cylinder) -->
    <inertial>
      <mass>0.1</mass>
    </inertial>
    <visual>
      <geometry>
        <cylinder>
          <radius>0.05</radius>
          <length>0.05</length>
        </cylinder>
      </geometry>
    </visual>

    <!-- LIDAR SENSOR DEFINITION -->
    <sensor name="lidar" type="gpu_lidar">
      <lidar>
        <!-- Horizontal scan parameters -->
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
          <!-- Vertical layers (1 = 2D scan, >1 = 3D) -->
          <vertical>
            <samples>1</samples>
            <resolution>1</resolution>
            <min_angle>0</min_angle>
            <max_angle>0</max_angle>
          </vertical>
        </scan>

        <!-- Range parameters -->
        <range>
          <min>0.1</min>
          <max>10</max>
        </range>

        <!-- Noise model (adds realism) -->
        <noise>
          <type>gaussian</type>
          <mean>0</mean>
          <stddev>0.01</stddev>
        </noise>
      </lidar>

      <!-- Publishing behavior -->
      <always_on>1</always_on>
      <update_rate>10</update_rate>

      <!-- Topic name where point cloud publishes -->
      <plugin name="lidar_controller" filename="libgazebo_ros_gpu_lidar.so">
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </link>

  <!-- Mount LIDAR on top of base_link -->
  <joint name="lidar_joint" type="fixed">
    <parent>base_link</parent>
    <child>lidar_link</child>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>
</model>
```

**What's happening:**
- `<sensor type="gpu_lidar">` uses GPU acceleration for fast rendering
- `<horizontal>` defines the main 360-degree sweep (360 samples)
- `<vertical>` defines vertical layers (1 = single horizontal plane, like a 2D laser)
- `<range>` defines min/max detection distance (0.1 to 10 meters)
- `<noise>` adds Gaussian noise for realistic data (optional but recommended)
- `<plugin>` configures ROS integration
- Joint mounts the LIDAR on top (z = 0.2 meters) of the base

### Step 2: Verify LIDAR Publishes

After launching Gazebo with your robot, check that LIDAR publishes data.

**List LIDAR topics:**
```bash
ros2 topic list | grep lidar
```

**Expected output:**
```
/lidar/scan
/lidar/points
```

Two types of data:
- `/lidar/scan`: LaserScan message (2D, like old-school laser range finder)
- `/lidar/points`: PointCloud2 message (3D, raw points x,y,z)

**Inspect LIDAR data:**
```bash
ros2 topic echo /lidar/scan --once
```

Output shows:
```
angle_min: -3.141592653589793
angle_max: 3.141592653589793
angle_increment: 0.017453292519943295
range_max: 10.0
ranges: [4.532, 4.521, 4.498, ...]  # 360 distance measurements
```

Each value in `ranges[]` is the distance detected in that direction.

### Step 3: Visualize LIDAR Output in RViz

RViz is the standard tool for visualizing 3D point clouds.

**Launch RViz:**
```bash
rviz2
```

**Add point cloud visualization:**
1. Click "Add" button
2. Select "PointCloud2"
3. Set topic to `/lidar/points`
4. You'll see colorful points representing detected surfaces

**Interpretation:**
- Each point is x,y,z position in 3D space
- Colors indicate distance (red = far, blue = close) or intensity
- Dense clouds = lots of surfaces (cluttered environment)
- Sparse clouds = open space (few obstacles)

---

## 2D vs 3D LIDAR

### 2D LIDAR (Planar Scan)

Scans a single horizontal plane. Ignores height information.

**SDF configuration:**
```xml
<vertical>
  <samples>1</samples>
  <min_angle>0</min_angle>
  <max_angle>0</max_angle>
</vertical>
```

**Use cases:**
- Navigation on flat ground (wheeled robots)
- 2D mapping and SLAM
- Obstacle detection in horizontal plane

**Pros:**
- Lower computation
- Simpler data processing
- Works well for indoor navigation

**Cons:**
- Misses obstacles above/below the scan plane (stairs, overpasses)
- Can't distinguish ground from obstacle

### 3D LIDAR (Vertical Stack)

Multiple horizontal planes stacked vertically. Captures full 3D environment.

**SDF configuration:**
```xml
<vertical>
  <samples>16</samples>
  <min_angle>-0.26</min_angle>
  <max_angle>0.26</max_angle>
</vertical>
```

(16 vertical layers, spanning about 30 degrees)

**Use cases:**
- Humanoid robots (need full 3D)
- Obstacle avoidance (stairs, overhangs)
- SLAM in 3D environments (forests, rubble)

**Pros:**
- Full 3D perception
- Detects obstacles at all heights
- Better for complex terrain

**Cons:**
- Significantly more data (16x or more)
- Slower processing
- More expensive sensor (in real hardware)

---

## Understanding LIDAR Data Sparsity

LIDAR produces sparse data: only points where the beam hit something. An open hallway generates few points. A cluttered room generates many points.

**Sparse hallway:**
```
Robot in empty hallway:
LIDAR → Wall left (x = -2m) → 1 point per angle
LIDAR → Wall right (x = 2m) → 1 point per angle
LIDAR → Forward (open space) → no return

Result: Only 2 lines of points (left and right walls)
```

**Dense room:**
```
Robot surrounded by objects:
LIDAR → Walls, furniture, boxes, people
Many rays return → Dense point cloud

Result: Points everywhere
```

**Why it matters for your robot:**
- Empty space = few points to process (faster)
- Cluttered space = many points (slower processing, needs optimization)

---

## Exercise: Add a 360-Degree LIDAR

In this exercise, you'll add a 360-degree LIDAR to your robot and visualize the point cloud.

**Task:**
1. Use the starter SDF from Lesson 11.1 (or your own robot)
2. Add a `lidar_link` with a `gpu_lidar` sensor
3. Configure for 360-degree horizontal scan, 1 vertical layer (2D)
4. Set range to 10 meters
5. Add Gaussian noise (stddev 0.01)
6. Launch Gazebo and your robot
7. Verify `/lidar/points` topic publishes
8. Open RViz and visualize the point cloud
9. Walk the robot around the environment and observe how the point cloud changes

**Expected result**: You see a 3D visualization of the environment as detected by your simulated LIDAR.

---

## Common LIDAR Issues and Fixes

### Issue 1: No LIDAR Topics

**Symptom**: No `/lidar/scan` or `/lidar/points` topics

**Likely cause:**
- LIDAR sensor not in SDF
- Wrong plugin: `libgazebo_ros_gpu_lidar.so` not found
- Gazebo not built with GPU LIDAR support

**Fix:**
1. Check SDF has `<sensor type="gpu_lidar">` block
2. Verify plugin filename is correct
3. Try fallback plugin: `libgazebo_ros_lidar.so` (CPU version, slower)

### Issue 2: LIDAR Range Very Short

**Symptom**: LIDAR only detects objects very close (less than 1 meter)

**Likely cause:**
- Max range too low
- Min range too high
- Noise model hiding distant points

**Fix:**
1. Increase max range: `<max>100</max>`
2. Decrease min range: `<min>0.05</min>`
3. Reduce noise stddev: `<stddev>0.001</stddev>`

### Issue 3: Too Many Points, Point Cloud Very Dense

**Symptom**: Point cloud is so dense it's hard to see structure. RViz is slow.

**Likely cause:**
- Too many horizontal samples (360 might be excessive)
- Multiple vertical layers creating redundant points

**Fix:**
1. Reduce horizontal samples: `<samples>180</samples>` (every 2 degrees)
2. Reduce vertical layers if 3D: `<samples>4</samples>` (down from 16)
3. Increase min/max angle to narrow beam: `<min_angle>-1.57</min_angle><max_angle>1.57</max_angle>`

---

## Validation Checklist

After adding a LIDAR:

- [ ] LIDAR sensor is defined in SDF with correct parameters
- [ ] LIDAR link is created and mounted on robot
- [ ] Gazebo launches without errors
- [ ] `ros2 topic list` shows `/lidar/scan` and `/lidar/points`
- [ ] `ros2 topic echo /lidar/scan` shows angle and range data
- [ ] RViz visualizes point cloud from `/lidar/points`
- [ ] Point cloud shows nearby obstacles (walls, objects)
- [ ] LIDAR data has reasonable sparsity (not all black, not impossibly dense)

---

## Try With AI

**Setup**: Open a chat and describe a LIDAR configuration problem.

**Prompt Set:**

```
Prompt 1 (Basic): "I'm adding a 2D LIDAR to a wheeled robot for
hallway navigation. The hallway is 20 meters long and 3 meters wide.
What range and angle parameters should I use?"

Prompt 2 (Intermediate): "My LIDAR is detecting a point cloud but
it's very sparse (only wall outlines visible). The robot is in a
furnished living room. Should I increase samples or something else?
What's the tradeoff if I increase from 360 to 720 samples?"

Prompt 3 (Advanced): "I need my humanoid robot to use a 3D LIDAR
for step detection and obstacle avoidance. It has 16 vertical layers
spanning 30 degrees. Should this update at 20 Hz or is that too
slow for a walking robot?"
```

**Expected Outcomes:**
- AI suggests appropriate range and angles based on environment
- AI explains the tradeoff between resolution and processing load
- AI helps optimize LIDAR configuration for specific robot morphology

**Safety Note**: LIDAR simulation assumes perfect reflections and no occlusion errors. Real LIDARs have failure modes: glass reflects poorly, black surfaces absorb, rain/dust scatter beams. When deploying real LIDAR, test in actual conditions and validate your range assumptions.

**Optional Stretch**: Write a ROS 2 node that counts how many points are closer than 1 meter. This is the basis for obstacle detection in later lessons.
