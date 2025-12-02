---
id: lesson-11-1-camera-simulation
title: "Lesson 11.1: Camera Simulation"
sidebar_position: 1
sidebar_label: "11.1 Camera Simulation"
description: "Adding and configuring camera sensors on simulated robots"
duration_minutes: 60
proficiency_level: "B1"
layer: "L1"
hardware_tier: 1
learning_objectives:
  - "Add a camera sensor to a robot model in SDF"
  - "Configure camera parameters (resolution, FOV, frame rate)"
  - "View camera output in Gazebo GUI"
  - "Understand camera frame hierarchy and transforms"
skills:
  - "sensor-simulation"
  - "camera-config"
cognitive_load:
  new_concepts: 7
tier_1_path: "TheConstruct cloud environment (basic rendering)"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 11.1: Camera Simulation

## Why Cameras Matter for Robots

Imagine a robot that can see. A factory bot inspects products with a camera. A delivery robot recognizes obstacles with a camera. A humanoid robot responds to hand gestures from a camera feed. Vision is one of the most powerful sensors a robot can have.

In simulation, adding a camera is straightforward: define a camera sensor in your robot's SDF file with specific parameters, and Gazebo will render images from that viewpoint. Those images publish to a ROS 2 topic, and your vision nodes can process them—object detection, lane following, gesture recognition, whatever you need.

This lesson teaches you how to add cameras to simulated robots and interpret the configuration options.

---

## Understanding Camera Parameters

Before adding a camera, understand what each parameter controls:

### Resolution (Image Size)

A camera captures a 2D grid of pixels. Resolution defines that grid size.

**Common resolutions:**
- **640x480**: Good balance of detail and speed
- **1920x1080**: High detail, slower processing
- **320x240**: Fast processing, less detail (mobile/embedded)

**In SDF:**
```xml
<image>
  <width>640</width>
  <height>480</height>
</image>
```

**Why it matters**: Higher resolution captures more detail but generates larger messages and slower vision processing. Lower resolution runs faster but loses fine details.

### Field of View (FOV)

Field of view describes how wide the camera's view is, measured in radians.

**Common FOVs:**
- **1.047 radians (~60 degrees)**: Telephoto, zoomed in, narrow view
- **1.571 radians (~90 degrees)**: Normal, like human eye
- **2.356 radians (~135 degrees)**: Wide angle, panoramic view

**In SDF:**
```xml
<horizontal_fov>1.047</horizontal_fov>
```

**Why it matters**: Wider FOV sees more of the scene but introduces distortion. Narrower FOV zooms in, sees less area but more detail.

### Near and Far Clip Planes

Cameras have a depth range. Anything closer than the near clip plane or farther than the far clip plane is not rendered.

**Typical values:**
- **Near**: 0.01 to 0.1 meters (how close the camera can focus)
- **Far**: 10 to 100 meters (how far it can see)

**In SDF:**
```xml
<clip>
  <near>0.1</near>
  <far>100</far>
</clip>
```

**Why it matters**: Objects outside this range don't appear in the image. If a robot navigates a room (5 meters wide), set far clip to at least 5 meters. If inspecting tiny parts (centimeters), set near clip very small.

### Update Rate (Frame Rate)

How often the camera captures and publishes frames, in Hz (frames per second).

**Common rates:**
- **10 Hz**: Real-time control, slower systems
- **30 Hz**: Video streaming, balance between detail and speed
- **60 Hz**: Fast control loops, high-speed response

**In SDF:**
```xml
<always_on>1</always_on>
<update_rate>30</update_rate>
```

**Why it matters**: Higher frame rates mean more messages on your ROS network and more processing load. Lower rates reduce latency sensitivity.

---

## Adding a Camera to Your Robot

### Step 1: Define the Camera Sensor in SDF

Edit your robot's SDF file. Add a `<sensor>` element to the link where you want the camera mounted (typically the robot's head or base).

**Example: Forward-facing camera on robot base**

```xml
<model name="my_robot">
  <link name="base_link">
    <!-- Robot body geometry... -->
  </link>

  <link name="camera_link">
    <!-- Camera physical representation (small box) -->
    <inertial>
      <mass>0.05</mass>
    </inertial>
    <visual>
      <geometry>
        <box>
          <size>0.05 0.05 0.05</size>
        </box>
      </geometry>
    </visual>

    <!-- CAMERA SENSOR DEFINITION -->
    <sensor name="camera" type="camera">
      <!-- Camera parameters -->
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>

      <!-- Publishing behavior -->
      <always_on>1</always_on>
      <update_rate>30</update_rate>

      <!-- Topic name where images publish -->
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <camera_name>camera</camera_name>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </link>

  <!-- Mount camera on base_link -->
  <joint name="camera_joint" type="fixed">
    <parent>base_link</parent>
    <child>camera_link</child>
    <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
  </joint>
</model>
```

**What's happening:**
- `<sensor type="camera">` defines this as a camera sensor
- `<camera>` contains parameters (FOV, resolution, clip planes)
- `<always_on>` means the camera is always publishing (not just on request)
- `<update_rate>30</update_rate>` publishes 30 frames per second
- `<plugin>` configures ROS integration (publishes to topics)
- `<joint>` mounts the camera at position (0.1, 0, 0.1) on the base_link

### Step 2: Verify the Camera Publishes

After launching Gazebo with your robot, check that the camera publishes data.

**List camera topics:**
```bash
ros2 topic list | grep camera
```

**Expected output:**
```
/camera/camera_info
/camera/image_raw
```

Two topics:
- `/camera/image_raw`: Raw pixel data (height x width x 3 for RGB)
- `/camera/camera_info`: Camera calibration (focal length, principal point, distortion)

**Inspect image data:**
```bash
ros2 topic echo /camera/image_raw --once
```

You'll see metadata about the image. The actual pixel data is binary (too large to print).

### Step 3: Visualize the Camera Output in Gazebo GUI

The Gazebo GUI can display camera output directly. This is your first real validation that the camera works.

**Method 1: Built-in Camera Display**
1. In Gazebo GUI, go to **Plugins → Camera → Camera Name**
2. A window opens showing the live camera feed

**Method 2: Using RViz (ROS Visualization)**
1. Launch RViz: `rviz2`
2. Add Image display
3. Select `/camera/image_raw` topic
4. See live camera output

---

## Understanding Camera Data Flow

Here's how camera data flows in the system:

```
Robot SDF
    ↓
Camera Sensor (renders image)
    ↓
ROS 2 Plugin (converts to message)
    ↓
/camera/image_raw Topic (geometry_msgs/Image)
    ↓
Your Vision Node
    ↓
Objects detected, lanes found, etc.
```

Each step matters:
- **SDF definition**: Specifies camera parameters
- **Gazebo rendering**: Generates pixel data from sensor viewpoint
- **ROS plugin**: Publishes as ROS messages
- **ROS topic**: Makes data available to your nodes
- **Your node**: Processes the image (OpenCV, TensorFlow, custom code)

---

## Exercise: Add a Forward-Facing Camera

In this exercise, you'll add a camera to a simple robot model and verify it publishes.

**Starter robot SDF (simple box robot):**

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="simple_robot">
    <static>false</static>

    <!-- Main body -->
    <link name="base_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.1</iyy>
          <iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.2 0.2 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.2 0.2 0.2</size>
          </box>
        </geometry>
      </collision>
    </link>

    <!-- TODO: Add camera_link here -->
    <!-- TODO: Add camera sensor in camera_link -->
    <!-- TODO: Add joint to mount camera on base_link -->

  </model>
</sdf>
```

**Your task:**
1. Copy this SDF into a file `my_robot.sdf`
2. Add a `camera_link` with a camera sensor (use parameters from Step 1)
3. Add a joint to mount it at position (0.1, 0, 0.1) on the base_link
4. Launch Gazebo: `gazebo my_robot.sdf`
5. Verify camera publishes: `ros2 topic list`
6. View output in Gazebo GUI (Plugins → Camera)

**Expected result**: You see a live camera feed of the Gazebo scene from your robot's viewpoint.

---

## Common Camera Issues and Fixes

### Issue 1: No Camera Topics

**Symptom**: You run `ros2 topic list` and see no `/camera/*` topics

**Likely cause**:
- Camera sensor not in SDF file
- ROS plugin not configured correctly
- Gazebo not built with ROS 2 support

**Fix**:
1. Double-check SDF file has `<sensor type="camera">` block
2. Verify `<plugin filename="libgazebo_ros_camera.so">` is present
3. Ensure Gazebo is launched with ROS 2: `LIBgazebo_PLUGIN_PATH=/opt/ros/humble/lib:$LIBGAZEBO_PLUGIN_PATH gazebo my_robot.sdf`

### Issue 2: All-Black Camera Feed

**Symptom**: Camera publishes images, but they're all black pixels

**Likely cause**:
- Camera is pointing at nothing (wrong orientation)
- Far clip plane is too close (everything is beyond visible range)
- Lighting is off in Gazebo scene

**Fix**:
1. Check camera orientation in joint `<origin rpy="...">` (should point forward: `rpy="0 0 0"`)
2. Increase far clip plane: `<far>1000</far>`
3. Verify Gazebo scene has lighting (check "Lighting" in menu)

### Issue 3: Camera Publishes Very Slowly

**Symptom**: Frames arrive but with long delays (not at expected frame rate)

**Likely cause**:
- Update rate set too low
- GPU rendering bottleneck (high resolution, complex scene)

**Fix**:
1. Lower resolution: `<width>320</width><height>240</height>`
2. Reduce update rate if 30 Hz is too much: `<update_rate>10</update_rate>`
3. If on Tier 1 (cloud), rendering is inherently slower—this is expected

---

## Validation Checklist

After adding a camera:

- [ ] Camera sensor is defined in SDF with proper parameters
- [ ] Camera link is created and mounted with a joint
- [ ] Gazebo launches without errors
- [ ] `ros2 topic list` shows `/camera/image_raw` and `/camera/camera_info`
- [ ] Gazebo GUI displays live camera feed
- [ ] Camera images are not all black or all white
- [ ] Frame rate matches expected rate (check with `ros2 topic bw /camera/image_raw`)

---

## Try With AI

**Setup**: Open a chat with Claude (or your preferred AI assistant) and describe a camera configuration challenge.

**Prompt Set:**

```
Prompt 1 (Basic): "I'm adding a camera to a robot in Gazebo.
The camera should see a 5-meter radius around the robot.
What FOV and clip plane values should I use?"

Prompt 2 (Intermediate): "My camera is publishing images but they're all black.
The camera is mounted at position (0.1, 0, 0.1) with orientation (0, 0, 0).
The scene has ground, walls, and a blue box 3 meters away.
What's likely wrong and how do I fix it?"

Prompt 3 (Advanced): "I need my robot to use two cameras:
one wide-angle (180 degree FOV) for navigation and one narrow (30 degree) for fine object inspection.
Should these be on separate links or the same link?
What resolution and frame rate would you recommend for each?"
```

**Expected Outcomes:**
- AI suggests appropriate FOV values based on use case
- AI helps diagnose camera issues by reasoning about geometry and rendering
- AI considers tradeoffs between resolution, frame rate, and processing load

**Safety Note**: Camera feeds are for simulation only at this stage. When deploying real cameras on physical robots, ensure you understand focal length, sensor size, and lens distortion—simulated cameras don't perfectly match real hardware. Validate in simulation first, test on real hardware second.

**Optional Stretch**: Write a simple ROS 2 node that subscribes to `/camera/image_raw` and counts how many blue pixels are visible. This prepares you for real vision processing in later lessons.
