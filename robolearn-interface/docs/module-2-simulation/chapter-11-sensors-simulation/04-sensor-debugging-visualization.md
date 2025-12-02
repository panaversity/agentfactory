---
id: lesson-11-4-sensor-debugging-visualization
title: "Lesson 11.4: Sensor Debugging and Visualization"
sidebar_position: 4
sidebar_label: "11.4 Sensor Debugging"
description: "Debugging sensor issues and visualizing sensor data effectively"
duration_minutes: 60
proficiency_level: "B1"
layer: "L2"
hardware_tier: 1
learning_objectives:
  - "Diagnose common sensor simulation problems"
  - "Use Gazebo visualization tools for sensor data"
  - "Apply AI collaboration to debug sensor configurations"
  - "Develop systematic debugging workflow"
skills:
  - "sensor-simulation"
  - "debugging"
  - "ai-collaboration"
cognitive_load:
  new_concepts: 6
tier_1_path: "TheConstruct cloud environment + AI assistant"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 11.4: Sensor Debugging and Visualization

## Why Sensor Debugging Is Critical

You've added cameras, LIDAR, and IMUs to your robot. Everything launches. But then:
- Your camera is publishing images, but they're all black
- Your LIDAR detects nothing even though walls are 2 meters away
- Your IMU orientation keeps drifting sideways

In professional robotics, debugging sensor issues is a core skill. Sensors fail silently (publishing wrong data) or noisily (not publishing at all). You need systematic approaches to diagnose problems and tools to visualize what's actually happening.

This lesson teaches you debugging methodology and AI-assisted problem solving.

---

## Sensor Debugging Workflow

When a sensor misbehaves, use this systematic workflow:

### Step 1: Verify Sensor is Publishing

**Check if topics exist:**
```bash
ros2 topic list | grep -E "camera|lidar|imu"
```

**If topics don't exist:**
- Sensor not in SDF file
- Plugin filename wrong (typo in `libgazebo_ros_camera.so`)
- Gazebo plugin path not set: `export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH`
- Gazebo not built with ROS 2 support

**Fix:** Verify SDF, check plugin filename, verify environment variables.

### Step 2: Verify Sensor is Publishing Data

**Check topic frequency:**
```bash
ros2 topic bw /camera/image_raw
```

**Output shows:**
```
Subscribed to [/camera/image_raw]
average: 300.00B, mean: 300.00B, min: 300B, max: 300B
Window size: 1000 messages
```

**If frequency is zero:**
- Sensor exists but not publishing data
- Likely cause: configuration error (update_rate = 0, always_on = false)

**If frequency is lower than expected:**
- Rendering bottleneck (scene too complex, resolution too high)
- GPU saturation (multiple heavy sensors, high frame rate)

**Fix:** Lower resolution/update_rate, simplify scene, use lower-fidelity rendering.

### Step 3: Inspect Actual Sensor Data

**For cameras:**
```bash
ros2 topic echo /camera/image_raw --once
```

Check `width`, `height`, `step`, and `data` size. If all black:
```
width: 640
height: 480
step: 1920
data: [0, 0, 0, 0, 0, ...]  # All zeros = black image
```

**For LIDAR:**
```bash
ros2 topic echo /lidar/scan --once
```

Check `ranges[]` values. If all zeros or all inf:
```
ranges: [0.0, 0.0, 0.0, ...]  # No detection
# OR
ranges: [inf, inf, inf, ...]  # Out of range
```

**For IMU:**
```bash
ros2 topic echo /imu --once
```

Check `linear_acceleration` and `angular_velocity`. Should see:
```
linear_acceleration: {x: small, y: small, z: 9.81}  # Gravity in z
angular_velocity: {x: small, y: small, z: small}
```

If all zeros:
```
linear_acceleration: {x: 0.0, y: 0.0, z: 0.0}  # Wrong!
```

This indicates configuration issue (sensor frame not updating).

### Step 4: Visualize in Gazebo GUI

**For cameras:**
- Click Plugins → Camera
- See live view
- If black: camera pointed wrong direction or far clip too close

**For LIDAR:**
- Launch RViz: `rviz2`
- Add PointCloud2 display, select `/lidar/points`
- See 3D point cloud
- If no points: range too short, beam doesn't hit anything

**For IMU:**
- No direct visualization (3D rotation hard to see)
- Use RViz to plot acceleration/velocity over time

---

## Common Sensor Problems and Diagnosis

### Problem 1: Camera Output All Black

**Symptoms:**
```
Camera publishing (/camera/image_raw topic exists)
But images are all zeros (black pixels)
```

**Possible causes:**
1. **Camera pointed wrong direction**
   - Check joint origin: `<origin xyz="..." rpy="...">`
   - rpy should point forward (0, 0, 0) or adjusted for actual mounting
   - Fix: Adjust joint orientation

2. **Far clip plane too close**
   - Check: `<far>1</far>` (only sees 1 meter)
   - Scene is beyond 1 meter
   - Fix: Increase: `<far>100</far>`

3. **Near clip plane too far**
   - Check: `<near>5</near>` (doesn't see anything closer than 5 meters)
   - Everything in view is closer
   - Fix: Decrease: `<near>0.1</near>`

4. **Scene has no lighting**
   - Gazebo scene is dark (no sun, no lights)
   - Fix: Add lighting in Gazebo GUI (click Light icon)

5. **Camera model incorrect**
   - Sensor type is `camera_depth` instead of `camera`
   - Fix: Use `<sensor type="camera">`

**Diagnosis workflow:**
```
Camera all black?
├─ Check topic publishes (step 1-2)
├─ Check image data size (should be 640*480*3 for RGB)
├─ Inspect in Gazebo GUI
│  ├─ Is camera visibly positioned? (can see it in 3D view)
│  ├─ Is it pointing at scene? (rotate with mouse, see if view changes)
│  └─ Check near/far clip planes in camera properties
└─ If nothing visible, likely clip plane or pointing issue
```

### Problem 2: LIDAR Range Very Short

**Symptoms:**
```
LIDAR publishes /lidar/scan
But ranges are all below 1 meter (max_range = 10, but nothing farther detected)
```

**Possible causes:**
1. **Max range too low**
   - Check: `<max>1</max>`
   - Should be 10 or more for most scenes
   - Fix: Increase: `<max>50</max>`

2. **Min range too high**
   - Check: `<min>5</min>`
   - Can't detect anything closer than 5 meters
   - Fix: Decrease: `<min>0.05</min>`

3. **Noise too high**
   - Gaussian noise is hiding distant points
   - Fix: Reduce stddev: `<stddev>0.001</stddev>`

4. **Scene has no walls/obstacles**
   - LIDAR works, but empty space returns no hits
   - Normal behavior (sparse cloud)
   - Fix: Add obstacles to scene

5. **LIDAR GPU acceleration failing**
   - Sensor type should be `gpu_lidar`
   - If GPU support unavailable, falls back to software rendering (slow)
   - Fix: Use `libgazebo_ros_lidar.so` plugin instead of `libgazebo_ros_gpu_lidar.so`

**Diagnosis workflow:**
```
LIDAR range too short?
├─ Inspect /lidar/scan data (step 3)
│  ├─ ranges: [1.0, 0.95, 1.1, ...] = all close? → clip plane issue
│  └─ ranges: [inf, inf, inf, ...] = all out of range? → max_range too low
├─ Check scene visualization
│  └─ Can you see walls/obstacles when rendered in Gazebo?
└─ If empty space, this is correct behavior (sparse cloud)
```

### Problem 3: IMU Not Updating

**Symptoms:**
```
IMU publishes /imu topic
But acceleration always (0, 0, 0)
Or orientation never changes even when robot moves
```

**Possible causes:**
1. **Sensor not on updating link**
   - IMU defined on static link
   - Fix: Move IMU sensor to moving link (base_link, not world)

2. **Sensor gravity not being applied**
   - IMU reads (0, 0, 0) when stationary (wrong!)
   - Should read (0, 0, 9.81)
   - Fix: Check if gravity is enabled in physics: `<gravity>1</gravity>`

3. **Update rate zero**
   - Check: `<update_rate>0</update_rate>`
   - Fix: Set to non-zero: `<update_rate>100</update_rate>`

4. **Frame reference wrong**
   - Check plugin: `<frame_name>base_link</frame_name>`
   - If link doesn't exist, no updates
   - Fix: Verify link name matches actual robot link

**Diagnosis workflow:**
```
IMU not updating?
├─ Check /imu topic frequency (step 2)
│  └─ 0 Hz? → update_rate is 0 or sensor not on moving link
├─ Inspect data (step 3)
│  ├─ All zeros? → gravity not applied, or sensor on static link
│  └─ Same value every frame? → sensor frozen, frame reference wrong
└─ Move robot and check if acceleration changes
```

---

## Visualization Tools

### Tool 1: ROS 2 Topic Commands

**Monitor data in real-time:**
```bash
# Continuous echo (press Ctrl+C to stop)
ros2 topic echo /camera/image_raw

# Show one message then exit
ros2 topic echo /camera/image_raw --once

# Show message rate
ros2 topic bw /camera/image_raw

# Show data type
ros2 topic info /camera/image_raw
```

### Tool 2: Gazebo GUI Visualization

**Built-in camera display:**
1. Gazebo window → Plugins → Camera
2. Select camera name (e.g., "camera")
3. Live video feed opens

**Point cloud visualization:**
1. Open RViz: `rviz2`
2. Click "Add" → "PointCloud2"
3. Set topic to `/lidar/points`
4. See 3D point cloud in 3D view

### Tool 3: RViz (ROS Visualization)

**Launch RViz:**
```bash
rviz2
```

**Add displays:**
1. Click "Add" button (bottom-left)
2. Choose display type:
   - **Image**: Shows camera feed
   - **PointCloud2**: Shows LIDAR point cloud
   - **Axes**: Shows robot orientation
   - **Plot**: Shows time-series data (acceleration, velocity)

**Configure displays:**
- Topic: Select `/camera/image_raw`, `/lidar/points`, etc.
- Size: Adjust point size for visibility
- Color scheme: Choose how to color points

---

## AI-Assisted Debugging

When standard debugging doesn't work, collaborate with an AI assistant. Describe the symptom clearly, and AI can suggest causes and fixes.

### Describing Sensor Problems Effectively

**Bad description:**
```
"My LIDAR isn't working"
```

**Good description:**
```
"My LIDAR is publishing /lidar/scan at 10 Hz.
Robot is in a 10-meter hallway with walls on left and right.
ranges[] shows all values between 8-10 meters.
But the walls are 2 meters away, not 9 meters.
Max range in SDF is <max>10</max>.
What's wrong?"
```

**Why the second is better:**
- Specifies what's publishing (topic, frequency)
- Describes environment (hallway, wall distance)
- Shows actual data (ranges 8-10 meters)
- Shows configuration (max_range = 10)
- Makes the problem clear (expected 2m, got 9m)

### Working with AI to Debug

**You present:**
1. What you expected to happen
2. What actually happened
3. Your configuration (relevant SDF snippet)
4. Your data (actual sensor output)

**AI provides:**
1. Likely cause(s)
2. How to test each cause
3. Fixes to try
4. Why each fix matters

**You iterate:**
1. Try one fix
2. Report results
3. AI refines diagnosis
4. Continue until fixed

---

## Exercise: Debug a Sensor Problem

In this exercise, you'll identify and fix sensor configuration issues using the debugging workflow.

**Broken robot SDF (deliberately configured wrong):**
```xml
<model name="broken_robot">
  <link name="base_link">
    <!-- Body -->
  </link>

  <!-- Camera pointed backwards (rpy="0 0 3.14159") -->
  <link name="camera_link">
    <sensor name="camera" type="camera">
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image><width>640</width><height>480</height></image>
        <clip><near>10</near><far>11</far></clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <camera_name>camera</camera_name>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </link>
  <joint name="camera_joint" type="fixed">
    <parent>base_link</parent>
    <child>camera_link</child>
    <!-- Points backward! -->
    <origin xyz="-0.1 0 0" rpy="0 0 3.14159"/>
  </joint>

  <!-- LIDAR with impossibly short range -->
  <link name="lidar_link">
    <sensor name="lidar" type="gpu_lidar">
      <lidar>
        <scan>
          <horizontal>
            <samples>180</samples>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <!-- Only sees 0.5 meters! -->
        <range><min>0.05</min><max>0.5</max></range>
      </lidar>
      <always_on>1</always_on>
      <update_rate>10</update_rate>
      <plugin name="lidar_controller" filename="libgazebo_ros_gpu_lidar.so">
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent>base_link</parent>
    <child>lidar_link</child>
    <origin xyz="0 0 0.2"/>
  </joint>
</model>
```

**Your task:**
1. Save this as `broken_robot.sdf`
2. Launch Gazebo with the robot
3. Check what each sensor publishes (topics exist? data present?)
4. Visualize in Gazebo GUI or RViz
5. Identify what's wrong (use the debugging workflow from earlier)
6. Fix each problem in the SDF
7. Verify fixes work (topics publish correct data)

**Problems to find and fix:**
- Camera pointed backward (rpy should be 0, not 3.14159)
- LIDAR range impossibly short (0.5 meters is too short for hallway)
- Camera near clip plane too large (near=10, far=11 creates 1-meter-thick visibility band)

**Expected result**: All sensors publish correct data, visualizations show meaningful output.

---

## Validation Checklist

After debugging sensors:

- [ ] All sensors have topics that publish at expected frequency
- [ ] Visual inspection (Gazebo GUI or RViz) shows meaningful data
- [ ] Data matches expectations (camera shows scene, LIDAR sees walls at correct distance, IMU shows gravity)
- [ ] No error messages in Gazebo console
- [ ] Sensor configurations make physical sense (near/far clip planes, orientation, mounting)

---

## Try With AI

**Setup**: Describe a sensor problem you've encountered or imagine, then work with AI to diagnose and fix it.

**Prompt Set:**

```
Prompt 1 (Basic Diagnosis): "My camera is publishing images but
they're all completely black. The robot is indoors, Gazebo is running,
the topic /camera/image_raw shows 640x480 images. What should I check?"

Prompt 2 (Intermediate Collaboration): "I'm debugging LIDAR. The
sensor publishes /lidar/scan at 10 Hz. When I echo the data, ranges
show [2.5, 2.5, 2.5, ...] in all directions. The robot is in the
center of a large room. Why would LIDAR detect everything at exactly
2.5 meters? How would I fix this?"

Prompt 3 (Advanced Problem-Solving): "I have a humanoid with two
cameras (wide and narrow angle), one LIDAR, and an IMU. All are
publishing topics, but RViz crashes when I try to visualize everything
together. The system becomes slow. How would you recommend I debug
which sensor is the bottleneck? What's the best visualization strategy?"
```

**Expected Outcomes:**
- AI helps systematically narrow down root causes
- AI suggests testable hypotheses (camera orientation, range limits, etc.)
- AI recommends visualization strategies based on sensor type

**Safety Note**: Sensor debugging in simulation is controlled and safe. When debugging real robot sensors, take physical safety precautions. A malfunctioning camera is harmless, but a malfunctioning motor/gripper could injure someone. Always have an emergency stop button, clear the workspace, and test incrementally.

**Optional Stretch**: Build a sensor diagnostics dashboard that shows all sensor topics, their frequencies, and last message timestamps. This helps quickly identify which sensors are healthy and which are broken.
