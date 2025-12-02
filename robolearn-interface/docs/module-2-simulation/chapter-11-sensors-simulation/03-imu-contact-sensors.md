---
id: lesson-11-3-imu-contact-sensors
title: "Lesson 11.3: IMU and Contact Sensors"
sidebar_position: 3
sidebar_label: "11.3 IMU & Contact"
description: "Adding inertial measurement and touch sensors to robots"
duration_minutes: 60
proficiency_level: "B1"
layer: "L1"
hardware_tier: 1
learning_objectives:
  - "Configure IMU sensors for orientation and acceleration data"
  - "Add contact sensors to detect collisions"
  - "Understand noise models for realistic sensor behavior"
  - "Interpret IMU data streams (acceleration, angular velocity, orientation)"
skills:
  - "sensor-simulation"
  - "imu-config"
cognitive_load:
  new_concepts: 6
tier_1_path: "TheConstruct cloud environment"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 11.3: IMU and Contact Sensors

## Why IMU and Touch Matter

**Inertial Measurement Units (IMUs)** measure the robot's own motion:
- Linear acceleration (is the robot accelerating forward?)
- Angular velocity (is the robot rotating?)
- Orientation (which way is the robot facing?)

IMUs are essential for:
- **Humanoid walking**: Detecting balance and adjusting leg position
- **Inertial navigation**: Dead reckoning when GPS fails
- **Fall detection**: Humanoids know when they're falling

**Contact sensors** detect when the robot touches something:
- **Feet sensors**: Humanoid knows when foot is on ground
- **Gripper sensors**: Manipulator knows when object is grasped
- **Bumpers**: Mobile robot detects collisions

Together, these sensors give robots proprioception (awareness of their own body) and exteroception (awareness of contacts).

---

## Understanding IMU Data

### The Three Components of IMU

An IMU typically contains three sensors fused into one package:

**1. Accelerometer** (measures linear acceleration in x, y, z)
```
Accelerometer output: (ax, ay, az) in m/s²

Stationary on a table:
  ax = 0, ay = 0, az = 9.81  (gravity pulls down!)

Accelerating forward:
  ax = 5.0, ay = 0, az = 9.81  (forward movement + gravity)

Jumping:
  ax = 0, ay = 0, az = 20  (upward acceleration + gravity)
```

**Important**: Accelerometer always measures gravity! Even a stationary sensor reads 9.81 m/s² downward (the acceleration you'd need to cancel gravity).

**2. Gyroscope** (measures angular velocity in roll, pitch, yaw)
```
Gyroscope output: (ωx, ωy, ωz) in rad/s

Stationary:
  ωx = 0, ωy = 0, ωz = 0  (no rotation)

Spinning around z-axis (yaw) at 1 rotation/second:
  ωx = 0, ωy = 0, ωz = 6.28  (2π rad/sec for full rotation)
```

**3. Magnetometer** (measures Earth's magnetic field, acts like compass)
```
Magnetometer output: (mx, my, mz)

Facing north:
  mx = large value, my = small, mz = small

Facing east:
  mx = small, my = large value, mz = small
```

**3-axis orientation** (derived from all three sensors):
```
Orientation: (roll, pitch, yaw)
- roll: rotation around x-axis (left/right tilt)
- pitch: rotation around y-axis (forward/backward tilt)
- yaw: rotation around z-axis (rotation around vertical)
```

### Noise in IMU Data

Real IMUs have errors:

**Bias**: Sensor reads non-zero when it should read zero
```
Gyroscope sitting still reports: (0.01, -0.02, 0.015) rad/s
(tiny drift, but compounds over time)
```

**Noise**: Random fluctuation around true value
```
Accelerometer reads: 9.81, 9.82, 9.79, 9.81, 9.80, ...
(varies by 0.01 m/s² randomly)
```

**Drift**: Errors accumulate over time
```
Integrating gyroscope velocity → orientation
Over 1 minute: small errors accumulate
Over 10 minutes: orientation estimate becomes unreliable
```

In simulation, you can add realistic noise models to make training data match real sensor behavior.

---

## Adding an IMU to Your Robot

### Step 1: Define the IMU Sensor in SDF

Add an `<sensor>` element of type `imu` to your robot's main link (usually `base_link`).

**Example: IMU on robot base**

```xml
<model name="my_robot">
  <link name="base_link">
    <!-- Robot body... -->

    <!-- IMU SENSOR DEFINITION -->
    <sensor name="imu" type="imu">
      <!-- IMU parameters -->
      <imu>
        <!-- Accelerometer noise -->
        <acceleration_xyzn>
          <x>
            <noise type="gaussian">
              <mean>0</mean>
              <stddev>0.05</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0</mean>
              <stddev>0.05</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0</mean>
              <stddev>0.05</stddev>
            </noise>
          </z>
        </acceleration_xyzn>

        <!-- Gyroscope noise -->
        <angular_velocity_xyzn>
          <x>
            <noise type="gaussian">
              <mean>0</mean>
              <stddev>0.01</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0</mean>
              <stddev>0.01</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0</mean>
              <stddev>0.01</stddev>
            </noise>
          </z>
        </angular_velocity_xyzn>
      </imu>

      <!-- Publishing behavior -->
      <always_on>1</always_on>
      <update_rate>100</update_rate>

      <!-- Topic name -->
      <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
        <frame_name>base_link</frame_name>
      </plugin>
    </sensor>
  </link>
</model>
```

**What's happening:**
- `<sensor type="imu">` defines this as an inertial measurement unit
- `<acceleration_xyzn>` defines noise for accelerometer (x, y, z axes)
- `<angular_velocity_xyzn>` defines noise for gyroscope
- `<stddev>0.05</stddev>` standard deviation of Gaussian noise
- `<update_rate>100</update_rate>` publishes 100 times per second (high frequency)

### Step 2: Verify IMU Publishes

**Check IMU topics:**
```bash
ros2 topic list | grep imu
```

**Expected output:**
```
/imu
```

**Inspect IMU data:**
```bash
ros2 topic echo /imu --once
```

**Output shows:**
```
header:
  seq: 1234
  stamp: {sec: 10, nsec: 500000000}
  frame_id: base_link
orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}  # Quaternion (rotated 90 deg)
orientation_covariance: [0.01, ...]  # Uncertainty in orientation
angular_velocity: {x: 0.0, y: 0.0, z: 0.1}  # Spinning at 0.1 rad/s
angular_velocity_covariance: [0.001, ...]
linear_acceleration: {x: 0.1, y: 0.0, z: 9.81}  # Gravity in z
linear_acceleration_covariance: [0.0025, ...]
```

**Interpretation:**
- `orientation`: Robot's pose as quaternion (encodes roll, pitch, yaw)
- `angular_velocity`: How fast robot is rotating
- `linear_acceleration`: Robot's acceleration plus gravity

---

## Adding Contact Sensors

Contact sensors detect when surfaces touch each other (collisions, grasping, foot contact).

### Example: Contact Sensor on Robot Bumper

```xml
<model name="my_robot">
  <link name="base_link">
    <!-- Robot body... -->

    <!-- Create a bumper link (collision detection area) -->
    <collision name="bumper_collision">
      <geometry>
        <box>
          <size>0.3 0.1 0.05</size>
        </box>
      </geometry>
    </collision>
  </link>

  <link name="bumper_link">
    <!-- Bumper sensor (just a collision shape) -->
    <inertial>
      <mass>0.01</mass>
    </inertial>
    <collision name="bumper_collision">
      <geometry>
        <box>
          <size>0.3 0.1 0.05</size>
        </box>
      </geometry>
    </collision>
  </link>

  <!-- Joint to mount bumper -->
  <joint name="bumper_joint" type="fixed">
    <parent>base_link</parent>
    <child>bumper_link</child>
    <origin xyz="0.15 0 0"/>  <!-- Front of robot -->
  </joint>

  <!-- Contact sensor definition -->
  <sensor name="bumper_sensor" type="contact">
    <contact>
      <collision>bumper_collision</collision>
    </contact>
    <plugin name="bumper_controller" filename="libgazebo_ros_bumper.so">
      <bumper_topic_name>bumper</bumper_topic_name>
    </plugin>
  </sensor>
</model>
```

**What's happening:**
- `bumper_link` is a separate link with collision shape
- `<sensor type="contact">` monitors this collision shape
- When robot bumps into something, the sensor publishes a contact event
- Plugin translates event into ROS 2 message

### Contact Sensor Topics

**Contact events publish to:**
```
/bumper
```

**Message contains:**
```
contacts:
  - contact1:
    collision1_name: bumper_collision
    collision2_name: ground_plane::ground_plane_collision
    position: [x, y, z]  # Where collision occurred
```

---

## Exercise: Add IMU to Your Robot

**Task:**
1. Use your robot SDF from previous lessons
2. Add an IMU sensor to the base_link
3. Configure accelerometer and gyroscope noise
4. Launch Gazebo with your robot
5. Verify `/imu` topic publishes
6. Move your robot around and observe IMU values changing
7. Stop your robot and verify acceleration still reads ~9.81 in z (gravity)

**Expected result**: You see realistic IMU data reflecting your robot's motion and orientation.

---

## Understanding Quaternions (Advanced Orientation Representation)

IMU data includes orientation as a **quaternion**: (x, y, z, w)

Quaternions represent rotations in 3D without gimbal lock (a limitation of Euler angles).

**Stationary robot (no rotation):**
```
Quaternion: (0, 0, 0, 1)
Euler angles: (0, 0, 0) = roll 0°, pitch 0°, yaw 0°
```

**Robot rotated 90 degrees around z-axis:**
```
Quaternion: (0, 0, 0.707, 0.707)  ≈ sin(45°), cos(45°)
Euler angles: (0, 0, 90°) = yaw 90°
```

**Conversion** (typically handled by libraries):
```python
import math
from tf_transformations import quaternion_from_euler

roll = 0
pitch = 0
yaw = math.radians(90)  # 90 degrees in radians

quat = quaternion_from_euler(roll, pitch, yaw)
# Result: (0, 0, 0.707, 0.707)
```

For most practical purposes, you don't need to understand quaternion math—ROS libraries handle conversion. Just know that orientation is represented this way.

---

## Common IMU and Contact Issues

### Issue 1: IMU Publishes Very Noisy Data

**Symptom**: IMU acceleration/rotation values fluctuate wildly

**Likely cause:**
- Noise stddev too high
- Sensor frequency too low (update_rate too low)

**Fix:**
1. Increase update_rate: `<update_rate>200</update_rate>`
2. Reduce noise: `<stddev>0.01</stddev>` (down from 0.05)
3. Post-process in your node (moving average filter)

### Issue 2: Contact Sensor Never Triggers

**Symptom**: Robot bumps into wall but no contact message

**Likely cause:**
- Collision shape not defined correctly
- Contact sensor not monitoring right collision geometry
- Collision groups not set to detect

**Fix:**
1. Verify collision shape exists in SDF
2. Double-check collision name in sensor definition
3. Enable collision detection in physics engine

### Issue 3: Gravity Constantly Affects Accelerometer

**Symptom**: IMU always reads 9.81 in z, making it hard to detect real acceleration

**This is expected!** Real IMUs have this problem too.

**Solution:**
- Subtract gravity (9.81) from z-axis in your processing node
- Use sensor fusion: combine gyroscope (which has no gravity) with accelerometer
- Many robotics libraries provide "IMU fusion" that removes gravity automatically

---

## Validation Checklist

After adding IMU and contact sensors:

- [ ] IMU sensor is defined in SDF
- [ ] Gazebo launches without errors
- [ ] `ros2 topic list` shows `/imu` and `/bumper` (if contact added)
- [ ] `ros2 topic echo /imu` shows changing values as robot moves
- [ ] Orientation quaternion values make sense (one value should be close to 1)
- [ ] Acceleration reads ~9.81 in z-axis when stationary
- [ ] Contact sensor triggers when robot collides (if contact added)

---

## Try With AI

**Setup**: Describe an IMU configuration problem to your AI assistant.

**Prompt Set:**

```
Prompt 1 (Basic): "I added an IMU to my robot. It's on the base_link.
What noise levels (stddev) would you recommend for a robot that
walks on uneven terrain? Should accelerometer noise be higher or
lower than gyroscope noise?"

Prompt 2 (Intermediate): "My robot has a humanoid that falls frequently.
I want the IMU to detect falling (sudden acceleration change).
What IMU update rate would you recommend? 10 Hz, 50 Hz, or 200 Hz?"

Prompt 3 (Advanced): "I'm fusing IMU data (accelerometer + gyroscope +
magnetometer) to estimate robot orientation. Magnetometer drifts in
indoor environments with metal. Should I rely more on accelerometer
or gyroscope for orientation estimation?"
```

**Expected Outcomes:**
- AI suggests appropriate noise levels for realistic simulation
- AI explains tradeoffs between update rate and processing load
- AI helps reason about sensor fusion strategies

**Safety Note**: IMU sensors in simulation are ideal (perfect noise models, no bias drift). Real IMUs have temperature-dependent bias, shock-induced errors, and environmental sensitivity. When deploying real robots, calibrate IMUs in the actual operating environment and validate your sensor fusion algorithms with real hardware.

**Optional Stretch**: Write a ROS 2 node that listens to `/imu`, removes gravity from the accelerometer, and detects sudden impacts (acceleration magnitude greater than 20 m/s²). This is the basis for fall detection in humanoid robots.
