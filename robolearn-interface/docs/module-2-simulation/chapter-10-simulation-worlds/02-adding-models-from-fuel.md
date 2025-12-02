---
id: lesson-10-2-adding-models-from-fuel
title: "Lesson 10.2: Adding Models from Fuel"
sidebar_position: 2
sidebar_label: "10.2 Models from Fuel"
description: "Using Gazebo Fuel to populate worlds with pre-built 3D models, positioning, and scaling."
duration_minutes: 60
proficiency_level: "A2"
layer: "L1"
hardware_tier: 1
learning_objectives:
  - "Navigate and search the Gazebo Fuel model repository"
  - "Include Fuel models in SDF world files using proper URI syntax"
  - "Position and scale imported models correctly in simulation space"
  - "Compose complete worlds by combining multiple Fuel models"
skills:
  - "gazebo-fuel"
  - "world-composition"
cognitive_load:
  new_concepts: 5
tier_1_path: "The Construct cloud environment + web browser"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 10.2: Adding Models from Fuel

Creating every 3D object from scratch (geometry, physics, appearance) would take days for a single room. Instead, **Gazebo Fuel** (https://app.gazebosim.org/fuel) is an online repository of thousands of pre-built 3D models contributed by the robotics community.

Gazebo Fuel contains tables, chairs, buildings, vegetation, traffic cones, robots, and more—all free to use. This lesson teaches you to find, include, and position these models in your SDF worlds.

## What is Gazebo Fuel?

Gazebo Fuel is a public repository of 3D models optimized for robotics simulation. Models are organized by category, quality, and licensing. Most models are free and open-source.

**Key features:**
- Searchable by name, category, or tag
- Models include physics properties, collision shapes, and visual meshes
- Each model has a unique URI (web address) for inclusion in SDF files
- Community-contributed: thousands of models from roboticists worldwide
- Quality varies: some are production-grade, others are educational

**Access**: https://app.gazebosim.org/fuel

## Finding Models in Gazebo Fuel

Navigate to https://app.gazebosim.org/fuel/models. You'll see a grid of available models.

**To search for a specific model:**
1. Use the search box (top right)
2. Type the model name (e.g., "table", "chair", "lamp")
3. Click on a result to view details

**Example search results:**

Search for "table":
- "Table" by OpenRobotics
- "Standing Desk" by user123
- "Picnic Table" by OpenRobotics
- "Conference Table" by user456

Each has a thumbnail (preview image), creator name, and download count.

**To find a specific model's URI:**
1. Click on the model
2. Look for a "URI" field or copy button
3. The URI format is: `https://fuel.gazebosim.org/1.0/[user]/models/[model-name]`

**Example URIs:**

Standard table:
```
https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master
```

Office chair:
```
https://fuel.gazebosim.org/1.0/openrobotics/models/chair/tip/master
```

## Including Fuel Models in SDF

Once you have a model URI, including it in your SDF world is simple. Use the `<include>` tag:

```xml
<include>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master</uri>
  <pose>0 0 0 0 0 0</pose>
  <name>my_table</name>
</include>
```

**Breaking down the include tag:**

- **uri**: The full web address of the Fuel model
- **pose**: Position and rotation (x y z roll pitch yaw)
- **name**: A unique name for this instance (you can include the same model multiple times with different names)

### Pose: Position and Rotation

The pose tag uses 6 values: **x y z roll pitch yaw**

**Position (first 3 values: x y z)**
- x: forward/backward (meters)
- y: left/right (meters)
- z: up/down (meters)

Example: `<pose>1 2 0.8 0 0 0</pose>` places the model 1m forward, 2m right, 0.8m up, with no rotation.

**Rotation (last 3 values: roll pitch yaw in radians)**
- roll: rotation around x-axis (0 to 2π ≈ 0 to 6.28)
- pitch: rotation around y-axis
- yaw: rotation around z-axis (rotation about vertical axis)

For now, keep rotations at 0. Rotations are measured in radians:
- 0 radians = no rotation
- π/2 radians ≈ 1.57 ≈ 90 degrees
- π radians ≈ 3.14 ≈ 180 degrees
- 2π radians ≈ 6.28 ≈ 360 degrees

Example with rotation:
```xml
<pose>2 0 0.8 0 0 1.57</pose>
```
This positions the model 2m forward, 0.8m up, and rotates it 90 degrees around the vertical axis (yaw).

## Example: Building a Simple Room

Let's create a room with a table and two chairs using Fuel models. Here's the complete SDF:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="furnished_room">

    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Table from Gazebo Fuel -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master</uri>
      <pose>0 0 0 0 0 0</pose>
      <name>dining_table</name>
    </include>

    <!-- Chair 1 (left side of table) -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/chair/tip/master</uri>
      <pose>-0.5 0 0 0 0 0</pose>
      <name>chair_left</name>
    </include>

    <!-- Chair 2 (right side of table) -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/chair/tip/master</uri>
      <pose>0.5 0 0 0 0 3.14</pose>
      <name>chair_right</name>
    </include>

  </world>
</sdf>
```

**What this creates:**
- A table at the origin (0, 0, 0)
- A chair 0.5m to the left of the table
- A chair 0.5m to the right of the table, rotated 180 degrees (so the person faces the other direction)

**Output**: In Gazebo, you'll see a furnished scene: a table with two chairs facing each other.

## Scaling Models

Some models may be too large or too small for your world. You can scale them using the `<scale>` tag:

```xml
<include>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master</uri>
  <pose>0 0 0 0 0 0</pose>
  <name>small_table</name>
  <scale>0.5 0.5 0.5</scale>
</include>
```

The scale values multiply the model's size:
- `1.0` = original size
- `0.5` = half size
- `2.0` = double size

You can also scale non-uniformly (different scales on different axes):
```xml
<scale>1.0 1.0 2.0</scale>
```
This scales the object to double height while keeping width and depth the same.

**When to scale:**
- Model is too big for your environment? Scale it down (0.5 to 0.8)
- Model is too small? Scale it up (1.5 to 2.0)
- Most models in Gazebo Fuel are already sized for human environments, so scaling is often unnecessary

## Understanding Model URIs

Every Fuel model has a unique URI. Let's break down the URI structure:

```
https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master
```

- `https://fuel.gazebosim.org/1.0/` - Base Gazebo Fuel domain
- `openrobotics` - Model owner/creator
- `models` - Type of asset (could be "worlds" for complete worlds)
- `table` - Model name
- `tip/master` - Version (usually "tip/master" for the latest)

**Alternative format** (without the /1.0/ prefix):
```
model://openrobotics/table
```
This is shorthand. Both forms work in SDF files.

## Practical Tips for Fuel Models

### Finding Models Effectively

**By category**: Browse https://app.gazebosim.org/fuel and filter by type:
- Furniture (tables, chairs, desks)
- Vegetation (trees, grass, flowers)
- Buildings (walls, doors, windows)
- Vehicles (cars, trucks)
- Objects (cones, boxes, balls)

**By reputation**: Look for models with high download counts. Popular models are usually well-made and tested.

**By creator**: OpenRobotics is the official Gazebo team. Their models are reliable.

### Checking Model Quality

Before including a model:
1. Download its preview image (check it looks right)
2. Look at the description (see if it has physics properties configured)
3. Check comments/reviews if available (users might report issues)
4. Test it in a simple world before using in complex scenarios

### Dealing with Large Models

Some Fuel models (especially detailed buildings or vegetation) have complex meshes. Large models can slow down simulation. If performance is an issue:
1. Use simpler models (search for "simple", "lowpoly", or "low-poly" variants)
2. Remove shadows from heavy models
3. Reduce the physics complexity (use "static" models where possible)

## Building a Complete World

Let's combine everything: ground plane, physics, lighting, and multiple Fuel models.

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="busy_room">

    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Work area -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master</uri>
      <pose>0 0 0 0 0 0</pose>
      <name>work_table</name>
    </include>

    <!-- Desk chair -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/chair/tip/master</uri>
      <pose>0 -0.5 0 0 0 0</pose>
      <name>desk_chair</name>
    </include>

    <!-- Decorative plant in corner -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/potted_plant/tip/master</uri>
      <pose>2 2 0 0 0 0</pose>
      <name>plant_corner</name>
    </include>

  </world>
</sdf>
```

**What's happening here:**
1. Ground and lighting (foundation)
2. Work table in the center
3. Chair positioned south of the table
4. Plant in the northeast corner

You now have a furnished environment ready for a robot to navigate through.

## Try With AI

**Setup**: Have a text editor open and be ready to write SDF include statements. Access the AI tool of your choice.

**Prompt 1** (Discovery):
```
I want to build a Gazebo world with:
1. A dining area (table + chairs)
2. A corner with a plant
3. A storage shelf unit

What Fuel models should I search for?
For each element, suggest a model name and describe what pose values I might use.
```

**Expected outcome**: AI suggests relevant Fuel models and gives approximate positioning guidance.

**Prompt 2** (Syntax):
```
I want to include this Fuel model in my SDF world:
https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master

Write the XML include statement to place this table:
- At position (2, 3, 0)
- With no rotation
- Named "conference_table"
```

**Expected outcome**: AI writes the correct XML include block.

**Prompt 3** (Debugging):
```
I included this model in my SDF world:
<include>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master</uri>
  <pose>1 2 0 0 0 0</pose>
  <name>table1</name>
</include>

When I open it in Gazebo, the table is floating in mid-air instead of resting on the ground.
What's likely wrong, and how should I fix the pose values?
```

**Expected outcome**: AI explains that the table's model origin might be above its base, and suggests trying a negative Z value or checking the model's documentation.

