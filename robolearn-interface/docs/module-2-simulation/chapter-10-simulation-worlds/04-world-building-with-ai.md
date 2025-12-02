---
id: lesson-10-4-world-building-with-ai
title: "Lesson 10.4: World Building with AI"
sidebar_position: 4
sidebar_label: "10.4 World Building with AI"
description: "Accelerating simulation world creation through AI collaboration, specification-first thinking, and iterative refinement."
duration_minutes: 60
proficiency_level: "B1"
layer: "L2"
hardware_tier: 1
learning_objectives:
  - "Describe desired simulation worlds in natural language"
  - "Evaluate AI-generated SDF for correctness and completeness"
  - "Iterate with AI to refine world configurations toward production quality"
  - "Apply specification-first thinking to accelerate world design"
  - "Validate AI outputs against simulation behavior"
skills:
  - "world-building"
  - "ai-collaboration"
  - "specification-first-design"
cognitive_load:
  new_concepts: 5
tier_1_path: "The Construct cloud environment + AI assistant (ChatGPT, Claude, or similar)"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 10.4: World Building with AI

You now understand SDF structure, Gazebo Fuel models, and physics configuration. You could manually create worlds by hand—but that's slow. An experienced SDF writer might spend 2-3 hours building a detailed world.

In this lesson, you'll accelerate world building through AI collaboration. Instead of writing SDF by hand, you'll:
1. Describe your desired world in plain English
2. Have AI generate the SDF
3. Evaluate the output for correctness
4. Iterate with AI to refine the world

This approach, called **specification-first thinking**, is how professional developers work with AI. You don't ask "write code"—you ask "build a world where X happens" and let AI handle the implementation details.

## The Specification-First Approach

**Old approach (manual)**:
- Decide what you want: "A warehouse with shelves and boxes"
- Manually write SDF by hand, 200 lines
- Test in Gazebo, debug issues, rewrite sections
- Total time: 2-3 hours

**New approach (specification-first with AI)**:
- Describe what you want in natural language
- AI writes the SDF (200 lines, mostly correct)
- Test in Gazebo, identify any issues
- Ask AI to fix specific issues
- Total time: 30-45 minutes

The key insight: **Specifying intent clearly saves more time than writing code manually.**

## Crafting Clear Specifications for AI

When asking AI to generate a world, clarity matters. Vague requests produce mediocre outputs. Specific, detailed requests produce better results.

### Example 1: Vague Specification

**Your request**:
```
Create a Gazebo world with a robot workspace.
```

**AI output**: Generic world with a table and nothing else. Not useful.

**Why it failed**: Too vague. What kind of robot? What should the workspace contain? What size?

### Example 2: Clear Specification

**Your request**:
```
Create a Gazebo SDF world for a mobile manipulation robot (like a TurtleBot with a gripper arm).
The world should include:
1. A ground plane with realistic friction (concrete)
2. A sturdy work table (1m x 1m x 0.8m height) in the center
3. On the table: 3 small objects (boxes, ~10cm cubes) that the robot can grasp
4. Around the table: free space for the robot to navigate
5. Physics: DART engine, 1ms step size, Earth gravity
6. Lighting: bright directional light (sun) with shadows

The robot needs to navigate to the table, identify objects, and manipulate them.
```

**AI output**: Detailed SDF world with all requested elements correctly placed and configured.

**Why it worked**: Specific intent, clear requirements, technical constraints included.

## Interactive World Building: A Live Example

Let's walk through a realistic scenario of building a world with AI feedback.

### Step 1: Initial Request

You want a simple office environment where a mobile robot can deliver items. Here's your specification:

```
I need a Gazebo SDF world for testing an office delivery robot.

Environment description:
- Size: ~10m x 10m open office space
- Ground: concrete (friction coefficient ~0.7)
- Features:
  1. A standing desk (2m x 1m, ~1.1m tall) in one corner
  2. A sofa (2m x 1m, ~0.5m tall) in another corner
  3. A small table (1m x 1m, ~0.5m tall) in the center
  4. 2-3 boxes/items on surfaces for delivery
- Physics: DART engine, 1ms steps, Earth gravity
- Lighting: bright sun, shadows enabled
- The robot needs clear pathways to navigate and deliver

Please generate the complete SDF file.
```

### Step 2: Receive AI Output

AI generates something like:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="office_delivery">
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
          <surface>
            <friction>
              <ode>
                <mu>0.7</mu>
                <mu2>0.7</mu2>
              </ode>
            </friction>
          </surface>
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
          </material>
        </visual>
      </link>
    </model>
    <!-- Furniture from Gazebo Fuel -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/desk/tip/master</uri>
      <pose>4 4 0 0 0 0</pose>
      <name>standing_desk</name>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/sofa/tip/master</uri>
      <pose>-4 -4 0 0 0 0</pose>
      <name>sofa</name>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/table/tip/master</uri>
      <pose>0 0 0 0 0 0</pose>
      <name>center_table</name>
    </include>
  </world>
</sdf>
```

### Step 3: Evaluate and Test

You save this file, open it in Gazebo, and observe:

**What's good:**
- Furniture is positioned in sensible locations
- Physics configuration looks correct
- Lighting and ground plane are present

**What needs improvement:**
- No delivery items on the tables (you wanted objects to deliver)
- World bounds aren't clear (no walls showing the 10m x 10m boundary)
- Furniture might be overlapping with navigation paths (hard to tell)
- No lighting at floor level (hard to see the ground plane)

### Step 4: Refine with Follow-Up Request

You send a follow-up message to AI:

```
Good start! The structure is correct. Now I need to:

1. Add 3 boxes on the center table (for the robot to deliver)
   - Each box ~10cm cubes, at different poses
   - Place them on top of the table surface

2. Add walls to show the 10m x 10m boundary
   - 4 walls, each ~10m long, ~1m tall
   - Gray material, minimal appearance (not distracting)

3. Add a point light at the ceiling (0, 0, 3m) for better floor visibility
   - White light, not too bright

Please modify the SDF to include these additions.
Show me the new include blocks and the light configuration.
```

### Step 5: Receive Refined Output

AI provides additions:

```xml
<!-- Delivery items on center table -->
<include>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/cube_20k/tip/master</uri>
  <pose>-0.2 -0.2 1.0 0 0 0</pose>
  <name>box_1</name>
  <scale>0.5 0.5 0.5</scale>
</include>
<include>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/cube_20k/tip/master</uri>
  <pose>0.2 -0.2 1.0 0 0 0</pose>
  <name>box_2</name>
  <scale>0.5 0.5 0.5</scale>
</include>
<include>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/cube_20k/tip/master</uri>
  <pose>0 0.3 1.0 0 0 0</pose>
  <name>box_3</name>
  <scale>0.5 0.5 0.5</scale>
</include>

<!-- Boundary walls -->
<model name="wall_north">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>10 0.1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>10 0.1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.5 0.5 0.5 1</ambient>
        <diffuse>0.7 0.7 0.7 1</diffuse>
      </material>
    </visual>
  </link>
  <pose>0 5 0.5 0 0 0</pose>
</model>

<!-- Similar for south, east, west walls... -->

<!-- Ceiling light -->
<light type="point" name="ceiling_light">
  <pose>0 0 3 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <attenuation>
    <range>10</range>
    <constant>1</constant>
    <linear>0.1</linear>
    <quadratic>0.01</quadratic>
  </attenuation>
</light>
```

### Step 6: Validate and Finalize

You integrate these additions into your SDF file, test in Gazebo:

**Validation checks:**
- Boxes are positioned on the table surface (z coordinate is correct)
- Walls are at the boundary
- Ceiling light illuminates the ground
- Robot can navigate between walls and furniture
- No objects overlapping or floating

**Result**: Production-ready world in 45 minutes instead of 3 hours.

## Common Mistakes When Using AI for World Building

### Mistake 1: Vague Specifications

**Bad**: "Create a warehouse world"
**Good**: "Create a warehouse world with:
- 5 aisles of shelving (2m tall)
- Shelves spaced 1.5m apart
- Concrete floor with high friction
- Boxes stacked on shelves (varies by aisle)
- Enough space for a 0.5m-wide mobile robot to navigate"

### Mistake 2: Not Specifying Physics

**Bad**: World generated without physics configuration details
**Good**: Explicitly state: "Use DART physics, 1ms step size, Earth gravity, concrete friction (mu=0.7)"

### Mistake 3: Forgetting to Validate

**Bad**: Copying AI output directly without testing
**Good**: Generate SDF → Test in Gazebo → Identify issues → Request fixes

### Mistake 4: Underspecifying Lighting

**Bad**: AI defaults to minimal lighting (hard to see details)
**Good**: Specify lighting: "Add directional sun + point ceiling light for full visibility"

## Specification Template for World Generation

Use this template when asking AI to build worlds:

```
I need a Gazebo SDF world for [robot type] to [task description].

Environment:
- Dimensions: [size description]
- Surfaces: [ground material, friction if known]
- Features:
  1. [Feature 1: description, size, location]
  2. [Feature 2: description, size, location]
  3. [Feature N: ...]

Physics:
- Engine: DART (or specify)
- Step size: 0.001s (or specify)
- Gravity: Earth standard (or specify)

Lighting:
- [Light setup description]

Obstacles:
- [Any obstacles robot must navigate around]

Grasping/Manipulation requirements (if applicable):
- [Objects to grasp, surfaces to interact with]

Please generate complete SDF file with:
- Properly positioned furniture from Gazebo Fuel
- Ground plane with correct friction
- All physics and lighting configuration
```

## Iterative Refinement Workflow

**This workflow repeats: Generate → Test → Refine → Test**

1. **Generate**: Ask AI for SDF based on specification
2. **Test**: Open in Gazebo, observe behavior
3. **Identify**: What's wrong? (Missing elements? Incorrect positioning? Physics issues?)
4. **Request**: Ask AI to fix specific issues (not vague "improve it")
5. **Validate**: Test the refined version
6. **Repeat**: Until world matches specification

Each iteration should take 10-15 minutes. After 3-4 iterations, you should have a production-ready world.

## Why This Matters for Sim-to-Real

Simulation accuracy depends on world accuracy. If your simulated world doesn't match the real world, your robot's learned behaviors will fail when deployed.

AI helps you create detailed, accurate worlds quickly. But YOU must validate that:
- Furniture matches real dimensions
- Friction coefficients match real surfaces
- Lighting approximates real conditions
- Physics configuration is realistic

The human (you) is responsible for correctness. AI is responsible for speed.

## Try With AI

**Setup**: Have Gazebo or similar simulator available. Open your AI assistant.

**Prompt 1** (Specification):
```
I'm building a simulation for a mobile robot that needs to navigate a small office and deliver a package to a desk.

The office:
- 5m x 5m space
- Hardwood floor
- One desk in corner (standard office desk)
- One chair at desk
- A sofa in opposite corner
- Clear navigation paths

Generate the Gazebo SDF world file.
Make sure:
- Physics is configured realistically
- Furniture from Gazebo Fuel is positioned correctly
- Ground has realistic friction
- Lighting is bright enough to see details
```

**Expected outcome**: AI generates complete SDF with proper structure, Fuel model includes, and physics configuration.

**Prompt 2** (Validation):
```
Here's the SDF you generated:
[Paste the SDF]

I tested this in Gazebo and notice:
1. The chair is floating in mid-air instead of at the desk
2. The objects are too small to see clearly

What's likely wrong, and what changes should I request for the next version?
```

**Expected outcome**: AI diagnoses issues (Z position for chair, object scaling) and suggests fixes.

**Prompt 3** (Refinement):
```
Please modify the SDF to:
1. Fix the chair Z position so it sits on the floor at the desk
2. Scale all objects 2x larger so they're more visible
3. Add a bright ceiling light so the robot can see the ground clearly

Show me the modified sections.
```

**Expected outcome**: AI provides corrected include statements and light configuration.

**Prompt 4** (Production Readiness):
```
The world is almost ready. One final check:
- Is the friction coefficient (0.7) realistic for hardwood floors?
- Are the physics step size and DART configuration appropriate for a mobile robot?
- Does the world accurately represent a real office environment for testing?

If anything needs adjustment for accuracy, what should it be?
```

**Expected outcome**: AI confirms physics is realistic or suggests tuning for better sim-to-real transfer.

