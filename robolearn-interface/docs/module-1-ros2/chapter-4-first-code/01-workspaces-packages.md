---
id: lesson-4-1-workspaces-packages
title: "Lesson 4.1: Workspaces and Packages"
sidebar_position: 1
sidebar_label: "4.1 Workspaces & Packages"
description: "Create your first ROS 2 workspace and package, understanding the build system structure."
chapter: 4
lesson: 1
duration_minutes: 60
proficiency_level: "A2-B1"
layer: "L1"
cognitive_load:
  new_concepts: 2
learning_objectives:
  - "Create a ROS 2 workspace from scratch"
  - "Understand package.xml purpose and structure"
  - "Build packages using colcon"
  - "Verify workspace environment sourcing"
skills:
  - "ros2-fundamentals"
  - "package-management"
hardware_tier: 1
tier_1_path: "Cloud ROS 2 (TheConstruct) or local installation with fallback to cloud"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Lesson 4.1: Workspaces and Packages

Welcome to your first hands-on ROS 2 experience! Until now, you've explored ROS 2 through the command line — running existing nodes, examining the node graph, calling services. Now it's time to create your own code and organize it properly.

Before you can write a ROS 2 node, you need to understand how ROS 2 projects are structured. Think of a ROS 2 workspace as a folder that contains all your source code, build artifacts, and compiled binaries. Packages within that workspace are individual projects — each one a self-contained module with its own code, configuration, and metadata.

In this lesson, you'll create your first workspace from scratch, understand what lives inside a package, and learn how to build your code with colcon.

---

## Understanding Workspaces

A ROS 2 workspace is simply a directory that follows a specific structure. When you build a workspace, ROS 2 tools know how to find your code, compile it, and set up your environment so you can run it.

### Typical Workspace Structure

When you create a workspace, it has three main directories:

```
ros2_ws/
├── src/                    # Your source code lives here
│   └── my_first_package/
│       ├── my_first_package/
│       ├── setup.py
│       ├── setup.cfg
│       ├── package.xml
│       └── resource/
├── build/                  # Build artifacts (created by colcon)
├── install/                # Compiled binaries and scripts
└── log/                     # Build logs
```

**The critical directories:**
- **src/**: Where you write code. This is the only directory you typically edit by hand.
- **build/**: Temporary build files. You can safely delete this — colcon will recreate it.
- **install/**: The compiled output. This is what you actually run.

### Why This Structure?

This three-directory approach separates concerns:
- **src/** = "what I wrote"
- **build/** = "temporary files while compiling"
- **install/** = "what I can run"

This means you can delete build/ and install/ without losing any work. It also means you can have multiple workspaces on the same machine, each with their own build and install directories.

---

## Creating Your First Workspace

Let's create a workspace. You'll do this once, then add packages inside it repeatedly.

### Step 1: Create the Workspace Structure

Open a terminal and run:

```bash
# Create the workspace directory
mkdir -p ~/ros2_ws/src

# Navigate into it
cd ~/ros2_ws
```

That's it. You've created a workspace. ROS 2 will automatically recognize this folder when you build packages inside src/.

**If you're using cloud ROS 2** (TheConstruct):
- Open your ROS 2 Humble workspace terminal
- Run the same commands above

### Step 2: Verify the Environment

Before you build anything, verify that ROS 2 is properly installed:

```bash
# Check that ros2 command is available
echo $ROS_DISTRO
```

Expected output: `humble`

If you get an error, you may need to source the ROS 2 setup script:

```bash
# For local installation
source /opt/ros/humble/setup.bash

# Verify again
echo $ROS_DISTRO
```

### Step 3: Build Your Workspace (Even Though It's Empty)

Navigate to your workspace and build it:

```bash
cd ~/ros2_ws
colcon build
```

This creates the build/ and install/ directories. Expected output:

```
Starting >>> ros2_ws
Finished <<< ros2_ws [0.05s]

Summary: 0 packages processed
```

You haven't added any packages yet, so it says "0 packages processed." That's normal.

---

## Creating Your First Package

Now let's create a package inside your workspace. A package is ROS 2's unit of organization — one folder containing code, dependencies, and metadata.

### Creating a Package

Use the `ros2 pkg create` command:

```bash
cd ~/ros2_ws
ros2 pkg create my_first_package --build-type ament_python
```

This creates a package named "my_first_package" using Python. Let's break down what just happened:

**Directory created:**
```
src/
└── my_first_package/
    ├── my_first_package/          # Python package directory
    │   └── __init__.py
    ├── test/                       # For unit tests (we'll skip for now)
    ├── setup.py                    # Python package metadata
    ├── setup.cfg                   # Python package config
    ├── package.xml                 # ROS 2 package metadata
    └── resource/
        └── my_first_package
```

The important file is **package.xml** — it tells ROS 2 what this package is and what it depends on.

### Understanding package.xml

Let's look at what was created:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_first_package</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="you@example.com">you</maintainer>
  <license>TODO</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_cmake_pytest</test_depend>

  <exec_depend>ros2launch</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**What each section means:**

| Section | Purpose |
|---------|---------|
| `<name>` | Package identifier (used in imports and commands) |
| `<version>` | Semantic version (0.0.0 is typical for new packages) |
| `<description>` | Human-readable description |
| `<maintainer>` | Who maintains this package |
| `<license>` | License type (MIT, Apache-2.0, etc.) |
| `<depend>` | Runtime dependencies (code you import) |
| `<test_depend>` | Test-only dependencies |
| `<exec_depend>` | Executable dependencies (like ros2launch) |
| `<build_type>` | Either ament_python or ament_cmake |

**The `<depend>` entries are important.** When you write code that imports rclpy (ROS 2's Python library), you need to list rclpy as a dependency. ROS 2 uses this to:
1. Check if dependencies are installed
2. Build packages in the correct order (if A depends on B, build B first)
3. Set up the Python import path correctly

---

## Building Your Workspace

Now that you have a package, build the entire workspace:

```bash
cd ~/ros2_ws
colcon build
```

Expected output:

```
Starting >>> my_first_package
Compiling Cython modules...
Finished <<< my_first_package [2.34s]

Summary: 1 package processed
```

**What happened?**
1. colcon found 1 package in src/ (my_first_package)
2. It ran the Python build process
3. It installed the package into install/

You can safely delete build/ and install/ anytime. To rebuild, just run colcon build again.

---

## Sourcing the Workspace

Before you can run any code from this workspace, you need to source the setup script. This modifies your shell environment to find your packages:

```bash
source ~/ros2_ws/install/setup.bash
```

Now your shell knows about everything in install/. If you write a Python script or launch file, ROS 2 will find it.

**Important:** You must source the workspace BEFORE running your code. If you open a new terminal, you need to source again.

### Verify Sourcing Worked

```bash
echo $ROS_PACKAGE_PATH
```

You should see a path that includes ~/ros2_ws/install. That confirms the workspace is in your search path.

---

## Workflow: Create → Build → Source → Run

This is the workflow you'll repeat whenever you add code:

```
1. Create/edit files in src/
   └── Edit Python code, package.xml, etc.

2. Build the workspace
   └── cd ~/ros2_ws && colcon build

3. Source the workspace
   └── source ~/ros2_ws/install/setup.bash

4. Run your node
   └── ros2 run my_first_package my_node_name
```

---

## Common Troubleshooting

**Error: "colcon: command not found"**
- You need to install colcon: `pip install colcon-common-extensions`
- Or use cloud ROS 2 (TheConstruct) where it's pre-installed

**Error: "package.xml not found"**
- Make sure you're in the src/ directory when you run `ros2 pkg create`
- Package folders must be direct children of src/

**Error: "No such file or directory: ./install/setup.bash"**
- Make sure you ran `colcon build` FIRST
- Source the correct path: `source ~/ros2_ws/install/setup.bash`

**Error: "colcon build" finds no packages**
- Check that your package has a package.xml in the right place
- Verify folder structure: `~/ros2_ws/src/my_first_package/package.xml`

---

## Why This Structure Matters

This three-directory structure (src/, build/, install/) is used industry-wide:
- **CMake projects** use the same pattern
- **ROS projects** (both ROS 1 and ROS 2) use the same pattern
- **Professional C++ projects** often use the same pattern

Understanding this structure now prepares you for professional robotics development.

---

## Try With AI

Now you have a workspace and a package. Let's explore what AI can teach you about package management.

**Ask your AI:**

> "I have a package called 'my_first_package' with dependencies on rclpy and std_msgs. What other dependencies might I need to add as my code grows? Give me a checklist of common ROS 2 dependencies and when to use each one."

**Expected outcome:** AI will suggest dependencies like:
- `geometry_msgs` for position/velocity data
- `sensor_msgs` for camera, LIDAR, IMU data
- `tf2` for coordinate transformations (advanced)
- `rclcpp` for C++ nodes (not Python, but good to know exists)

Notice how AI knows patterns that took humans time to discover. You didn't have to read 50 ROS 2 tutorials — AI suggested the key dependencies upfront.

**Challenge your assumption:**

> "Is it better to depend on everything I might use, or add dependencies only when needed?"

**Expected outcome:** AI will explain the trade-off:
- **Add early**: Simpler dependency list, but larger build
- **Add only when needed**: Cleaner dependency list, but you might miss something
- **Best practice**: Add when needed, document why

This is the kind of reasoning that comes from experience. By asking AI, you're learning from patterns it's seen across thousands of ROS 2 projects.

---

## Reflect

Consider these questions:

1. **Why does ROS 2 use workspaces?** What problems would arise if all packages were installed globally?

2. **Why does colcon separate build/ and install/?** What benefit does this separation provide during development?

3. **What happens if you forget to source?** How would you diagnose the error "package not found" after a successful build?

In the next lesson, you'll write your first ROS 2 node — a publisher that sends data to a topic.
