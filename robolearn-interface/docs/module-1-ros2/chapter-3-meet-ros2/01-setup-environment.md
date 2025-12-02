---
id: lesson-3-1-setup-environment
title: "Lesson 3.1: Setting Up Your ROS 2 Environment"
sidebar_position: 1
sidebar_label: "3.1 Environment Setup"
description: "Install ROS 2 Humble locally or access cloud ROS 2 environment with workspace structure verification."
duration_minutes: 60
proficiency_level: "A2"
layer: "L1"
hardware_tier: 1
learning_objectives:
  - "Install ROS 2 Humble OR access cloud ROS 2 environment"
  - "Understand workspace structure and package paths"
  - "Verify environment with diagnostic commands"
skills:
  - "ros2-fundamentals"
tier_1_path: "Cloud ROS 2 (TheConstruct) or local Ubuntu/WSL installation"
generated_by: "content-implementer v1.0.0"
created: "2025-11-29"
version: "1.0.0"
---

# Setting Up Your ROS 2 Environment

You've learned why ROS 2 matters. Now it's time to get it running on your machine (or in the cloud).

This lesson offers two paths: **Local Installation** (if you have Ubuntu or WSL) or **Cloud ROS 2** (if you prefer browser-based access). Both paths lead to the same destination—a working ROS 2 Humble environment where you can run commands and launch applications.

**Duration**: 60 minutes | **Hardware Tier**: Tier 1 (cloud OR local) | **No coding yet—setup and verification only**

---

## Understanding Your Two Paths

Before you choose, understand what each path offers:

### Path A: Cloud ROS 2 (Easiest, Recommended)

**Best for**: Students without Ubuntu, minimal installation hassle, guaranteed compatibility

**What you get**:
- Browser-based ROS 2 terminal (no installation)
- Pre-configured ROS 2 Humble environment
- VNC remote desktop for GUI applications
- No dependency conflicts

**Time investment**: 5 minutes (signup + launch)

**Where**: The Construct (TheConstruct.org) — free tier includes cloud ROS 2

---

### Path B: Local Installation (Fastest Execution, More Setup)

**Best for**: Students with Ubuntu Linux or WSL, who want local execution speed

**What you get**:
- Native ROS 2 on your machine
- Faster command execution than cloud
- Full control over installation
- Works offline after setup

**Time investment**: 15-20 minutes (installation + verification)

**Where**: Your Ubuntu 22.04 machine (or WSL on Windows)

---

## Choose Your Path

**Decision Guide**:

| Question | Path A (Cloud) | Path B (Local) |
|----------|----------------|----------------|
| Do you have Ubuntu 22.04? | No problem! | Use this ✅ |
| Do you want fastest execution? | Skip this | Try this ✅ |
| Want to avoid installation? | Perfect ✅ | Skip this |
| Have limited disk space? | Yes ✅ | Requires ~5GB |
| Work offline often? | No | Yes ✅ |

**Most students choose Path A first, then graduate to Path B for speed.**

---

## Path A: Cloud ROS 2 (TheConstruct)

### Step 1: Create a Free Account

1. Go to **TheConstruct.org**
2. Click **Sign Up**
3. Create account with email/password
4. Verify email (check spam folder)

**Estimated time**: 3 minutes

---

### Step 2: Launch a ROS 2 Environment

1. Log in to your The Construct account
2. Click **MY COURSES** → **Create New Rosject**
3. Configure:
   - **Name**: "ROS 2 Learning" (or your choice)
   - **ROS Distribution**: Select **ROS 2 Humble**
   - **Type**: Select **ROS Desktop** (includes visualization tools)
4. Click **Create Rosject**

**Wait**: The system launches your environment (~30 seconds)

---

### Step 3: Access Your Terminal

Once launched, you'll see a web interface with:
- **Left sidebar**: File browser
- **Center**: Text editor (not needed yet)
- **Right panel**: **Open Shell** button

Click **Open Shell** to open a terminal.

You now have a working ROS 2 Humble environment in your browser.

**Verify it works:**

```bash
echo $ROS_DISTRO
```

**Expected output**:
```
humble
```

If you see `humble`, you're done with setup! Skip to "Environment Verification" below.

---

## Path B: Local Installation (Ubuntu/WSL)

### Prerequisites

- **Ubuntu 22.04 LTS** (native or WSL on Windows)
- **Sudo access** (ability to run `sudo` commands)
- **~5GB disk space** for ROS 2 + dependencies
- **15-20 minutes** of patience

---

### Step 1: Add ROS 2 Repository

Open a terminal and run:

```bash
sudo apt update
sudo apt install curl gnupg lsb-release

sudo curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros2.org/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

This adds the official ROS 2 repository to your package manager.

**If errors occur**: Run `sudo apt update` and retry the commands above.

---

### Step 2: Install ROS 2 Humble

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

**What happens**:
- Terminal shows progress (may take 5-10 minutes)
- Asks for your password (enter it)
- Installs ROS 2, Python libraries, build tools

**When complete**: You see a fresh prompt with no errors.

---

### Step 3: Source the Setup Script

Add ROS 2 to your shell environment:

```bash
source /opt/ros/humble/setup.bash
```

**What this does**:
- Adds ROS 2 executables to your PATH
- Sets `ROS_DISTRO` environment variable
- Configures Python paths for ROS 2 packages

**Make it automatic** (recommended):

Add the source command to your shell startup file so it runs every time you open a terminal:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Now every new terminal automatically loads ROS 2.

---

## Environment Verification (Both Paths)

Regardless of which path you chose, verify your installation works:

### Check 1: ROS Distro

```bash
echo $ROS_DISTRO
```

**Expected output**: `humble`

**If blank or error**: You skipped the sourcing step. Run:
```bash
source /opt/ros/humble/setup.bash  # Cloud: skip this, already sourced
source ~/.bashrc                     # Local: verify setup
```

---

### Check 2: ROS 2 Commands Available

```bash
ros2 --version
```

**Expected output**:
```
ROS 2 humble (release date: 2023-05-23)
```

---

### Check 3: Find ROS Packages

```bash
ros2 pkg list | head -10
```

This shows the first 10 installed ROS 2 packages. You should see output like:

```
ament_cmake
ament_cmake_auto
ament_cmake_gmock
ament_cmake_google_benchmark
...
```

**If no output**: Sourcing didn't work. Restart your terminal and retry Check 1.

---

### Check 4: Python Path for ROS 2

```bash
python3 -c "import rclpy; print('✓ rclpy installed')"
```

**Expected output**: `✓ rclpy installed`

**If ModuleNotFoundError**: ROS 2 isn't sourced. Retry sourcing and restart terminal.

---

## Troubleshooting

### "Command ros2 not found"

**Cause**: ROS 2 not sourced or installation incomplete

**Fix**:
1. Run `source /opt/ros/humble/setup.bash`
2. Restart terminal
3. Try again

For permanent fix: Add to `~/.bashrc` (already done if you followed Step B3)

---

### "ROS_DISTRO is empty"

**Cause**: Sourcing failed

**Fix**:
1. Verify file exists: `ls /opt/ros/humble/setup.bash` (local) or check The Construct terminal
2. Run source command: `source /opt/ros/humble/setup.bash`
3. Verify: `echo $ROS_DISTRO`

---

### Installation Failed (Local Path)

**If `apt install` gave errors**:

1. Check Ubuntu version: `lsb_release -cs` (should be `jammy` for 22.04)
2. Re-add repository (might have typo)
3. Try again: `sudo apt install ros-humble-desktop`

**If still failing**: Use Cloud Path instead (no installation needed)

---

### "Permission denied" errors

**Cause**: Missing sudo

**Fix**: Prefix commands with `sudo`:
```bash
sudo apt update
sudo apt install ...
```

---

## Understanding Your Environment

Now that ROS 2 is running, understand what you just set up:

### Workspace Structure (Local Installation)

When you install ROS 2 locally, it creates:

```
/opt/ros/humble/           # ROS 2 installation directory
├── bin/                   # Executables (ros2 command, etc.)
├── lib/                   # Libraries
├── share/                 # Packages and data files
└── setup.bash             # Script that sets up environment
```

**When you run `source setup.bash`**, the script:
1. Adds `/opt/ros/humble/bin/` to PATH (so `ros2` command is found)
2. Sets `ROS_DISTRO=humble` (tells ROS 2 which version to use)
3. Configures `PYTHONPATH` (so Python finds `rclpy` library)

---

### Package Discovery (Cloud and Local)

When you run `ros2 pkg list`, ROS 2 searches for packages in directories specified by `ROS_PACKAGE_PATH`.

**For a clean installation**, these are all in `/opt/ros/humble/share/`.

Later (Chapters 4-7), you'll create your own packages in a **workspace** (`~/ros2_ws/`), and they'll automatically be discoverable once sourced.

---

## Next Steps

You now have a working ROS 2 Humble environment. In the next lesson, you'll:

1. Launch **turtlesim**—a simple 2D robot simulator
2. Control it with keyboard input
3. See ROS 2's publish/subscribe system in action

But before that, let's lock in your understanding with a brief checkpoint.

---

## Try With AI

**Setup**: Open your favorite AI tool (ChatGPT, Claude, or similar) and your ROS 2 terminal side by side.

**Prompt 1** (Understanding PATH):

Ask your AI:

```
I just ran 'source /opt/ros/humble/setup.bash'.
What does this command actually do to my shell?
Explain what PATH environment variable is and why ROS 2 needs to modify it.
```

**Expected insight**: PATH tells your shell where to find executables. Understanding this helps you troubleshoot "command not found" errors later.

---

**Prompt 2** (Verification Check):

Ask your AI:

```
I'm verifying my ROS 2 installation. I ran these checks:
- echo $ROS_DISTRO → outputs "humble" ✓
- ros2 --version → outputs "ROS 2 humble (release date: 2023-05-23)" ✓
- ros2 pkg list | head → shows packages ✓
- python3 -c "import rclpy" → outputs nothing (success) ✓

Is my installation complete and correct? What might still be missing?
```

**Expected insight**: AI may suggest verifying `ROS_DOMAIN_ID` or `RMW_IMPLEMENTATION` (advanced settings you don't need yet). Installation can have depth beyond the basics.

---

**Prompt 3** (Failure Recovery):

Ask your AI this scenario:

```
I get the error: "ros2: command not found" even after running "source /opt/ros/humble/setup.bash".
What are 3 things I should check, and how would I diagnose each one?
```

**What you learn**: Systematic troubleshooting. The AI teaches you that errors have multiple possible causes (wrong path, sourcing in wrong shell, incomplete installation), and you can rule them out one by one.

---

**Optional Extension**:

If you're curious about your specific installation, ask:

```
Show me what happens when I run 'source /opt/ros/humble/setup.bash' step-by-step.
What environment variables change? Which files get added to PATH?
```

This deepens your mental model of how ROS 2 installation works.

---

## Checkpoint

Before moving to Lesson 3.2, verify you can:

- [ ] Run `echo $ROS_DISTRO` and see `humble`
- [ ] Run `ros2 --version` and see version output
- [ ] Run `ros2 pkg list | head -5` and see packages
- [ ] (If local path) Understand where ROS 2 is installed (`/opt/ros/humble/`)
- [ ] (If cloud path) Know how to access your The Construct terminal

If all boxes are checked, you're ready to launch turtlesim in Lesson 3.2.

If any failed, re-read the troubleshooting section or ask your AI tool for help.

**Next lesson**: [→ Lesson 3.2: Turtlesim in Action](./02-turtlesim-action.md)
