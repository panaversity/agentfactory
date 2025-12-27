---
sidebar_position: 1
title: "Docker Installation & Setup"
chapter: 49
lesson: 1
duration_minutes: 30

# HIDDEN SKILLS METADATA
skills:
  - name: "Understanding Container Architecture"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "3. Digital Content Creation"
    measurable_at_this_level: "Student can explain why containers exist, how they differ from VMs, and describe the Docker architecture stack (Docker Desktop, Engine, containerd)"

  - name: "Installing Development Tools"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student can install Docker Desktop on their operating system following official installation procedures"

  - name: "Verifying Development Environment"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "5.3 Using digital tools to solve problems"
    measurable_at_this_level: "Student can verify Docker installation using docker version command and run hello-world container to validate complete workflow"

  - name: "Configuring Resources for AI Workloads"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "1.2 Understanding digital concepts and terminology"
    measurable_at_this_level: "Student can configure Docker Desktop resources (CPU, memory, disk) appropriately for AI/ML workloads"

learning_objectives:
  - objective: "Explain why containers exist and how they differ from virtual machines"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Verbal explanation of container vs VM architecture with correct identification of kernel sharing as key differentiator"

  - objective: "Install Docker Desktop on your operating system (macOS, Windows, or Linux)"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful installation verified by docker version showing both Client and Server sections"

  - objective: "Verify Docker Engine is running with docker version command"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Command execution showing Client and Server components with version numbers"

  - objective: "Run your first container with docker run hello-world to validate the complete workflow"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful execution of hello-world container with expected output message"

  - objective: "Configure Docker Desktop resources for AI workloads (memory, CPU, disk)"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Docker Desktop Resources tab showing 4+ CPUs and 8+ GB RAM allocated"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (containers vs VMs, Docker Desktop, Docker Engine, containerd, images, containers, resource configuration) at upper limit for B1 tier (7-10 concepts) - appropriate given heavy scaffolding with step-by-step installation guides"

differentiation:
  extension_for_advanced: "Research Docker's OCI (Open Container Initiative) compatibility; explore how containerd works independently of Docker for Kubernetes deployments"
  remedial_for_struggling: "Focus solely on Docker Desktop GUI; skip CLI verification until GUI concepts are solid; use the hello-world container as the primary success validation"
---

# Docker Installation & Setup

Your FastAPI agent runs perfectly on your machine. But "works on my machine" doesn't scale to production or your team's machines. Docker solves this fundamental problem by packaging your agent, its dependencies, and its runtime into a container that runs identically everywhere—your laptop, a teammate's Mac, or a cloud server.

Before you experience the power of containerization through AI collaboration, you need to understand what Docker actually is. This lesson walks you through installation and initial setup manually, building the mental model you'll need to optimize containers and debug issues later.

---

## Why Containers? The Problem They Solve

Before diving into Docker, understand the problem it solves and why it's become essential for deploying AI services.

### The Deployment Problem

Your FastAPI agent from Part 6 works on your laptop. But to make it useful, you need to run it somewhere accessible—a server in the cloud, your company's data center, or a colleague's machine. This is where things break:

| Your Machine | Production Server |
|--------------|-------------------|
| Python 3.12 | Python 3.9 |
| macOS | Ubuntu Linux |
| Dependencies installed globally | Different versions installed |
| Environment variables set in .zshrc | No environment configured |
| Model files in ~/Downloads | Where are the model files? |

Every difference is a potential bug. The server says "Module not found." You say "But it works on my machine!"

### Three Ways to Deploy Software

**Option 1: Manual Setup (Fragile)**

SSH into the server, install Python, pip install dependencies, copy files, configure environment variables, hope nothing changed since yesterday.

Problems: Slow, error-prone, not reproducible. Works until someone updates a system package.

**Option 2: Virtual Machines (Heavy)**

Package the entire operating system—kernel, libraries, your application—into a VM image. Run the VM on any hypervisor (VMware, VirtualBox, cloud providers).

```
┌─────────────────────────────────────┐
│           Your Application          │
├─────────────────────────────────────┤
│    Python, Dependencies, Files      │
├─────────────────────────────────────┤
│         Guest OS (Ubuntu)           │
├─────────────────────────────────────┤
│         Hypervisor (VMware)         │
├─────────────────────────────────────┤
│         Host OS (macOS/Windows)     │
├─────────────────────────────────────┤
│            Hardware                 │
└─────────────────────────────────────┘
```

Problems: Each VM needs its own OS (gigabytes of storage), boots in minutes, wastes RAM running duplicate kernels. Running 10 services means 10 operating systems.

**Option 3: Containers (Lightweight)**

Package your application and dependencies, but **share the host kernel**. No duplicate operating system. Start in milliseconds. Use megabytes instead of gigabytes.

```
┌──────────┐  ┌──────────┐  ┌──────────┐
│  App 1   │  │  App 2   │  │  App 3   │
├──────────┤  ├──────────┤  ├──────────┤
│  Deps    │  │  Deps    │  │  Deps    │
└──────────┴──┴──────────┴──┴──────────┘
           Container Runtime
├─────────────────────────────────────┤
│           Host OS (Linux)           │
├─────────────────────────────────────┤
│            Hardware                 │
└─────────────────────────────────────┘
```

**Key insight**: Containers share the host's kernel. They're isolated processes, not full operating systems. This makes them:
- **Fast**: Start in under a second (VMs take minutes)
- **Small**: 50-200MB typical (VMs are 2-10GB)
- **Efficient**: Run 100 containers on a laptop (10 VMs would exhaust RAM)
- **Portable**: Same container runs on any Linux kernel (development to production)

### Why Containers Matter for AI Services

Your AI agent has specific requirements:

1. **Large dependencies**: PyTorch, transformers, numpy—hundreds of megabytes of packages
2. **Specific versions**: Model trained on transformers 4.35.0 breaks on 4.36.0
3. **Environment variables**: API keys, model paths, configuration
4. **GPU access**: Some AI workloads need NVIDIA CUDA (containers support this)
5. **Reproducibility**: Must reproduce exact behavior for debugging

Containers solve all of these by freezing your entire environment into an immutable, portable package. When you deploy to the cloud, you're not hoping the server is configured correctly—you're shipping the exact environment that works.

### Cloud Computing Context

If you're new to cloud computing, here's the essential context:

**Cloud providers** (AWS, Google Cloud, Azure, DigitalOcean) rent you servers by the hour. These servers run Linux. You deploy your application to these servers.

**Without containers**: You configure each server manually, install dependencies, copy files. Different servers drift out of sync.

**With containers**: You build once, push to a registry (like Docker Hub), and pull onto any server. The container is identical everywhere.

**Kubernetes** (Chapter 50) orchestrates many containers across many servers—handling load balancing, failover, and scaling. But first, you need to know how to build and run a single container. That's what Docker teaches you.

---

## Prerequisites: What You Need Before Starting

Check that your system meets these baseline requirements:

| Requirement | How to Check |
|------------|-------------|
| **macOS 11.0+** OR **Windows 10/11** OR **Linux (Ubuntu 18.04+, Fedora, etc.)** | You're reading this, so you have a system |
| **4 GB RAM minimum** | macOS: Apple menu → About This Mac; Windows: Settings → System → About; Linux: `free -h` |
| **2 CPU cores minimum** | For AI workloads, we'll allocate 4GB RAM + 2 CPUs minimum |
| **10 GB free disk space** | For Docker images and containers |
| **Reliable internet connection** | We'll download ~2GB during setup |

### Don't Have Minimum Resources?

If your machine is underpowered, you have options:

1. **Cloud alternative**: Install Docker on a cloud VM (AWS EC2 t3.small, Google Cloud e2-standard-2, or DigitalOcean $5/month droplet)
2. **Shared machine**: Use a lab computer or colleague's system
3. **Defer chapter**: Complete Part 6 (agent fundamentals) while planning infrastructure upgrades

---

## What Is Docker? The Mental Model

Before installing anything, understand what we're installing. Docker has three essential components:

### Component 1: Docker Engine (The Runtime)

This is the core. The Docker Engine is a lightweight process that runs on your operating system and:
- Creates isolated containers from images
- Manages container lifecycle (start, stop, remove)
- Handles networking between containers
- Manages storage and volumes

**Think of it like**: A process manager—like `systemd` on Linux or Task Manager on Windows, but specialized for containers.

### Component 2: Docker Desktop (The Complete Package)

On macOS and Windows, you can't install Docker Engine directly (it's Linux-native). Docker Desktop solves this by:
- Running a lightweight Linux VM (using Hypervisor.framework on macOS, Hyper-V on Windows)
- Installing Docker Engine inside that VM
- Providing a GUI dashboard for viewing containers and images
- Handling networking so containers feel like they're on your machine

**Important**: Docker Desktop is NOT Docker Engine. Desktop is the *packaging and UI* around Engine.

### Component 3: containerd (The Container Runtime)

Inside Docker Engine runs containerd, a lower-level component that actually:
- Pulls container images from registries
- Extracts images to filesystems
- Creates cgroups and namespaces (Linux kernel features that provide isolation)
- Starts container processes

**You rarely interact with containerd directly**, but it's the reason containers are so lightweight—they don't need a full operating system like VMs do.

### The Architecture Stack

```
Your Machine (macOS/Windows)
    ↓
Docker Desktop (GUI + VM)
    ↓
Linux VM (inside Docker Desktop)
    ↓
Docker Engine
    ↓
containerd
    ↓
Containers (your FastAPI agent, databases, etc.)
```

On Linux, the stack is simpler (no VM needed):

```
Your Linux Machine
    ↓
Docker Engine
    ↓
containerd
    ↓
Containers
```

---

## Installation by Operating System

::::os-tabs

::macos
**Supported versions**: macOS 11.0 (Big Sur) and later

**Step 1: Download Docker Desktop**

Visit [Docker's official download page](https://www.docker.com/products/docker-desktop) and click the macOS download button. You'll see two options:

- **Apple Silicon (M1/M2/M3)**: For newer Macs with Apple chips
- **Intel**: For Intel-based Macs

Check which you have: Apple menu → About This Mac → Look for "Chip: Apple M2" (Silicon) or "Processor: Intel Core" (Intel).

**Step 2: Install Docker Desktop**

1. Open your Downloads folder
2. Double-click `Docker.dmg`
3. Drag the Docker icon to the Applications folder
4. Wait for copy to complete (usually 1-2 minutes)
5. Eject the disk image by dragging it to Trash

**Step 3: Launch Docker**

1. Open Applications folder
2. Double-click Docker.app
3. Enter your password when prompted (Docker needs to install system components)
4. Wait for "Docker is running" to appear in the menu bar (top right)

Docker Desktop is now running. You'll see the Docker icon in your menu bar at the top right.

::windows
**Supported versions**: Windows 10 (21H2 and later) or Windows 11

**Prerequisites check**: Docker Desktop on Windows requires Hyper-V. Check if it's enabled:

```powershell
# In PowerShell (as Administrator)
Get-WindowsOptionalFeature -Online -FeatureName Hyper-V | Select-Object -Property Name, State
```

**Expected output**:
```
Name  State
----  -----
Hyper-V Enabled
```

If State is "Disabled", you need to enable Hyper-V first:

```powershell
# In PowerShell (as Administrator)
Enable-WindowsOptionalFeature -Online -FeatureName Hyper-V -All
```

Restart your computer when prompted.

**Step 1: Download Docker Desktop**

Visit [Docker's official download page](https://www.docker.com/products/docker-desktop) and click the Windows download button. This downloads `Docker Desktop Installer.exe`.

**Step 2: Install**

1. Double-click `Docker Desktop Installer.exe`
2. Follow the installation wizard (use default settings)
3. Check "Use WSL 2 instead of Hyper-V" (WSL 2 is recommended for performance)
4. Click Install

Installation takes 2-5 minutes.

**Step 3: Launch Docker**

1. Search "Docker Desktop" in your Start menu
2. Click to launch
3. Wait for "Docker Desktop is running" notification (taskbar shows Docker icon)

You'll see Docker icon in your system tray (bottom right).

::linux
**Supported distributions**: Ubuntu 18.04+, Fedora, Debian, CentOS, etc.

Docker Engine runs natively on Linux, so installation is simpler (no VM needed).

**Ubuntu/Debian:**

```bash
# Update package index
sudo apt-get update

# Install prerequisites
sudo apt-get install -y ca-certificates curl gnupg lsb-release

# Add Docker's official GPG key
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# Add Docker repository
echo \
  "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Add your user to docker group (so you don't need sudo every time)
sudo usermod -aG docker $USER

# Activate the change
newgrp docker
```

**Expected output** (after installation completes):
```
Setting up docker-ce (5:24.0.0~3-0~ubuntu-jammy) ...
...
Processing triggers for man-db (2.11.2-1) ...
```

**Fedora/RHEL:**

```bash
sudo dnf -y install dnf-plugins-core
sudo dnf config-manager --add-repo https://download.docker.com/linux/fedora/docker-ce.repo
sudo dnf install docker-ce docker-ce-cli containerd.io docker-compose-plugin
sudo systemctl start docker
sudo usermod -aG docker $USER
newgrp docker
```

::::

---

## Verify Installation

No matter your OS, verify Docker installed correctly by checking the version:

```bash
docker version
```

**Expected output** (macOS/Windows/Linux):
```
Client:
 Cloud integration: v1.0.35
 Version:           24.0.0
 API version:       1.43
 Go version:        go1.20.4
 Git commit:        62409b4
 Built:             Thu May 25 11:30:12 2023
 OS/Arch:           darwin/arm64
 Context:           default

Server: Docker Desktop
 Engine:
  Version:          24.0.0
  API version:      1.43 (minimum version 1.12)
  Go version:       go1.20.4
  Git commit:       61876b8
  Built:            Thu May 25 11:30:34 2023
  OS/Arch:          linux/arm64
  Experimental:     false
 containerd:
  Version:          1.6.21
  GitCommit:        3dce8eb055cbb6872793272b4f20ed16117344f8
 runc:
  Version:          1.1.7
  GitCommit:        v1.1.7-0-g860f061
```

Key confirmation: You see **both Client and Server** sections. The Client is your terminal, the Server is Docker Engine running behind the scenes.

If you see only "Client" without "Server," Docker Engine isn't running. Restart Docker Desktop and try again.

---

## Run Your First Container

Now test that the entire system works end-to-end:

```bash
docker run hello-world
```

**Expected output**:
```
Unable to find image 'hello-world:latest' locally
latest: Pulling from library/hello-world
719385e32844: Pull complete
Digest: sha256:926fac19d22f26fc3d2d91fa13a...
Status: Downloaded newer image for hello-world:latest

Hello from Docker!
This message shows that your installation appears to be working correctly.

To generate this message, Docker took the following steps:
 1. The Docker client contacted the Docker daemon.
 2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
 3. The Docker daemon created a new container from that image which runs the
    executable that produces the output you are currently reading.
 4. The Docker daemon streamed that output to the Docker client, which sent it
    to your terminal.
```

What just happened:

1. Docker looked for `hello-world` image locally (not found on first run)
2. Docker pulled the image from Docker Hub (the public registry)
3. Docker created and started a container from that image
4. The container printed its message and exited

**Congratulations**: You've just created and run your first container. This single command validates:
- Docker Engine is running
- Your network connection works
- Image pulling works
- Container creation works

---

## Configure Docker Desktop for AI Workloads

By default, Docker Desktop allocates limited resources. For AI services (which are memory-hungry), you need to configure resources.

::::os-tabs

::macos
1. Click Docker icon in menu bar → Preferences (or Settings)
2. Select **Resources** tab
3. Adjust these settings:
   - **CPUs**: 4 (minimum for AI workloads)
   - **Memory**: 8 GB (minimum 4GB, but AI models benefit from more)
   - **Disk Image Size**: 100 GB (default may be too small for large model files)
   - **Swap**: 2 GB (helps when memory is tight)

4. Click **Apply & Restart**
5. Docker restarts with new limits

::windows
1. Right-click Docker icon in system tray → Settings
2. Select **Resources** tab
3. Adjust these settings:
   - **CPUs**: 4
   - **Memory**: 8 GB
   - **Disk space**: 100 GB
   - **Virtual disk limit**: 100 GB

4. Click **Apply & Restart**

::linux
On Linux, Docker uses system resources directly (no VM overhead). Configure at container level instead:

When you run your agent container, specify resource limits:

```bash
docker run \
  --memory=4g \
  --cpus=2 \
  your-agent-image
```

**Expected output** (after container starts):
```
Container started with 4GB memory limit and 2 CPU cores
```

This reserves 4GB RAM and 2 CPUs for the container.

::::

---

## Understanding Docker Desktop GUI

Docker Desktop provides a GUI for visualizing your containers and images. Explore it now:

**macOS**: Click Docker icon (top right) → Dashboard opens

**Windows**: Click Docker icon (system tray, bottom right) → Dashboard opens

**Linux**: Not available (Docker runs natively without GUI)

### Containers Tab

Shows all containers on your system. You'll see:

- **hello-world** container you just ran (status: Exited)
- Any other containers you've created

Columns show:
- Container name
- Image it was created from
- Current status (Running, Exited)
- Ports being exposed
- When it was created

### Images Tab

Shows all images stored locally:

- **hello-world** image you pulled earlier
- When you build custom Dockerfiles, they appear here
- Shows image size

**Experiment**: Click on the **Containers** tab and delete the hello-world container by clicking the trash icon. The container is removed (the image remains for future use).

---

## Common Installation Issues & Recovery

### Issue: "Docker Desktop won't start" (macOS/Windows)

**Diagnosis**: Check that virtualization is enabled:

- **macOS**: Virtualization is typically enabled by default
- **Windows**: Check that Hyper-V is enabled (see Windows installation section)

**Recovery**:
1. Restart your computer
2. Try launching Docker Desktop again
3. If still fails: Uninstall Docker Desktop, restart, reinstall

### Issue: "Permission denied" when running docker commands (Linux)

**Cause**: Your user is not in the docker group

**Recovery**:
```bash
sudo usermod -aG docker $USER
newgrp docker
docker run hello-world  # Try again
```

**Expected output** (after fix):
```
Hello from Docker!
This message shows that your installation appears to be working correctly.
```

### Issue: "Cannot connect to Docker daemon" (All platforms)

**Cause**: Docker Engine isn't running

**Recovery**:
- **macOS/Windows**: Open Docker Desktop application
- **Linux**: `sudo systemctl start docker`

Then retry your command.

### Issue: "Image pull failed" during `docker run`

**Cause**: Network issue or Docker Hub is temporarily down

**Recovery**:
1. Check your internet connection
2. Wait a few minutes (Docker Hub may be temporarily unavailable)
3. Try again: `docker pull hello-world` then `docker run hello-world`

---

## Architecture Review: Putting It Together

Now that Docker is running, you understand these key relationships:

**Docker Desktop** (macOS/Windows) = lightweight VM + Docker Engine + GUI dashboard

**Docker Engine** = the actual process managing containers on the Linux kernel

**containerd** = the lower-level runtime creating isolated processes

**Images** = blueprints (like a class definition)

**Containers** = running instances (like an object instantiated from that class)

When you run `docker run hello-world`:

1. Docker Engine checks if hello-world image exists locally
2. If not, containerd pulls it from Docker Hub
3. containerd extracts the image layers to a filesystem
4. containerd creates a new isolated process (using cgroups/namespaces)
5. That process runs the hello-world program
6. Process exits, container stops (but image and container remain on disk)

For your FastAPI agent in future lessons, this same flow applies:

1. Docker Engine pulls your custom agent image
2. containerd sets up an isolated filesystem with your code + dependencies
3. containerd starts the Python process running your FastAPI server
4. Your agent is now isolated—it can't access other system files or crash other processes

---

## Try With AI

Now that Docker is installed and configured, explore container concepts further with your AI companion.

### Prompt 1: Understand the Architecture Deeply

```
I just installed Docker and learned about Docker Desktop, Docker Engine, and
containerd. I understand they form a stack, but I'm not clear on when I'd
interact with each layer. Can you explain:

1. When would a developer interact directly with containerd vs Docker Engine?
2. Why does Docker Desktop need a Linux VM on macOS/Windows?
3. If containers share the host kernel, how are they actually isolated?

Use concrete examples from deploying a Python FastAPI application.
```

**What you're learning**: Understanding architectural layers helps you debug issues at the right level. When something breaks, you'll know whether it's a Docker Desktop issue, Engine configuration, or container runtime problem.

### Prompt 2: Compare with Your Mental Model

```
I've been thinking of Docker containers as "lightweight VMs." But I learned
containers share the kernel while VMs don't. Help me refine my mental model:

1. What can go wrong if two containers share the same kernel?
2. Why are containers faster to start than VMs?
3. For my AI agent that needs specific Python packages and environment
   variables, what exactly gets "frozen" in a container vs what's dynamic?

Challenge my understanding if I have misconceptions.
```

**What you're learning**: Refining mental models through dialogue. Your AI partner identifies gaps in understanding and helps you build accurate intuitions about container technology.

### Prompt 3: Plan for AI Workload Requirements

```
I configured Docker Desktop with 8GB RAM and 4 CPUs for AI workloads. But
I'm not sure if that's enough or overkill. My AI agent uses:
- FastAPI for the API layer
- A small language model (maybe 2-3GB)
- Some numpy/pandas processing

Help me think through:
1. How do I estimate memory needs for this workload?
2. What happens if my container exceeds the memory limit?
3. Should I set resource limits per container, or rely on Docker Desktop limits?

Ask me clarifying questions about my specific use case if needed.
```

**What you're learning**: Resource planning for production workloads. Understanding memory and CPU allocation prevents out-of-memory crashes and helps you size cloud infrastructure correctly.

### Safety Note

Docker provides process isolation, not security isolation. Containers share the host kernel, so a kernel vulnerability could affect all containers. For production deployments with sensitive data, use additional security layers (network policies, secrets management) covered in later lessons.
