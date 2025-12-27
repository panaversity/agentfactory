---
sidebar_position: 2
title: "Container Fundamentals: Images, Containers, and Layers"
description: "Master Docker's core concepts through hands-on exploration—pulling images, running containers, and understanding how layers enable efficient image sharing."
keywords: [docker, containers, images, layers, docker pull, docker run, docker ps, docker exec]
chapter: 49
lesson: 2
duration_minutes: 40
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Hands-on exploration builds intuition for image/container relationship"

# HIDDEN SKILLS METADATA
skills:
  - name: "Container Architecture Understanding"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "5. Problem Solving"
    measurable_at_this_level: "Student can explain the distinction between images (immutable templates) and containers (running instances with writable layers)"

  - name: "Docker Image Management"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "3. Digital Content Creation"
    measurable_at_this_level: "Student can pull images from Docker Hub, list local images, and remove unused images using docker commands"

  - name: "Container Lifecycle Operations"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "3. Digital Content Creation"
    measurable_at_this_level: "Student can run, stop, start, and remove containers using interactive and detached modes"

  - name: "Container Debugging"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "5. Problem Solving"
    measurable_at_this_level: "Student can execute commands inside running containers and access shell for inspection"

learning_objectives:
  - objective: "Explain the difference between images (templates) and containers (running instances)"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student can articulate that images are immutable blueprints while containers are live processes with writable layers"

  - objective: "Pull images from Docker Hub with docker pull"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student successfully pulls python:3.12-slim and nginx:alpine images and verifies with docker images"

  - objective: "Explain how layers create efficient, reusable images"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student can describe copy-on-write mechanism and how shared layers reduce disk usage"

  - objective: "List and manage images with docker images and docker rmi"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student can list images and remove unused ones from local storage"

  - objective: "List running containers with docker ps"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student uses docker ps and docker ps -a to distinguish running vs stopped containers"

  - objective: "Start, stop, and remove containers with docker start/stop/rm"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student demonstrates full container lifecycle from run through removal"

cognitive_load:
  new_concepts: 6
  assessment: "6 core concepts (images, containers, layers, pull, run, exec) within B1 limit (7-10 concepts). Moderate scaffolding with step-by-step commands."

differentiation:
  extension_for_advanced: "Explore docker inspect output in detail; compare layer digests across images sharing the same base; experiment with volume mounts for data persistence"
  remedial_for_struggling: "Focus on the coffee analogy (recipe vs cup); run only the Python interactive example before moving to Nginx; use docker run --rm to auto-cleanup containers"
---

# Container Fundamentals: Images, Containers, and Layers

Think of containers like shipping containers in the real world. A shipping container is a standardized box: 20 feet or 40 feet long, built to a spec that works on trucks, ships, and trains. What's inside changes—steel coils, electronics, clothing—but the container itself is identical. You can move it anywhere, and the contents stay protected and organized.

Software containers work the same way. A container is a standardized package holding your application, its dependencies, and configuration. It runs identically on your laptop, a colleague's machine, or a cloud server. The operating system might be different, but the container guarantees consistency.

In this lesson, you'll explore the mechanics of containers by hands-on discovery: pulling actual images, running them, stopping them, and examining their internal structure. Through this exploration, you'll build the mental model that enables you to write and optimize containers effectively.

---

## The Core Distinction: Images vs Containers

Here's the fundamental concept that unlocks everything:

**Images are templates. Containers are instances.**

Just like a class in Python defines a blueprint (the image) and objects are instantiated from that class (the containers), Docker works the same way.

### Images: The Blueprint

An image is a **read-only template**. It contains:
- A minimal operating system (Alpine Linux, Ubuntu, Debian)
- Your application code
- All dependencies (Python, Node, Java, libraries)
- Configuration files
- Instructions for how to start the application

Images live in registries (Docker Hub, GitHub Container Registry, cloud provider registries). You pull them from the registry to your machine.

Example image names:
- `python:3.12-slim` — Python 3.12 with minimal OS
- `nginx:alpine` — Nginx web server with Alpine Linux
- `node:20-alpine` — Node.js runtime with Alpine Linux

### Containers: Running Instances

A container is a **running instance** created from an image. It's:
- What actually executes on your machine
- A live process with a file system, network, and memory
- Writable (changes happen at runtime)
- Isolated from other containers and the host

When you run a container, Docker:
1. Takes the image (the template)
2. Creates a writable layer on top
3. Starts the process inside
4. Connects it to the network and file system

Multiple containers can run from the same image simultaneously, each isolated from the others.

### The Analogy in Action

Think of making coffee:

- **Image**: The recipe (beans, water, filter, brewing time)
- **Container**: The actual cup of coffee you make right now
- **Run another**: You can make 10 cups from the same recipe at the same time

Each cup exists independently. Changes to one cup (adding sugar) don't affect others.

---

## The Critical Concept: Images Are Immutable

This is one of the most important concepts in Docker: **images are immutable** (unchangeable).

Once an image is built, it never changes. Not modified. Not updated. Not patched. It's frozen forever.

### Why Immutability Matters

**Reproducibility**: If you pull `python:3.12-slim` today and again in 6 months, you get the exact same image (identified by its SHA256 digest). No surprises.

**Security**: You can verify an image hasn't been tampered with. The digest `sha256:8a3f4d9e5c2b...` is a cryptographic fingerprint. If a single byte changes, the digest changes.

**Rollbacks**: Running a bad version? Switch back to the previous image instantly. The old image still exists unchanged.

**Caching**: Docker can aggressively cache because layers never change. If layer `a803e7c4b030` exists locally, Docker doesn't re-download it—ever.

### But Containers Can Write Files?

Yes. Here's how Docker reconciles immutability with runtime changes:

```
┌─────────────────────────────────────┐
│     Container (writable layer)      │  ← Your runtime changes go here
├─────────────────────────────────────┤
│     Image Layer 4 (read-only)       │  ← Application code
├─────────────────────────────────────┤
│     Image Layer 3 (read-only)       │  ← Dependencies
├─────────────────────────────────────┤
│     Image Layer 2 (read-only)       │  ← Package updates
├─────────────────────────────────────┤
│     Image Layer 1 (read-only)       │  ← Base OS (Alpine, Debian)
└─────────────────────────────────────┘
```

When you run a container, Docker adds a thin **writable layer** on top of the immutable image layers. This is called **copy-on-write**:

- **Reading a file**: Docker looks through layers top-to-bottom until it finds the file
- **Writing a file**: Docker copies the file to the writable layer, then modifies the copy
- **Deleting a file**: Docker marks it deleted in the writable layer (original unchanged)

When you delete the container, the writable layer is discarded. The image remains pristine.

### Practical Implications

1. **Don't store important data in containers**: When the container dies, writable layer data is lost. Use volumes for persistence (Lesson 6).

2. **Multiple containers, same image**: 10 containers from `nginx:alpine` share the same image layers. Only their writable layers differ. This is why containers are so lightweight.

3. **Debugging in production**: You can `docker exec` into a container to investigate, but any files you create disappear when the container restarts. Logs and metrics should go to external systems.

4. **Image tags can be reassigned**: `python:3.12-slim` might point to different images over time (as Python releases patches). For true immutability, reference images by digest: `python@sha256:8a3f4d9e5c2b...`

---

## Pulling Images from Docker Hub

Docker Hub is the default registry where images live. Think of it like GitHub for Docker images.

Let's pull a real image and see it arrive on your system.

### Pull Python Image

```bash
docker pull python:3.12-slim
```

**Output:**

```
3.12-slim: Pulling from library/python
7264a8db6058: Pull complete
28ffb91f2e74: Pull complete
5abc33a71234: Pull complete
a12c5a6b1c00: Pull complete
Digest: sha256:8a3f4d9e5c2b1a9f7c6e4d3b2a1f9e8d7c6b5a4f3e2d1c0b9a8f7e6d5c4b3a
Status: Downloaded newer image for python:3.12-slim
```

### Pull Nginx Image

```bash
docker pull nginx:alpine
```

**Output:**

```
alpine: Pulling from library/nginx
a803e7c4b030: Pull complete
8c2be06b0893: Pull complete
68b0f6f0e0d6: Pull complete
Digest: sha256:a8a6e48d1a8c4c6b2d1a0f9e8d7c6b5a4f3e2d1c0b9a8f7e6d5c4b3a2f1e0d
Status: Downloaded newer image for nginx:alpine
```

When you pull an image, you're downloading the image **layers** (we'll explore those shortly). Notice the `Pull complete` messages—each line represents a layer being downloaded.

### List Downloaded Images

Now that images are on your machine, list them:

```bash
docker images
```

**Output:**

```
REPOSITORY    TAG       IMAGE ID       CREATED       SIZE
nginx         alpine    f5ae1a5d5c8b   2 weeks ago   41.2MB
python        3.12-slim e9b5c4a3d2c1   1 week ago    126MB
```

You now have two image templates on your machine. Neither is running—they're just available to create containers from.

---

## Running Containers: Interactive and Detached

An image is inert until you run it. Let's create containers and see them become alive.

### Run Python Interactively (Interactive Mode)

```bash
docker run -it python:3.12-slim python
```

**Output:**

```
Python 3.12.1 (main, Dec 19 2024, 19:52:33) [GCC 12.2.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>>
```

What happened:
- `-i` (interactive): Keeps STDIN open even if not attached
- `-t` (tty): Allocates a pseudo-terminal
- Together (`-it`): You can type commands and see output

You're now inside a Python REPL running in a container. Try:

```python
>>> print("Hello from inside a container!")
Hello from inside a container!
>>> exit()
```

When you exit, the container stops. The image remains unchanged.

### Run Nginx in Background (Detached Mode)

Running a web server in interactive mode would block your terminal. Instead, run it detached:

```bash
docker run -d --name web-server -p 8080:80 nginx:alpine
```

**Output:**

```
c7d9e4a6f5b2a1c8d3e9f4a6b5c2d1e0
```

What happened:
- `-d` (detached): Run in background
- `--name web-server`: Give the container a readable name
- `-p 8080:80`: Map port 8080 on your machine to port 80 in the container

The container is now running. Test it:

```bash
curl http://localhost:8080
```

**Output:**

```
<!DOCTYPE html>
<html>
<head>
<title>Welcome to nginx!</title>
<style>
    body {
        width: 35em;
        margin: 0 auto;
        font-family: Tahoma, Verdana, Arial, sans-serif;
    }
</style>
</head>
<body>
<h1>Welcome to nginx!</h1>
...
```

The Nginx container is serving web traffic. Perfect.

---

## Container Lifecycle: Inspect, Stop, Restart, Remove

Containers have a lifecycle. Let's see all the operations:

### List Running Containers

```bash
docker ps
```

**Output:**

```
CONTAINER ID   IMAGE           COMMAND                  CREATED        STATUS       PORTS                  NAMES
c7d9e4a6f5b2   nginx:alpine    "/docker-entrypoint.…"   2 minutes ago  Up 2 mins    0.0.0.0:8080->80/tcp   web-server
```

Only running containers appear. The Python container we exited is gone (it stopped when we exited Python).

### List All Containers (Including Stopped)

```bash
docker ps -a
```

**Output:**

```
CONTAINER ID   IMAGE              COMMAND               CREATED         STATUS                     PORTS     NAMES
c7d9e4a6f5b2   nginx:alpine       "/docker-entrypoint.…"   5 minutes ago   Up 5 minutes               8080->80  web-server
f2e1d9c8b7a6   python:3.12-slim   "python"              10 minutes ago  Exited (0) 8 minutes ago            practical_archimedes
```

The Python container still exists (in stopped state) but won't restart automatically.

### Stop a Running Container

```bash
docker stop web-server
```

**Output:**

```
web-server
```

The container gracefully stops. Test that the web server no longer responds:

```bash
curl http://localhost:8080
```

**Output:**

```
curl: (7) Failed to connect to localhost port 8080: Connection refused
```

### Start a Stopped Container

```bash
docker start web-server
```

**Output:**

```
web-server
```

The container restarts. Test the web server again:

```bash
curl http://localhost:8080
```

**Output:**

```
<!DOCTYPE html>
<html>
...
```

Running again.

### Remove a Container

```bash
docker rm web-server
```

**Output:**

```
Error response from daemon: You cannot remove a running container. Stop the container before removing or force remove with option '-f'.
```

Right—can't delete a running container. Stop it first:

```bash
docker stop web-server
docker rm web-server
```

**Output:**

```
web-server
web-server
```

The container is completely deleted. Its file system, networking, and state are gone.

---

## Execute Commands Inside Containers

Sometimes you need to run commands inside a running container without stopping it.

Start Nginx again:

```bash
docker run -d --name web-server -p 8080:80 nginx:alpine
```

Now execute a command inside it:

```bash
docker exec web-server ls /usr/share/nginx/html/
```

**Output:**

```
50x.html
index.html
```

You're listing the directory where Nginx serves files, all from inside the running container.

### Access a Shell Inside the Container

```bash
docker exec -it web-server sh
```

**Output:**

```
/ #
```

You now have a shell prompt inside the Nginx container. Try:

```
# cat /etc/os-release
```

**Output:**

```
NAME="Alpine Linux"
ID=alpine
VERSION_ID=3.18.4
PRETTY_NAME="Alpine Linux v3.18.4"
HOME_URL="https://alpinelinux.org/"
BUG_REPORT_URL="https://bugs.alpinelinux.org/issues"
```

You're running Alpine Linux inside the container. Exit:

```
# exit
```

---

## Understanding Layers: How Images Are Built

Images aren't monolithic blobs. They're built from **layers**, stacked like cake layers.

### Inspect Image Layers

```bash
docker inspect nginx:alpine
```

**Output (abbreviated):**

```json
[
  {
    "Id": "sha256:f5ae1a5d5c8b...",
    "RepoTags": ["nginx:alpine"],
    "RepoDigests": ["nginx@sha256:a8a6..."],
    "Size": 41203456,
    "VirtualSize": 41203456,
    "Layers": [
      "sha256:a803e7c4b030...",
      "sha256:8c2be06b0893...",
      "sha256:68b0f6f0e0d6...",
      "sha256:2b3f1a6c8d9e..."
    ]
  }
]
```

An Nginx image might have 4-5 layers:
1. Base OS (Alpine Linux)
2. Package manager updates
3. Nginx installation
4. Configuration files
5. Entrypoint script

Each layer is **independent** and **reusable**. If you create multiple images that share the same base OS layer, Docker only stores that layer once on disk.

### Why Layers Matter

Layers enable:

**Caching**: When you rebuild an image, Docker reuses unchanged layers (super fast)

**Sharing**: Multiple images sharing a base layer means only one copy on disk

**Efficiency**: You only download layers that don't exist locally (pull is fast)

**Auditability**: Each layer has a hash you can verify

When you write a Dockerfile (later), each instruction creates a layer. Understanding layers helps you optimize image size and build speed.

---

## Try With AI

You now have the foundational understanding. Use AI to deepen your hands-on exploration.

### Setup

You have Docker running. Open a terminal with:
- Docker Desktop running
- Python and Nginx images already pulled (from earlier)

### Prompt 1: Explore Layer Differences

```
I have two images: python:3.12-slim and nginx:alpine. How can I compare
their layers using docker inspect? What do the layers tell me about what
software is installed in each?
```

**What you're learning**: Docker's layer architecture reveals how images are constructed. By comparing layer counts and sizes, you can understand why Python images are larger (more dependencies) and how base images (Alpine) get reused across different applications.

### Prompt 2: Verify Container Independence

```
I'm running two Nginx containers from the same image on the same machine.
Show me how to verify they're independent. If I create a file in one
container's file system, does it affect the other?
```

**What you're learning**: Container isolation in practice. You'll see that each container has its own writable layer, so changes in one never affect another—even when they share the same image. This is the foundation of reproducible deployments.

### Prompt 3: Investigate Container State

```
I have a container that exited. Can I see what command it ran before
exiting? Can I see its logs? How would I know why it stopped?
```

**What you're learning**: Container forensics. You'll use `docker logs`, `docker inspect`, and exit codes to understand container behavior. This is essential for debugging containers that fail in production.

### Safety Note

Remember that container commands (`docker rm`, `docker rmi`) are destructive. Always double-check container names before removing. In production, use `docker rm` with caution—stopped containers may contain logs or state you need to investigate.
