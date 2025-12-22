---
sidebar_position: 3
chapter: 49
lesson: 3
duration_minutes: 50
title: "Writing Your First Dockerfile"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Manual Dockerfile writing builds understanding of container build process"
cognitive_load:
  concepts_count: 9
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Write a Dockerfile from scratch using FROM, WORKDIR, COPY, RUN, CMD"
    bloom_level: "Create"
  - id: LO2
    description: "Build an image with docker build -t and tag it appropriately"
    bloom_level: "Apply"
  - id: LO3
    description: "Run a container from your custom image and verify it works"
    bloom_level: "Apply"
  - id: LO4
    description: "Create a .dockerignore file to exclude unnecessary files"
    bloom_level: "Apply"
  - id: LO5
    description: "Order Dockerfile instructions to maximize layer cache hits"
    bloom_level: "Analyze"
  - id: LO6
    description: "Explain what happens during each build step"
    bloom_level: "Understand"
  - id: LO7
    description: "Pass environment variables to containers with -e flag"
    bloom_level: "Apply"
  - id: LO8
    description: "Map ports from container to host with -p flag"
    bloom_level: "Apply"
  - id: LO9
    description: "Debug build failures by reading build output"
    bloom_level: "Analyze"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO2
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO5
    competency_area: "2. Digital Communication and Collaboration"
    competency: "2.1 Problem Solving"
---

# Writing Your First Dockerfile

A Dockerfile is a recipe for creating container images. Like a cooking recipe that lists ingredients and steps, a Dockerfile lists the base environment, files to include, and commands to run—building up layers until you have a complete, runnable image.

In this lesson, you'll write a complete Dockerfile for a Python FastAPI service from scratch. You won't use generators or templates. You'll write each instruction by hand, understanding what it does and why it matters. By the end, you'll have built, run, and tested your first containerized application.

---

## Setting Up Your Application Files

Before writing a Dockerfile, you need a Python application to containerize. Let's create a simple FastAPI service.

Create a new directory for your project:

```bash
mkdir my-fastapi-app
cd my-fastapi-app
```

**Output:**
```
$ mkdir my-fastapi-app
$ cd my-fastapi-app
$ pwd
/Users/you/my-fastapi-app
```

Now create the application file:

**File: main.py**
```python
from fastapi import FastAPI

app = FastAPI()

@app.get("/")
def read_root():
    return {"message": "Hello from Docker!"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}
```

**File: requirements.txt**
```
fastapi==0.115.0
uvicorn[standard]==0.32.0
```

Verify both files exist in your directory:

```bash
ls -la
```

**Output:**
```
$ ls -la
total 16
-rw-r--r--  1 you  staff  237 Dec 22 10:30 main.py
-rw-r--r--  1 you  staff   42 Dec 22 10:30 requirements.txt
drwxr-xr-x  3 you  staff   96 Dec 22 10:30 .
drwxr-xr-x  5 you  staff  160 Dec 22 10:30 ..
```

Now you have the application. Let's write the Dockerfile to package it.

---

## Writing Your First Dockerfile

Create a new file named `Dockerfile` (no extension) in your project directory:

```bash
touch Dockerfile
```

**Output:**
```
$ touch Dockerfile
```

Now open it in your editor and write each instruction. We'll build this step-by-step, understanding each line.

### Instruction 1: FROM (Base Image)

Every Dockerfile starts with `FROM`. It specifies the base image—the starting environment that already has the operating system and runtime.

```dockerfile
FROM python:3.12-slim
```

**What this does:**
- `FROM` tells Docker: "Start with this pre-built image"
- `python:3.12-slim` is a minimal Python 3.12 image (only 130 MB)
- The image comes from Docker Hub, the public image repository
- Alternative: `python:3.12` (larger, ~900 MB, includes build tools you don't need)

**Why slim**: For production, smaller base images mean faster downloads, faster deploys, and smaller security surface. We use slim instead of full.

### Instruction 2: WORKDIR (Working Directory)

```dockerfile
WORKDIR /app
```

**What this does:**
- `WORKDIR` sets the container's working directory to `/app`
- All subsequent RUN, COPY, and CMD commands run relative to this directory
- If `/app` doesn't exist, Docker creates it automatically

**Why this matters**: Without setting a workdir, files scatter in the root filesystem. Setting a dedicated directory keeps things organized.

### Instruction 3 & 4: COPY + RUN (Install Dependencies)

This is where we install Python dependencies. Notice the order—we copy `requirements.txt` BEFORE copying `main.py`. This is intentional (we'll explain why in the layer caching section).

```dockerfile
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
```

**What these do:**
- `COPY requirements.txt .` copies the requirements file FROM your machine TO the container's `/app` directory
  - First `.` = file on your machine
  - Second `.` = destination in container (which is `/app` because of WORKDIR)
- `RUN pip install --no-cache-dir -r requirements.txt` executes this command INSIDE the container
  - `--no-cache-dir` tells pip not to store downloaded packages in cache (saves space)
  - This layer creates a Docker layer with all installed packages

**Why two instructions**: COPY brings files in. RUN executes commands. They're separate operations.

### Instruction 5: COPY (Application Code)

```dockerfile
COPY main.py .
```

**What this does:**
- Copies your application code into the container's `/app` directory

**Important**: We copy dependencies FIRST (previous step), then code SECOND. This matters for layer caching (explained later).

### Instruction 6: EXPOSE (Declare Port)

```dockerfile
EXPOSE 8000
```

**What this does:**
- `EXPOSE` documents that the container listens on port 8000
- This doesn't actually open the port—it's documentation + metadata
- When running the container, you'll use `-p` flag to actually map ports

**Why it exists**: It tells other developers (and orchestrators like Kubernetes) what port your service uses.

### Instruction 7: CMD (Default Command)

```dockerfile
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

**What this does:**
- `CMD` specifies the default command when the container starts
- This tells uvicorn to run your FastAPI app on all network interfaces (0.0.0.0) at port 8000

**Difference from RUN**:
- `RUN` executes DURING image build (creates a layer)
- `CMD` executes when the container STARTS (doesn't create a layer)

**Why "0.0.0.0"**: Inside a container, localhost (127.0.0.1) is isolated from outside. Using 0.0.0.0 makes the service accessible from your machine.

### Complete Dockerfile

Here's your finished Dockerfile with all instructions together:

```dockerfile
FROM python:3.12-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY main.py .

EXPOSE 8000

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Copy this into your editor and save the file.

---

## Building Your Image

Now that you have a Dockerfile, build an image from it:

```bash
docker build -t my-fastapi-app:v1 .
```

**Output:**
```
$ docker build -t my-fastapi-app:v1 .
[1/6] FROM docker.io/library/python:3.12-slim
[2/6] WORKDIR /app
[3/6] RUN pip install --no-cache-dir -r requirements.txt
Collecting fastapi==0.115.0
Collecting uvicorn[standard]==0.32.0
...
Successfully installed fastapi-0.115.0 uvicorn-0.32.0
[4/6] COPY main.py .
[5/6] EXPOSE 8000
[6/6] CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
Successfully built 1a2b3c4d
Successfully tagged my-fastapi-app:v1
```

**What happened:**
- `docker build` creates an image from your Dockerfile
- `-t my-fastapi-app:v1` names and tags it (`-t` = tag)
- `.` says "use the Dockerfile in the current directory"
- Docker executed each instruction as a "step" [1/6], [2/6], etc.
- Each step creates a layer (we'll discuss this soon)
- Finally: `Successfully tagged my-fastapi-app:v1` means your image is ready

**Image size check:**

```bash
docker images | grep my-fastapi-app
```

**Output:**
```
$ docker images | grep my-fastapi-app
my-fastapi-app    v1        1a2b3c4d        17 minutes ago   182MB
```

Your image is 182 MB. This is reasonable for a Python application with dependencies.

---

## Running Your Container

Now run a container from your image:

```bash
docker run -p 8000:8000 my-fastapi-app:v1
```

**Output:**
```
$ docker run -p 8000:8000 my-fastapi-app:v1
INFO:     Uvicorn running on http://0.0.0.0:8000
INFO:     Application startup complete
```

**What `-p 8000:8000` does:**
- Maps port 8000 on your machine TO port 8000 in the container
- Left 8000 = your machine's port
- Right 8000 = container's port
- Now you can reach the app at `http://localhost:8000`

The container is running. In another terminal, test the API:

```bash
curl http://localhost:8000/
```

**Output:**
```
$ curl http://localhost:8000/
{"message":"Hello from Docker!"}
```

Test the health endpoint:

```bash
curl http://localhost:8000/health
```

**Output:**
```
$ curl http://localhost:8000/health
{"status":"healthy"}
```

Success! Your containerized application is running and responding to requests.

Stop the container by pressing Ctrl+C in the terminal where it's running:

```
^C
INFO:     Shutting down
INFO:     Application shutdown complete
```

---

## Understanding Build Context and .dockerignore

When you run `docker build .`, Docker sends everything in your directory to the build context. For a small project this is fine, but imagine if you had:
- `node_modules/` (100,000 files)
- `.git/` (git history)
- `.venv/` (virtual environment)
- `__pycache__/` (Python cache)

Docker would waste time processing files it doesn't need.

### Creating .dockerignore

Create a `.dockerignore` file to exclude unnecessary files:

**File: .dockerignore**
```
__pycache__
*.pyc
.venv
.git
.gitignore
.DS_Store
*.log
.env
.pytest_cache
```

**What this does:**
- Works like `.gitignore` for Docker
- Excludes these patterns from the build context
- Speeds up builds and keeps image clean

Verify the file exists:

```bash
ls -la | grep dockerignore
```

**Output:**
```
$ ls -la | grep dockerignore
-rw-r--r--  1 you  staff  112 Dec 22 10:45 .dockerignore
```

Now when you rebuild, Docker ignores these patterns:

```bash
docker build -t my-fastapi-app:v2 .
```

The build should be slightly faster since Docker isn't processing unnecessary files.

---

## Layer Caching: Why Instruction Order Matters

Docker builds images in layers. Each instruction creates a layer. If you change one layer, Docker can reuse unchanged layers from previous builds—this is layer caching.

Here's the key insight: **If you change `main.py`, do you need to reinstall dependencies?**

Look at our Dockerfile again:

```dockerfile
FROM python:3.12-slim

WORKDIR /app

COPY requirements.txt .            # Layer: Install dependencies
RUN pip install --no-cache-dir -r requirements.txt

COPY main.py .                     # Layer: Copy application code

EXPOSE 8000

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

**Why we copy requirements.txt BEFORE main.py:**

When you edit `main.py` and rebuild:
- Docker checks each layer against cached layers
- Layers 1-3 haven't changed (FROM, WORKDIR, COPY requirements.txt, RUN pip install)
- Docker REUSES those cached layers ✓
- Only the COPY main.py layer rebuilds (fast—just file copy)
- Total build time: ~5 seconds

**If we had reversed the order** (COPY main.py first, then requirements.txt):

When you edit `main.py` and rebuild:
- COPY main.py changes → Layer invalidated ✗
- RUN pip install invalidated too (everything after changes must rebuild) ✗
- Docker reinstalls dependencies from scratch
- Total build time: ~60 seconds (downloading and installing packages)

**General rule**: Put instructions that change frequently (your code) AFTER instructions that change rarely (dependencies).

### Demonstrating Cache Hit

Build again without any changes:

```bash
docker build -t my-fastapi-app:v3 .
```

**Output:**
```
$ docker build -t my-fastapi-app:v3 .
[1/6] FROM docker.io/library/python:3.12-slim
[2/6] WORKDIR /app
[3/6] RUN pip install --no-cache-dir -r requirements.txt
...
Using cache
Using cache
...
Successfully tagged my-fastapi-app:v3
```

Notice "Using cache" appears multiple times. Docker didn't re-execute those steps—it used cached layers. That's why the build was instant.

---

## Environment Variables and Port Flexibility

Sometimes you want to run the same image with different configurations (different port, different log level, etc.). Use environment variables.

Run your container with custom port:

```bash
docker run -p 9000:8000 -e PORT=8000 my-fastapi-app:v1
```

**Output:**
```
$ docker run -p 9000:8000 -e PORT=8000 my-fastapi-app:v1
INFO:     Uvicorn running on http://0.0.0.0:8000
```

The `-e PORT=8000` sets an environment variable. The application still runs on 8000 (the CMD hard-codes it), but you map it to 9000 on your machine.

This is simple here, but in more complex applications, you'd modify the CMD to read environment variables:

```dockerfile
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "${PORT:-8000}"]
```

(This uses bash substitution: PORT environment variable, default to 8000 if not set.)

---

## Common Build Failures and How to Read Them

### Failure 1: Missing File

If you run `docker build` without a `requirements.txt` file:

```
COPY requirements.txt .
COPY: source file does not exist
```

**What this means**: Docker can't find the file in the build context.
**Fix**: Verify the file exists: `ls requirements.txt`

### Failure 2: Failed Dependency Installation

If `pip install` fails during build:

```
RUN pip install --no-cache-dir -r requirements.txt
ERROR: Could not find a version that satisfies the requirement ...
```

**What this means**: The package doesn't exist or the version specified is invalid.
**Fix**: Check `requirements.txt` for typos or invalid version numbers.

### Failure 3: Port Already in Use

If you try to run a container with a port already in use:

```
Error response from daemon: driver failed programming external connectivity
... Bind for 0.0.0.0:8000 failed
```

**What this means**: Another container (or service) is using port 8000.
**Fix**: Use a different port: `docker run -p 9000:8000 my-fastapi-app:v1`

---

## Building Your Mental Model

You now understand:

1. **Dockerfile structure**: FROM (base) → WORKDIR (directory) → COPY (files) → RUN (install) → EXPOSE (document port) → CMD (run command)

2. **Build process**: Docker executes each instruction as a step, creating layers, and tags the final image

3. **Running containers**: `docker run` starts a container from an image, with `-p` mapping ports and `-e` setting environment variables

4. **Layer caching**: Order matters—put dependencies before code so changes only rebuild what changed

5. **Build context**: `.dockerignore` excludes unnecessary files from builds

This foundation prepares you for advanced patterns: multi-stage builds to shrink images, Docker Compose for multi-container setups, and security best practices. But first, let's practice.

---

## Try With AI

### Part 1: Modify Your Application

Edit `main.py` to add a new endpoint:

```python
@app.get("/version")
def get_version():
    return {"version": "1.0.0", "service": "FastAPI in Docker"}
```

Save the file.

### Part 2: Rebuild Without Changing Dependencies

Rebuild your image:

```bash
docker build -t my-fastapi-app:v4 .
```

Pay attention to the output. Does it show "Using cache" for the pip install layer? Why? (Hint: You only changed application code, not dependencies.)

### Part 3: Build a New Image and Test the Endpoint

Run your container from the new image:

```bash
docker run -p 8000:8000 my-fastapi-app:v4
```

In another terminal, test the new endpoint:

```bash
curl http://localhost:8000/version
```

You should get:

```json
{"version":"1.0.0","service":"FastAPI in Docker"}
```

### Part 4: Challenge — Optimize Image Size (Optional)

Your current Dockerfile is 182 MB. Try these optimizations:

1. Use `python:3.12-alpine` instead of `python:3.12-slim` (Alpine is even smaller)
2. Check image size: `docker images my-fastapi-app`
3. Does the application still work?

When you're confident with Dockerfiles, you're ready to use AI to optimize builds for production. But this manual practice builds the intuition you need to evaluate AI-generated Dockerfiles critically.
