---
sidebar_position: 3
title: "Writing Your First Dockerfile"
chapter: 49
lesson: 3
duration_minutes: 45

# HIDDEN SKILLS METADATA
skills:
  - name: "Dockerfile Authoring"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "3. Digital Content Creation"
    measurable_at_this_level: "Student can write a complete Dockerfile from scratch using FROM, WORKDIR, COPY, RUN, CMD, and EXPOSE instructions"

  - name: "Container Image Building"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "3. Digital Content Creation"
    measurable_at_this_level: "Student can build and tag images using docker build -t and verify successful builds"

  - name: "Container Runtime Configuration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "3. Digital Content Creation"
    measurable_at_this_level: "Student can run containers with port mapping (-p) and environment variables (-e)"

  - name: "Layer Cache Optimization"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "2. Problem Solving"
    measurable_at_this_level: "Student can order Dockerfile instructions to maximize cache hits and explain why order matters"

  - name: "Build Context Management"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "3. Digital Content Creation"
    measurable_at_this_level: "Student can create effective .dockerignore files to exclude unnecessary files from builds"

learning_objectives:
  - objective: "Write a Dockerfile from scratch using FROM, WORKDIR, COPY, RUN, CMD, EXPOSE"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Create a working Dockerfile that containerizes the Task API"

  - objective: "Build an image with docker build -t and tag appropriately"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Execute build command and verify image appears in docker images output"

  - objective: "Run a container from custom image and verify functionality"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Run container and test API endpoints with curl"

  - objective: "Create a .dockerignore file to exclude unnecessary files"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Create .dockerignore that excludes common Python artifacts"

  - objective: "Order Dockerfile instructions to maximize layer cache hits"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Explain why dependencies are copied before application code"

  - objective: "Pass environment variables with -e flag"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Run container with custom environment variable and verify it's accessible"

  - objective: "Map ports from container to host with -p flag"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Run container with port mapping and access service from host"

cognitive_load:
  new_concepts: 9
  assessment: "9 concepts (FROM, WORKDIR, COPY, RUN, CMD, EXPOSE, -t flag, -p flag, -e flag, .dockerignore, layer caching) at upper limit for B1 tier (7-10 concepts). Scaffolding through step-by-step instruction building manages load."

differentiation:
  extension_for_advanced: "Experiment with multi-stage builds to reduce image size; compare python:slim vs python:alpine base images"
  remedial_for_struggling: "Focus on the first 4 instructions (FROM, WORKDIR, COPY, RUN) before adding EXPOSE and CMD"
---

# Writing Your First Dockerfile

A Dockerfile is a blueprint for creating container images. Like a recipe that lists ingredients and cooking steps, a Dockerfile specifies the base environment, files to include, and commands to execute—building layer upon layer until you have a complete, runnable application image.

In Part 6, you built a Task API with FastAPI—a REST service for managing tasks with full CRUD operations. That API is exactly what we'll containerize in this lesson. You'll write every Dockerfile instruction by hand, understanding what each one does and why it matters. By the end, you'll have built, run, and tested your first containerized Python application using the modern UV package manager.

---

## The Task API: Your Application to Containerize

We'll containerize the In-Memory Task API you built in Chapter 40. This simple but complete FastAPI application demonstrates all the patterns you'll use when containerizing more complex services.

Create a new directory for your Docker project:

```bash
mkdir task-api-docker
cd task-api-docker
```

**Output:**
```
$ mkdir task-api-docker
$ cd task-api-docker
$ pwd
/Users/you/task-api-docker
```

Now create the application file:

**File: main.py**
```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

app = FastAPI(title="Task API")


class Task(BaseModel):
    id: int | None = None
    title: str
    completed: bool = False


tasks: list[Task] = []
next_id = 1


@app.post("/tasks", response_model=Task)
def create_task(task: Task) -> Task:
    global next_id
    task.id = next_id
    next_id += 1
    tasks.append(task)
    return task


@app.get("/tasks", response_model=list[Task])
def list_tasks() -> list[Task]:
    return tasks


@app.get("/tasks/{task_id}", response_model=Task)
def get_task(task_id: int) -> Task:
    for task in tasks:
        if task.id == task_id:
            return task
    raise HTTPException(status_code=404, detail="Task not found")


@app.put("/tasks/{task_id}", response_model=Task)
def update_task(task_id: int, updated: Task) -> Task:
    for i, task in enumerate(tasks):
        if task.id == task_id:
            updated.id = task_id
            tasks[i] = updated
            return updated
    raise HTTPException(status_code=404, detail="Task not found")


@app.delete("/tasks/{task_id}")
def delete_task(task_id: int) -> dict:
    for i, task in enumerate(tasks):
        if task.id == task_id:
            tasks.pop(i)
            return {"message": "Task deleted"}
    raise HTTPException(status_code=404, detail="Task not found")


@app.get("/health")
def health_check() -> dict:
    return {"status": "healthy"}
```

This Task API provides:
- **POST /tasks**: Create a new task
- **GET /tasks**: List all tasks
- **GET /tasks/{id}**: Get a specific task
- **PUT /tasks/{id}**: Update a task
- **DELETE /tasks/{id}**: Delete a task
- **GET /health**: Health check endpoint (essential for container orchestration)

Next, create the `pyproject.toml` for UV package management:

**File: pyproject.toml**
```toml
[project]
name = "task-api"
version = "0.1.0"
description = "In-Memory Task API for Docker containerization"
requires-python = ">=3.12"
dependencies = [
    "fastapi>=0.115.0",
    "uvicorn[standard]>=0.32.0",
]
```

Verify both files exist:

```bash
ls -la
```

**Output:**
```
$ ls -la
total 16
drwxr-xr-x  4 you  staff  128 Dec 27 10:30 .
drwxr-xr-x  5 you  staff  160 Dec 27 10:30 ..
-rw-r--r--  1 you  staff 1247 Dec 27 10:30 main.py
-rw-r--r--  1 you  staff  198 Dec 27 10:30 pyproject.toml
```

Now you have a complete FastAPI application. Let's write the Dockerfile to package it.

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

Open the file in your editor. We'll build it step-by-step, understanding each instruction.

### Instruction 1: FROM (Base Image)

Every Dockerfile starts with `FROM`. It specifies the base image—the starting environment with an operating system and runtime already configured.

```dockerfile
FROM python:3.12-slim
```

**What this does:**
- `FROM` tells Docker: "Start with this pre-built image"
- `python:3.12-slim` is a minimal Python 3.12 image (~130 MB)
- The image comes from Docker Hub, the public image registry
- Alternative: `python:3.12` (larger at ~900 MB, includes build tools)

**Why slim?** For production, smaller base images mean faster downloads, faster deploys, and smaller attack surface. We use slim because our application doesn't need compilation tools.

### Instruction 2: Install UV Package Manager

UV is a modern, fast Python package manager written in Rust. It's significantly faster than pip and provides better dependency resolution.

```dockerfile
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/
```

**What this does:**
- Uses a multi-stage copy to get UV binaries from the official UV image
- `/uv` is the main package manager binary
- `/uvx` is for running tools without installing them
- Places both in `/bin/` so they're in PATH

**Why UV instead of pip?** UV installs packages 10-100x faster than pip and handles dependency conflicts better. For containerized applications, faster builds mean faster deployments.

### Instruction 3: WORKDIR (Working Directory)

```dockerfile
WORKDIR /app
```

**What this does:**
- `WORKDIR` sets the container's working directory to `/app`
- All subsequent RUN, COPY, and CMD commands execute relative to this directory
- If `/app` doesn't exist, Docker creates it automatically

**Why this matters:** Without a WORKDIR, files scatter across the root filesystem. Setting a dedicated directory keeps your application organized.

### Instruction 4 & 5: COPY + RUN (Install Dependencies)

This is where we install Python dependencies. Notice the order—we copy `pyproject.toml` BEFORE copying `main.py`. This is intentional for layer caching (explained later).

```dockerfile
COPY pyproject.toml .
RUN uv sync --no-dev
```

**What these do:**
- `COPY pyproject.toml .` copies the dependency file FROM your machine TO the container's `/app` directory
- `RUN uv sync --no-dev` executes inside the container:
  - `uv sync` installs all dependencies from pyproject.toml
  - `--no-dev` skips development dependencies (like pytest, mypy)
  - This creates a Docker layer with all installed packages

**Why two instructions?** COPY brings files in. RUN executes commands. They're separate operations that create separate layers.

### Instruction 6: COPY (Application Code)

```dockerfile
COPY main.py .
```

**What this does:**
- Copies your application code into the container's `/app` directory

**Important:** We copy dependencies FIRST (previous step), then code SECOND. This matters critically for layer caching—when you edit `main.py`, Docker reuses the cached dependency layer instead of reinstalling everything.

### Instruction 7: EXPOSE (Declare Port)

```dockerfile
EXPOSE 8000
```

**What this does:**
- `EXPOSE` documents that the container listens on port 8000
- This doesn't actually open the port—it's documentation and metadata
- When running the container, you'll use `-p` flag to actually map ports

**Why it exists:** It tells other developers (and orchestrators like Kubernetes) which port your service uses.

### Instruction 8: CMD (Default Command)

```dockerfile
CMD ["uv", "run", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

**What this does:**
- `CMD` specifies the default command when the container starts
- `uv run` executes a command in the UV-managed environment
- `uvicorn main:app` starts the ASGI server running your FastAPI application
- `--host 0.0.0.0` binds to all network interfaces (required for container networking)
- `--port 8000` specifies the port

**Difference from RUN:**
- `RUN` executes DURING image build (creates a layer)
- `CMD` executes when the container STARTS (doesn't create a layer)

**Why "0.0.0.0"?** Inside a container, localhost (127.0.0.1) is isolated from the host. Using 0.0.0.0 makes the service accessible from your machine when you map ports.

### Complete Dockerfile

Here's your finished Dockerfile with all instructions:

```dockerfile
FROM python:3.12-slim

# Install UV package manager
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

WORKDIR /app

# Install dependencies first (for layer caching)
COPY pyproject.toml .
RUN uv sync --no-dev

# Copy application code
COPY main.py .

EXPOSE 8000

CMD ["uv", "run", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Copy this into your editor and save the file.

---

## Building Your Image

Now that you have a Dockerfile, build an image from it:

```bash
docker build -t task-api:v1 .
```

**Output:**
```
$ docker build -t task-api:v1 .
[+] Building 45.2s (10/10) FINISHED
 => [internal] load build definition from Dockerfile
 => [internal] load .dockerignore
 => [internal] load metadata for docker.io/library/python:3.12-slim
 => [internal] load metadata for ghcr.io/astral-sh/uv:latest
 => [1/5] FROM docker.io/library/python:3.12-slim
 => [2/5] COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/
 => [3/5] WORKDIR /app
 => [4/5] COPY pyproject.toml .
 => [5/5] RUN uv sync --no-dev
Resolved 11 packages in 156ms
Prepared 11 packages in 892ms
Installed 11 packages in 45ms
 + annotated-types==0.7.0
 + anyio==4.7.0
 + click==8.1.8
 + fastapi==0.115.6
 + h11==0.14.0
 + pydantic==2.10.4
 + pydantic-core==2.27.2
 + sniffio==1.3.1
 + starlette==0.41.3
 + typing-extensions==4.12.2
 + uvicorn==0.34.0
 => [6/5] COPY main.py .
 => exporting to image
 => => naming to docker.io/library/task-api:v1
```

**What happened:**
- `docker build` creates an image from your Dockerfile
- `-t task-api:v1` names and tags it (`-t` = tag)
- `.` says "use the Dockerfile in the current directory"
- Docker executed each instruction as a step
- UV installed all dependencies in under 1 second (much faster than pip)
- Finally: `naming to docker.io/library/task-api:v1` means your image is ready

**Check image size:**

```bash
docker images | grep task-api
```

**Output:**
```
$ docker images | grep task-api
task-api    v1    a1b2c3d4e5f6    2 minutes ago    195MB
```

Your image is ~195 MB. This includes Python 3.12, UV, FastAPI, Uvicorn, and all dependencies—a complete runtime environment.

---

## Running Your Container

Now run a container from your image:

```bash
docker run -p 8000:8000 task-api:v1
```

**Output:**
```
$ docker run -p 8000:8000 task-api:v1
INFO:     Started server process [1]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
```

**What `-p 8000:8000` does:**
- Maps port 8000 on your machine TO port 8000 in the container
- Left 8000 = your machine's port (host)
- Right 8000 = container's port
- Now you can reach the app at `http://localhost:8000`

The container is running. Open a **new terminal** and test the API:

**Test health endpoint:**
```bash
curl http://localhost:8000/health
```

**Output:**
```
$ curl http://localhost:8000/health
{"status":"healthy"}
```

**Create a task:**
```bash
curl -X POST http://localhost:8000/tasks \
  -H "Content-Type: application/json" \
  -d '{"title": "Learn Docker", "completed": false}'
```

**Output:**
```
$ curl -X POST http://localhost:8000/tasks \
  -H "Content-Type: application/json" \
  -d '{"title": "Learn Docker", "completed": false}'
{"id":1,"title":"Learn Docker","completed":false}
```

**List all tasks:**
```bash
curl http://localhost:8000/tasks
```

**Output:**
```
$ curl http://localhost:8000/tasks
[{"id":1,"title":"Learn Docker","completed":false}]
```

**Update a task:**
```bash
curl -X PUT http://localhost:8000/tasks/1 \
  -H "Content-Type: application/json" \
  -d '{"title": "Learn Docker", "completed": true}'
```

**Output:**
```
$ curl -X PUT http://localhost:8000/tasks/1 \
  -H "Content-Type: application/json" \
  -d '{"title": "Learn Docker", "completed": true}'
{"id":1,"title":"Learn Docker","completed":true}
```

Your containerized Task API is fully functional! All CRUD operations work exactly as they did locally.

Stop the container by pressing Ctrl+C in the terminal where it's running:

```
^C
INFO:     Shutting down
INFO:     Finished server process [1]
```

---

## Understanding Build Context and .dockerignore

When you run `docker build .`, Docker sends everything in your directory to the build context. For a small project this is fine, but imagine if you had:
- `__pycache__/` directories (Python bytecode)
- `.venv/` (virtual environment—could be 500 MB+)
- `.git/` (repository history)
- `.env` (secrets that shouldn't be in images)

Docker would waste time and bandwidth processing files it doesn't need—and worse, you might accidentally include secrets in your image.

### Creating .dockerignore

Create a `.dockerignore` file to exclude unnecessary files:

**File: .dockerignore**
```
# Python artifacts
__pycache__/
*.py[cod]
*$py.class
.pytest_cache/
*.egg-info/
.eggs/
dist/
build/

# Virtual environments
.venv/
venv/
ENV/

# UV cache
.uv/

# IDE and editor files
.idea/
.vscode/
*.swp
*.swo

# Git
.git/
.gitignore

# Environment and secrets
.env
.env.*
*.pem
*.key

# OS files
.DS_Store
Thumbs.db

# Test and development
tests/
*.log
htmlcov/
.coverage
```

**What this does:**
- Works like `.gitignore` but for Docker builds
- Excludes these patterns from the build context
- Speeds up builds and keeps images clean
- **Critically**: Prevents secrets from ending up in images

Verify the file exists:

```bash
ls -la | grep dockerignore
```

**Output:**
```
$ ls -la | grep dockerignore
-rw-r--r--  1 you  staff  425 Dec 27 10:45 .dockerignore
```

Now rebuild your image:

```bash
docker build -t task-api:v2 .
```

The build should be faster since Docker isn't processing unnecessary files in the context.

---

## Layer Caching: Why Instruction Order Matters

Docker builds images in layers. Each instruction creates a layer. When you rebuild, Docker checks if anything changed—if a layer is identical to a cached version, Docker reuses it instead of rebuilding.

Here's the key insight: **If you change `main.py`, do you need to reinstall dependencies?**

Look at our Dockerfile again:

```dockerfile
FROM python:3.12-slim
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/
WORKDIR /app

COPY pyproject.toml .              # Layer: Dependencies definition
RUN uv sync --no-dev               # Layer: Installed packages

COPY main.py .                     # Layer: Application code

EXPOSE 8000
CMD ["uv", "run", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

**Why we copy pyproject.toml BEFORE main.py:**

When you edit `main.py` and rebuild:
1. Layers 1-5 haven't changed (FROM, UV, WORKDIR, COPY pyproject.toml, RUN uv sync)
2. Docker REUSES those cached layers
3. Only the `COPY main.py` layer rebuilds (instant—just a file copy)
4. **Total rebuild time: ~1 second**

**If we had reversed the order** (COPY main.py first, then pyproject.toml):

When you edit `main.py` and rebuild:
1. `COPY main.py` changed → Layer invalidated
2. `COPY pyproject.toml` must rebuild (everything after a change rebuilds)
3. `RUN uv sync` must reinstall all packages
4. **Total rebuild time: 30-60 seconds** (re-downloading and installing packages)

**General rule**: Put instructions that change frequently (your code) AFTER instructions that change rarely (dependencies).

### Demonstrating Cache Hit

Let's prove this works. Edit `main.py` to add a new endpoint:

```python
@app.get("/version")
def get_version() -> dict:
    return {"version": "1.0.0", "service": "Task API"}
```

Rebuild:

```bash
docker build -t task-api:v3 .
```

**Output:**
```
$ docker build -t task-api:v3 .
[+] Building 1.2s (10/10) FINISHED
 => [internal] load build definition from Dockerfile
 => [internal] load .dockerignore
 => CACHED [1/5] FROM docker.io/library/python:3.12-slim
 => CACHED [2/5] COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/
 => CACHED [3/5] WORKDIR /app
 => CACHED [4/5] COPY pyproject.toml .
 => CACHED [5/5] RUN uv sync --no-dev
 => [6/5] COPY main.py .
 => exporting to image
Successfully tagged task-api:v3
```

Notice `CACHED` appears for steps 1-5. Docker reused the dependency layer because `pyproject.toml` didn't change. Only the `COPY main.py` step ran, completing in about 1 second.

**This is why instruction order matters.** A poorly ordered Dockerfile rebuilds everything on every code change. A well-ordered Dockerfile only rebuilds what actually changed.

---

## Environment Variables and Port Flexibility

Sometimes you want to run the same image with different configurations. Environment variables provide this flexibility.

### Running on a Different Host Port

```bash
docker run -p 9000:8000 task-api:v3
```

**Output:**
```
$ docker run -p 9000:8000 task-api:v3
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
```

Now test it on port 9000:

```bash
curl http://localhost:9000/health
```

**Output:**
```
$ curl http://localhost:9000/health
{"status":"healthy"}
```

The container still runs on port 8000 internally, but you mapped it to 9000 on your machine.

### Passing Environment Variables

Use `-e` to pass environment variables:

```bash
docker run -p 8000:8000 -e LOG_LEVEL=debug task-api:v3
```

Your application can read these via `os.environ["LOG_LEVEL"]`. This is how you configure database connections, API keys, and feature flags in containerized applications—without changing the image.

### Running in Detached Mode

For development, you might want the container to run in the background:

```bash
docker run -d -p 8000:8000 --name my-task-api task-api:v3
```

**Output:**
```
$ docker run -d -p 8000:8000 --name my-task-api task-api:v3
a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6q7r8s9t0u1v2w3x4y5z
```

**What the flags do:**
- `-d` = detached mode (runs in background)
- `--name my-task-api` = gives the container a memorable name

Check it's running:

```bash
docker ps
```

**Output:**
```
$ docker ps
CONTAINER ID   IMAGE         STATUS          PORTS                    NAMES
a1b2c3d4e5f6   task-api:v3   Up 5 seconds    0.0.0.0:8000->8000/tcp   my-task-api
```

Stop and remove it when done:

```bash
docker stop my-task-api
docker rm my-task-api
```

**Output:**
```
$ docker stop my-task-api
my-task-api
$ docker rm my-task-api
my-task-api
```

---

## Common Build Failures and How to Debug Them

### Failure 1: Missing File

If you run `docker build` without a required file:

```
COPY pyproject.toml .
COPY: source file does not exist
```

**What this means:** Docker can't find the file in the build context.
**Fix:** Verify the file exists: `ls pyproject.toml`

### Failure 2: Failed Dependency Installation

If `uv sync` fails:

```
RUN uv sync --no-dev
error: Failed to resolve dependencies
  Caused by: No solution found when resolving dependencies
```

**What this means:** The package versions in `pyproject.toml` are incompatible.
**Fix:** Check for typos in package names or invalid version constraints.

### Failure 3: Port Already in Use

```
Error response from daemon: driver failed programming external connectivity
... Bind for 0.0.0.0:8000 failed: port is already allocated
```

**What this means:** Another container (or process) is using port 8000.
**Fix:** Use a different port: `docker run -p 9000:8000 task-api:v1`

Or find and stop the conflicting container:

```bash
docker ps | grep 8000
docker stop <container_id>
```

### Failure 4: Application Crashes on Start

If the container starts but immediately exits:

```bash
docker run task-api:v1
# (exits immediately with no output)
```

Check the logs:

```bash
docker logs $(docker ps -lq)
```

**Output might show:**
```
ModuleNotFoundError: No module named 'fastapi'
```

**What this means:** Dependencies weren't installed correctly.
**Fix:** Verify `RUN uv sync --no-dev` succeeded in the build output.

---

## Building Your Mental Model

You now understand:

1. **Dockerfile structure**: FROM (base) → COPY UV → WORKDIR (directory) → COPY deps → RUN install → COPY code → EXPOSE (port) → CMD (run command)

2. **Build process**: Docker executes each instruction as a step, creating layers, and tags the final image

3. **Running containers**: `docker run` starts a container from an image, with `-p` mapping ports, `-e` setting environment variables, and `-d` for background mode

4. **Layer caching**: Order matters—put dependencies before code so changes only rebuild what changed

5. **Build context**: `.dockerignore` excludes unnecessary files and secrets from builds

This foundation prepares you for multi-stage builds to optimize image size, Docker Compose for multi-container applications, and production security best practices.

---

## Try With AI

### Prompt 1: Diagnose a Dockerfile Problem

```
I have this Dockerfile for a Python FastAPI app, but my builds are slow—about
60 seconds every time I change my code. Here's my Dockerfile:

FROM python:3.12-slim
WORKDIR /app
COPY . .
RUN pip install -r requirements.txt
CMD ["uvicorn", "main:app", "--host", "0.0.0.0"]

What's wrong with my layer ordering, and how would you fix it for faster
rebuilds?
```

**What you're learning:** Analyzing layer cache invalidation patterns—understanding how instruction order affects build performance.

### Prompt 2: Explain a Build Failure

```
I'm getting this error when building my Docker image:

[5/5] RUN uv sync --no-dev
error: Failed to build `pydantic-core==2.27.2`
  Caused by: Failed to build wheel

The package needs a Rust compiler. I'm using python:3.12-slim as my base image.
What are my options? Explain the tradeoffs between using a larger base image
versus multi-stage builds.
```

**What you're learning:** Troubleshooting build failures that require native compilation—a common challenge when containerizing Python applications with binary dependencies.

### Prompt 3: Design a Dockerfile for Your Own API

```
I'm building a [describe your API - e.g., "recipe management API with SQLite
database" or "weather data API that calls external services"]. Based on the
Task API Dockerfile I learned, help me design the Dockerfile for my service.

Ask me about:
1. What external services does it call?
2. Does it need any system-level dependencies?
3. What environment variables does it require?
4. Does it need to persist any data?

Then write a Dockerfile with comments explaining each choice.
```

**What you're learning:** Translating Dockerfile patterns to your own applications—moving from following instructions to making design decisions based on your specific requirements.

### Safety Note

When containerizing applications, never include secrets (API keys, passwords, database credentials) in your Dockerfile or image. Use environment variables passed at runtime (`-e` flag) or Docker secrets for sensitive configuration. Images may be shared or pushed to registries where secrets would be exposed.
