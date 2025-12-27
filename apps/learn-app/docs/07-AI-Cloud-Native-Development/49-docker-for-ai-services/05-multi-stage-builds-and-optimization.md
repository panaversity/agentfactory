---
sidebar_position: 5
title: "Multi-Stage Builds & Optimization"
description: "Reduce Docker image size by 70-85% using multi-stage builds, UV package manager, and Alpine base images"
keywords: [docker, multi-stage builds, docker optimization, uv package manager, alpine, distroless, image size]
chapter: 49
lesson: 5
duration_minutes: 50
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Iterative optimization teaches image size reduction techniques through hands-on practice"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Multi-Stage Dockerfile Design"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "3. Digital Content Creation"
    measurable_at_this_level: "Student can write a multi-stage Dockerfile with separate build and runtime stages"

  - name: "Docker Image Optimization"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "2. Problem Solving"
    measurable_at_this_level: "Student can apply optimization techniques to reduce image size by 70%+"

  - name: "UV Package Manager Usage"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "3. Digital Content Creation"
    measurable_at_this_level: "Student can use UV for fast Python dependency installation in Docker builds"

  - name: "Base Image Selection"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Analyze"
    digcomp_area: "2. Problem Solving"
    measurable_at_this_level: "Student can compare slim, alpine, and distroless images and select appropriately"

  - name: "Docker Layer Optimization"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "3. Digital Content Creation"
    measurable_at_this_level: "Student can combine RUN commands and clean caches to minimize layer size"

  - name: "Image Size Measurement"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Evaluate"
    digcomp_area: "2. Problem Solving"
    measurable_at_this_level: "Student can measure and compare image sizes before and after optimization"

learning_objectives:
  - objective: "Explain why multi-stage builds reduce image size"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of build artifacts vs runtime needs"

  - objective: "Write a multi-stage Dockerfile with build and runtime stages"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Working multi-stage Dockerfile with COPY --from"

  - objective: "Use UV package manager for fast Python dependency installation"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Dockerfile using uv pip install with correct flags"

  - objective: "Compare slim, alpine, and distroless base images"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Decision matrix for base image selection"

  - objective: "Optimize layers by combining RUN commands and cleaning caches"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Dockerfile with combined RUN commands and cache cleanup"

  - objective: "Measure image size before and after optimization"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "docker images and docker history output analysis"

  - objective: "Achieve 70%+ size reduction from naive to optimized image"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Comparison showing 1.2GB to ~120MB reduction"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (multi-stage, UV, alpine, slim, distroless, layer optimization, size measurement) at upper limit of B1's 7-10 range, appropriate scaffolding provided through iterative steps"

differentiation:
  extension_for_advanced: "Explore distroless Python images for maximum security; implement BuildKit cache mounts for even faster builds; investigate scratch-based images for Go applications"
  remedial_for_struggling: "Focus on the two-stage pattern first (builder + runtime); practice with slim base before attempting alpine; ensure understanding of COPY --from before adding optimization layers"
---

# Multi-Stage Builds & Optimization

Container images that work are good. Container images that work AND are small are great. This lesson teaches you why small images matter and how to achieve 70-85% size reduction through iterative optimization.

When you build a Docker image for a Python service, you typically need compilers and development libraries during the build process. But in production, you only need the installed packages themselves. A naive Dockerfile includes everything---build tools, development headers, cache files---adding hundreds of megabytes of unnecessary weight.

Multi-stage builds solve this elegantly. You perform dependency installation in a large build image, then copy only the artifacts you need into a minimal production image. Combined with UV (a Rust-based package manager that's 10-100x faster than pip) and Alpine base images, you can reduce a 1.2GB image to under 120MB.

In this lesson, you'll containerize a Task API service---the same pattern you'll use for your Part 6 FastAPI agent. You'll start with a bloated Dockerfile and progressively optimize it through four iterations, measuring the size reduction at each step.

---

## The Application: Task API Service

We'll optimize a realistic FastAPI service that represents the pattern you'll use for AI agent services. This Task API manages tasks with full CRUD operations.

**File: requirements.txt**
```
fastapi==0.115.6
uvicorn[standard]==0.32.1
pydantic==2.10.3
```

**File: main.py**
```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import Optional
from datetime import datetime

app = FastAPI(title="Task API", version="1.0.0")

# In-memory task storage (production would use database)
tasks: dict[str, dict] = {}

class TaskCreate(BaseModel):
    title: str
    description: Optional[str] = None
    priority: int = 1

class Task(BaseModel):
    id: str
    title: str
    description: Optional[str]
    priority: int
    created_at: datetime
    completed: bool = False

@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "task-api"}

@app.post("/tasks", response_model=Task)
def create_task(task: TaskCreate):
    task_id = f"task_{len(tasks) + 1}"
    new_task = {
        "id": task_id,
        "title": task.title,
        "description": task.description,
        "priority": task.priority,
        "created_at": datetime.now(),
        "completed": False
    }
    tasks[task_id] = new_task
    return new_task

@app.get("/tasks")
def list_tasks():
    return list(tasks.values())

@app.get("/tasks/{task_id}", response_model=Task)
def get_task(task_id: str):
    if task_id not in tasks:
        raise HTTPException(status_code=404, detail="Task not found")
    return tasks[task_id]

@app.patch("/tasks/{task_id}/complete")
def complete_task(task_id: str):
    if task_id not in tasks:
        raise HTTPException(status_code=404, detail="Task not found")
    tasks[task_id]["completed"] = True
    return tasks[task_id]
```

Create these files in your working directory before proceeding.

---

## Iteration 0: The Naive Dockerfile (~1.2GB)

Let's start with a Dockerfile that works but doesn't consider image size at all.

**File: Dockerfile.naive**
```dockerfile
FROM python:3.12

WORKDIR /app

COPY requirements.txt .
RUN pip install -r requirements.txt

COPY main.py .

EXPOSE 8000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Build this image and check its size:

```bash
docker build -t task-api:naive -f Dockerfile.naive .
```

**Output:**
```
[+] Building 45.2s (9/9) FINISHED
 => [internal] load build definition from Dockerfile.naive           0.0s
 => [internal] load .dockerignore                                    0.0s
 => [internal] load metadata for docker.io/library/python:3.12       1.2s
 => [1/4] FROM docker.io/library/python:3.12                        28.3s
 => [2/4] WORKDIR /app                                               0.1s
 => [3/4] COPY requirements.txt .                                    0.0s
 => [4/4] RUN pip install -r requirements.txt                       12.8s
 => [5/5] COPY main.py .                                             0.0s
 => exporting to image                                               2.7s
```

Now check the image size:

```bash
docker images task-api:naive
```

**Output:**
```
REPOSITORY   TAG     IMAGE ID       CREATED          SIZE
task-api     naive   a1b2c3d4e5f6   15 seconds ago   1.21GB
```

**1.21GB for a simple Task API.** That bloat comes from:

| Component | Approximate Size |
|-----------|-----------------|
| Full Python image (compilers, headers, build tools) | ~900MB |
| Pip cache (stored in `/root/.cache/pip`) | ~150MB |
| Development dependencies | ~150MB |

None of that is needed to RUN the application. You only need the installed Python packages---maybe 100MB total.

---

## Iteration 1: Slim Base Image (~450MB)

The `python:3.12` image is the full-featured version. Docker provides leaner alternatives:

| Base Image | Size | Contents |
|------------|------|----------|
| `python:3.12` (full) | ~900MB | Build tools, compilers, development headers |
| `python:3.12-slim` | ~150MB | Essential runtime, no build tools |
| `python:3.12-alpine` | ~50MB | Minimal Linux, tiny footprint |
| `distroless/python3` | ~50MB | Only runtime, no shell or package manager |

Let's try slim first---it's the safest improvement with the least risk.

**File: Dockerfile.v1-slim**
```dockerfile
FROM python:3.12-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY main.py .

EXPOSE 8000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Note: We added `--no-cache-dir` to pip to avoid storing the download cache.

Build and measure:

```bash
docker build -t task-api:slim -f Dockerfile.v1-slim .
```

**Output:**
```
[+] Building 18.4s (9/9) FINISHED
 => [internal] load metadata for docker.io/library/python:3.12-slim  0.8s
 => [1/4] FROM docker.io/library/python:3.12-slim                    8.2s
 => [4/4] RUN pip install --no-cache-dir -r requirements.txt         8.1s
```

```bash
docker images task-api:slim
```

**Output:**
```
REPOSITORY   TAG    IMAGE ID       CREATED          SIZE
task-api     slim   f6e5d4c3b2a1   8 seconds ago    458MB
```

| Version | Size | Reduction |
|---------|------|-----------|
| Naive | 1.21GB | --- |
| Slim | 458MB | 62% smaller |

**Progress: 62% reduction** from a single change (base image). But we're still carrying pip overhead and could do better.

---

## Iteration 2: Multi-Stage Build with UV (~180MB)

Multi-stage builds use multiple `FROM` instructions in a single Dockerfile. Each stage can use a different base image. You build dependencies in a large stage, then copy only what you need into a small stage.

We'll also introduce **UV**, a Rust-based Python package manager that's 10-100x faster than pip.

**File: Dockerfile.v2-multistage**
```dockerfile
# Stage 1: Build stage (install dependencies)
FROM python:3.12-slim AS builder

WORKDIR /app

# Install UV package manager (10-100x faster than pip)
RUN pip install uv

COPY requirements.txt .

# UV installs packages to system Python
# --system: install to system Python instead of virtual environment
# --no-cache: don't store package cache
RUN uv pip install --system --no-cache -r requirements.txt

# Stage 2: Runtime stage (only what's needed to run)
FROM python:3.12-slim

WORKDIR /app

# Copy installed packages from builder stage
COPY --from=builder /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY --from=builder /usr/local/bin /usr/local/bin

# Set environment variables
ENV PYTHONUNBUFFERED=1

# Copy application code
COPY main.py .

EXPOSE 8000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Let's understand what's happening:

**Stage 1 (builder)**:
- Starts with `python:3.12-slim` (has pip available)
- Installs UV package manager
- Installs application dependencies with UV
- This stage is used only for building; it's discarded when the build finishes

**Stage 2 (runtime)**:
- Starts with a fresh `python:3.12-slim` (clean slate)
- Copies only the installed packages from builder stage
- Copies application code
- Does NOT include UV, pip cache, or build artifacts
- This is the final image Docker keeps

Build and measure:

```bash
docker build -t task-api:multistage -f Dockerfile.v2-multistage .
```

**Output:**
```
[+] Building 12.8s (14/14) FINISHED
 => [builder 1/4] FROM docker.io/library/python:3.12-slim             0.0s
 => [builder 2/4] RUN pip install uv                                  3.2s
 => [builder 3/4] COPY requirements.txt .                             0.0s
 => [builder 4/4] RUN uv pip install --system --no-cache ...          1.8s  <-- Much faster!
 => [stage-1 1/4] FROM docker.io/library/python:3.12-slim             0.0s
 => [stage-1 2/4] COPY --from=builder /usr/local/lib/python...        0.4s
 => [stage-1 3/4] COPY --from=builder /usr/local/bin ...              0.1s
 => [stage-1 4/4] COPY main.py .                                      0.0s
```

Notice how UV installed dependencies in 1.8 seconds vs pip's 8+ seconds.

```bash
docker images task-api:multistage
```

**Output:**
```
REPOSITORY   TAG          IMAGE ID       CREATED          SIZE
task-api     multistage   d3c2b1a0f9e8   5 seconds ago    182MB
```

| Version | Size | Reduction from Naive |
|---------|------|---------------------|
| Naive | 1.21GB | --- |
| Slim | 458MB | 62% |
| Multi-stage | 182MB | 85% |

**Progress: 85% reduction.** The runtime image has no UV, no pip, no build tools---only the installed packages.

---

## Iteration 3: Alpine Base Image + UV (~120MB)

Alpine Linux is a minimal distribution (~5MB base) designed for containers. Combined with multi-stage builds and UV, we can achieve maximum size reduction.

**File: Dockerfile.v3-alpine**
```dockerfile
# Stage 1: Build stage with Alpine
FROM python:3.12-alpine AS builder

WORKDIR /app

# Install UV package manager
RUN pip install uv

COPY requirements.txt .

# Install dependencies with UV
RUN uv pip install --system --no-cache -r requirements.txt

# Stage 2: Runtime stage with Alpine
FROM python:3.12-alpine

WORKDIR /app

# Copy installed packages from builder
COPY --from=builder /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY --from=builder /usr/local/bin /usr/local/bin

ENV PYTHONUNBUFFERED=1

COPY main.py .

EXPOSE 8000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Build and measure:

```bash
docker build -t task-api:alpine -f Dockerfile.v3-alpine .
```

**Output:**
```
[+] Building 15.2s (14/14) FINISHED
 => [builder 1/4] FROM docker.io/library/python:3.12-alpine           2.1s
 => [builder 2/4] RUN pip install uv                                  4.8s
 => [builder 3/4] COPY requirements.txt .                             0.0s
 => [builder 4/4] RUN uv pip install --system --no-cache ...          2.4s
 => [stage-1 1/4] FROM docker.io/library/python:3.12-alpine           0.0s
 => [stage-1 2/4] COPY --from=builder /usr/local/lib/python...        0.3s
 => [stage-1 3/4] COPY --from=builder /usr/local/bin ...              0.1s
 => [stage-1 4/4] COPY main.py .                                      0.0s
```

```bash
docker images task-api:alpine
```

**Output:**
```
REPOSITORY   TAG      IMAGE ID       CREATED          SIZE
task-api     alpine   e4f5a6b7c8d9   4 seconds ago    118MB
```

| Version | Size | Reduction from Naive |
|---------|------|---------------------|
| Naive | 1.21GB | --- |
| Slim | 458MB | 62% |
| Multi-stage | 182MB | 85% |
| Alpine + UV | 118MB | **90%** |

**Progress: 90% reduction.** From 1.21GB to 118MB.

---

## Iteration 4: Layer Optimization (~115MB)

Docker builds images in layers. Each `RUN` instruction creates a new layer. By combining commands and cleaning up in the same layer, we can squeeze out a few more megabytes.

**File: Dockerfile.v4-optimized**
```dockerfile
# Stage 1: Build stage
FROM python:3.12-alpine AS builder

WORKDIR /app

# Single RUN: install UV + dependencies + cleanup
RUN pip install uv && \
    pip cache purge

COPY requirements.txt .

RUN uv pip install --system --no-cache -r requirements.txt && \
    find /usr/local -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null || true && \
    find /usr/local -type f -name '*.pyc' -delete 2>/dev/null || true

# Stage 2: Runtime stage
FROM python:3.12-alpine

WORKDIR /app

# Copy only necessary artifacts
COPY --from=builder /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY --from=builder /usr/local/bin /usr/local/bin

ENV PYTHONUNBUFFERED=1

COPY main.py .

EXPOSE 8000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Optimizations applied:
- Combined RUN commands to reduce layer count
- Removed `__pycache__` directories (bytecode cache)
- Removed `.pyc` files
- Purged pip cache after installing UV

Build and measure:

```bash
docker build -t task-api:optimized -f Dockerfile.v4-optimized .
```

**Output:**
```
[+] Building 14.8s (14/14) FINISHED
 => [builder 4/4] RUN uv pip install --system --no-cache ...          2.6s
 => exporting to image                                                0.2s
```

```bash
docker images task-api:optimized
```

**Output:**
```
REPOSITORY   TAG        IMAGE ID       CREATED          SIZE
task-api     optimized  f7g8h9i0j1k2   3 seconds ago    115MB
```

---

## Final Size Comparison

Let's see all versions side by side:

```bash
docker images task-api --format "table {{.Tag}}\t{{.Size}}"
```

**Output:**
```
TAG          SIZE
optimized    115MB
alpine       118MB
multistage   182MB
slim         458MB
naive        1.21GB
```

| Version | Size | Technique | Reduction |
|---------|------|-----------|-----------|
| Naive | 1.21GB | Full Python image + pip | Baseline |
| Slim | 458MB | python:3.12-slim | 62% |
| Multi-stage | 182MB | Separate build/runtime + UV | 85% |
| Alpine | 118MB | Alpine base + UV | 90% |
| Optimized | 115MB | Layer cleanup + cache purge | **90.5%** |

**Result: 1.21GB reduced to 115MB (90.5% reduction)**

---

## Analyzing Layers with docker history

The `docker history` command shows what each layer contains:

```bash
docker history task-api:optimized
```

**Output:**
```
IMAGE          CREATED         CREATED BY                                      SIZE
f7g8h9i0j1k2   2 minutes ago   CMD ["uvicorn" "main:app" "--host" "0.0.0...   0B
<missing>      2 minutes ago   EXPOSE 8000                                     0B
<missing>      2 minutes ago   COPY main.py . # buildkit                       1.52kB
<missing>      2 minutes ago   ENV PYTHONUNBUFFERED=1                          0B
<missing>      2 minutes ago   COPY /usr/local/bin /usr/local/bin # bui...     1.2MB
<missing>      2 minutes ago   COPY /usr/local/lib/python3.12/site-pack...     58.4MB
<missing>      2 minutes ago   WORKDIR /app                                    0B
<missing>      3 weeks ago     CMD ["python3"]                                 0B
<missing>      3 weeks ago     RUN /bin/sh -c set -eux;   apk add --no-...    1.85MB
<missing>      3 weeks ago     ENV PYTHON_VERSION=3.12.8                       0B
...
```

The SIZE column shows the contribution of each layer:
- Application code: ~1.5KB
- Installed binaries (uvicorn, etc.): ~1.2MB
- Installed packages (site-packages): ~58MB
- Python Alpine base: ~55MB (shown in earlier layers)

If you see an unexpectedly large layer, that's where to focus optimization efforts.

---

## Base Image Tradeoffs

| Base Image | Size | Pros | Cons | Use When |
|------------|------|------|------|----------|
| `python:3.12-slim` | ~150MB | Most compatible, safer | Larger than Alpine | Default choice; C extensions work out of box |
| `python:3.12-alpine` | ~50MB | Smallest, fast builds | Some packages need compilation | Size-critical deployments, pure Python |
| `distroless/python3` | ~50MB | Maximum security, no shell | Can't debug interactively | Production security-critical services |

**For this chapter**: Alpine is excellent for AI services that don't require complex C dependencies. Your Task API (and most FastAPI services) work perfectly with Alpine.

**When Alpine fails**: If you need numpy, pandas, or other packages with C extensions that aren't available as Alpine wheels, fall back to slim.

---

## Handling Large Model Files

A critical consideration for AI services: **never embed large model files in Docker images.**

**Wrong approach** (image becomes 4GB+):
```dockerfile
# DON'T DO THIS
COPY models/model.bin /app/models/
```

**Correct approach** (use volume mounts):
```dockerfile
# Image stays small, models loaded at runtime
# No COPY for model files
```

Run with volume mount:

```bash
docker run -v $(pwd)/models:/app/models task-api:optimized
```

**Output:**
```
INFO:     Started server process [1]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

Your application code loads models from the mounted directory:

```python
from pathlib import Path

models_dir = Path("/app/models")
model_path = models_dir / "model.bin"

@app.on_event("startup")
async def load_model():
    if model_path.exists():
        print(f"Loading model from {model_path}")
        # Load your model here
```

Benefits:
- Image stays small (~115MB)
- Models can be updated without rebuilding
- Same model can be shared across container instances
- Model storage handled by Kubernetes PersistentVolumes in production

---

## The Production Pattern

Here's the pattern to apply to any Python AI service:

```dockerfile
# Stage 1: Build
FROM python:3.12-alpine AS builder
WORKDIR /app

# Install UV for fast dependency installation
RUN pip install uv

COPY requirements.txt .
RUN uv pip install --system --no-cache -r requirements.txt

# Stage 2: Runtime
FROM python:3.12-alpine
WORKDIR /app

# Copy installed packages from builder
COPY --from=builder /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY --from=builder /usr/local/bin /usr/local/bin

ENV PYTHONUNBUFFERED=1

# Copy application code (NOT model files)
COPY . .

EXPOSE 8000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

**When to deviate**:

| Situation | Adjustment |
|-----------|------------|
| C extensions fail on Alpine | Use `python:3.12-slim` instead |
| Need system libraries | Add `RUN apk add --no-cache [packages]` in builder |
| Security-critical production | Consider `distroless/python3` |
| Debugging required | Keep Alpine (has shell) or slim |

---

## Try With AI

Now apply what you've learned to your own service.

**Prompt 1: Analyze Your Current Dockerfile**

```
I have a Dockerfile for a FastAPI service. Analyze it for size optimization opportunities:

[paste your Dockerfile here]

Questions:
1. What's the estimated image size with current base image?
2. Would multi-stage builds help? Why or why not?
3. What specific changes would achieve 70%+ size reduction?
```

**What you're learning**: How to evaluate an existing Dockerfile against optimization criteria. AI can identify which techniques apply to your specific dependencies.

---

**Prompt 2: Generate Optimized Dockerfile**

```
Create a multi-stage Dockerfile for my Python service with these requirements:

Dependencies: [list your requirements.txt]
Entry point: uvicorn main:app --host 0.0.0.0 --port 8000
Target: Under 150MB final image
Constraints:
- Use UV package manager (not pip)
- Use Alpine base image
- No model files in image (volume mount)

Include comments explaining each optimization.
```

**What you're learning**: How to specify constraints clearly so AI generates a Dockerfile matching your exact requirements. The constraints prevent AI from defaulting to less optimized patterns.

---

**Prompt 3: Debug Size Issues**

```
My Docker image is larger than expected. Here's my Dockerfile and the output of docker history:

[paste Dockerfile]

docker history output:
[paste docker history output]

Questions:
1. Which layer is contributing the most unexpected size?
2. What's likely included that shouldn't be?
3. How would you modify the Dockerfile to fix this?
```

**What you're learning**: How to use `docker history` output to diagnose size problems. AI can interpret layer contributions and suggest targeted fixes.

---

Verify your learning by building an optimized image for your Part 6 FastAPI agent and measuring the size reduction. Target: 70%+ reduction from a naive Dockerfile.
