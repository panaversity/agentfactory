---
sidebar_position: 5
chapter: 49
lesson: 5
duration_minutes: 55
title: "Multi-Stage Builds & Optimization"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Iterative optimization teaches image size reduction techniques through hands-on practice"
cognitive_load:
  concepts_count: 9
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Explain why multi-stage builds reduce image size"
    bloom_level: "Understand"
  - id: LO2
    description: "Write a multi-stage Dockerfile with build and runtime stages"
    bloom_level: "Create"
  - id: LO3
    description: "Use UV package manager for faster Python dependency installation"
    bloom_level: "Apply"
  - id: LO4
    description: "Compare slim, alpine, and distroless base images"
    bloom_level: "Analyze"
  - id: LO5
    description: "Optimize layers by combining RUN commands and cleaning caches"
    bloom_level: "Apply"
  - id: LO6
    description: "Copy only necessary artifacts from build to runtime stage"
    bloom_level: "Apply"
  - id: LO7
    description: "Measure image size before and after optimization"
    bloom_level: "Evaluate"
  - id: LO8
    description: "Handle large model files (>1GB) with volume mounts"
    bloom_level: "Apply"
  - id: LO9
    description: "Use BuildKit for faster, more efficient builds"
    bloom_level: "Apply"
digcomp_mapping:
  - objective_id: LO2
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO5
    competency_area: "2. Digital Communication and Collaboration"
    competency: "2.1 Problem Solving"
---

# Multi-Stage Builds & Optimization

Container images that work are good. Container images that work AND are small are great. This lesson teaches you why—and more importantly, how.

When you build a Docker image for a Python service, you typically need two things during the build process: a compiler and development libraries to install dependencies. But in production, you only need the installed packages themselves. A naive Dockerfile includes everything—build tools, development headers, cache files—all of which add hundreds of megabytes of unnecessary weight.

Multi-stage builds solve this elegantly. You perform the heavy lifting (dependency installation, compilation) in a large build image, then copy only the artifacts you need into a small production image. The result: images that are 85-90% smaller, faster to push, faster to pull, and have smaller attack surfaces.

In this lesson, you'll start with a bloated Dockerfile and progressively optimize it through multiple iterations. You'll measure the size reduction at each step, understand the tradeoffs, and learn techniques you'll use in every production Dockerfile going forward.

---

## The Problem: Bloated Images

Let's start with a naive Dockerfile that doesn't think about image size at all.

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

And a minimal FastAPI application to containerize:

**File: requirements.txt**
```
fastapi==0.115.0
uvicorn==0.30.0
pydantic==2.6.0
```

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

Build this image and check its size:

```bash
docker build -t naive-image:latest -f Dockerfile.naive .
docker images naive-image:latest
```

**Output:**
```
REPOSITORY     TAG      IMAGE ID       CREATED         SIZE
naive-image    latest   a1b2c3d4e5f6   10 seconds ago  1.2GB
```

1.2 gigabytes for a tiny FastAPI app. That bloat comes from:
- **Full Python image**: ~900MB (includes compilers, development headers, build tools)
- **Pip cache**: ~150MB (stored in `/root/.cache/pip`)
- **Development dependencies**: ~150MB (libraries needed only during installation)

None of that is needed to RUN the application. You only need the installed Python packages themselves—maybe 100-150MB total.

---

## Iteration 1: Use a Slim Base Image

The `python:3.12` image is the full-featured version. Docker provides alternatives:

- **python:3.12** (full) — ~900MB, includes build tools, development headers, compilers
- **python:3.12-slim** (slim) — ~150MB, includes essentials but no build tools
- **python:3.12-alpine** (alpine) — ~50MB, minimal Linux, tiny footprint
- **distroless/python3.12** (distroless) — ~50MB, only runtime, no shell or package manager

For most cases, **slim** strikes the right balance: small enough to matter, but large enough to have essential libraries for most Python packages.

**File: Dockerfile.v1-slim**
```dockerfile
FROM python:3.12-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install -r requirements.txt

COPY main.py .

EXPOSE 8000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Build and measure:

```bash
docker build -t slim-image:latest -f Dockerfile.v1-slim .
docker images slim-image:latest
```

**Output:**
```
REPOSITORY    TAG      IMAGE ID       CREATED        SIZE
slim-image    latest   f6e5d4c3b2a1   5 seconds ago  450MB
```

**Progress**: 1.2GB → 450MB (62% reduction)

Better, but we're still carrying unnecessary files. The Python slim image has development libraries needed during installation (like gcc for compiling C extensions), but we don't need them in the final image.

---

## Iteration 2: Multi-Stage Builds (Separate Build & Runtime)

Multi-stage builds use multiple `FROM` instructions in a single Dockerfile. Each stage can use a different base image. You build dependencies in a large stage, then copy only what you need into a small stage.

**File: Dockerfile.v2-multistage**
```dockerfile
# Stage 1: Build stage (large image with build tools)
FROM python:3.12-slim AS builder

WORKDIR /app

COPY requirements.txt .

# Install dependencies, but keep them in the builder stage
RUN pip install --user --no-cache-dir -r requirements.txt

# Stage 2: Runtime stage (small image with only what's needed)
FROM python:3.12-slim

WORKDIR /app

# Copy installed packages from builder stage
COPY --from=builder /root/.local /root/.local

# Set PATH so Python finds installed packages
ENV PATH=/root/.local/bin:$PATH \
    PYTHONUNBUFFERED=1

# Copy application code
COPY main.py .

EXPOSE 8000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Let's understand what's happening:

**Stage 1 (builder)**:
- Starts with `python:3.12-slim` (all build tools available)
- Installs dependencies with `pip install --user` (stores in `/root/.local`)
- This stage is used only for building; it's discarded when the build finishes

**Stage 2 (runtime)**:
- Also starts with `python:3.12-slim` (fresh, clean slate)
- Copies `/root/.local` from builder stage (all installed packages)
- Copies application code
- Does NOT include build tools, development headers, or pip cache
- This is the final image Docker keeps

Build and measure:

```bash
docker build -t multistage-image:latest -f Dockerfile.v2-multistage .
docker images multistage-image:latest
```

**Output:**
```
REPOSITORY       TAG      IMAGE ID       CREATED        SIZE
multistage-image latest   d3c2b1a0f9e8   3 seconds ago  180MB
```

**Progress**: 1.2GB → 180MB (85% reduction from original)

---

## Iteration 3: Use Alpine Base Image + UV Package Manager

Alpine is a minimal Linux distribution (50MB vs 150MB for slim). It's tiny but requires careful package selection because some Python packages expect standard Linux tools.

More importantly, we'll introduce **UV**, a Rust-based Python package manager that's 10-100x faster than pip while using less memory.

**File: Dockerfile.v3-alpine-uv**
```dockerfile
# Stage 1: Build stage with Alpine (minimal)
FROM python:3.12-alpine AS builder

WORKDIR /app

# Install UV package manager
RUN pip install uv

COPY requirements.txt .

# UV installs 10-100x faster than pip and uses less memory
# --system installs to system Python instead of virtual environment
RUN uv pip install --system --no-cache -r requirements.txt

# Stage 2: Runtime stage with Alpine (minimal)
FROM python:3.12-alpine

WORKDIR /app

COPY --from=builder /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY --from=builder /usr/local/bin /usr/local/bin

ENV PYTHONUNBUFFERED=1

COPY main.py .

EXPOSE 8000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Key changes:

- **Alpine base**: 50MB instead of 150MB
- **UV package manager**: Installs dependencies 10-100x faster
- **--system flag**: UV installs to system Python, not a virtual environment (simpler copying)
- **--no-cache**: Doesn't cache pip data in the builder stage

Build and measure:

```bash
docker build -t alpine-uv-image:latest -f Dockerfile.v3-alpine-uv .
docker images alpine-uv-image:latest
```

**Output:**
```
REPOSITORY     TAG      IMAGE ID       CREATED        SIZE
alpine-uv-image latest  e4f5a6b7c8d9   2 seconds ago  120MB
```

**Progress**: 1.2GB → 120MB (90% reduction from original)

---

## Iteration 4: Combine RUN Commands & Clean Caches

Docker builds images in layers. Each `RUN` instruction creates a new layer. Layers are cached, which speeds up subsequent builds, but it also means intermediate files are retained.

By combining `RUN` commands, you reduce layers and can clean up intermediate files in the same layer (so they don't persist).

**File: Dockerfile.v4-optimized**
```dockerfile
# Stage 1: Build stage
FROM python:3.12-alpine AS builder

WORKDIR /app

# Single RUN command: install UV + dependencies + clean cache
RUN pip install uv && \
    pip cache purge

COPY requirements.txt .

RUN uv pip install --system --no-cache -r requirements.txt && \
    rm -rf /root/.cache && \
    find /usr/local -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null || true

# Stage 2: Runtime stage
FROM python:3.12-alpine

WORKDIR /app

# Minimal runtime: copy only what's needed
COPY --from=builder /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY --from=builder /usr/local/bin /usr/local/bin

# Set environment variables in a single RUN to reduce layers
RUN echo "PYTHONUNBUFFERED=1" >> /etc/environment

COPY main.py .

EXPOSE 8000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Optimizations:

- **Combined RUN commands**: Fewer layers = smaller overhead
- **Explicit cache cleanup**: `pip cache purge`, removed `__pycache__` directories
- **Clean intermediate artifacts**: Anything created during build but not needed is discarded

Build and measure:

```bash
docker build -t optimized-image:latest -f Dockerfile.v4-optimized .
docker images optimized-image:latest
```

**Output:**
```
REPOSITORY      TAG      IMAGE ID       CREATED        SIZE
optimized-image latest   f7g8h9i0j1k2   2 seconds ago  118MB
```

**Progress**: Nearly the same (118MB vs 120MB). Cleanup saved only 2MB because UV already doesn't cache. But the technique is important for other package managers and dependencies.

---

## Understanding Base Image Tradeoffs

Let's summarize the three base image options and when to use each:

| Base Image | Size | Use Case | Tradeoff |
|------------|------|----------|----------|
| **python:3.12-slim** | 150MB | Default choice for most apps | Includes build tools you might not need |
| **python:3.12-alpine** | 50MB | Size-critical deployments (Kubernetes, edge) | Missing some standard libraries; C extensions may not compile |
| **distroless/python3.12** | 50MB | Maximum security (no shell, no package manager) | Can't debug in container; requires all dependencies pre-installed |

For this lesson, **slim is the safest**, and **alpine is the best** for containerized AI services where size matters. **Distroless** is advanced—useful for production security but harder to debug.

---

## Handling Large Model Files (>1GB)

A Dockerfile with `COPY model.bin .` would embed a 4GB model file into the image itself. That's wasteful: the image would be 4GB+, slow to push, slow to pull, and duplicated on every machine.

Instead, use **volume mounts** to inject model files at runtime:

**File: Dockerfile.v4-optimized (no model file)**
```dockerfile
# Same as before—NO COPY of model files
FROM python:3.12-alpine AS builder
# ... rest same ...

FROM python:3.12-alpine
# ... rest same ...
```

**Run with volume mount:**
```bash
docker run -v $(pwd)/models:/app/models optimized-image:latest
```

**Or in docker-compose.yaml:**
```yaml
services:
  app:
    image: optimized-image:latest
    volumes:
      - ./models:/app/models
    ports:
      - "8000:8000"
```

**Application code** (main.py):
```python
from fastapi import FastAPI
from pathlib import Path

app = FastAPI()

# Models loaded from volume-mounted directory at runtime
models_dir = Path("/app/models")

@app.on_event("startup")
async def load_model():
    global model
    model_path = models_dir / "model.bin"
    # Load model from file
    print(f"Loading model from {model_path}")
    # model = load_model_function(model_path)

@app.get("/")
def read_root():
    return {"message": "Model loaded from volume mount"}
```

**Benefits**:
- Image stays small (no model embedded)
- Models can be shared across containers (single volume mount point)
- Models can be updated without rebuilding image
- Perfect for AI services with large model files

---

## Measuring Progress: docker images and docker history

Docker provides commands to inspect image size and layers:

**docker images** shows the final image size:
```bash
docker images
```

**Output:**
```
REPOSITORY      TAG       IMAGE ID       CREATED        SIZE
optimized-image latest    f7g8h9i0j1k2   2 seconds ago  118MB
naive-image     latest    a1b2c3d4e5f6   15 mins ago    1.2GB
slim-image      latest    f6e5d4c3b2a1   10 mins ago    450MB
```

**docker history** shows what each layer contains:
```bash
docker history optimized-image:latest
```

**Output:**
```
IMAGE          CREATED        CREATED BY                                      SIZE
f7g8h9i0j1k2   2 seconds ago  CMD ["uvicorn" "main:app" "--host" "0.0.0.0"]  0B
<missing>      2 seconds ago  COPY main.py . # buildkit                       5.2kB
<missing>      2 seconds ago  RUN /bin/sh -c echo "PYTHONUNBUFFERED=1"...    42B
<missing>      2 seconds ago  COPY --from=builder /usr/local/bin...          15MB
<missing>      2 seconds ago  COPY --from=builder /usr/local/lib/python...   103MB
```

Each line is a layer. The SIZE column shows how much that layer added. If you see a large layer, that's where to optimize.

---

## BuildKit: Faster, Smarter Builds

Docker's modern build system uses **BuildKit**, which offers advanced caching and parallel stage execution.

BuildKit is enabled by default in Docker Desktop and modern versions. You can explicitly enable it:

```bash
export DOCKER_BUILDKIT=1
docker build -t optimized-image:latest -f Dockerfile.v4-optimized .
```

**Output:**
```
[+] Building 8.2s (13/13) FINISHED
 => [internal] load build definition from Dockerfile              0.0s
 => => transferring dockerfile: 1.2kB                             0.0s
 => [builder 1/3] FROM python:3.12-alpine                         2.3s
 => [builder 2/3] RUN pip install uv && pip cache purge           4.1s
 => [builder 3/3] RUN uv pip install --system --no-cache...       1.5s  <<---- Fast!
 => [stage-1 1/5] FROM python:3.12-alpine                         0.0s  (reused)
 => [stage-1 2/5] COPY --from=builder /usr/local/lib/python...   0.2s
 => [stage-1 3/5] COPY --from=builder /usr/local/bin ...          0.1s
 => [stage-1 4/5] RUN echo "PYTHONUNBUFFERED=1"...               0.2s
 => [stage-1 5/5] COPY main.py .                                  0.1s
 => exporting to image                                            0.3s
```

BuildKit advantages:
- **Parallel stage execution**: Stages with no dependencies run in parallel
- **Advanced caching**: Smarter cache invalidation
- **Faster builds**: Noticeably quicker for large Dockerfiles

---

## Practice: Optimize Your Own Dockerfile

You now have a template for optimized multi-stage builds. Here's the pattern to apply to any Python service:

**Pattern: Multi-stage Dockerfile for Python AI Services**

```dockerfile
# Stage 1: Build (large, has build tools)
FROM python:3.12-alpine AS builder

WORKDIR /app

# Install UV for fast dependency installation
RUN pip install uv

COPY requirements.txt .

# Install dependencies with UV
RUN uv pip install --system --no-cache -r requirements.txt

# Stage 2: Runtime (small, only what's needed)
FROM python:3.12-alpine

WORKDIR /app

# Copy installed packages from builder
COPY --from=builder /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY --from=builder /usr/local/bin /usr/local/bin

# Set environment
ENV PYTHONUNBUFFERED=1

# Copy application code
COPY . .

# For AI services with models: models come via volume mount, not COPY
# COPY models /app/models  <<---- Don't do this for large files!

EXPOSE 8000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

**When to deviate from this pattern**:

- **Alpine not available for your stack**: Use python:3.12-slim
- **Static files needed at build time**: COPY them in builder stage, but only if <100MB
- **Model files required**: Use volume mounts, never COPY large files
- **Security-critical**: Consider distroless base image (but lose debugging capability)
- **Need specific system libraries**: Add RUN apk add ... in builder, copy results to runtime stage

---

## Try With AI

**Setup**: You have a FastAPI agent service from Part 6. Now you'll containerize it with an optimized multi-stage Dockerfile.

**Prompts**:

**Part 1: Initial Dockerfile**
Ask AI: "I have a FastAPI service with dependencies [list your requirements.txt]. Create a multi-stage Dockerfile that minimizes image size. Use python:3.12-alpine as base image and UV for package installation."

**Part 2: Critical Evaluation**
Review AI's response. Ask yourself:
- Are there two stages (build and runtime)?
- Does the runtime stage copy only necessary files from builder?
- Does it use UV with `--no-cache` flag?
- Is alpine used as the final base image?

**Part 3: Size Validation**
Build the image and check its size:
```bash
docker build -t my-agent:optimized .
docker images my-agent:optimized
```

Compare to a naive version:
```bash
# Naive: single stage, full Python image
docker build -t my-agent:naive -f Dockerfile.naive .
docker images
```

Measure the size reduction. You should see 70-85% reduction from naive to optimized.

**Part 4: Layer Analysis**
Examine the layers:
```bash
docker history my-agent:optimized
```

Ask yourself:
- What's the largest layer?
- Are RUN commands combined where possible?
- Could you remove any intermediate files?

**Part 5: Production Readiness**
Consider:
- If you have model files >1GB, did you remove COPY and plan for volume mount instead?
- Is the image size now suitable for pushing to a registry?
- Does the image run your service correctly (test locally first)?

Compare your final optimized image to the initial naive version. Document the size reduction and the key optimizations that made the biggest difference.
