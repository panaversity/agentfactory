---
sidebar_position: 6
title: "Production Hardening"
description: "Secure and harden Docker containers for production deployment with environment variables, health checks, and non-root users"
keywords:
  - docker
  - production
  - security
  - health checks
  - non-root user
  - environment variables
  - container hardening
chapter: 49
lesson: 6
duration_minutes: 45
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Manual hardening practices build security awareness before AI assistance"

skills:
  - name: "Container Security Configuration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "4. Safety"
    measurable_at_this_level: "Configure non-root users and health checks in production Dockerfiles"
  - name: "Environment Variable Management"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "3. Digital Content Creation"
    measurable_at_this_level: "Use ENV and ARG instructions appropriately for build-time and runtime configuration"
  - name: "Health Check Implementation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "5. Problem Solving"
    measurable_at_this_level: "Implement health check endpoints and Docker HEALTHCHECK instruction"

learning_objectives:
  - objective: "Configure environment variables for container configuration using ENV instruction"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Write Dockerfile with ENV instruction and verify at runtime"
  - objective: "Implement health check endpoints and Docker HEALTHCHECK instruction"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Add /health endpoint and HEALTHCHECK instruction, verify with docker inspect"
  - objective: "Run containers as non-root user for security"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Create non-root user in Dockerfile and verify container runs as that user"
  - objective: "Distinguish between ARG (build-time) and ENV (runtime) instructions"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Use both ARG and ENV in a Dockerfile and explain the difference"
  - objective: "Create production-ready Dockerfile combining all hardening patterns"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Write complete Dockerfile with all hardening patterns and verify it works"
  - objective: "Verify health check status using docker inspect"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Use docker inspect to check container health status"

cognitive_load:
  new_concepts: 8
  assessment: "B1-appropriate load with moderate scaffolding; builds directly on L05 multi-stage patterns"

differentiation:
  extension_for_advanced: "Implement custom health check scripts that verify database connectivity"
  remedial_for_struggling: "Focus on non-root user pattern first, add health checks after mastery"

digcomp_mapping:
  - objective_id: LO1
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO2
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
  - objective_id: LO3
    competency_area: "4. Safety"
    competency: "4.1 Protecting devices"
---

# Production Hardening

Your container builds and runs. The image is optimized with multi-stage builds. But production environments demand more than a working container—they require security, observability, and resilience.

Consider what happens when your containerized FastAPI agent service goes to production. Kubernetes needs to know if your service is healthy before routing traffic to it. If your container runs as root, a security vulnerability could give an attacker full control of the host. If configuration is hardcoded, you'll need to rebuild images for every environment (dev, staging, production).

Production hardening addresses these concerns through three patterns: environment variable configuration (flexibility), health checks (observability), and non-root users (security). These aren't optional extras—they're requirements for any serious deployment. In this lesson, you'll add each pattern to your Dockerfile, understand why it matters, and end up with a production-ready container template you'll use for every AI service you build.

---

## The Three Pillars of Production Hardening

Before diving into implementation, understand what we're solving:

| Pillar | Problem | Solution |
|--------|---------|----------|
| **Configuration** | Hardcoded values require image rebuilds | Environment variables at runtime |
| **Observability** | Orchestrators can't detect unhealthy containers | Health check endpoints + HEALTHCHECK instruction |
| **Security** | Root containers enable privilege escalation | Non-root user execution |

Each pillar is independent but together they form the foundation of production-ready containers. Let's implement each one.

---

## Environment Variables for Configuration

Hardcoded configuration creates fragile containers. If your Dockerfile specifies `LOG_LEVEL=INFO`, you need a new image for debug logging. If `API_HOST=production.example.com`, you can't run locally.

Docker provides two instructions for configuration:

- **ARG**: Build-time variables (available during `docker build`, NOT in running container)
- **ENV**: Runtime variables (available when container runs)

### Understanding ENV

The `ENV` instruction sets environment variables that persist into the running container:

```dockerfile
ENV PYTHONUNBUFFERED=1
ENV LOG_LEVEL=INFO
ENV API_HOST=0.0.0.0
ENV API_PORT=8000
```

Your application reads these values at runtime:

**File: main.py**
```python
import os
from fastapi import FastAPI

app = FastAPI()

LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")
API_HOST = os.getenv("API_HOST", "0.0.0.0")

@app.get("/")
def read_root():
    return {"message": "Hello from Docker!", "log_level": LOG_LEVEL}

@app.get("/health")
def health_check():
    return {"status": "healthy"}
```

**Output:**
```
$ docker run -p 8000:8000 my-app:latest
INFO:     Uvicorn running on http://0.0.0.0:8000

$ curl http://localhost:8000/
{"message":"Hello from Docker!","log_level":"INFO"}
```

### Overriding ENV at Runtime

The `-e` flag overrides environment variables when starting a container:

```bash
docker run -e LOG_LEVEL=DEBUG -e API_PORT=9000 -p 9000:9000 my-app:latest
```

**Output:**
```
$ docker run -e LOG_LEVEL=DEBUG -p 8000:8000 my-app:latest
INFO:     Uvicorn running on http://0.0.0.0:8000

$ curl http://localhost:8000/
{"message":"Hello from Docker!","log_level":"DEBUG"}
```

The container uses `DEBUG` instead of the default `INFO` without rebuilding the image.

### Understanding ARG

`ARG` defines build-time variables. They're available during `docker build` but not when the container runs:

```dockerfile
ARG PYTHON_VERSION=3.12

FROM python:${PYTHON_VERSION}-alpine

# ARG value is accessible here during build
RUN echo "Building with Python ${PYTHON_VERSION}"

# But NOT accessible at runtime
# The following would fail because ARG is gone after build:
# CMD ["echo", "${PYTHON_VERSION}"]
```

Build with different Python versions:

```bash
docker build --build-arg PYTHON_VERSION=3.11 -t my-app:py311 .
docker build --build-arg PYTHON_VERSION=3.12 -t my-app:py312 .
```

**Output:**
```
$ docker build --build-arg PYTHON_VERSION=3.11 -t my-app:py311 .
[+] Building 2.1s (8/8) FINISHED
 => [1/4] FROM docker.io/library/python:3.11-alpine
 => [2/4] RUN echo "Building with Python 3.11"
Building with Python 3.11
...
```

### When to Use ARG vs ENV

| Use Case | Instruction | Example |
|----------|-------------|---------|
| Python version for base image | ARG | `ARG PYTHON_VERSION=3.12` |
| Log level for running app | ENV | `ENV LOG_LEVEL=INFO` |
| Git commit hash for image tag | ARG | `ARG GIT_SHA` |
| API keys (runtime secrets) | ENV (via `-e`) | `-e API_KEY=abc123` |
| Package versions during build | ARG | `ARG UV_VERSION=0.4.0` |
| Feature flags in running container | ENV | `ENV ENABLE_CACHING=true` |

**Key distinction**: If you need the value when the container RUNS, use `ENV`. If you only need it during BUILD, use `ARG`.

---

## Health Check Implementation

Orchestrators like Kubernetes need to know if your container is healthy. A container can be "running" but completely broken—the process exists but crashes on every request. Health checks detect this.

### Adding a Health Endpoint to FastAPI

First, ensure your FastAPI service has a health endpoint:

**File: main.py**
```python
from fastapi import FastAPI
import os

app = FastAPI()

@app.get("/")
def read_root():
    return {"message": "Hello from Docker!"}

@app.get("/health")
def health_check():
    """Health check endpoint for Docker HEALTHCHECK and Kubernetes probes."""
    return {"status": "healthy"}
```

Test the endpoint:

```bash
uvicorn main:app --host 0.0.0.0 --port 8000 &
curl http://localhost:8000/health
```

**Output:**
```
$ curl http://localhost:8000/health
{"status":"healthy"}
```

### Docker HEALTHCHECK Instruction

The `HEALTHCHECK` instruction tells Docker how to verify container health:

```dockerfile
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD wget --no-verbose --tries=1 --spider http://localhost:8000/health || exit 1
```

Let's break down each component:

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `--interval=30s` | 30 seconds | How often to run the check |
| `--timeout=10s` | 10 seconds | How long to wait for response |
| `--start-period=5s` | 5 seconds | Grace period for container startup |
| `--retries=3` | 3 attempts | Failed checks before marking unhealthy |
| `CMD` | wget command | The actual health check command |

**Why wget instead of curl?**

Alpine images include `wget` by default but not `curl`. Using `wget` avoids adding dependencies. For slim-based images, use `curl`:

```dockerfile
# For Alpine-based images (wget built-in):
HEALTHCHECK CMD wget --no-verbose --tries=1 --spider http://localhost:8000/health || exit 1

# For slim-based images (curl built-in):
HEALTHCHECK CMD curl --fail http://localhost:8000/health || exit 1
```

### Verifying Health Check Status

Build and run a container with the health check:

```bash
docker build -t health-app:latest .
docker run -d --name health-test -p 8000:8000 health-app:latest
```

Wait 30 seconds for the first health check, then inspect:

```bash
docker inspect --format='{{json .State.Health}}' health-test | python -m json.tool
```

**Output:**
```json
{
    "Status": "healthy",
    "FailingStreak": 0,
    "Log": [
        {
            "Start": "2024-12-27T10:30:00.123456Z",
            "End": "2024-12-27T10:30:00.234567Z",
            "ExitCode": 0,
            "Output": ""
        }
    ]
}
```

The `"Status": "healthy"` confirms the health check passed. If the endpoint fails, status becomes `"unhealthy"` and `FailingStreak` increments.

### Health Check Status in docker ps

```bash
docker ps
```

**Output:**
```
CONTAINER ID   IMAGE             COMMAND                  STATUS                    PORTS
a1b2c3d4e5f6   health-app:latest "uvicorn main:app..."   Up 2 minutes (healthy)   0.0.0.0:8000->8000/tcp
```

Notice `(healthy)` in the STATUS column. This is how you quickly verify container health.

Clean up:

```bash
docker stop health-test && docker rm health-test
```

---

## Non-Root User Security

By default, Docker containers run as root. If an attacker exploits a vulnerability in your application, they have root access to the container—and potentially the host.

Running as a non-root user limits damage. Even if compromised, the attacker has limited privileges.

### Creating a Non-Root User

Add a dedicated user in your Dockerfile:

```dockerfile
# Create non-root user with specific UID
RUN adduser -D -u 1000 appuser
```

The flags:
- `-D`: Don't assign a password (non-interactive)
- `-u 1000`: Assign user ID 1000 (conventional for app users)
- `appuser`: Username

### Switching to Non-Root User

After creating the user, switch to it with `USER`:

```dockerfile
# Create user
RUN adduser -D -u 1000 appuser

# ... copy files with ownership ...

# Switch to non-root user BEFORE CMD
USER appuser

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Copying Files with Correct Ownership

Files copied into the container are owned by root by default. Use `--chown` to set ownership:

```dockerfile
# Copy with ownership set to appuser
COPY --chown=appuser:appuser main.py .
```

Without `--chown`, the non-root user can't read the files:

```bash
# Without --chown:
$ docker exec my-container ls -la /app/main.py
-rw-r--r-- 1 root root 237 Dec 27 10:00 main.py  # Owned by root

# With --chown:
$ docker exec my-container ls -la /app/main.py
-rw-r--r-- 1 appuser appuser 237 Dec 27 10:00 main.py  # Owned by appuser
```

### Verifying Non-Root Execution

Build and run, then check who's running the process:

```bash
docker build -t secure-app:latest .
docker run -d --name secure-test secure-app:latest
docker exec secure-test whoami
```

**Output:**
```
$ docker exec secure-test whoami
appuser
```

Not root. The container runs with limited privileges.

Clean up:

```bash
docker stop secure-test && docker rm secure-test
```

---

## The Production Dockerfile Template

Combining all three pillars—configuration, health checks, and non-root user—creates a production-ready template:

```dockerfile
# Stage 1: Build
FROM python:3.12-alpine AS builder

WORKDIR /app

# Install UV for fast dependency installation
RUN pip install uv

COPY requirements.txt .

# Install dependencies with UV
RUN uv pip install --system --no-cache -r requirements.txt

# Stage 2: Runtime
FROM python:3.12-alpine

# Create non-root user
RUN adduser -D -u 1000 appuser

WORKDIR /app

# Copy dependencies from builder
COPY --from=builder /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY --from=builder /usr/local/bin /usr/local/bin

# Copy application code with ownership
COPY --chown=appuser:appuser main.py .

# Environment configuration
ENV PYTHONUNBUFFERED=1
ENV LOG_LEVEL=INFO

# Switch to non-root user
USER appuser

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD wget --no-verbose --tries=1 --spider http://localhost:8000/health || exit 1

EXPOSE 8000

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

This template includes:
- Multi-stage build (from Lesson 5)
- UV for fast dependency installation
- Non-root user (`appuser`)
- Correct file ownership (`--chown`)
- Environment variable defaults
- Health check with appropriate timing
- Exposed port documentation

### Building and Testing the Complete Template

Create the required files:

**File: requirements.txt**
```
fastapi==0.115.0
uvicorn==0.30.0
```

**File: main.py**
```python
import os
from fastapi import FastAPI

app = FastAPI()

LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")

@app.get("/")
def read_root():
    return {"message": "Production hardened!", "log_level": LOG_LEVEL}

@app.get("/health")
def health_check():
    return {"status": "healthy"}
```

Build and run:

```bash
docker build -t production-app:latest .
docker run -d --name prod-test -p 8000:8000 production-app:latest
```

**Output:**
```
$ docker build -t production-app:latest .
[+] Building 12.3s (14/14) FINISHED
 => [builder 1/4] FROM python:3.12-alpine
 => [builder 2/4] WORKDIR /app
 => [builder 3/4] RUN pip install uv
 => [builder 4/4] RUN uv pip install --system --no-cache -r requirements.txt
 => [stage-1 1/6] FROM python:3.12-alpine
 => [stage-1 2/6] RUN adduser -D -u 1000 appuser
 => [stage-1 3/6] WORKDIR /app
 => [stage-1 4/6] COPY --from=builder /usr/local/lib/python3.12/site-packages...
 => [stage-1 5/6] COPY --from=builder /usr/local/bin /usr/local/bin
 => [stage-1 6/6] COPY --chown=appuser:appuser main.py .
 => exporting to image
Successfully tagged production-app:latest
```

Verify all three pillars:

```bash
# 1. Configuration: Override LOG_LEVEL
docker run --rm -e LOG_LEVEL=DEBUG production-app:latest sh -c 'echo $LOG_LEVEL'
```

**Output:**
```
DEBUG
```

```bash
# 2. Health check: Wait 30s, then check status
sleep 35
docker inspect --format='{{.State.Health.Status}}' prod-test
```

**Output:**
```
healthy
```

```bash
# 3. Non-root user: Verify process owner
docker exec prod-test whoami
```

**Output:**
```
appuser
```

All three pillars verified. Clean up:

```bash
docker stop prod-test && docker rm prod-test
```

---

## Common Hardening Mistakes

### Mistake 1: USER Before COPY

Placing `USER appuser` before `COPY` commands can cause permission errors:

```dockerfile
# WRONG: User can't copy because it doesn't own /app yet
USER appuser
COPY main.py .  # Fails: permission denied

# CORRECT: Copy first with ownership, then switch user
COPY --chown=appuser:appuser main.py .
USER appuser
```

### Mistake 2: Missing Health Endpoint

Adding `HEALTHCHECK` without implementing the endpoint:

```dockerfile
# Docker checks /health, but your app doesn't have that route
HEALTHCHECK CMD wget --spider http://localhost:8000/health || exit 1
```

**Result**: Container marked unhealthy immediately.

**Fix**: Always implement the health endpoint in your application code.

### Mistake 3: Wrong Port in HEALTHCHECK

The health check runs INSIDE the container. Use the container's internal port:

```dockerfile
# WRONG: 9000 is the host port, not container port
HEALTHCHECK CMD wget --spider http://localhost:9000/health || exit 1

# CORRECT: 8000 is the port your app listens on inside the container
HEALTHCHECK CMD wget --spider http://localhost:8000/health || exit 1
```

### Mistake 4: Secrets in ENV

Never put sensitive data in Dockerfile ENV instructions:

```dockerfile
# WRONG: Secret visible in image history and inspect
ENV API_KEY=sk-abc123secret

# CORRECT: Pass at runtime, never store in image
# docker run -e API_KEY=sk-abc123secret my-app:latest
```

Use `-e` flag at runtime or Docker secrets for sensitive configuration.

---

## Production Hardening Checklist

Before deploying any container to production, verify:

| Check | Command | Expected |
|-------|---------|----------|
| Non-root user | `docker exec <container> whoami` | Not `root` |
| Health check exists | `docker inspect --format='{{.Config.Healthcheck}}' <image>` | Non-empty |
| Health status | `docker inspect --format='{{.State.Health.Status}}' <container>` | `healthy` |
| No hardcoded secrets | `docker history <image>` | No API keys visible |
| Config via ENV | `docker run --rm -e LOG_LEVEL=DEBUG <image> env` | Variable overridable |

---

## Try With AI

**Setup**: You have the Task API from previous lessons. Now you'll apply production hardening patterns.

**Part 1: Analyze Your Dockerfile for Security Gaps**

Start with your existing Dockerfile from Lesson 5. Ask AI:

```
Review this Dockerfile for production readiness. Check for:
1. Running as root (security risk)
2. Missing health check (observability gap)
3. Hardcoded configuration (flexibility issue)

[paste your Dockerfile]

What hardening patterns are missing?
```

**What you're learning**: Identifying security and operational gaps before they cause production incidents. A systematic review catches issues that "it works" testing misses.

**Part 2: Design Health Check Strategy**

Your Task API has a `/tasks` endpoint. Ask AI:

```
My FastAPI service has these endpoints:
- GET /tasks (list tasks)
- POST /tasks (create task)
- GET /tasks/{id} (get specific task)

Design a health check strategy:
1. Should I create a dedicated /health endpoint or use an existing one?
2. What should the health check verify (database connection, API availability)?
3. What HEALTHCHECK parameters make sense for a task API?
```

**What you're learning**: Health check design requires thinking about what "healthy" means for your specific service. A simple HTTP 200 might miss database connectivity issues.

**Part 3: Apply Hardening to Your Own Project**

Take your Dockerfile and apply all three pillars. Ask AI:

```
Transform this Dockerfile to production-ready:
1. Add non-root user (appuser, UID 1000)
2. Add HEALTHCHECK using wget for Alpine
3. Add ENV defaults for LOG_LEVEL and PYTHONUNBUFFERED
4. Ensure correct file ownership with --chown

[paste your Dockerfile]

Show me the hardened version with comments explaining each security measure.
```

**What you're learning**: Applying patterns systematically transforms a development Dockerfile into production infrastructure. The comments help you understand each change so you can apply the same patterns to future projects.

**Safety Note**: Never include real API keys or secrets in Dockerfiles you share with AI. Use placeholder values like `API_KEY=your-key-here` and replace them with actual secrets at runtime using `-e` flags or Docker secrets.
