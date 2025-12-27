---
sidebar_position: 8
title: "Capstone: Containerize Your API"
description: "Apply specification-first containerization to build a production-ready Docker image for your SQLModel + Neon Task API"
keywords: [docker, capstone, containerization, production, registry, specification]
chapter: 49
lesson: 8
duration_minutes: 60
proficiency_level: B1
teaching_stage: 4
stage_name: "Spec-Driven Integration"
stage_description: "Apply all Docker skills through specification-first capstone project"

skills:
  - name: "Specification-First Containerization"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Create"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student writes containerization specification before implementation"

  - name: "Multi-Stage Dockerfile Design"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student applies build/runtime separation pattern"

  - name: "Environment Variable Configuration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student configures database connection via environment variable"

  - name: "Container Registry Workflow"
    proficiency_level: "B1"
    category: "Procedural"
    bloom_level: "Apply"
    digcomp_area: "3.4 Programming"
    measurable_at_this_level: "Student pushes image to Docker Hub or GHCR"

  - name: "Cross-Machine Validation"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Evaluate"
    digcomp_area: "5.3 Problem Solving"
    measurable_at_this_level: "Student validates container works on different machine"

learning_objectives:
  - objective: "Write specification for containerization project BEFORE implementation"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Complete specification document with intent, constraints, and success criteria"

  - objective: "Apply multi-stage build pattern to generate production Dockerfile"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Dockerfile has separate build and runtime stages"

  - objective: "Containerize SQLModel + Neon Task API from Chapter 40"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Container runs Task API with database connectivity"

  - objective: "Configure database connection via environment variable"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Container accepts DATABASE_URL at runtime"

  - objective: "Build production-ready container image under 200MB"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "docker images shows final image under 200MB"

  - objective: "Push container image to registry"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Image accessible on Docker Hub or GHCR"

  - objective: "Validate container works on different machine"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Container pulled and run successfully on separate machine"

cognitive_load:
  new_concepts: 5
  assessment: "Specification-first workflow, skill composition, DATABASE_URL configuration, registry push, cross-machine validation"

differentiation:
  extension_for_advanced: "Add health check endpoint, implement semantic versioning for tags"
  remedial_for_struggling: "Use provided specification template, focus on single registry"
---

# Capstone: Containerize Your API

Throughout this chapter, you've built Docker knowledge step by step: container fundamentals, Dockerfile syntax, lifecycle management, multi-stage builds. Now it's time to apply everything to a real production scenario.

In Chapter 40, you built a Task API with SQLModel and Neon PostgreSQL. It works on your machine. But "works on my machine" doesn't ship products. Your teammates can't run it without matching your Python version, installing the same dependencies, and configuring their environment variables.

This capstone changes that. You'll write a specification FIRST, then containerize your API using the patterns from this chapter. The result: a portable container image that runs identically on your laptop, a teammate's machine, or a cloud server.

The specification-first approach is critical. Jumping straight to code is the Vibe Coding anti-pattern. Writing the spec first forces you to think about constraints (image size, security, configuration) before touching any Docker commands.

---

## Phase 1: Write the Specification FIRST

Before any implementation, you write a specification. This is the specification-first approach that separates professional development from Vibe Coding. The spec defines WHAT you're building and HOW you'll know it works.

**Create this file in your project directory:**

**File: containerization-spec.md**

```markdown
# Containerization Specification: Task API

## Intent

Containerize the SQLModel + Neon Task API for production deployment.

**Business Goal**: Enable any developer to run this API without environment setup.

**Technical Goal**: Create a portable, optimized container image that works anywhere Docker runs.

## Constraints

### Image Size
- **Target**: Under 200MB final image
- **Rationale**: Smaller images push/pull faster, reduce storage costs

### Security
- **Non-root user**: Container runs as unprivileged user
- **Health check**: Built-in endpoint for orchestrator monitoring
- **No secrets in image**: Database URL passed at runtime

### Configuration
- **DATABASE_URL**: Environment variable (not hardcoded)
- **PORT**: Configurable, defaults to 8000

### Base Image
- **Choice**: python:3.12-alpine (small, secure)
- **Alternative**: python:3.12-slim (if Alpine compatibility issues)

## Success Criteria

- [ ] Container builds successfully without errors
- [ ] Image size under 200MB (verify with `docker images`)
- [ ] All CRUD endpoints work when running containerized
- [ ] Health check endpoint responds at `/health`
- [ ] Container can connect to Neon database with provided DATABASE_URL
- [ ] Image can be pushed to registry (Docker Hub or GHCR)
- [ ] Image can be pulled and run on different machine
- [ ] Container runs as non-root user

## Non-Goals (What We're NOT Doing)

- [ ] Docker Compose multi-service setup (separate lesson)
- [ ] Kubernetes deployment (Chapter 50)
- [ ] CI/CD automation (future topic)
- [ ] GPU support (not needed for this API)

## Dependencies

- SQLModel Task API code from Chapter 40 Lesson 7
- Neon PostgreSQL database with connection string
- Docker Desktop installed and running
- Registry account (Docker Hub or GitHub)
```

**Why specification first?**

Without a spec, you'd start typing `FROM python:3.12` and figure things out as you go. That's Vibe Coding. You might forget security constraints. You might not consider image size until it's 1.2GB. You might hardcode secrets.

The spec makes constraints explicit BEFORE you start. It's your contract with yourself.

---

## Phase 2: Prepare the Application

Before writing the Dockerfile, ensure your Task API code is ready for containerization.

**Your project should have this structure:**

```
task-api/
├── main.py           # FastAPI application
├── models.py         # SQLModel Task definition
├── database.py       # Engine and session management
├── config.py         # Settings with DATABASE_URL
├── requirements.txt  # Dependencies
└── containerization-spec.md  # The spec you just wrote
```

**Verify requirements.txt includes all dependencies:**

```
fastapi==0.115.0
uvicorn==0.30.0
sqlmodel==0.0.22
psycopg2-binary==2.9.9
pydantic-settings==2.5.2
```

**Update config.py to read DATABASE_URL from environment:**

```python
# config.py
from pydantic_settings import BaseSettings
from functools import lru_cache


class Settings(BaseSettings):
    database_url: str

    class Config:
        env_file = ".env"


@lru_cache
def get_settings() -> Settings:
    return Settings()
```

**Add a health check endpoint to main.py:**

```python
# Add this to main.py (if not already present)

@app.get("/health")
def health_check():
    """Health check endpoint for container orchestrators."""
    return {"status": "healthy", "service": "task-api"}
```

**Output:**
```json
{"status": "healthy", "service": "task-api"}
```

---

## Phase 3: Apply Multi-Stage Build Pattern

Now apply the multi-stage build pattern from Lesson 5. Reference your specification: under 200MB, Alpine base, non-root user.

**File: Dockerfile**

```dockerfile
# =============================================================================
# Stage 1: Build Stage
# Purpose: Install dependencies with build tools (discarded after build)
# =============================================================================
FROM python:3.12-alpine AS builder

WORKDIR /app

# Install UV for fast dependency installation
RUN pip install --no-cache-dir uv

# Copy requirements first (layer caching)
COPY requirements.txt .

# Install dependencies to user directory
RUN uv pip install --system --no-cache -r requirements.txt

# =============================================================================
# Stage 2: Runtime Stage
# Purpose: Minimal production image with only necessary files
# =============================================================================
FROM python:3.12-alpine

WORKDIR /app

# Create non-root user for security
RUN adduser -D -u 1000 appuser

# Copy installed packages from builder
COPY --from=builder /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY --from=builder /usr/local/bin /usr/local/bin

# Copy application code
COPY main.py .
COPY models.py .
COPY database.py .
COPY config.py .

# Set ownership to non-root user
RUN chown -R appuser:appuser /app

# Switch to non-root user
USER appuser

# Environment configuration
ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

# Expose port (documentation, doesn't publish)
EXPOSE 8000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD wget --no-verbose --tries=1 --spider http://localhost:8000/health || exit 1

# Run the application
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

**Key design decisions (trace back to spec):**

| Spec Requirement | Dockerfile Implementation |
|------------------|---------------------------|
| Under 200MB | python:3.12-alpine base, multi-stage build |
| Non-root user | `adduser appuser`, `USER appuser` |
| Health check | HEALTHCHECK instruction with wget |
| No secrets in image | DATABASE_URL passed at runtime via -e flag |
| Configurable port | Exposed via EXPOSE, configurable in CMD |

---

## Phase 4: Build and Validate Locally

Build the image and validate against success criteria from your spec.

**Build the image:**

```bash
docker build -t task-api:v1 .
```

**Output:**
```
[+] Building 12.3s (15/15) FINISHED
 => [internal] load build definition from Dockerfile
 => [builder 1/4] FROM python:3.12-alpine
 => [builder 2/4] RUN pip install --no-cache-dir uv
 => [builder 3/4] COPY requirements.txt .
 => [builder 4/4] RUN uv pip install --system --no-cache -r requirements.txt
 => [stage-1 1/7] FROM python:3.12-alpine
 => [stage-1 2/7] COPY --from=builder /usr/local/lib/python...
 => exporting to image
```

**Check image size (spec: under 200MB):**

```bash
docker images task-api:v1
```

**Output:**
```
REPOSITORY   TAG   IMAGE ID       CREATED         SIZE
task-api     v1    a1b2c3d4e5f6   30 seconds ago  145MB
```

145MB is well under the 200MB target from your specification.

**Run the container with DATABASE_URL:**

```bash
docker run -d \
  -p 8000:8000 \
  -e DATABASE_URL="postgresql://user:pass@ep-xxx.region.aws.neon.tech/neondb?sslmode=require" \
  --name task-api-container \
  task-api:v1
```

**Verify container is running:**

```bash
docker ps
```

**Output:**
```
CONTAINER ID   IMAGE        COMMAND                  STATUS          PORTS
f7g8h9i0j1k2   task-api:v1  "uvicorn main:app..."   Up 10 seconds   0.0.0.0:8000->8000/tcp
```

**Test health check (spec: health endpoint responds):**

```bash
curl http://localhost:8000/health
```

**Output:**
```json
{"status":"healthy","service":"task-api"}
```

**Test CRUD endpoints (spec: all endpoints work):**

```bash
# Create a task
curl -X POST http://localhost:8000/tasks \
  -H "Content-Type: application/json" \
  -d '{"title": "Test containerized API"}'

# List tasks
curl http://localhost:8000/tasks
```

**Output:**
```json
{"id":1,"title":"Test containerized API","description":null,"status":"pending","created_at":"2024-01-15T10:30:00"}
```

**Verify non-root user (spec: container runs as non-root):**

```bash
docker exec task-api-container whoami
```

**Output:**
```
appuser
```

---

## Phase 5: Push to Container Registry

Your image works locally. Now push it to a registry so anyone can pull and run it.

### Option A: Docker Hub

**Step 1: Log in to Docker Hub**

```bash
docker login
```

Enter your Docker Hub username and password when prompted.

**Step 2: Tag the image for your Docker Hub account**

```bash
docker tag task-api:v1 yourusername/task-api:v1
docker tag task-api:v1 yourusername/task-api:latest
```

Replace `yourusername` with your actual Docker Hub username.

**Step 3: Push to Docker Hub**

```bash
docker push yourusername/task-api:v1
docker push yourusername/task-api:latest
```

**Output:**
```
The push refers to repository [docker.io/yourusername/task-api]
a1b2c3d4e5f6: Pushed
b2c3d4e5f6a7: Pushed
v1: digest: sha256:abc123... size: 1234
```

### Option B: GitHub Container Registry (GHCR)

**Step 1: Create a personal access token**

Go to GitHub Settings > Developer settings > Personal access tokens > Generate new token. Select `write:packages` scope.

**Step 2: Log in to GHCR**

```bash
echo $GITHUB_TOKEN | docker login ghcr.io -u yourusername --password-stdin
```

**Step 3: Tag and push**

```bash
docker tag task-api:v1 ghcr.io/yourusername/task-api:v1
docker push ghcr.io/yourusername/task-api:v1
```

---

## Phase 6: Cross-Machine Validation

The ultimate test: can someone else run your container? This validates the spec requirement "Image can be pulled and run on different machine."

**On a different machine (or a cloud VM):**

**Step 1: Pull the image**

```bash
docker pull yourusername/task-api:v1
```

**Step 2: Run with your Neon DATABASE_URL**

```bash
docker run -d \
  -p 8000:8000 \
  -e DATABASE_URL="postgresql://user:pass@ep-xxx.region.aws.neon.tech/neondb?sslmode=require" \
  --name task-api \
  yourusername/task-api:v1
```

**Step 3: Verify endpoints work**

```bash
curl http://localhost:8000/health
curl http://localhost:8000/tasks
```

**What you just proved:**

"Works on my machine" is now "Works everywhere Docker runs." Your teammate doesn't need:
- The same Python version
- The same operating system
- To run `pip install` for 50 packages
- To configure environment variables manually

They run one command, and the entire environment is identical to yours.

---

## Specification Checklist: Final Validation

Go back to your specification and verify each success criterion:

| Success Criterion | Status | Evidence |
|-------------------|--------|----------|
| Container builds successfully | PASS | `docker build` completed without errors |
| Image size under 200MB | PASS | `docker images` shows 145MB |
| All CRUD endpoints work | PASS | curl commands return expected responses |
| Health check responds | PASS | `/health` returns `{"status":"healthy"}` |
| Connects to Neon database | PASS | Tasks persist across container restarts |
| Pushed to registry | PASS | Image visible on Docker Hub/GHCR |
| Runs on different machine | PASS | Pulled and executed successfully |
| Runs as non-root | PASS | `whoami` returns `appuser` |

**All criteria met. Specification satisfied.**

---

## Common Issues and Solutions

### Issue: Container exits immediately

**Check logs:**
```bash
docker logs task-api-container
```

**Common causes:**
- Missing DATABASE_URL environment variable
- Invalid database connection string
- Python import errors

### Issue: Cannot connect to database

**Verify DATABASE_URL is passed correctly:**
```bash
docker exec task-api-container env | grep DATABASE
```

**Ensure sslmode=require is present** (required for Neon):
```
DATABASE_URL=postgresql://user:pass@host/db?sslmode=require
```

### Issue: Health check failing

**Debug by running health check manually:**
```bash
docker exec task-api-container wget --spider http://localhost:8000/health
```

**Check if uvicorn started:**
```bash
docker logs task-api-container | head -20
```

### Issue: Permission denied errors

**Check if non-root user has access to files:**
```bash
docker exec task-api-container ls -la /app
```

All files should be owned by `appuser`.

---

## Try With AI

You've completed the capstone manually. Now extend your containerization skills through AI collaboration.

**Prompt 1: Specification Review**

```text
Review my containerization specification for gaps:

[Paste your containerization-spec.md content]

Questions to consider:
- What security constraints am I missing?
- Should I add any non-goals to prevent scope creep?
- What edge cases should my success criteria cover?
```

**What you're learning:** AI can review specifications and identify blind spots you might have missed. It might suggest constraints you hadn't considered, like secrets rotation, log aggregation, or graceful shutdown handling. You evaluate each suggestion against your project's actual needs.

**Prompt 2: Dockerfile Optimization**

```text
Here's my production Dockerfile for a FastAPI + SQLModel service:

[Paste your Dockerfile]

Analyze it against these criteria:
- Is the image as small as possible?
- Are there security improvements I'm missing?
- Is layer caching optimized for rebuild speed?

Suggest specific improvements with explanations.
```

**What you're learning:** AI can identify optimization opportunities in your Dockerfile. It might suggest removing unused binaries, combining RUN commands, or using more specific COPY paths. You evaluate each suggestion against your specification constraints.

**Prompt 3: Tagging Strategy**

```text
I'm pushing my task-api image to Docker Hub. Help me design a tagging strategy that includes:
- Latest tag for convenience
- Semantic version tags (v1.0.0, v1.1.0)
- Git commit hash tags for traceability

Show me the docker tag and docker push commands for this workflow.
Explain when I would use each tag type.
```

**What you're learning:** Production registries need thoughtful tagging. `latest` is convenient but dangerous in production. Semantic versions communicate compatibility. Git hashes enable exact reproduction. AI helps you design a strategy that balances convenience and safety.

**Safety note:** When sharing specifications or Dockerfiles with AI, redact actual database credentials and sensitive configuration. Replace real values with placeholders like `user:pass` or `YOUR_CONNECTION_STRING`.
