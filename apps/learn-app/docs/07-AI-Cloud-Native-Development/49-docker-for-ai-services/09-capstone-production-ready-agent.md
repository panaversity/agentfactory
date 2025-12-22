---
sidebar_position: 9
chapter: 49
lesson: 9
duration_minutes: 90
title: "Capstone: Production-Ready Agent Container"
proficiency_level: B1
teaching_stage: 4
stage_name: "Spec-Driven Development"
stage_description: "Translate a natural language specification into a production-ready containerized AI agent"
cognitive_load:
  concepts_count: 10
  scaffolding_level: "Light"
learning_objectives:
  - id: LO1
    description: "Transform natural language requirements into a formal Docker specification"
    bloom_level: "Create"
  - id: LO2
    description: "Write production multi-stage Dockerfiles applying all Docker lessons"
    bloom_level: "Create"
  - id: LO3
    description: "Implement security best practices from Lesson 7 in a complete application"
    bloom_level: "Apply"
  - id: LO4
    description: "Create compose.yaml orchestrating agent, database, and cache services"
    bloom_level: "Create"
  - id: LO5
    description: "Add health checks and readiness probes for Kubernetes deployment"
    bloom_level: "Apply"
  - id: LO6
    description: "Configure CI/CD pipelines for automated image building and testing"
    bloom_level: "Create"
  - id: LO7
    description: "Tag and push images to container registries (Docker Hub, GHCR)"
    bloom_level: "Apply"
  - id: LO8
    description: "Verify container functionality in clean deployment environments"
    bloom_level: "Evaluate"
  - id: LO9
    description: "Optimize final image size using layer caching and dependency analysis"
    bloom_level: "Apply"
  - id: LO10
    description: "Document container API, configuration, and deployment procedures"
    bloom_level: "Create"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO2
    competency_area: "2. Digital Communication and Collaboration"
    competency: "2.1 Problem Solving"
---

# Capstone: Production-Ready Agent Container

Everything you've learned about Docker comes together here. You have a working FastAPI AI agent from Part 6, and now you'll package it for production—a container that runs identically on your laptop, a teammate's machine, or a cloud cluster.

This isn't a tutorial where you follow steps blindly. This is specification-first development: you'll transform a natural language requirement into a formal specification, then use that specification to guide your implementation. By the end, you'll have a production-grade container that's ready for Kubernetes deployment.

---

## Starting Point: The Natural Language Specification

Your goal:

> "Containerize my FastAPI AI agent for production deployment. The container should run on any system with Docker, include health checks for Kubernetes, work with PostgreSQL and Redis dependencies, and be ready to push to a container registry."

This is what stakeholders give you. Now you'll translate it into a specification precise enough to drive implementation.

---

## Formalizing Your Specification

### What Are We Building?

**Container image:** A complete, self-contained FastAPI agent service with all dependencies, security hardening, and operational readiness.

**Success criteria:**
- Multi-stage build produces image under 200MB
- Non-root user enforces security (no running as root)
- Health check endpoint responds within 3 seconds
- Environment-based configuration (no hardcoded values)
- PostgreSQL and Redis connectivity optional but detected
- Passes security scanning (no critical vulnerabilities)
- Pushed to registry with semantic versioning (1.0.0, 1.0.1, etc.)
- Runs identically in clean environment (verified by running on different machine)

### Constraints

- Target Python 3.12 slim base image (minimal footprint)
- No GPU support in this capstone (cloud GPU availability is variable)
- Cold start time under 10 seconds from container launch
- Health check must be self-contained (no external dependencies)

### Non-Goals

We're NOT covering:
- Multi-architecture builds (ARM64 + x86 in single image)
- GPU image variants
- Private image registry authentication
- Helm chart packaging (that's Chapter 51)

---

## Your FastAPI Agent Application

You'll containerize this complete application. Create these files in a new directory:

**Directory structure:**
```
my-ai-agent/
├── main.py
├── requirements.txt
├── .dockerignore
├── Dockerfile
├── docker-compose.yaml
└── .github/workflows/build-push.yml
```

**File: main.py**
```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import os
import logging

# Configure logging
logging.basicConfig(level=os.getenv("LOG_LEVEL", "INFO"))
logger = logging.getLogger(__name__)

app = FastAPI(
    title="AI Agent Service",
    version=os.getenv("APP_VERSION", "1.0.0"),
    docs_url="/docs",
    openapi_url="/openapi.json"
)

class QueryRequest(BaseModel):
    prompt: str
    max_tokens: int = 100

class QueryResponse(BaseModel):
    response: str
    tokens_used: int

@app.get("/health", tags=["operations"])
async def health_check():
    """Liveness probe: Is the container running?"""
    return {
        "status": "healthy",
        "version": os.getenv("APP_VERSION", "1.0.0")
    }

@app.get("/ready", tags=["operations"])
async def readiness_check():
    """Readiness probe: Is the service ready for traffic?"""
    db_url = os.getenv("DATABASE_URL")
    redis_url = os.getenv("REDIS_URL")

    return {
        "ready": True,
        "checks": {
            "database": "configured" if db_url else "not_configured",
            "cache": "configured" if redis_url else "not_configured",
            "api": "responsive"
        }
    }

@app.post("/query", response_model=QueryResponse, tags=["agent"])
async def query_agent(request: QueryRequest):
    """Process a query through the AI agent."""
    if not request.prompt or len(request.prompt) == 0:
        raise HTTPException(status_code=400, detail="Prompt cannot be empty")

    if request.max_tokens < 10 or request.max_tokens > 2000:
        raise HTTPException(status_code=400, detail="max_tokens must be between 10 and 2000")

    logger.info(f"Processing query: {request.prompt[:50]}...")

    # Simulated agent response
    response_text = f"Processed: {request.prompt[:100]}..." if len(request.prompt) > 100 else f"Processed: {request.prompt}"
    tokens = len(request.prompt.split())

    return QueryResponse(
        response=response_text,
        tokens_used=tokens
    )

@app.get("/metrics", tags=["operations"])
async def metrics():
    """Expose basic metrics for monitoring."""
    return {
        "app_name": "AI Agent Service",
        "version": os.getenv("APP_VERSION", "1.0.0"),
        "environment": os.getenv("ENVIRONMENT", "development")
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=int(os.getenv("PORT", 8000))
    )
```

**File: requirements.txt**
```
fastapi==0.115.0
uvicorn[standard]==0.32.0
pydantic==2.9.0
pydantic-settings==2.3.0
python-dotenv==1.0.1
```

**Output:** (when you install these dependencies)
```
$ pip install -r requirements.txt
Successfully installed fastapi-0.115.0 uvicorn-0.32.0 pydantic-2.9.0 pydantic-settings-2.3.0 python-dotenv-1.0.1
```

---

## Writing the Production Dockerfile

This Dockerfile applies everything from Lessons 1-6: multi-stage builds (Lesson 4), security hardening (Lesson 7), and optimization.

**File: Dockerfile**
```dockerfile
# Stage 1: Build
FROM python:3.12-slim AS builder

WORKDIR /build

# Install build tools minimally
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install UV for fast, deterministic dependency installation
RUN pip install --no-cache-dir uv==0.4.0

# Copy and install dependencies
COPY requirements.txt .
RUN uv pip install --system --no-cache-dir -r requirements.txt

# Stage 2: Runtime
FROM python:3.12-slim

LABEL org.opencontainers.image.title="AI Agent Service"
LABEL org.opencontainers.image.version="1.0.0"
LABEL org.opencontainers.image.description="Production-ready FastAPI AI agent container"

# Create non-root user (security best practice from Lesson 7)
RUN groupadd -r agent && useradd -r -g agent agent

WORKDIR /app

# Copy dependencies from builder stage (multi-stage optimization)
COPY --from=builder /usr/local/lib/python3.12/site-packages /usr/local/lib/python3.12/site-packages
COPY --from=builder /usr/local/bin /usr/local/bin

# Copy application code
COPY --chown=agent:agent main.py .

# Create non-writable app directory to prevent tampering
RUN chmod 555 /app

# Switch to non-root user
USER agent

# Health check (Lesson 5 - readiness checks)
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD python -c "import urllib.request; urllib.request.urlopen('http://localhost:8000/health', timeout=2).read()" || exit 1

# Expose port
EXPOSE 8000

# Run application
CMD ["python", "-m", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

**Building the image:**
```bash
docker build -t ai-agent-service:1.0.0 .
```

**Expected output:**
```
[+] Building 45.3s (9/9) FINISHED
 => [internal] load build definition from Dockerfile
 => [builder 2/4] RUN apt-get update && apt-get install -y --no-install-recommends build-essential
 => [builder 3/4] RUN pip install --no-cache-dir uv==0.4.0
 => [builder 4/4] COPY requirements.txt .
 => [builder 5/4] RUN uv pip install --system --no-cache-dir -r requirements.txt
 => [stage-1 6/9] COPY --from=builder /usr/local/lib/python3.12/site-packages
 => [stage-1 7/9] COPY --chown=agent:agent main.py .
 => [stage-1 8/9] USER agent
 => => naming to docker.io/library/ai-agent-service:1.0.0
```

**Verify image size:**
```bash
docker images | grep ai-agent-service
```

**Expected output:**
```
REPOSITORY              TAG       IMAGE ID       CREATED              SIZE
ai-agent-service        1.0.0     a1b2c3d4e5f6   Less than 1 minute   187MB
```

Under 200MB—specification met.

---

## Docker Compose for Local Development

Now create a compose file that orchestrates the agent with PostgreSQL and Redis, matching your specification requirements.

**File: docker-compose.yaml**
```yaml
version: '3.8'

services:
  agent:
    build: .
    container_name: ai-agent
    ports:
      - "8000:8000"
    environment:
      APP_VERSION: "1.0.0"
      ENVIRONMENT: "development"
      DATABASE_URL: "postgresql://agent:password@postgres:5432/agents_db"
      REDIS_URL: "redis://redis:6379/0"
      LOG_LEVEL: "DEBUG"
    depends_on:
      postgres:
        condition: service_healthy
      redis:
        condition: service_healthy
    volumes:
      - ./main.py:/app/main.py  # Hot reload for development
    networks:
      - agent-network
    healthcheck:
      test: ["CMD", "python", "-c", "import urllib.request; urllib.request.urlopen('http://localhost:8000/health').read()"]
      interval: 10s
      timeout: 3s
      retries: 3
      start_period: 5s

  postgres:
    image: postgres:16-alpine
    container_name: agent-postgres
    environment:
      POSTGRES_USER: agent
      POSTGRES_PASSWORD: password
      POSTGRES_DB: agents_db
    volumes:
      - postgres_data:/var/lib/postgresql/data
    networks:
      - agent-network
    healthcheck:
      test: ["CMD-SHELL", "pg_isready -U agent"]
      interval: 10s
      timeout: 5s
      retries: 5

  redis:
    image: redis:7-alpine
    container_name: agent-redis
    volumes:
      - redis_data:/data
    networks:
      - agent-network
    healthcheck:
      test: ["CMD", "redis-cli", "ping"]
      interval: 10s
      timeout: 3s
      retries: 5

volumes:
  postgres_data:
  redis_data:

networks:
  agent-network:
    driver: bridge
```

**Test the compose setup:**
```bash
docker-compose up -d
```

**Expected output:**
```
[+] Running 4/4
 ✔ Network agent-network Created
 ✔ Container agent-postgres Created
 ✔ Container agent-redis Created
 ✔ Container ai-agent Created
```

**Verify all services are healthy:**
```bash
docker-compose ps
```

**Expected output:**
```
NAME               COMMAND                  SERVICE    STATUS           PORTS
ai-agent           "python -m uvicorn..."   agent      Up (healthy)     0.0.0.0:8000->8000/tcp
agent-postgres     "postgres"               postgres   Up (healthy)
agent-redis        "redis-server"           redis      Up (healthy)
```

**Test the health endpoint:**
```bash
curl -s http://localhost:8000/health | python -m json.tool
```

**Expected output:**
```json
{
  "status": "healthy",
  "version": "1.0.0"
}
```

---

## Pushing to a Container Registry

Your specification requires pushing to a registry. Let's push to Docker Hub (you can use GitHub Container Registry instead).

**Tag the image with your Docker username:**
```bash
docker tag ai-agent-service:1.0.0 <your-docker-username>/ai-agent-service:1.0.0
docker tag ai-agent-service:1.0.0 <your-docker-username>/ai-agent-service:latest
```

**Log in to Docker Hub:**
```bash
docker login
```

**Push the image:**
```bash
docker push <your-docker-username>/ai-agent-service:1.0.0
docker push <your-docker-username>/ai-agent-service:latest
```

**Expected output:**
```
1.0.0: Pushing
a1b2c3d4e5f6: Pushed
b2c3d4e5f6a7: Pushed
...
1.0.0: digest: sha256:abc123def456... size: 5432

latest: Pushing
...
latest: digest: sha256:abc123def456... size: 5432
```

**Verify the image is accessible:**
```bash
docker pull <your-docker-username>/ai-agent-service:1.0.0
```

**Expected output:**
```
1.0.0: Pulling from your-username/ai-agent-service
Pulling fs layer a1b2c3d4e5f6
Download complete
Digest: sha256:abc123def456...
Status: Downloaded newer image for your-username/ai-agent-service:1.0.0
```

---

## Verifying in a Clean Environment

Now test that your container works on a different system (or simulate this by removing the local image).

**Remove your local image:**
```bash
docker rmi ai-agent-service:1.0.0
```

**Pull from registry and run:**
```bash
docker run -d \
  --name test-agent \
  -p 9000:8000 \
  -e APP_VERSION="1.0.0" \
  <your-docker-username>/ai-agent-service:1.0.0
```

**Expected output:**
```
a1b2c3d4e5f6789abcdef0123456789abcdef012 (container ID)
```

**Test the running container:**
```bash
curl -s http://localhost:9000/health | python -m json.tool
```

**Expected output:**
```json
{
  "status": "healthy",
  "version": "1.0.0"
}
```

**Test a query:**
```bash
curl -X POST http://localhost:9000/query \
  -H "Content-Type: application/json" \
  -d '{"prompt": "Explain containerization in one sentence", "max_tokens": 50}'
```

**Expected output:**
```json
{
  "response": "Processed: Explain containerization in one sentence",
  "tokens_used": 7
}
```

---

## .dockerignore Configuration

Create a `.dockerignore` file to reduce image context and exclude unnecessary files from the build.

**File: .dockerignore**
```
__pycache__
*.pyc
*.pyo
*.pyd
.Python
env/
venv/
.venv
.git
.gitignore
.dockerignore
docker-compose*.yaml
.env
.env.local
.DS_Store
*.log
.pytest_cache
.coverage
htmlcov/
dist/
build/
*.egg-info/
.vscode/
.idea/
*.swp
*.swo
*~
node_modules/
README.md
LICENSE
```

---

## What You've Applied

This capstone integrates all Docker lessons:

1. **Container Fundamentals** (Lesson 1) — Understanding layers, images, and containers
2. **Writing Dockerfiles** (Lesson 3) — FROM, WORKDIR, COPY, RUN, CMD instructions
3. **Multi-Stage Builds** (Lesson 4) — Separate builder stage reducing final image size
4. **Dependency Management** (Lesson 5) — Using UV for deterministic installations, health checks
5. **Docker Compose** (Lesson 5) — Orchestrating agent, PostgreSQL, Redis with networks
6. **Security Best Practices** (Lesson 7) — Non-root user, minimal base image, read-only app dir
7. **Optimization** (throughout) — Multi-stage builds, layer caching, .dockerignore
8. **Registry Integration** — Tagging, pushing, pulling from Docker Hub

---

## Try With AI

**Part 1: Initial Implementation**

Ask AI: "I need to add environment-based logging configuration to my FastAPI container. Show me how to use Python's logging module with environment variables for LOG_LEVEL and LOG_FORMAT."

Review AI's response. Ask yourself:

- Does this logging approach work inside a container (stdout to Docker logs)?
- Can the LOG_LEVEL be changed at runtime with `-e` flags?
- Does this add security vulnerabilities (logging sensitive data)?

**Part 2: Constraint Teaching**

Based on your evaluation, tell AI your constraints:

"The logging must output to stdout (not files) so Docker can capture container logs. LOG_LEVEL should be configurable via environment variable but default to INFO. Don't log request bodies (they might contain sensitive prompts)."

**Part 3: Refinement**

Ask AI to revise: "Update the logging configuration to match these constraints. Show me the updated main.py."

**Part 4: Integration**

Compare the output to your current main.py:

- What changed in the logging implementation?
- How would you test this in the compose environment?
- Does this meet your specification requirements?

**Part 5: Final Verification**

Run your compose environment with the updated logging:

```bash
docker-compose down
docker-compose up -d
docker-compose logs agent
```

Does the logging output appear in Docker logs? Can you change LOG_LEVEL and see it take effect?

What improved through this iteration? What did you learn about logging in containerized environments that you might apply to future projects?
