---
sidebar_position: 6
chapter: 49
lesson: 6
duration_minutes: 55
title: "Docker Compose for Development"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Declarative compose files teach infrastructure-as-code thinking before automation"
cognitive_load:
  concepts_count: 9
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Write a docker compose.yaml file with multiple services"
    bloom_level: "Create"
  - id: LO2
    description: "Configure service dependencies with depends_on and health checks"
    bloom_level: "Apply"
  - id: LO3
    description: "Use networks to enable service-to-service communication"
    bloom_level: "Apply"
  - id: LO4
    description: "Persist data with named volumes across container restarts"
    bloom_level: "Apply"
  - id: LO5
    description: "Configure environment variables with .env files"
    bloom_level: "Apply"
  - id: LO6
    description: "Use bind mounts for live code reloading during development"
    bloom_level: "Apply"
  - id: LO7
    description: "Manage multi-container lifecycle with docker compose up/down"
    bloom_level: "Apply"
  - id: LO8
    description: "Override base compose configuration with compose.override.yaml"
    bloom_level: "Apply"
  - id: LO9
    description: "View logs across all services with docker compose logs"
    bloom_level: "Apply"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO2
    competency_area: "2. Digital Communication and Collaboration"
    competency: "2.1 Problem Solving"
---

# Docker Compose for Development

So far, you've containerized a single FastAPI service. But real applications are orchestras—your agent needs a database to store memory, a cache to serve common responses fast, and sometimes other microservices. Managing three separate containers by hand (starting each one, networking them, configuring volumes) is tedious and error-prone.

Docker Compose is a declarative tool that orchestrates multi-container applications. You write a `compose.yaml` file that describes your entire application—all services, their dependencies, networks, volumes, and environment—then a single command brings everything up. It's like an infrastructure blueprint that turns many manual steps into one reproducible workflow.

In this lesson, you'll design and build a development environment for your FastAPI agent that includes PostgreSQL (for data persistence) and Redis (for session caching). You won't use generators. You'll write the compose.yaml by hand, understanding each service and how they communicate.

---

## Real Application Architecture: Why Multiple Services?

Your FastAPI agent from previous lessons works, but it only lives in memory. Restart the container and all conversation history is lost. Real applications need:

1. **API Service** (FastAPI) — Your agent logic
2. **Database** (PostgreSQL) — Persist memories, user conversations, agent state
3. **Cache** (Redis) — Session data, frequently accessed responses

These three services need to:
- Start together reliably
- Communicate over a private network (service A calls service B by name)
- Persist data even when containers restart (database and cache volumes)
- Load environment variables for configuration

Docker Compose handles all of this.

---

## Understanding Compose.yaml Structure

Before writing code, let's understand the specification. A `compose.yaml` file has this structure:

```yaml
version: '3.9'
services:
  service_name_1:
    # Service definition
  service_name_2:
    # Service definition
networks:
  # Network definitions
volumes:
  # Volume definitions
```

The four main sections:

1. **services** — Each service is a container (API, database, cache)
2. **networks** — Virtual networks that services use to communicate
3. **volumes** — Persistent storage that survives container restarts
4. **environment** — Configuration values (from .env files)

Let's apply this to your agent architecture.

---

## Your Development Environment Specification

Here's the architecture you're building:

```
┌─────────────────────────────────────────────┐
│           Docker Compose Network             │
├─────────────────────────────────────────────┤
│                                              │
│  ┌──────────────┐  ┌──────────┐  ┌────────┐ │
│  │   FastAPI    │  │PostgreSQL│  │ Redis  │ │
│  │   Agent      │  │Database  │  │ Cache  │ │
│  │ (port 8000)  │  │ (5432)   │  │(6379)  │ │
│  │              │  │          │  │        │ │
│  │ Bind mount   │  │Named     │  │Named   │ │
│  │ for live     │  │volume    │  │volume  │ │
│  │ reload       │  │pgdata    │  │redisdb │ │
│  │              │  │          │  │        │ │
│  └──────────────┘  └──────────┘  └────────┘ │
│        │                │            │       │
│        └────────────────┴────────────┘       │
│         (Communicate via service names)      │
│                                              │
└─────────────────────────────────────────────┘
```

**Services communicate by name**:
- FastAPI connects to database at `postgresql://postgres:postgres@db:5432/agent_db`
- FastAPI connects to cache at `redis://cache:6379`
- Service names (db, cache) resolve automatically in the Compose network

---

## Writing Your Compose File

Create a new directory for your multi-service project:

```bash
mkdir my-agent-app
cd my-agent-app
```

**Output:**
```
$ mkdir my-agent-app
$ cd my-agent-app
```

Now create the `compose.yaml` file. This is the complete specification:

**File: compose.yaml**

```yaml
version: '3.9'

services:
  # FastAPI Agent Service
  api:
    build: .
    container_name: agent-api
    ports:
      - "8000:8000"
    environment:
      - DATABASE_URL=postgresql://postgres:postgres@db:5432/agent_db
      - REDIS_URL=redis://cache:6379
      - LOG_LEVEL=info
    depends_on:
      db:
        condition: service_healthy
      cache:
        condition: service_started
    volumes:
      - .:/app                    # Bind mount: sync local code
      - /app/__pycache__          # Exclude Python cache
    networks:
      - agent-network
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8000/health"]
      interval: 10s
      timeout: 5s
      retries: 3
      start_period: 10s

  # PostgreSQL Database
  db:
    image: postgres:16-alpine
    container_name: agent-db
    environment:
      POSTGRES_USER: postgres
      POSTGRES_PASSWORD: postgres
      POSTGRES_DB: agent_db
    volumes:
      - pgdata:/var/lib/postgresql/data
    networks:
      - agent-network
    healthcheck:
      test: ["CMD-SHELL", "pg_isready -U postgres"]
      interval: 5s
      timeout: 5s
      retries: 5
    ports:
      - "5432:5432"

  # Redis Cache
  cache:
    image: redis:7-alpine
    container_name: agent-cache
    volumes:
      - redisdata:/data
    networks:
      - agent-network
    ports:
      - "6379:6379"
    command: redis-server --appendonly yes

networks:
  agent-network:
    driver: bridge

volumes:
  pgdata:
    driver: local
  redisdata:
    driver: local
```

Save this file as `compose.yaml` in your project directory.

---

## Understanding Each Service Configuration

### The API Service

```yaml
api:
  build: .                         # Build image from Dockerfile in current dir
  container_name: agent-api        # Name the container
  ports:
    - "8000:8000"                  # Map host:container port
  environment:                     # Pass environment variables
    - DATABASE_URL=...             # Database connection string
    - REDIS_URL=...                # Cache connection string
  depends_on:                      # Wait for dependencies
    db:
      condition: service_healthy   # Wait for health check pass
    cache:
      condition: service_started   # Cache doesn't need health check
  volumes:
    - .:/app                       # Bind mount current dir
    - /app/__pycache__             # Named mount to exclude Python cache
  networks:
    - agent-network                # Connect to this network
```

**Key points:**

- `build: .` means "use the Dockerfile in the current directory" (same Dockerfile from Lesson 3)
- `environment` sets variables the Python app reads (more on this later)
- `depends_on` ensures database is ready before API starts
- `volumes` with `.:/app` syncs your local code into the container (live reload)

### The Database Service

```yaml
db:
  image: postgres:16-alpine        # Use pre-built image
  environment:
    POSTGRES_USER: postgres        # Default user
    POSTGRES_PASSWORD: postgres    # Default password
    POSTGRES_DB: agent_db          # Database name
  volumes:
    - pgdata:/var/lib/postgresql/data  # Persist data
  healthcheck:
    test: ["CMD-SHELL", "pg_isready -U postgres"]
    interval: 5s
    timeout: 5s
    retries: 5
```

**Key points:**

- `image` pulls a pre-built PostgreSQL image (no Dockerfile needed)
- `healthcheck` ensures the database is ready before other services depend on it
- `volumes` with named volume `pgdata:` persists data even if container restarts

### The Cache Service

```yaml
cache:
  image: redis:7-alpine
  volumes:
    - redisdata:/data
  command: redis-server --appendonly yes
```

**Key points:**

- Lightweight Redis image
- `command` overrides the default startup command (enables persistence with `--appendonly yes`)
- Named volume persists cache data

### Networks and Volumes

```yaml
networks:
  agent-network:
    driver: bridge                 # Bridge network for service-to-service communication

volumes:
  pgdata:
    driver: local
  redisdata:
    driver: local
```

**Key points:**

- Network creates a virtual bridge so services communicate by name
- Volumes create persistent storage outside containers
- Both are created automatically by Compose

---

## Creating the Application Files

Now create the Python files that use the environment variables:

**File: main.py**

```python
from fastapi import FastAPI
import os
import psycopg2
import redis

app = FastAPI()

# Read environment variables
DATABASE_URL = os.getenv("DATABASE_URL", "not configured")
REDIS_URL = os.getenv("REDIS_URL", "not configured")
LOG_LEVEL = os.getenv("LOG_LEVEL", "info")

@app.get("/")
def read_root():
    return {
        "message": "Agent ready with full infrastructure",
        "database": DATABASE_URL.split("@")[1] if "@" in DATABASE_URL else "not connected",
        "cache": REDIS_URL.split("//")[1] if "//" in REDIS_URL else "not connected"
    }

@app.get("/health")
def health_check():
    return {"status": "healthy"}

@app.get("/config")
def get_config():
    return {
        "log_level": LOG_LEVEL,
        "database_configured": bool(DATABASE_URL and "not configured" not in DATABASE_URL),
        "cache_configured": bool(REDIS_URL and "not configured" not in REDIS_URL)
    }
```

**File: requirements.txt**

```
fastapi==0.115.0
uvicorn[standard]==0.32.0
psycopg2-binary==2.9.9
redis==5.0.1
```

**File: Dockerfile**

(Same as Lesson 3)

```dockerfile
FROM python:3.12-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY main.py .

EXPOSE 8000

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000", "--reload"]
```

Note: This Dockerfile includes `--reload` flag so changes to `main.py` restart the server automatically.

**Verify all files exist:**

```bash
ls -la
```

**Output:**

```
$ ls -la
total 48
-rw-r--r--  1 you  staff  1247 Dec 22 11:00 compose.yaml
-rw-r--r--  1 you  staff   589 Dec 22 11:00 main.py
-rw-r--r--  1 you  staff  1047 Dec 22 11:00 requirements.txt
-rw-r--r--  1 you  staff  112 Dec 22 11:00 Dockerfile
-rw-r--r--  1 you  staff   96 Dec 22 11:00 .dockerignore
```

---

## Starting Your Multi-Container Application

Now bring everything up with a single command:

```bash
docker compose up
```

**Output:**

```
$ docker compose up
[+] Building 45.2s (7/7) FINISHED
[+] Running 3/3
 ✔ Container agent-db     Healthy
 ✔ Container agent-cache  Running
 ✔ Container agent-api    Running

agent-db        | 2024-12-22 11:05:15.123 UTC [1] LOG:  database system is ready to accept connections
agent-cache    | 1:M 22 Dec 2024 11:05:15.234 * Ready to accept connections
agent-api       | INFO:     Uvicorn running on http://0.0.0.0:8000
agent-api       | INFO:     Application startup complete
```

All three services are running. The database and cache are healthy. Your API is listening on port 8000.

**In another terminal, test the API:**

```bash
curl http://localhost:8000/
```

**Output:**

```json
{
  "message": "Agent ready with full infrastructure",
  "database": "db:5432/agent_db",
  "cache": "cache:6379"
}
```

The API successfully read the environment variables pointing to the database and cache services.

**Check configuration:**

```bash
curl http://localhost:8000/config
```

**Output:**

```json
{
  "log_level": "info",
  "database_configured": true,
  "cache_configured": true
}
```

---

## Service-to-Service Communication

Services communicate by name through the Compose network. Let's verify this works:

**Connect to the API container and test from inside:**

```bash
docker compose exec api bash
```

**Output:**

```
root@agent-api:/app#
```

You're inside the API container. Try to reach the database by name:

```bash
apt-get update && apt-get install -y postgresql-client
psql -h db -U postgres -d agent_db -c "SELECT 1;"
```

**Output:**

```
 ?column?
----------
        1
(1 row)
```

The database is reachable at hostname `db` from inside the network. This automatic service discovery is what makes Compose powerful.

Exit the container:

```bash
exit
```

---

## Persistent Storage: Named Volumes

Stop the containers:

```bash
docker compose down
```

**Output:**

```
[+] Running 3/3
 ✔ Container agent-api     Removed
 ✔ Container agent-db      Removed
 ✔ Container agent-cache   Removed
```

Now start them again:

```bash
docker compose up -d
```

**Output:**

```
[+] Creating 3/3
 ✔ Container agent-db     Running
 ✔ Container agent-cache  Running
 ✔ Container agent-api    Running
```

The containers restarted. But did the data persist? The named volumes (`pgdata`, `redisdata`) are still on your system, independent of containers.

Verify by listing volumes:

```bash
docker volume ls | grep agent
```

**Output:**

```
DRIVER    VOLUME NAME
local     my-agent-app_pgdata
local     my-agent-app_redisdata
```

Both volumes exist and persist data across container restarts.

---

## Live Code Reloading with Bind Mounts

The `compose.yaml` includes this configuration:

```yaml
volumes:
  - .:/app                    # Bind mount: sync local code
  - /app/__pycache__          # Exclude Python cache
```

The bind mount (`.:/app`) syncs your local directory into the container. The Dockerfile's `--reload` flag restarts the server when files change.

**Test this:**

Edit `main.py` and add a new endpoint:

```python
@app.get("/agent-status")
def agent_status():
    return {"status": "running", "mode": "development with live reload"}
```

Save the file. Check the API logs:

```bash
docker compose logs api
```

**Output:**

```
agent-api | INFO:     Shutting down
agent-api | INFO:     Restarting due to changes in '/app/main.py'
agent-api | INFO:     Uvicorn running on http://0.0.0.0:8000
agent-api | INFO:     Application startup complete
```

The server restarted automatically! Test the new endpoint:

```bash
curl http://localhost:8000/agent-status
```

**Output:**

```json
{"status": "running", "mode": "development with live reload"}
```

This is development efficiency: edit code, refresh browser/rerun curl, see changes instantly.

---

## Viewing Logs from All Services

When multiple containers run, viewing logs gets complex. Docker Compose aggregates them:

```bash
docker compose logs
```

**Output:**

```
agent-api    | INFO:     Application startup complete
agent-db     | 2024-12-22 11:15:03.456 UTC [1] LOG:  database system is ready to accept connections
agent-cache  | 1:M 22 Dec 2024 11:15:03.567 * Ready to accept connections
```

View logs from a specific service:

```bash
docker compose logs db
```

**Output:**

```
agent-db | 2024-12-22 11:15:03.456 UTC [1] LOG:  database system is ready to accept connections
```

Follow logs in real-time:

```bash
docker compose logs -f api
```

**Output:**

```
agent-api | INFO:     Application startup complete
agent-api | INFO:     Uvicorn running on http://0.0.0.0:8000
(Waiting for new logs...)
```

Press Ctrl+C to stop following.

---

## Managing Containers: Up, Down, and Restart

Stop all containers without removing them:

```bash
docker compose stop
```

**Output:**

```
[+] Stopping 3/3
 ✔ Container agent-api     Stopped
 ✔ Container agent-db      Stopped
 ✔ Container agent-cache   Stopped
```

Restart them:

```bash
docker compose start
```

**Output:**

```
[+] Starting 3/3
 ✔ Container agent-api     Started
 ✔ Container agent-db      Started
 ✔ Container agent-cache   Started
```

Stop and remove everything (but keep volumes):

```bash
docker compose down
```

**Output:**

```
[+] Removing 3/3
 ✔ Container agent-api     Removed
 ✔ Container agent-db      Removed
 ✔ Container agent-cache   Removed
```

Verify:

```bash
docker compose ps
```

**Output:**

```
NAME      COMMAND   SERVICE   STATUS      PORTS
```

All gone. But volumes still exist:

```bash
docker volume ls | grep agent
```

**Output:**

```
DRIVER    VOLUME NAME
local     my-agent-app_pgdata
local     my-agent-app_redisdata
```

To remove everything INCLUDING volumes:

```bash
docker compose down -v
```

**Output:**

```
[+] Removing 3/3
 ✔ Container agent-api     Removed
 ✔ Container agent-db      Removed
 ✔ Container agent-cache   Removed
[+] Removing 2/2
 ✔ Volume my-agent-app_pgdata    Removed
 ✔ Volume my-agent-app_redisdata Removed
```

Complete cleanup.

---

## Development vs Production: Using Compose Overrides

In development you want live reload and all ports exposed. In production you want minimal resource usage and security hardening.

Docker Compose supports `compose.override.yaml` for development-specific settings:

**File: compose.override.yaml**

(Only needed if different from production. For now, your `compose.yaml` IS your development config.)

When you're ready for production, you'd create a separate `compose.prod.yaml` and use:

```bash
docker compose -f compose.yaml -f compose.prod.yaml up
```

This merges the base config with production overrides.

For now, you have one `compose.yaml` that defines your complete development environment.

---

## Your Mental Model

You now understand:

1. **Multi-container architecture**: API, database, and cache services working together
2. **Service communication**: Services reach each other by name through the Compose network
3. **Environment variables**: Applications read configuration from Compose environment definitions
4. **Persistent storage**: Named volumes survive container restarts
5. **Live development**: Bind mounts enable code changes to reload without rebuilding
6. **Lifecycle management**: `up`, `down`, `stop`, `start` commands control the entire application
7. **Logging**: `docker compose logs` aggregates output from all services
8. **Health checks**: Dependencies wait for services to be healthy before starting
9. **Specification first**: Your `compose.yaml` is the infrastructure-as-code blueprint

This foundation prepares you for Kubernetes (Chapter 50), where you'll orchestrate containers at production scale. But first, let's practice.

---

## Try With AI

### Part 1: Examine Current Services

List all running containers:

```bash
docker compose ps
```

Ask yourself: What is the role of each service (api, db, cache)? Why do they need to be separate?

### Part 2: Modify the API

Edit `main.py` to add a `/stats` endpoint that returns memory usage or connection count. Save the file.

Verify in logs that the server reloaded:

```bash
docker compose logs api | grep -i "restarting\|startup"
```

Test the new endpoint. Did it work without restarting the entire application?

### Part 3: Challenge — Add a New Service

Try adding a fourth service to your `compose.yaml`:
- An Nginx reverse proxy on port 80 that routes to the API

Hint: Use `image: nginx:latest` and configure port mapping. Can you start everything with `docker compose up` without errors?

### Part 4: Inspect the Network

Services communicate via the internal network. View network details:

```bash
docker network ls | grep agent
docker network inspect <network-name>
```

How many containers are connected to the `agent-network`? What are their IP addresses?

Your next step: Push this multi-container application to a container registry (Docker Hub), then orchestrate it with Kubernetes.
