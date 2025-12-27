---
sidebar_position: 4
title: "Container Lifecycle and Debugging"
description: "Master container lifecycle management and debugging techniques using docker logs, docker exec, and docker inspect to troubleshoot containerized FastAPI services"
keywords: [docker logs, docker exec, docker inspect, container debugging, port conflicts, restart policies, OOM kills, container lifecycle]
chapter: 49
lesson: 4
duration_minutes: 40
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Debugging broken containers teaches troubleshooting skills before AI assistance"

# HIDDEN SKILLS METADATA
skills:
  - name: "Container Debugging"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "5. Problem Solving"
    measurable_at_this_level: "Diagnose container failures using logs and inspection tools"
  - name: "Container Lifecycle Management"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "3. Digital Content Creation"
    measurable_at_this_level: "Configure restart policies and resource limits"

learning_objectives:
  - objective: "View container logs with docker logs to diagnose startup failures"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Practical exercise: extract error message from failed container"
  - objective: "Execute commands inside running containers with docker exec for live inspection"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Exercise: verify file permissions inside container"
  - objective: "Inspect container configuration with docker inspect to verify settings"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Extract environment variables and port mappings from running container"
  - objective: "Debug port conflicts and resolve them using alternative port mappings"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Diagnose port conflict error and implement solution"
  - objective: "Configure restart policies for container resilience"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Configure unless-stopped policy and verify behavior"
  - objective: "Identify and fix common container startup failures including missing dependencies"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Debug container that fails due to missing environment variable"

cognitive_load:
  new_concepts: 6
  assessment: "Moderate scaffolding - builds on L03 Dockerfile knowledge with focused debugging scenarios"

differentiation:
  extension_for_advanced: "Debug multi-container networking issues, implement health checks with restart policies"
  remedial_for_struggling: "Focus on docker logs and docker ps before moving to docker exec and inspect"

digcomp_mapping:
  - objective_id: LO1
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
  - objective_id: LO2
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
  - objective_id: LO3
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
  - objective_id: LO4
    competency_area: "5. Problem Solving"
    competency: "5.2 Identifying needs and responses"
  - objective_id: LO5
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO6
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
---

# Container Lifecycle and Debugging

Your FastAPI service from Lesson 3 is running in a container. It responds to requests, returns JSON, and everything works. Then you deploy it to a server. It crashes. No error on your screen, no stack trace, nothing. The container simply stops.

This is where container debugging skills become essential. Unlike local development where errors appear in your terminal, containerized applications fail silently unless you know where to look. The container's logs, its internal state, its configuration, and its resource usage are all hidden behind Docker's abstraction layer.

In this lesson, you'll learn the debugging toolkit that every container developer needs: reading logs to understand what happened, executing commands inside containers to inspect their state, and using inspection tools to verify configuration. You'll practice these skills using the FastAPI application you built in Lesson 3, intentionally breaking it to develop debugging intuition.

---

## Running Your FastAPI App in Detached Mode

Before debugging, let's run your Lesson 3 FastAPI container in the background. Navigate to your `my-fastapi-app` directory where you created the Dockerfile:

```bash
cd my-fastapi-app
docker build -t task-api:v1 .
```

**Output:**
```
$ docker build -t task-api:v1 .
[+] Building 2.1s (8/8) FINISHED
 => [1/5] FROM docker.io/library/python:3.12-slim
 => CACHED [2/5] WORKDIR /app
 => CACHED [3/5] COPY requirements.txt .
 => CACHED [4/5] RUN pip install --no-cache-dir -r requirements.txt
 => CACHED [5/5] COPY main.py .
 => exporting to image
Successfully tagged task-api:v1
```

Now run it in detached mode (`-d`) so it runs in the background:

```bash
docker run -d -p 8000:8000 --name task-api task-api:v1
```

**Output:**
```
$ docker run -d -p 8000:8000 --name task-api task-api:v1
a7b8c9d0e1f2g3h4i5j6k7l8m9n0o1p2q3r4s5t6
```

The long string is the container ID. The `-d` flag means "detached"—the container runs in the background and you get your terminal back.

Verify the container is running:

```bash
docker ps
```

**Output:**
```
$ docker ps
CONTAINER ID   IMAGE         COMMAND                  STATUS         PORTS                    NAMES
a7b8c9d0e1f2   task-api:v1   "uvicorn main:app ..."   Up 5 seconds   0.0.0.0:8000->8000/tcp   task-api
```

Test that it's responding:

```bash
curl http://localhost:8000/health
```

**Output:**
```
$ curl http://localhost:8000/health
{"status":"healthy"}
```

Now you have a running container to debug. Let's explore the debugging tools.

---

## Reading Container Logs

The most important debugging command is `docker logs`. It shows everything your application writes to stdout and stderr—print statements, uvicorn startup messages, errors, and stack traces.

View logs from your running container:

```bash
docker logs task-api
```

**Output:**
```
$ docker logs task-api
INFO:     Started server process [1]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
```

These logs show uvicorn started successfully. Now make a request and check logs again:

```bash
curl http://localhost:8000/
docker logs task-api
```

**Output:**
```
$ curl http://localhost:8000/
{"message":"Hello from Docker!"}

$ docker logs task-api
INFO:     Started server process [1]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     172.17.0.1:54321 - "GET / HTTP/1.1" 200 OK
```

The new log line shows the request: the client IP, the endpoint accessed, and the HTTP response code (200 OK).

### Following Logs in Real-Time

For live debugging, use the `-f` (follow) flag to stream logs continuously:

```bash
docker logs -f task-api
```

**Output:**
```
$ docker logs -f task-api
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     172.17.0.1:54321 - "GET / HTTP/1.1" 200 OK
[cursor waiting for new logs...]
```

Now in another terminal, make requests and watch them appear in real-time. Press Ctrl+C to stop following.

### Viewing Recent Logs

For large log files, use `--tail` to see only the last N lines:

```bash
docker logs --tail 5 task-api
```

**Output:**
```
$ docker logs --tail 5 task-api
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     172.17.0.1:54321 - "GET / HTTP/1.1" 200 OK
INFO:     172.17.0.1:54322 - "GET /health HTTP/1.1" 200 OK
```

---

## Debugging a Failed Container

Now let's create a container that fails on startup. Stop and remove the current container:

```bash
docker stop task-api
docker rm task-api
```

**Output:**
```
$ docker stop task-api
task-api

$ docker rm task-api
task-api
```

Create a Python script that simulates a startup failure:

**File: broken_main.py**
```python
import os
import sys

print("Task API starting...")
print("Checking for required configuration...")

# Simulate missing required environment variable
api_key = os.environ.get("API_KEY")
if not api_key:
    print("ERROR: API_KEY environment variable is required but not set")
    sys.exit(1)

print(f"API_KEY configured: {api_key[:4]}****")
print("Starting server...")
```

Create a Dockerfile for this broken app:

**File: Dockerfile.broken**
```dockerfile
FROM python:3.12-slim
WORKDIR /app
COPY broken_main.py .
CMD ["python", "broken_main.py"]
```

Build and run it:

```bash
docker build -f Dockerfile.broken -t task-api-broken:v1 .
docker run -d --name broken-api task-api-broken:v1
```

**Output:**
```
$ docker build -f Dockerfile.broken -t task-api-broken:v1 .
[+] Building 1.2s (7/7) FINISHED
Successfully tagged task-api-broken:v1

$ docker run -d --name broken-api task-api-broken:v1
b1c2d3e4f5g6h7i8j9k0l1m2n3o4p5q6r7s8t9u0
```

Check if it's running:

```bash
docker ps
```

**Output:**
```
$ docker ps
CONTAINER ID   IMAGE   COMMAND   STATUS   PORTS   NAMES
```

Empty! The container isn't running. Check all containers, including stopped ones:

```bash
docker ps -a
```

**Output:**
```
$ docker ps -a
CONTAINER ID   IMAGE                 COMMAND                  STATUS                     NAMES
b1c2d3e4f5g6   task-api-broken:v1    "python broken_main..."  Exited (1) 5 seconds ago   broken-api
```

Status shows "Exited (1)" - the container crashed with exit code 1. Now use `docker logs` to find out why:

```bash
docker logs broken-api
```

**Output:**
```
$ docker logs broken-api
Task API starting...
Checking for required configuration...
ERROR: API_KEY environment variable is required but not set
```

The logs tell you exactly what went wrong: the API_KEY environment variable is missing.

### Fixing the Broken Container

Now that you know the problem, run it correctly with the environment variable:

```bash
docker rm broken-api
docker run -d --name broken-api -e API_KEY=sk-test-12345 task-api-broken:v1
docker logs broken-api
```

**Output:**
```
$ docker rm broken-api
broken-api

$ docker run -d --name broken-api -e API_KEY=sk-test-12345 task-api-broken:v1
c2d3e4f5g6h7i8j9k0l1m2n3o4p5q6r7s8t9u0v1

$ docker logs broken-api
Task API starting...
Checking for required configuration...
API_KEY configured: sk-t****
Starting server...
```

The container now starts successfully because the required environment variable is set.

---

## Executing Commands Inside Containers

Logs show what happened, but sometimes you need to inspect the container's current state. `docker exec` lets you run commands inside a running container.

First, restart your working FastAPI container:

```bash
docker rm -f broken-api
docker run -d -p 8000:8000 --name task-api task-api:v1
```

**Output:**
```
$ docker rm -f broken-api
broken-api

$ docker run -d -p 8000:8000 --name task-api task-api:v1
d3e4f5g6h7i8j9k0l1m2n3o4p5q6r7s8t9u0v1w2
```

Now execute commands inside the running container:

```bash
docker exec task-api pwd
```

**Output:**
```
$ docker exec task-api pwd
/app
```

The container's working directory is `/app`, as set by WORKDIR in the Dockerfile.

List files in the container:

```bash
docker exec task-api ls -la
```

**Output:**
```
$ docker exec task-api ls -la
total 16
drwxr-xr-x 1 root root 4096 Dec 22 10:30 .
drwxr-xr-x 1 root root 4096 Dec 22 10:30 ..
-rw-r--r-- 1 root root  237 Dec 22 10:30 main.py
-rw-r--r-- 1 root root   42 Dec 22 10:30 requirements.txt
```

Check what user is running the process:

```bash
docker exec task-api whoami
```

**Output:**
```
$ docker exec task-api whoami
root
```

### Interactive Shell Access

For deeper debugging, launch an interactive shell inside the container:

```bash
docker exec -it task-api sh
```

**Output:**
```
$ docker exec -it task-api sh
#
```

The `-it` flags mean "interactive" and "allocate a TTY" (terminal). You're now inside the container. Try some commands:

```bash
# pwd
/app
# cat main.py
from fastapi import FastAPI

app = FastAPI()

@app.get("/")
def read_root():
    return {"message": "Hello from Docker!"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}
# exit
```

**Output:**
```
# pwd
/app
# cat main.py
from fastapi import FastAPI
...
# exit
$
```

The `exit` command returns you to your host machine.

### Testing API Endpoints from Inside the Container

You can even test your API from inside the container:

```bash
docker exec task-api curl -s http://localhost:8000/health
```

**Output:**
```
$ docker exec task-api curl -s http://localhost:8000/health
{"status":"healthy"}
```

This confirms the API is responding on port 8000 inside the container. If this works but external requests fail, you have a port mapping problem, not an application problem.

---

## Inspecting Container Configuration

The `docker inspect` command shows complete configuration and runtime state in JSON format. This is essential for verifying that a container was started with the correct settings.

Inspect your running container:

```bash
docker inspect task-api
```

This outputs hundreds of lines. Let's extract specific information.

### Check Container Status

```bash
docker inspect --format='{{.State.Status}}' task-api
```

**Output:**
```
$ docker inspect --format='{{.State.Status}}' task-api
running
```

### Check the Running Command

```bash
docker inspect --format='{{json .Config.Cmd}}' task-api
```

**Output:**
```
$ docker inspect --format='{{json .Config.Cmd}}' task-api
["uvicorn","main:app","--host","0.0.0.0","--port","8000"]
```

This confirms exactly what command the container is running.

### Check Environment Variables

```bash
docker inspect --format='{{json .Config.Env}}' task-api
```

**Output:**
```
$ docker inspect --format='{{json .Config.Env}}' task-api
["PATH=/usr/local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin","LANG=C.UTF-8","GPG_KEY=...","PYTHON_VERSION=3.12.7","PYTHON_PIP_VERSION=24.0"]
```

### Check Port Mappings

```bash
docker inspect --format='{{json .NetworkSettings.Ports}}' task-api
```

**Output:**
```
$ docker inspect --format='{{json .NetworkSettings.Ports}}' task-api
{"8000/tcp":[{"HostIp":"0.0.0.0","HostPort":"8000"}]}
```

This shows port 8000 in the container is mapped to port 8000 on the host.

### Practical Use: Verify Exit Codes

For crashed containers, inspect shows why they stopped:

```bash
docker inspect --format='{{.State.ExitCode}}' broken-api
```

**Output:**
```
$ docker inspect --format='{{.State.ExitCode}}' broken-api
1
```

Exit code 1 means the application exited with an error. Exit code 0 means success. Exit code 137 means the kernel killed the process (usually out of memory).

---

## Resolving Port Conflicts

A common debugging scenario: you try to start a container and it fails because the port is already in use.

Try to start another container on port 8000 while `task-api` is running:

```bash
docker run -d -p 8000:8000 --name task-api-2 task-api:v1
```

**Output:**
```
$ docker run -d -p 8000:8000 --name task-api-2 task-api:v1
docker: Error response from daemon: driver failed programming external connectivity
on endpoint task-api-2: Bind for 0.0.0.0:8000 failed: port is already allocated.
```

The error is clear: port 8000 is already allocated (by your first container).

### Solution 1: Use a Different Host Port

```bash
docker run -d -p 8001:8000 --name task-api-2 task-api:v1
```

**Output:**
```
$ docker run -d -p 8001:8000 --name task-api-2 task-api:v1
e4f5g6h7i8j9k0l1m2n3o4p5q6r7s8t9u0v1w2x3
```

Now you have two containers:
- `task-api` on port 8000
- `task-api-2` on port 8001

Verify both work:

```bash
curl http://localhost:8000/health
curl http://localhost:8001/health
```

**Output:**
```
$ curl http://localhost:8000/health
{"status":"healthy"}

$ curl http://localhost:8001/health
{"status":"healthy"}
```

### Solution 2: Find and Stop the Conflicting Container

If you need port 8000 specifically, find what's using it:

```bash
docker ps --format "table {{.Names}}\t{{.Ports}}"
```

**Output:**
```
$ docker ps --format "table {{.Names}}\t{{.Ports}}"
NAMES        PORTS
task-api-2   0.0.0.0:8001->8000/tcp
task-api     0.0.0.0:8000->8000/tcp
```

Stop the container using your desired port:

```bash
docker stop task-api
docker rm task-api
```

Now you can start a new container on port 8000.

Clean up for the next section:

```bash
docker stop task-api-2
docker rm task-api-2
```

---

## Restart Policies for Resilience

Containers can crash due to bugs, resource exhaustion, or temporary failures. Instead of manually restarting them, configure Docker to restart them automatically.

### The `--restart` Flag

Docker supports several restart policies:

| Policy | Behavior |
|--------|----------|
| `no` | Never restart (default) |
| `always` | Always restart, even after successful exit |
| `unless-stopped` | Restart unless manually stopped |
| `on-failure:N` | Restart only on non-zero exit, up to N times |

### Testing Restart Policies

Create a container that crashes sometimes:

**File: flaky_main.py**
```python
import random
import sys
import time

print("Task API starting...")
time.sleep(1)

if random.random() < 0.3:
    print("ERROR: Random failure occurred!")
    sys.exit(1)

print("Task API started successfully!")
while True:
    time.sleep(10)
```

Build it:

```bash
docker build -f Dockerfile.broken -t flaky-api:v1 .
```

Wait, that Dockerfile won't work for this script. Create a new one:

**File: Dockerfile.flaky**
```dockerfile
FROM python:3.12-slim
WORKDIR /app
COPY flaky_main.py .
CMD ["python", "-u", "flaky_main.py"]
```

The `-u` flag means unbuffered output so logs appear immediately.

```bash
docker build -f Dockerfile.flaky -t flaky-api:v1 .
```

**Output:**
```
$ docker build -f Dockerfile.flaky -t flaky-api:v1 .
[+] Building 0.8s (7/7) FINISHED
Successfully tagged flaky-api:v1
```

Run without restart policy:

```bash
docker run -d --name flaky-no-restart flaky-api:v1
sleep 3
docker ps -a --filter name=flaky
```

**Output:**
```
$ docker ps -a --filter name=flaky
CONTAINER ID   IMAGE          STATUS                     NAMES
f5g6h7i8j9k0   flaky-api:v1   Exited (1) 2 seconds ago   flaky-no-restart
```

If the container crashed (30% chance), it stays dead. Remove it:

```bash
docker rm flaky-no-restart
```

Now run with automatic restart:

```bash
docker run -d --restart=unless-stopped --name flaky-restart flaky-api:v1
```

Watch it recover from failures:

```bash
docker logs -f flaky-restart
```

**Output (if it fails and restarts):**
```
$ docker logs -f flaky-restart
Task API starting...
ERROR: Random failure occurred!
Task API starting...
Task API started successfully!
```

The container automatically restarted after the failure. Check restart count:

```bash
docker inspect --format='{{.RestartCount}}' flaky-restart
```

**Output:**
```
$ docker inspect --format='{{.RestartCount}}' flaky-restart
1
```

### Production Recommendation

For production services, use `--restart=unless-stopped`. This ensures:
- Containers restart after crashes
- Containers restart after host reboots
- You can still manually stop them with `docker stop`

Clean up:

```bash
docker stop flaky-restart
docker rm flaky-restart
```

---

## Try With AI

Now that you understand container debugging fundamentals, practice these skills with increasingly complex scenarios.

**Prompt 1: Diagnose a Startup Failure**

Create a FastAPI application that requires a DATABASE_URL environment variable. Run it without the variable and use the debugging tools you learned to identify the problem:

```
Create a FastAPI app that:
1. Checks for DATABASE_URL environment variable on startup
2. Prints an error and exits with code 1 if missing
3. Prints "Connected to: [masked URL]" if present

Help me create the Dockerfile and show me how to:
- See the error when DATABASE_URL is missing
- Verify the container exit code
- Run it successfully with the environment variable
```

**What you're learning:** This reinforces the pattern of using `docker logs` and `docker inspect --format='{{.State.ExitCode}}'` to diagnose startup failures, and `-e` to provide environment variables.

**Prompt 2: Debug a Port Mapping Issue**

Sometimes your application seems to start but doesn't respond to requests. Practice debugging this:

```
My FastAPI container starts successfully (docker logs shows "Uvicorn running")
but curl http://localhost:8000/ returns "Connection refused".

Help me debug this using:
1. docker inspect to check port mappings
2. docker exec to test the app from inside the container
3. Common causes of this problem
```

**What you're learning:** This teaches you to systematically isolate whether the problem is the application, the port mapping, or network configuration using `docker exec` to test from inside the container.

**Prompt 3: Configure a Resilient Service**

Production services need to handle crashes gracefully:

```
I have a FastAPI service that occasionally crashes due to memory pressure.
Help me configure it with:
1. --restart=unless-stopped for automatic recovery
2. Memory limits to prevent runaway usage
3. How to monitor restart count

Show me the docker run command and how to verify the configuration.
```

**What you're learning:** This reinforces restart policies and introduces memory limits (`--memory`), which become critical when deploying AI services that can consume large amounts of RAM.

**Safety note:** When debugging containers in production, use read-only commands (`docker logs`, `docker inspect`) before interactive commands (`docker exec`). Avoid running shells in production containers unless absolutely necessary, as it can affect running services.
