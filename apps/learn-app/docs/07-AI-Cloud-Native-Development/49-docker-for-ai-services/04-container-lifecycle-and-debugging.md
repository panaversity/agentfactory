---
sidebar_position: 4
chapter: 49
lesson: 4
duration_minutes: 45
title: "Container Lifecycle & Debugging"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Debugging broken containers teaches troubleshooting skills"
cognitive_load:
  concepts_count: 8
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Diagnose container failures using docker logs"
    bloom_level: "Analyze"
  - id: LO2
    description: "Execute commands in running containers with docker exec"
    bloom_level: "Apply"
  - id: LO3
    description: "Inspect container configuration and state with docker inspect"
    bloom_level: "Analyze"
  - id: LO4
    description: "Identify and resolve port conflicts"
    bloom_level: "Analyze"
  - id: LO5
    description: "Diagnose file permission errors in containers"
    bloom_level: "Analyze"
  - id: LO6
    description: "Configure restart policies for resilient containers"
    bloom_level: "Apply"
  - id: LO7
    description: "Set memory and CPU limits for AI workloads"
    bloom_level: "Apply"
  - id: LO8
    description: "Interpret OOM (Out of Memory) kills and resolve them"
    bloom_level: "Analyze"
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
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
  - objective_id: LO6
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO7
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO8
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
---

# Container Lifecycle & Debugging

Containers fail. Your image builds successfully, you run it locally, and it crashes in production. Your agent service starts, but nothing responds on the expected port. A memory-intensive model loads fine on your 64GB workstation but kills your container on a cloud instance with memory limits.

Debugging containers requires different skills than debugging local applications. You can't attach a debugger to a container running in a Kubernetes cluster. You can't inspect files on a machine you don't have SSH access to. Container debugging relies on logs, introspection tools, and understanding the container lifecycle.

In this lesson, you'll diagnose intentionally broken containers to develop debugging competence. By working through real failure patterns, you'll build intuition for what goes wrong and how to fix it.

---

## Reading Container Logs

When a container crashes or behaves unexpectedly, logs are your first source of truth. The `docker logs` command shows you everything the main process writes to stdout and stderr.

Create a broken application to see how logs reveal problems. Start with a Python script that fails immediately:

**File: broken_app.py**
```python
import sys

print("Application starting...")
print("Attempting to open configuration file...")
sys.exit("ERROR: Configuration file not found at /etc/app/config.json")
```

Now create a Dockerfile for this broken app:

**File: Dockerfile.broken**
```dockerfile
FROM python:3.12-slim

WORKDIR /app

COPY broken_app.py .

CMD ["python", "broken_app.py"]
```

Build the image:

```bash
docker build -f Dockerfile.broken -t broken-app:v1 .
```

**Output:**
```
$ docker build -f Dockerfile.broken -t broken-app:v1 .
[+] Building 2.3s (7/7) FINISHED
 => [internal] load build context
 => => transferring context: 51B
 => [1/4] FROM python:3.12-slim:latest
 => => pull from library/python
 => => STATUS Downloaded newer image for python:3.12-slim:latest
 => [2/4] WORKDIR /app
 => [3/4] COPY broken_app.py .
 => [4/4] RUN echo "Image built successfully"
 => => RUN echo "Image built successfully"
 => => # Image built successfully
 [+] FINISHED Successfully built successfully
```

Now run the container:

```bash
docker run broken-app:v1
```

**Output:**
```
Application starting...
Attempting to open configuration file...
ERROR: Configuration file not found at /etc/app/config.json
```

The container exited immediately. To see what happened after it died, use `docker logs` with the container ID. First, run the container again and keep it running so you can query it:

```bash
docker run -d --name broken-instance broken-app:v1
sleep 2
docker logs broken-instance
```

**Output:**
```
$ docker run -d --name broken-instance broken-app:v1
a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6

$ sleep 2

$ docker logs broken-instance
Application starting...
Attempting to open configuration file...
ERROR: Configuration file not found at /etc/app/config.json
```

**Key insight:** The logs show exactly what the application printed. This is why print statements or structured logging matters in containerized applications. If your application fails silently (no output), the logs will be empty and you'll have no clues about what went wrong.

Always ensure your applications log errors to stdout or stderr, not to files that don't persist after container death.

---

## Executing Commands in Running Containers

Logs reveal what happened, but sometimes you need to inspect the running system state. The `docker exec` command lets you run commands inside a running container, like SSH-ing into a machine.

Create a working application first:

**File: app_with_debug.py**
```python
import os
import time

print("Application starting...")
print(f"Home directory: {os.path.expanduser('~')}")
print(f"Current working directory: {os.getcwd()}")
print(f"Python version: {os.sys.version}")
print("Server ready. Listening indefinitely...")

while True:
    time.sleep(1)
```

Create a Dockerfile:

**File: Dockerfile.debug**
```dockerfile
FROM python:3.12-slim

WORKDIR /app

COPY app_with_debug.py .

CMD ["python", "app_with_debug.py"]
```

Build and run:

```bash
docker build -f Dockerfile.debug -t debug-app:v1 .
docker run -d --name debug-instance debug-app:v1
docker logs debug-instance
```

**Output:**
```
$ docker run -d --name debug-instance debug-app:v1
c7d8e9f0g1h2i3j4k5l6m7n8o9p0q1r2

$ docker logs debug-instance
Application starting...
Home directory: /root
Current working directory: /app
Python version: 3.12.7 (main, Oct 1 2024, 00:00:00)
[GCC 12.2.0]
Server ready. Listening indefinitely...
```

Now use `docker exec` to run commands inside the running container:

```bash
docker exec debug-instance pwd
```

**Output:**
```
/app
```

List files in the container:

```bash
docker exec debug-instance ls -la
```

**Output:**
```
total 12
drwxr-xr-x 1 root root 4096 Dec 22 10:15 .
drwxr-xr-x 1 root root 4096 Dec 22 10:15 ..
-rw-r--r-- 1 root root  285 Dec 22 10:15 app_with_debug.py
```

Check what user is running the application:

```bash
docker exec debug-instance whoami
```

**Output:**
```
root
```

Launch an interactive shell to explore the container:

```bash
docker exec -it debug-instance /bin/bash
```

**Output:**
```
root@c7d8e9f0g1h2:/app#
```

From here you can navigate, inspect files, and run commands. Type `exit` to return to your host:

```bash
root@c7d8e9f0g1h2:/app# exit
exit
```

**Key insight:** `docker exec` is powerful but expensive. It's a sign something is wrong if you need to debug interactively inside production containers. The fact that you CAN inspect a running container is valuable for learning, but production debugging should rely on logs, metrics, and distributed tracing, not interactive container inspection.

---

## Inspecting Container Configuration

The `docker inspect` command gives you the complete configuration and state of a container in JSON format. It shows environment variables, volume mounts, network settings, restart policies, and the exact command that started the container.

Inspect the container you just created:

```bash
docker inspect debug-instance | head -50
```

**Output:**
```json
[
  {
    "Id": "c7d8e9f0g1h2i3j4k5l6m7n8o9p0q1r2...",
    "Created": "2024-12-22T10:15:00.000000000Z",
    "Path": "/usr/local/bin/python",
    "Args": ["app_with_debug.py"],
    "State": {
      "Status": "running",
      "Pid": 12345,
      "ExitCode": 0,
      "StartedAt": "2024-12-22T10:15:00.000000000Z",
      "FinishedAt": "0001-01-01T00:00:00Z"
    },
    "Image": "sha256:abc123def456...",
    "Name": "/debug-instance",
    "RestartCount": 0,
    "Driver": "overlay2",
    "Config": {
      "Hostname": "c7d8e9f0g1h2",
      "Image": "debug-app:v1",
      "Cmd": ["python", "app_with_debug.py"],
      "WorkingDir": "/app",
      "Env": ["PATH=/usr/local/bin:/usr/bin:/bin", "PYTHONUNBUFFERED=1"]
    }
  }
]
```

The most useful fields are:

- **State.Status**: Is the container running, exited, or stopped?
- **State.ExitCode**: If exited, what exit code did the process return? (0 = success, non-zero = failure)
- **Args**: What arguments were passed to the entrypoint?
- **Config.Cmd**: What command does the container run?
- **Config.Env**: What environment variables are set?
- **Config.ExposedPorts**: What ports does the container listen on?

To extract just the command that's running:

```bash
docker inspect debug-instance --format='{{json .Config.Cmd}}'
```

**Output:**
```json
["python","app_with_debug.py"]
```

Extract environment variables:

```bash
docker inspect debug-instance --format='{{json .Config.Env}}'
```

**Output:**
```json
["PATH=/usr/local/bin:/usr/bin:/bin","PYTHONUNBUFFERED=1"]
```

**Key insight:** Use `docker inspect` when you need to verify that a container was started with the configuration you expected. Common questions: "Is PORT 8000 really exposed?" "What environment variables are set?" "What user is running this?" All answered by inspect.

---

## Port Conflicts and Binding Errors

A common container failure pattern: You run a service that should listen on port 8000, but something else is already using that port. The container might start, but it can't bind to the expected port and crashes or silently fails to listen.

Create an application that listens on a specific port:

**File: server.py**
```python
import socket
import sys

PORT = 8000

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', PORT))
    sock.listen(1)
    print(f"Server listening on port {PORT}")
    sock.accept()
except OSError as e:
    print(f"ERROR: Could not bind to port {PORT}: {e}")
    sys.exit(1)
```

Dockerfile:

```dockerfile
FROM python:3.12-slim
WORKDIR /app
COPY server.py .
CMD ["python", "server.py"]
```

Build it:

```bash
docker build -f Dockerfile -t port-server:v1 .
```

**Output:**
```
$ docker build -f Dockerfile -t port-server:v1 .
[+] Building 1.2s (6/6) FINISHED
[+] FINISHED
```

Run it with port mapping:

```bash
docker run -d -p 8000:8000 --name server1 port-server:v1
docker logs server1
```

**Output:**
```
$ docker run -d -p 8000:8000 --name server1 port-server:v1
a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6

$ docker logs server1
Server listening on port 8000
```

Now try to run a second container on the same host port:

```bash
docker run -d -p 8000:8000 --name server2 port-server:v1
```

**Output:**
```
docker: Error response from daemon: driver failed programming endpoint
server2 on bridge network: Bind for 0.0.0.0:8000 failed: port is already allocated
```

The host port 8000 can only be used by one container. You have options:

**Option 1: Use a different host port**

```bash
docker run -d -p 8001:8000 --name server2 port-server:v1
docker logs server2
```

**Output:**
```
$ docker run -d -p 8001:8000 --name server2 port-server:v1
b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6q7

$ docker logs server2
Server listening on port 8000
```

Notice the container still listens on 8000 (the container port), but the host maps port 8001 to the container's 8000. If a client connects to your machine on port 8001, traffic routes to the container's 8000.

**Option 2: Use a random host port**

```bash
docker run -d -p 8000 --name server3 port-server:v1
docker port server3
```

**Output:**
```
$ docker port server3
8000/tcp -> 0.0.0.0:32768
```

Docker assigns a random port (32768 in this case). Check which port was assigned:

```bash
docker run -d -p 8000 --name server4 port-server:v1
docker port server4
```

**Output:**
```
$ docker port server4
8000/tcp -> 0.0.0.0:32769
```

**Key insight:** In local development, port conflicts are easy to debug. In production (especially Kubernetes), port binding is internal to the cluster, so this specific error is rare. But understanding port mapping helps you reason about how containers expose services.

Clean up:

```bash
docker rm -f server1 server2 server3 server4
```

---

## Permission Errors in Containers

By default, Docker runs containers as the root user. This is convenient for development but dangerous in production. When you switch to a non-root user (a security best practice), you encounter permission errors if that user can't write to necessary directories.

Create an application that writes to a log file:

**File: logging_app.py**
```python
import os
import sys

LOG_DIR = "/var/log/app"
LOG_FILE = f"{LOG_DIR}/application.log"

try:
    with open(LOG_FILE, "a") as f:
        f.write("Application started\n")
    print(f"Logged successfully to {LOG_FILE}")
except PermissionError as e:
    print(f"ERROR: Permission denied writing to {LOG_FILE}")
    print(f"Current user: {os.getuid()}")
    print(f"Current directory: {os.getcwd()}")
    sys.exit(1)
```

Create a Dockerfile that runs as a non-root user:

**File: Dockerfile.nonroot**
```dockerfile
FROM python:3.12-slim

# Create the log directory and a non-root user
RUN mkdir -p /var/log/app && \
    useradd -m -s /bin/bash appuser

# Copy the application
WORKDIR /app
COPY logging_app.py .

# Switch to non-root user
USER appuser

CMD ["python", "logging_app.py"]
```

Build and run:

```bash
docker build -f Dockerfile.nonroot -t logging-app:v1 .
docker run --name logger logging-app:v1
```

**Output:**
```
$ docker run --name logger logging-app:v1
ERROR: Permission denied writing to /var/log/app/application.log
Current user: 1000
Current directory: /app
```

The user ID 1000 (the `appuser`) doesn't have write permissions to `/var/log/app`, which is owned by root. To fix this, the directory needs to be writable by the user:

**File: Dockerfile.fixed**
```dockerfile
FROM python:3.12-slim

# Create the log directory and a non-root user
RUN mkdir -p /var/log/app && \
    useradd -m -s /bin/bash appuser && \
    chown -R appuser:appuser /var/log/app

# Copy the application
WORKDIR /app
COPY logging_app.py .

# Switch to non-root user
USER appuser

CMD ["python", "logging_app.py"]
```

Rebuild and run:

```bash
docker build -f Dockerfile.fixed -t logging-app:v2 .
docker run --name logger2 logging-app:v2
```

**Output:**
```
$ docker run --name logger2 logging-app:v2
Logged successfully to /var/log/app/application.log
```

**Key insight:** When you see "Permission denied" errors in containers, check:

1. What user is the container running as? (`docker exec CONTAINER whoami`)
2. What are the directory permissions? (`docker exec CONTAINER ls -la /path/to/dir`)
3. Do you need to `chown` directories to the non-root user in the Dockerfile?

This is a security + correctness tradeoff: Running as non-root is safer, but requires careful permission planning.

---

## Memory Limits and Out-of-Memory Kills

AI workloads are memory-intensive. Loading a large language model might require 8GB. Your development machine has 64GB, so it runs fine. But when deployed to a cloud instance with 4GB total, the container gets killed.

Docker enforces memory limits with the `--memory` flag. When a container exceeds its limit, the kernel terminates the process with an "Out of Memory" (OOM) kill.

Create an application that allocates memory:

**File: memory_hog.py**
```python
import sys

print("Starting memory allocation...")

try:
    # Allocate 100MB chunks until we hit the limit
    chunks = []
    for i in range(100):
        chunk = bytearray(1024 * 1024)  # 1MB
        chunks.append(chunk)
        print(f"Allocated {i+1}MB")
        sys.stdout.flush()
except MemoryError:
    print("MemoryError: Out of memory!")
    sys.exit(1)
```

Dockerfile:

```dockerfile
FROM python:3.12-slim
WORKDIR /app
COPY memory_hog.py .
CMD ["python", "memory_hog.py"]
```

Build it:

```bash
docker build -f Dockerfile -t memory-app:v1 .
```

**Output:**
```
$ docker build -f Dockerfile -t memory-app:v1 .
[+] Building 0.9s (6/6) FINISHED
[+] FINISHED
```

Run with a 10MB memory limit:

```bash
docker run --memory=10m --name memory-test memory-app:v1
echo "Exit code: $?"
```

**Output:**
```
Starting memory allocation...
Allocated 1MB
Allocated 2MB
Allocated 3MB
Allocated 4MB
Allocated 5MB
Allocated 6MB
Allocated 7MB
Allocated 8MB
Allocated 9MB
Exit code: 137
```

Exit code 137 means the kernel killed the process (SIGKILL, signal 9). The container was OOM-killed.

To verify this is an OOM kill, inspect the container:

```bash
docker inspect memory-test --format='{{json .State}}'
```

**Output:**
```json
{
  "Status": "exited",
  "Pid": 0,
  "ExitCode": 137,
  "OOMKilled": true,
  "FinishedAt": "2024-12-22T10:20:15.000000000Z"
}
```

The `OOMKilled: true` field confirms the kernel terminated the process due to memory pressure.

**How to fix OOM kills:**

1. **Increase the memory limit** if you know the process needs more:

```bash
docker run --memory=256m --name memory-test2 memory-app:v1
echo "Exit code: $?"
```

**Output:**
```
Starting memory allocation...
Allocated 1MB
...
Allocated 100MB
[Container keeps running, has more memory available]
```

2. **Set memory reservations** to request guaranteed memory:

```bash
docker run --memory=512m --memory-reservation=256m --name memory-reserved memory-app:v1
```

The reservation tells Docker to ensure this container has at least 256MB available.

3. **Monitor memory usage** before deploying:

```bash
docker run --memory=512m --name memory-monitor memory-app:v1 &
docker stats memory-monitor
```

**Output:**
```
CONTAINER ID   NAME              CPU %   MEM USAGE / LIMIT
a1b2c3d4e5f6   memory-monitor    0.50%   150MiB / 512MiB
```

**Key insight:** OOM kills are silent failures in production. The container just dies with exit code 137. Always set appropriate memory limits during testing to catch OOM issues before production. For AI workloads, this means testing on resource-constrained environments that match your production deployment targets.

---

## Restart Policies for Resilience

Containers fail. Networks are unreliable. Services crash. A resilient system automatically restarts failed containers rather than leaving them dead.

Docker restart policies handle this. The most common production policy is `--restart unless-stopped`, which automatically restarts a container every time it exits, unless you explicitly stop it.

Create a flaky application that fails sometimes:

**File: flaky_app.py**
```python
import random
import sys

attempt = 0

while True:
    attempt += 1
    print(f"Attempt {attempt}: Running...")

    # 70% chance to succeed, 30% chance to crash
    if random.random() > 0.7:
        print("CRASH! Exiting with error")
        sys.exit(1)
    else:
        print("Success!")
        sys.exit(0)
```

Dockerfile:

```dockerfile
FROM python:3.12-slim
WORKDIR /app
COPY flaky_app.py .
CMD ["python", "flaky_app.py"]
```

Build it:

```bash
docker build -f Dockerfile -t flaky-app:v1 .
```

Run with no restart policy:

```bash
docker run --name flaky-no-restart flaky-app:v1
echo "Exit code: $?"
```

**Output:**
```
Attempt 1: Running...
CRASH! Exiting with error
Exit code: 1
```

The container exits and stays dead. If this were a critical service, users would experience downtime.

Now run with automatic restart:

```bash
docker run -d --restart=always --name flaky-always flaky-app:v1
sleep 1
docker logs flaky-always
```

**Output:**
```
$ docker logs flaky-always
Attempt 1: Running...
CRASH! Exiting with error
Attempt 2: Running...
Success!
```

With `--restart=always`, Docker automatically restarts the container after it exits. Eventually it succeeds.

The most common restart policies are:

- **no**: Don't automatically restart (default)
- **always**: Always restart, even if the exit code is 0 (not recommended—wastes resources on successful exits)
- **unless-stopped**: Restart unless you explicitly run `docker stop` (production standard)
- **on-failure:max-retries**: Restart only if exit code is non-zero, up to a max number of times

For production, use `unless-stopped`:

```bash
docker run -d --restart=unless-stopped --name flaky-unless-stopped flaky-app:v1
docker logs flaky-unless-stopped
```

**Output:**
```
$ docker logs flaky-unless-stopped
Attempt 1: Running...
Success!
```

To stop a container with `unless-stopped` policy, explicitly stop it:

```bash
docker stop flaky-unless-stopped
docker logs flaky-unless-stopped
```

**Output:**
```
Attempt 1: Running...
Success!
```

The logs don't change because the container doesn't restart (you explicitly stopped it).

**Key insight:** Use `--restart=unless-stopped` for production services. It ensures containers come back to life if they crash, while respecting explicit stop commands during maintenance.

---

## CPU Limits for Shared Environments

Memory limits prevent processes from consuming infinite RAM. CPU limits prevent runaway compute from monopolizing the host system.

The `--cpus` flag limits how much of the CPU a container can use:

```bash
docker run --cpus 1 --name cpu-limited ...
```

This allows the container to use up to 1 full CPU core (100%). On a machine with 4 cores, the container can use up to 25% of total CPU.

Create a CPU-intensive application:

**File: cpu_intensive.py**
```python
import os
import multiprocessing

def cpu_work():
    count = 0
    while True:
        count += 1

if __name__ == "__main__":
    cores = os.cpu_count()
    print(f"System reports {cores} CPU cores")
    print(f"Starting {cores} worker processes...")

    processes = []
    for i in range(cores):
        p = multiprocessing.Process(target=cpu_work)
        p.start()
        processes.append(p)

    for p in processes:
        p.join()
```

Dockerfile:

```dockerfile
FROM python:3.12-slim
WORKDIR /app
COPY cpu_intensive.py .
CMD ["python", "cpu_intensive.py"]
```

Build:

```bash
docker build -f Dockerfile -t cpu-app:v1 .
```

Run with a 1-core CPU limit:

```bash
docker run --cpus=1 -d --name cpu-limited cpu-app:v1
docker stats cpu-limited
```

**Output:**
```
CONTAINER ID   NAME           CPU %   MEM USAGE / LIMIT
a1b2c3d4e5f6   cpu-limited    25.0%   8MiB / 512MiB
```

The CPU usage is capped at around 25% (1 core on a 4-core system). Without the limit, the process would consume all 4 cores.

**Key insight:** For shared environments (especially Kubernetes clusters), set both `--memory` and `--cpus` limits. This prevents one workload from starving others. For AI workloads, be generous with CPU allocation for inference (models often need multiple cores), but strict with memory to prevent OOM kills.

---

## Try With AI

In real deployments, debugging containers involves combining these tools—logs, inspect, exec, and understanding restart policies. You'll encounter containers that fail for multiple reasons simultaneously: permission issues causing crashes that trigger restarts, which then cause OOM kills because memory wasn't configured, which generates logs you need to interpret.

**Part 1: Create a Realistic Broken Service**

Ask AI to help you build a more complex application that demonstrates multiple failure modes:

```
Create a FastAPI service that:
1. Listens on port 8000
2. Loads a large model from a file at /models/model.bin
3. Uses an environment variable LOG_LEVEL to control logging
4. Tries to write logs to /var/log/app/service.log

Include intentional bugs:
- The model file doesn't exist initially
- The log directory isn't writable by the non-root user
- The model uses ~200MB of memory

Show me the complete main.py, requirements.txt, and Dockerfile
for the broken service, plus a fixed version.
```

**Part 2: Debug the Broken Service**

Once AI generates the broken service, run it:

```bash
docker build -f Dockerfile.broken -t broken-service:v1 .
docker run -d --memory=512m -p 8000:8000 --name broken broken-service:v1
```

Then work through these debugging steps:

- Check `docker logs broken` - What failed?
- Use `docker inspect broken` to see what configuration was applied
- Ask yourself: Is it a port issue? A missing file? A permission error?
- Compare against `docker logs` of the fixed version

**Part 3: Refine Your Mental Model**

As you debug, ask yourself:

- What would happen if I increase the memory limit?
- What would the logs show if the permission was correct but the model file was missing?
- If I add `--restart=unless-stopped`, would the container eventually succeed?
- How would I know if this was an OOM kill vs a permission error?

Your goal: Build intuition for translating container errors into specific causes and solutions. The tools (logs, inspect, exec) are easy. The skill is diagnosing correctly.
