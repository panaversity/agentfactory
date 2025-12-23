---
sidebar_position: 18
chapter: 50
lesson: 18
duration_minutes: 40
title: "Health Checks: Liveness, Readiness, Startup Probes"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Manual probe configuration builds understanding of container health patterns"
cognitive_load:
  concepts_count: 10
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Explain liveness, readiness, and startup probes and when to use each"
    bloom_level: "Understand"
  - id: LO2
    description: "Configure HTTP GET probes for health endpoints"
    bloom_level: "Apply"
  - id: LO3
    description: "Configure TCP socket probes for port availability"
    bloom_level: "Apply"
  - id: LO4
    description: "Configure exec command probes for custom health checks"
    bloom_level: "Apply"
  - id: LO5
    description: "Set appropriate timing parameters for different workload types"
    bloom_level: "Apply"
  - id: LO6
    description: "Debug probe failures using kubectl describe and logs"
    bloom_level: "Apply"
  - id: LO7
    description: "Design health check strategies for AI agents with slow initialization"
    bloom_level: "Apply"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "5. Problem Solving"
    competency: "5.2 Identifying needs and technological responses"
  - objective_id: LO2
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO3
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO4
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO5
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO6
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
  - objective_id: LO7
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
---

# Health Checks: Liveness, Readiness, Startup Probes

Your agent is now running in a Deployment (Lesson 4). Kubernetes monitors it and restarts it if the container crashes. But there's a critical problem: **a container that's running might not be healthy**.

Imagine this: Your agent's model loads asynchronously. For the first 30 seconds after startup, the container is running but can't handle requests. Kubernetes sees it as "ready" and sends traffic to it immediately. Users get errors. The container didn't crash, so Kubernetes doesn't restart it. Your service is degraded but technically "up."

This lesson teaches health checks—the mechanism that lets Kubernetes know whether your container is actually ready to serve traffic, whether it's still alive, and how long to wait during startup before expecting it to respond.

---

## The Problem: Running ≠ Ready

Consider your AI agent startup sequence:

```
0s   - Container starts, main process begins
0.5s - Python initializes, imports libraries
5s   - Model weights load into memory
10s  - Embedding vectors cache builds
30s  - Application ready to serve requests
```

**Without health checks:**
- Kubernetes sends traffic at 1s (while model is still loading)
- Requests fail, users see errors
- Container doesn't crash (exit code 0 is still valid)
- Kubernetes doesn't restart it
- You have a running but broken service

**With health checks:**
- Readiness probe fails for first 30 seconds (container is running but not ready)
- Kubernetes removes pod from service endpoints
- Traffic doesn't reach it until model is loaded
- Once ready, traffic flows
- If it becomes unhealthy, Kubernetes routes around it or restarts it

---

## Three Types of Probes

Kubernetes provides three health check mechanisms:

### 1. Readiness Probe (Is this pod ready to serve traffic?)

**Purpose**: Determines if a pod should receive traffic from Services.

**When to use**:
- Application startup is slow (models loading, caches building)
- Container is running but dependencies aren't ready
- Need to temporarily remove pod from load balancer during updates

**Behavior**:
- Fails → Pod removed from Service endpoints (traffic stops)
- Recovers → Pod re-added to Service endpoints (traffic resumes)
- Success → Pod receives traffic normally

**Key insight**: Readiness probes are about external traffic. Even a healthy, running pod might not be ready to serve external requests.

### 2. Liveness Probe (Is this pod still alive?)

**Purpose**: Detects if container is in a broken/stuck state and needs restart.

**When to use**:
- Detect deadlocks or infinite loops
- Identify containers that are running but unresponsive
- Force restart of unhealthy containers (different from crash)

**Behavior**:
- Fails continuously → Kubernetes restarts pod
- Success → No action, pod continues running

**Key insight**: Liveness probes are about pod lifecycle. A pod can be running but logically dead (stuck waiting, infinite loop, memory leak).

### 3. Startup Probe (Has this pod finished initializing?)

**Purpose**: Prevents liveness/readiness probes from triggering during slow startup.

**When to use**:
- Container has long initialization time (30+ seconds)
- Need different probe behavior during startup vs steady state
- AI agents loading models, warming caches

**Behavior**:
- Fails during startup → No restart (give it more time)
- Succeeds → Liveness/readiness probes take over
- Fails after startup complete → Kubernetes restarts pod (treated as liveness failure)

**Key insight**: Startup probes buy time for initialization. Once startup succeeds, other probes begin their work.

---

## HTTP GET Probes (Most Common)

HTTP probes call a health endpoint and check the response code.

### Basic HTTP Probe Structure

```yaml
spec:
  containers:
  - name: agent
    image: my-agent:v1
    ports:
    - containerPort: 8000
    readinessProbe:
      httpGet:
        path: /health/ready
        port: 8000
      initialDelaySeconds: 10
      periodSeconds: 5
      timeoutSeconds: 1
      failureThreshold: 3
```

**Explanation**:
- `httpGet.path`: Endpoint to call
- `httpGet.port`: Container port (can be number or name)
- `initialDelaySeconds`: Wait 10s before first probe (let container start)
- `periodSeconds`: Check every 5 seconds
- `timeoutSeconds`: If response takes >1s, count as failure
- `failureThreshold`: After 3 failures, take action (remove from service or restart)

### HTTP Readiness Probe Example

Your FastAPI agent needs a readiness endpoint that returns 200 only when model is loaded:

```python
# main.py - FastAPI agent with readiness endpoint
from fastapi import FastAPI
from fastapi.responses import JSONResponse
import asyncio
import time

app = FastAPI()

# Simulate model loading time
model_loaded = False
load_start = None

@app.on_event("startup")
async def load_model():
    global model_loaded, load_start
    load_start = time.time()
    print("Starting model load...")
    # Simulate 15s model loading
    await asyncio.sleep(15)
    model_loaded = True
    load_time = time.time() - load_start
    print(f"Model loaded in {load_time:.1f}s")

@app.get("/health/ready")
async def readiness():
    """Returns 200 only when model is fully loaded"""
    if not model_loaded:
        return JSONResponse(
            {"status": "loading", "ready": False},
            status_code=503
        )
    return JSONResponse({"status": "ready", "ready": True})

@app.get("/health/live")
async def liveness():
    """Always returns 200 if main process is running"""
    return JSONResponse({"status": "alive"})

@app.post("/predict")
async def predict(data: dict):
    """Agent inference endpoint"""
    if not model_loaded:
        return JSONResponse(
            {"error": "Model still loading, try again soon"},
            status_code=503
        )
    # Do actual inference
    return {"prediction": "example output"}
```

**Output** (when you test these endpoints):

```
Starting model load...
```

```bash
# First request during loading
curl http://localhost:8000/health/ready
```

**Output:**
```
{"status": "loading", "ready": false}
```

```bash
# After 15 seconds, try again
curl http://localhost:8000/health/ready
```

**Output:**
```
{"status": "ready", "ready": true}
```

```bash
# Liveness endpoint always responds
curl http://localhost:8000/health/live
```

**Output:**
```
{"status": "alive"}
```

### Deployment with HTTP Readiness and Liveness Probes

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: agent-deployment
spec:
  replicas: 2
  selector:
    matchLabels:
      app: agent
  template:
    metadata:
      labels:
        app: agent
    spec:
      containers:
      - name: agent
        image: my-agent:v1
        ports:
        - containerPort: 8000
          name: http

        # Wait for model to load before sending traffic
        readinessProbe:
          httpGet:
            path: /health/ready
            port: http
          initialDelaySeconds: 5    # Check after 5s (model loads in 15s)
          periodSeconds: 5          # Check every 5s
          timeoutSeconds: 2         # Request must complete in 2s
          failureThreshold: 2       # 2 failures = not ready (10s total)

        # Detect if container is stuck or crashed
        livenessProbe:
          httpGet:
            path: /health/live
            port: http
          initialDelaySeconds: 20   # Wait for startup before checking
          periodSeconds: 10         # Check every 10s (less frequent)
          timeoutSeconds: 2
          failureThreshold: 3       # 3 failures = restart pod (30s total)

        # Buy time for initialization before liveness checks
        startupProbe:
          httpGet:
            path: /health/ready
            port: http
          periodSeconds: 5          # Check every 5s during startup
          failureThreshold: 6       # Allow 30s for startup (6 * 5s)
```

**What happens when you deploy this:**

```bash
kubectl apply -f deployment.yaml
```

**Output:**
```
deployment.apps/agent-deployment created

# Watch pod startup
kubectl get pods -w

NAME                                 READY   STATUS    RESTARTS
agent-deployment-7d4f5c6b9f-abc123   0/1     Running   0
agent-deployment-7d4f5c6b9f-xyz789   0/1     Running   0

# Startup probe running (checking every 5s)
# At 5s: startup probe fails (model still loading)
# At 10s: startup probe fails
# At 15s: startup probe succeeds (model loaded)
#
# Now readiness and liveness probes take over

# After ~20s:
agent-deployment-7d4f5c6b9f-abc123   1/1     Running   0
agent-deployment-7d4f5c6b9f-xyz789   1/1     Running   0

# View probe events
kubectl describe pod agent-deployment-7d4f5c6b9f-abc123

# Output excerpt:
# Events:
#   Type     Reason     Age   From               Message
#   ----     ------     ---   ----               -------
#   Normal   Created    45s   kubelet            Created container agent
#   Normal   Started    45s   kubelet            Started container agent
#   Warning  Unhealthy  40s   kubelet            Readiness probe failed: HTTP probe failed
#   Warning  Unhealthy  35s   kubelet            Readiness probe failed: HTTP probe failed
#   Normal   Ready      30s   kubelet            Container is ready
```

---

## TCP Socket Probes (Port Availability)

Use TCP probes when you don't have an HTTP endpoint, or for protocols like gRPC/database connections.

### TCP Probe Configuration

```yaml
spec:
  containers:
  - name: cache
    image: redis:7
    ports:
    - containerPort: 6379

    livenessProbe:
      tcpSocket:
        port: 6379
      initialDelaySeconds: 10
      periodSeconds: 10
      timeoutSeconds: 1
      failureThreshold: 3
```

**How it works**:
- Kubernetes attempts TCP connection to port
- Success → Port is open and accepting connections
- Timeout/failure → Port not responding

**Example**: Redis cache in your cluster

```bash
# Redis container starts but is slow to open port
kubectl logs deployment/redis-deployment

# Output:
Ready to accept connections
```

**kubectl describe** shows probe activity:

```bash
kubectl describe pod redis-deployment-xxxxx

# Output excerpt:
# Conditions:
#   Ready          True
#   ContainersReady True
# Events:
#   Type    Reason     Age  From     Message
#   ----    ------     ---  ----     -------
#   Normal  Created    1m   kubelet  Created container cache
#   Normal  Started    1m   kubelet  Started container cache
#   Normal  Healthy    55s  kubelet  Tcp-socket probe succeeded
```

---

## Exec Probes (Custom Commands)

Run arbitrary shell commands for custom health checks.

### Exec Probe Example

Your agent needs a complex health check:

```yaml
spec:
  containers:
  - name: agent
    image: my-agent:v1

    readinessProbe:
      exec:
        command:
        - /bin/sh
        - -c
        - |
          # Check if critical files exist and are recent
          [ -f /app/model.pkl ] && \
          [ $(find /app/model.pkl -mmin -2) ] && \
          curl -sf http://localhost:8000/health/ready > /dev/null
      initialDelaySeconds: 5
      periodSeconds: 10
      timeoutSeconds: 5
      failureThreshold: 2
```

**Explanation**:
- `exec.command`: Run shell commands
- First part: Verify model file exists and was modified in last 2 minutes
- Second part: Verify HTTP endpoint responds
- Exit code 0 = healthy, non-zero = unhealthy

### Example with Database Check

```yaml
livenessProbe:
  exec:
    command:
    - python
    - -c
    - |
      import sqlite3
      conn = sqlite3.connect('/data/cache.db')
      cursor = conn.cursor()
      cursor.execute('SELECT 1')
      cursor.close()
      print('Database healthy')
```

**Output** when you check the pod:

```bash
kubectl logs agent-deployment-xxxxx -c agent

# Output:
Database healthy
Database healthy
Database healthy

# If it fails:
kubectl describe pod agent-deployment-xxxxx

# Output excerpt:
# Events:
#   Type     Reason     Age   Message
#   ----     ------     ---   -------
#   Warning  Unhealthy  30s   Exec probe failed: Database connection failed
```

---

## Timing Parameters (Critical Decisions)

Probe timing parameters dramatically affect pod lifecycle. Choosing wrong values causes cascading failures.

### initialDelaySeconds: Wait Before First Check

**Purpose**: Give container time to start before first health check.

**Decision framework**:
- **Fast startup** (web server): 5-10s
- **Moderate startup** (cache warming): 15-30s
- **Slow startup** (model loading): 30-60s
- **Very slow startup** (fine-tuning, large models): Use `startupProbe` instead

**Wrong value too low (5s for 15s load time)**:
```
0s   Container starts
5s   First readiness probe → FAILS (model still loading)
15s  Model finished loading
20s  Readiness probe finally succeeds
→ Pod unavailable for 15s unnecessarily
```

**Right value (25s for 15s load time)**:
```
0s   Container starts
5s   Startup probe checking
15s  Model loaded
20s  Startup probe succeeds
25s  First readiness check (succeeds)
→ Cleaner transition
```

### periodSeconds: How Often to Check

**Purpose**: Frequency of health checks after first check.

**Decision framework**:
- **Readiness probe**: 5-10s (frequent, pod removal is gentle)
- **Liveness probe**: 10-30s (less frequent, restart is drastic)
- **Startup probe**: 5s (frequent during init, then liveness takes over)

**Why different frequencies?**
- Readiness: Removing pod from service is non-destructive
- Liveness: Restarting pod loses in-flight requests

### timeoutSeconds: Wait for Response

**Purpose**: How long before probe response is considered failure.

**Decision framework**:
- **Healthy endpoint**: 1-2s
- **Slow endpoint**: 5s
- **Database query**: 5-10s

**Watch out**: If timeout >= period, you get overlapping probes.

```
Wrong: periodSeconds: 5, timeoutSeconds: 10
→ Probe takes 10s to timeout, but next probe starts at 5s
→ Multiple overlapping probes, confusion

Right: periodSeconds: 5, timeoutSeconds: 2
→ Probe completes in 2s, next starts at 5s, clean
```

### failureThreshold: How Many Failures Until Action

**Purpose**: Prevent flaky endpoints from triggering cascading restarts.

**Decision framework**:
- **Readiness probe**: 2-3 (quicker reaction to real issues)
- **Liveness probe**: 3-5 (more tolerant, restart is drastic)
- **Startup probe**: 6-10 (allow slow initialization)

**Math example**:
```
readiness: periodSeconds=5, failureThreshold=3
→ 3 failed checks * 5s = 15s total before pod removed

liveness: periodSeconds=10, failureThreshold=3
→ 3 failed checks * 10s = 30s total before restart

startup: periodSeconds=5, failureThreshold=6
→ 6 failed checks * 5s = 30s allowed for initialization
```

---

## Debugging Probe Failures

When pods aren't becoming ready or are restarting, probes are usually the culprit.

### Step 1: Check Pod Status

```bash
kubectl get pods
```

**Output:**
```
NAME                                 READY   STATUS    RESTARTS
agent-deployment-7d4f5c6b9f-abc123   0/1     Running   0
```

**Interpretation**:
- READY 0/1 = Pod running but not ready (readiness probe failing)
- RESTARTS increasing = Liveness probe killing pod

### Step 2: Describe Pod for Events

```bash
kubectl describe pod agent-deployment-7d4f5c6b9f-abc123
```

**Output** (showing readiness probe failure):

```
Name:           agent-deployment-7d4f5c6b9f-abc123
Namespace:      default
Status:         Running
Containers:
  agent:
    Image:      my-agent:v1
    State:      Running
      Started:  Sat, 22 Dec 2024 15:42:30 UTC
    Ready:      False
    Restart Count: 0
    Readiness:  exec Exec probe failed

Events:
  Type     Reason     Age    From               Message
  ----     ------     ---    ----               -------
  Normal   Created    2m18s  kubelet            Created container agent
  Normal   Started    2m18s  kubelet            Started container agent
  Warning  Unhealthy  2m13s  kubelet            Readiness probe failed: exec probe failed
  Warning  Unhealthy  2m08s  kubelet            Readiness probe failed: exec probe failed
  Warning  Unhealthy  2m03s  kubelet            Readiness probe failed: exec probe failed
```

**Key info**:
- `Ready: False` = Readiness probe failing
- `Readiness: exec Exec probe failed` = Which probe is failing
- Events show exact failure messages

### Step 3: Check Container Logs

```bash
kubectl logs agent-deployment-7d4f5c6b9f-abc123

# Output:
Starting model load...
Model loaded in 15.2s
WARNING: POST /predict called before startup complete
```

**Diagnosis**: Model takes 15s to load, but readiness probe starts checking at 5s.

### Step 4: Test Probe Manually

Port forward to test endpoint directly:

```bash
kubectl port-forward pod/agent-deployment-7d4f5c6b9f-abc123 8000:8000
```

**Output:**
```
Forwarding from 127.0.0.1:8000 -> 8000
Forwarding from [::1]:8000 -> 8000
```

In another terminal, test the readiness endpoint:

```bash
curl http://localhost:8000/health/ready
```

**Output** (if model is still loading):
```
{"error": "Model still loading"}
```

**Output** (if model is ready):
```
{"status": "ready", "ready": true}
```

### Step 5: Fix and Retest

Common fixes:

```yaml
# Problem: initialDelaySeconds too low
# Fix: Increase delay
readinessProbe:
  httpGet:
    path: /health/ready
    port: 8000
  initialDelaySeconds: 30  # Was 5, now 30

# Problem: Endpoint too slow
# Fix: Increase timeout
  timeoutSeconds: 5  # Was 1

# Problem: Flaky endpoint
# Fix: Increase failureThreshold
  failureThreshold: 3  # Was 1
```

Apply updated deployment:

```bash
kubectl apply -f deployment.yaml

# Watch pods restart with new config
kubectl get pods -w

NAME                                 READY   STATUS     RESTARTS
agent-deployment-7d4f5c6b9f-abc123   0/1     Pending    0
agent-deployment-7d4f5c6b9f-abc123   0/1     Running    0
agent-deployment-7d4f5c6b9f-abc123   1/1     Running    0

# Now ready!
```

---

## AI-Native Health Checks

For AI agents with variable startup times and complex dependencies, design probes thoughtfully.

### Pattern: Startup Probe + Readiness Probe + Liveness Probe

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: ai-agent
spec:
  replicas: 3
  template:
    spec:
      containers:
      - name: agent
        image: my-agent:latest

        # Buy time for model initialization
        startupProbe:
          httpGet:
            path: /health/startup
            port: 8000
          periodSeconds: 5
          failureThreshold: 30  # 150s total (5 * 30)

        # Check if model is loaded and ready
        readinessProbe:
          httpGet:
            path: /health/ready
            port: 8000
          initialDelaySeconds: 5
          periodSeconds: 5
          timeoutSeconds: 2
          failureThreshold: 2

        # Detect if model inference is broken
        livenessProbe:
          httpGet:
            path: /health/live
            port: 8000
          initialDelaySeconds: 30
          periodSeconds: 10
          timeoutSeconds: 2
          failureThreshold: 3
```

### Probe Implementation Strategy

```python
# health.py - Health check endpoints for AI agent
from fastapi import FastAPI
from datetime import datetime
import os

# Global state
model = None
model_loaded_at = None
last_inference_at = None
last_inference_error = None

@app.on_event("startup")
async def initialize_agent():
    global model, model_loaded_at, last_inference_at
    print("Loading model...")
    # Simulate loading process
    await asyncio.sleep(30)  # Real: model.load(), cache.warm()
    model = True
    model_loaded_at = datetime.now()
    last_inference_at = datetime.now()

@app.get("/health/startup")
async def startup():
    """Startup probe: Is initialization complete?"""
    if model is None:
        return JSONResponse({"status": "initializing"}, status_code=503)
    return JSONResponse({"status": "started"})

@app.get("/health/ready")
async def readiness():
    """Readiness probe: Is this pod ready for traffic?"""
    if model is None:
        return JSONResponse({"status": "not_ready"}, status_code=503)

    # Optional: Check dependencies
    if not check_cache_healthy():
        return JSONResponse({"status": "cache_unhealthy"}, status_code=503)

    return JSONResponse(
        {"status": "ready", "model_loaded_at": model_loaded_at.isoformat()}
    )

@app.get("/health/live")
async def liveness():
    """Liveness probe: Is inference still working?"""
    if model is None:
        return JSONResponse({"status": "not_initialized"}, status_code=503)

    # Check for stuck state (no inference in 5 minutes)
    if (datetime.now() - last_inference_at).total_seconds() > 300:
        return JSONResponse(
            {"status": "stale", "age_seconds": 300},
            status_code=503
        )

    # Check for recent errors
    if last_inference_error:
        return JSONResponse(
            {"status": "error", "last_error": last_inference_error},
            status_code=503
        )

    return JSONResponse({"status": "healthy"})

@app.post("/predict")
async def predict(data: dict):
    global last_inference_at, last_inference_error

    try:
        if model is None:
            raise RuntimeError("Model not loaded")

        result = run_inference(data)
        last_inference_at = datetime.now()
        last_inference_error = None
        return result
    except Exception as e:
        last_inference_error = str(e)
        return JSONResponse({"error": str(e)}, status_code=500)
```

---

## Try With AI

Now you've learned to configure probes manually. In the next lesson, you'll use kubectl-ai to generate probe configurations automatically and debug real cluster issues with AI guidance.

**Setup**: You have a containerized agent with a health endpoint.

**Challenge**: Configure probes for an agent with these characteristics:
- Model loads in 20 seconds
- Health endpoint responds in under 500ms when healthy
- You want aggressive failure detection for readiness (pod removed quickly if unhealthy)
- You want conservative failure detection for liveness (don't restart on transient errors)

**Action Prompt**: Write a Deployment manifest with startup, readiness, and liveness probes. Consider:
1. How long should initialDelaySeconds be?
2. How frequent should readiness checks be (periodSeconds)?
3. How many failures before removing pod from service (failureThreshold)?
4. What should failureThreshold be for liveness (more conservative)?

Test your configuration by deploying to Minikube and simulating failures (make your health endpoint return 500 temporarily to test probe behavior).
