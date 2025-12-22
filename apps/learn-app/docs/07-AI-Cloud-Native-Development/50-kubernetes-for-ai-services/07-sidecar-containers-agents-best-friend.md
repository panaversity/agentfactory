---
sidebar_position: 7
chapter: 50
lesson: 7
duration_minutes: 40
title: "Sidecar Containers: The Agent's Best Friend"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Manual sidecar configuration builds understanding of multi-container Pod patterns"
cognitive_load:
  concepts_count: 8
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Explain the sidecar pattern and native sidecar support in K8s 1.28+"
    bloom_level: "Understand"
  - id: LO2
    description: "Create native sidecars with restartPolicy: Always"
    bloom_level: "Apply"
  - id: LO3
    description: "Configure sidecars for logging, monitoring, and proxying"
    bloom_level: "Apply"
  - id: LO4
    description: "Share volumes between main and sidecar containers"
    bloom_level: "Apply"
  - id: LO5
    description: "Understand sidecar startup and shutdown ordering"
    bloom_level: "Understand"
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
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
---

# Sidecar Containers: The Agent's Best Friend

You've deployed your agent to Kubernetes. Requests arrive. Your agent handles them. But where do the logs go? Where's the monitoring data? How do you trace which request took how long?

Traditional single-container deployments mix your application logic with logging, monitoring, and debugging concerns. Your agent code becomes cluttered. Operational concerns become tangled with business logic. Testing gets harder.

**Sidecars** solve this. A sidecar is a helper container that runs alongside your main application container—in the same Pod, sharing the same network and storage. Your agent focuses on its job. The sidecar handles logging, metrics, proxying, or security concerns independently. They coordinate through shared volumes and localhost networking.

This lesson teaches you to design and deploy sidecars using Kubernetes' native sidecar support (available since Kubernetes 1.28). By the end, you'll deploy an agent with a logging sidecar and a metrics sidecar, keeping operational concerns separated from application logic.

---

## The Sidecar Pattern: Separation of Concerns

### The Problem: Tangled Concerns

Imagine your FastAPI agent needs to:
1. Handle incoming requests
2. Write structured logs to a file
3. Expose Prometheus metrics
4. Trace request latency

If all this logic lives in one container, your code becomes messy:

```python
@app.post("/inference")
async def inference(request: Request):
    # Business logic
    result = model.predict(request.text)

    # Logging concern
    with open("/var/log/requests.log", "a") as f:
        f.write(json.dumps({"request": request.text, "result": result}))

    # Metrics concern
    inference_counter.inc()
    inference_latency.observe(time.time() - start)

    # Return result
    return {"prediction": result}
```

Your application code is entangled with operational concerns. Testing requires mocking file I/O and metrics. Scaling requires coordinating all concerns together.

### The Solution: Sidecars

Instead, separate concerns into independent containers:

```
┌──────────────────────────────────────────────────┐
│             Pod (Shared Network)                 │
├──────────────────────┬──────────────────────────┤
│                      │                          │
│  Main Container      │  Logging Sidecar         │
│  (FastAPI Agent)     │                          │
│                      │  - Reads /var/log/agent  │
│  @app.post("/")      │  - Streams to stdout     │
│  Handle requests     │  - Or sends to central   │
│                      │    logging system        │
│                      │                          │
├──────────────────────┼──────────────────────────┤
│  Metrics Sidecar     │  Proxy Sidecar           │
│                      │  (Optional)              │
│  - Scrapes metrics   │                          │
│  - Exposes on :9090  │  - TLS termination       │
│                      │  - Service mesh inject   │
│                      │  - Request routing       │
└──────────────────────┴──────────────────────────┘
      Shared Volume (/var/log)
      Shared Network (localhost)
```

Your agent writes logs to `/var/log/agent/requests.log`. The logging sidecar watches that file and streams it to a central logging service. Your agent exposes metrics on `localhost:8000/metrics`. A metrics sidecar scrapes and forwards those metrics.

Each container has one job. Testing is simpler. Scaling is predictable.

---

## Native Sidecars: Kubernetes 1.28+

Before Kubernetes 1.28, sidecars were regular containers in the `containers` field, but Kubernetes couldn't distinguish them from the main application container. This created ambiguity about startup ordering and lifecycle.

Kubernetes 1.28 introduced the `initContainers` field with a new `restartPolicy: Always` option specifically for sidecars:

### How It Works

1. **Init containers with `restartPolicy: Always`** start before main containers
2. They run continuously (unlike traditional init containers that exit after setup)
3. If a sidecar crashes, Kubernetes restarts it independently
4. Main container startup is NOT blocked by sidecar readiness

This guarantees:
- Sidecars are ready before main application starts processing requests
- Sidecars stay alive throughout the Pod's lifetime
- Main container failures don't restart sidecars unnecessarily

---

## Concept 1: Writing Logs to Shared Volumes

Your agent and sidecars must communicate. The most reliable method: **shared volumes**.

Create a Pod with:
- Main container: Writes logs to `/var/log/agent/`
- Logging sidecar: Reads from `/var/log/agent/`, streams to stdout or external service

### Step 1: Create Agent with Log Volume

Create `agent-with-logging.yaml`:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: agent-with-logging
spec:
  containers:
  - name: agent
    image: myregistry.azurecr.io/agent:v1
    ports:
    - containerPort: 8000
    volumeMounts:
    - name: log-volume
      mountPath: /var/log/agent
    env:
    - name: LOG_FILE
      value: /var/log/agent/requests.log

  initContainers:
  - name: logging-sidecar
    image: fluent/fluent-bit:latest
    restartPolicy: Always  # <-- Native sidecar: Always restarts
    volumeMounts:
    - name: log-volume
      mountPath: /var/log/agent
    args:
    - -c
    - /fluent-bit/etc/fluent-bit.conf

  volumes:
  - name: log-volume
    emptyDir: {}
```

Key points:
- **emptyDir**: Temporary storage, shared across containers in the Pod
- **volumeMounts**: Both containers mount the same volume at `/var/log/agent`
- **initContainers with restartPolicy: Always**: Logging sidecar starts before agent and stays running

Deploy it:

```bash
kubectl apply -f agent-with-logging.yaml
```

**Output:**
```
pod/agent-with-logging created
```

Verify the Pod is running:

```bash
kubectl get pods
```

**Output:**
```
NAME                   READY   STATUS    RESTARTS   AGE
agent-with-logging     2/2     Running   0          5s
```

The `2/2` shows both containers are ready. Without the sidecar, you'd see `1/1`.

### Step 2: Verify Log Volume Sharing

Exec into the agent container and create a test log:

```bash
kubectl exec -it agent-with-logging -c agent -- sh
```

Inside the container:

```bash
echo '{"request": "test", "result": "success"}' >> /var/log/agent/requests.log
cat /var/log/agent/requests.log
```

**Output:**
```
{"request": "test", "result": "success"}
```

Exit:

```bash
exit
```

Now exec into the logging sidecar and read the same file:

```bash
kubectl exec -it agent-with-logging -c logging-sidecar -- sh
```

Inside the sidecar:

```bash
cat /var/log/agent/requests.log
```

**Output:**
```
{"request": "test", "result": "success"}
```

Both containers see the same logs. The sidecar can now stream this data to a centralized logging service (e.g., Elasticsearch, Splunk, or Stackdriver).

---

## Concept 2: Multi-Sidecar Pod with Metrics

Most production Pods run multiple sidecars. Let's add a metrics sidecar alongside logging.

Create `agent-with-logging-and-metrics.yaml`:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: agent-with-sidecars
spec:
  containers:
  - name: agent
    image: myregistry.azurecr.io/agent:v1
    ports:
    - containerPort: 8000
      name: http
    - containerPort: 8001
      name: metrics  # <-- Agent exposes metrics here
    volumeMounts:
    - name: log-volume
      mountPath: /var/log/agent
    env:
    - name: LOG_FILE
      value: /var/log/agent/requests.log

  initContainers:
  - name: logging-sidecar
    image: fluent/fluent-bit:latest
    restartPolicy: Always
    volumeMounts:
    - name: log-volume
      mountPath: /var/log/agent

  - name: metrics-sidecar
    image: prom/prometheus:latest
    restartPolicy: Always
    ports:
    - containerPort: 9090
      name: prometheus
    volumeMounts:
    - name: prometheus-config
      mountPath: /etc/prometheus
    args:
    - --config.file=/etc/prometheus/prometheus.yml

  volumes:
  - name: log-volume
    emptyDir: {}
  - name: prometheus-config
    configMap:
      name: prometheus-config
```

Deploy:

```bash
kubectl apply -f agent-with-logging-and-metrics.yaml
```

**Output:**
```
pod/agent-with-sidecars created
```

Check Pod status:

```bash
kubectl get pods
```

**Output:**
```
NAME                     READY   STATUS    RESTARTS   AGE
agent-with-sidecars      3/3     Running   0          8s
```

Three containers running in one Pod: agent, logging sidecar, metrics sidecar. All share the Pod's network namespace, so the metrics sidecar can reach the agent's metrics endpoint at `localhost:8001/metrics` without any network configuration.

---

## Concept 3: Sidecar Lifecycle and Ordering

### Init Containers with restartPolicy: Always

When you use `initContainers` with `restartPolicy: Always`, Kubernetes guarantees:

1. **Startup order**: Init containers start BEFORE main containers
2. **Independence**: Each init container runs to completion before the next starts
3. **Restart behavior**: If an init container crashes, it restarts (not the entire Pod)
4. **Main container waits**: Main containers don't start until all init containers are healthy

This is different from:
- **Traditional init containers** (`restartPolicy: OnFailure` or default): Run once, exit, never restart
- **Regular containers**: Can be restarted independently, but don't block main container startup

### Example: Startup Sequence

```
Time 0:    Pod created
           └─ Init container 1 (logging-sidecar) starts
Time 1:    Logging sidecar ready
           └─ Init container 2 (metrics-sidecar) starts
Time 2:    Metrics sidecar ready
           └─ Main container (agent) starts
Time 3:    Agent running, processing requests
           └─ All sidecars handle logging/metrics independently

Time 10:   Metrics sidecar crashes
           └─ Kubernetes immediately restarts it
           └─ Agent continues running (unaffected)

Time 15:   Metrics sidecar recovered
           └─ Continues scraping metrics from agent
```

This ordering prevents race conditions where the agent starts before sidecars are ready.

### Shutdown Sequence

When a Pod terminates:

```
Time 0:    Pod receives termination signal
           └─ All containers receive SIGTERM

Time 0-30: Containers have grace period to shut down
           └─ Agent stops accepting requests
           └─ Sidecars flush any buffered data
           └─ Connections close gracefully

Time 30:   Grace period expires
           └─ Kubernetes sends SIGKILL
           └─ Pod and all containers terminated
```

All containers terminate together, which is why sidecars (not separate sidecar Pods) are essential—they share the Pod's lifecycle.

---

## Practical Configuration: Logging Sidecar for Agent Inference

Let's build a realistic logging sidecar for your agent. Your agent logs inference requests and latencies. The sidecar streams these to stdout for collection.

Create `agent-inference-logging.yaml`:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: agent-inference-logger
spec:
  containers:
  - name: agent
    image: myregistry.azurecr.io/agent:v1
    ports:
    - containerPort: 8000
    volumeMounts:
    - name: logs
      mountPath: /var/log/agent
    env:
    - name: LOG_FILE
      value: /var/log/agent/inference.log

  initContainers:
  - name: log-collector
    image: fluent/fluent-bit:latest
    restartPolicy: Always
    volumeMounts:
    - name: logs
      mountPath: /var/log/agent
    - name: fluent-config
      mountPath: /fluent-bit/etc
    args:
    - -c
    - /fluent-bit/etc/fluent-bit.conf

  volumes:
  - name: logs
    emptyDir: {}
  - name: fluent-config
    configMap:
      name: fluent-bit-config
```

Deploy:

```bash
kubectl apply -f agent-inference-logging.yaml
```

**Output:**
```
pod/agent-inference-logger created
```

View logs from the sidecar:

```bash
kubectl logs agent-inference-logger -c log-collector
```

**Output:**
```
[2025-12-22 14:35:12.123] [info] [fluent-bit] version 2.1.0, commit hash: abc1234
[2025-12-22 14:35:12.234] [info] starting log collection from /var/log/agent/inference.log
[2025-12-22 14:35:23.456] [info] collected: {"request_id": "abc123", "latency_ms": 245, "status": "success"}
[2025-12-22 14:35:45.789] [info] collected: {"request_id": "def456", "latency_ms": 198, "status": "success"}
```

The sidecar reads and streams logs, which Kubernetes collects and makes available via `kubectl logs`.

---

## Summary

Sidecars are helper containers that live in the same Pod as your main application. They handle operational concerns (logging, metrics, proxying, security) independently, keeping your application code clean.

**Key takeaways:**

1. **Native sidecars** (Kubernetes 1.28+) use `initContainers` with `restartPolicy: Always`
2. **Shared volumes** (`emptyDir`) let containers communicate reliably
3. **Shared network** (`localhost`) means sidecar and app can reach each other without service discovery
4. **Lifecycle guarantees**: Sidecars start first, restart independently, and shutdown with the Pod
5. **Separation of concerns**: Application logic stays separate from operational concerns

Your FastAPI agent writes logs to `/var/log/agent/`. The logging sidecar reads them. Your agent exposes metrics on `localhost:8001/metrics`. The metrics sidecar scrapes them. Clean. Scalable. Testable.

---

## Try With AI

Open a terminal and work through these scenarios with an AI assistant's help:

### Scenario 1: Design a Logging Sidecar

**Your task:**
Design a Pod manifest where:
- Main container: A Python FastAPI agent running on port 8000
- Sidecar: Fluent Bit collecting logs from `/var/log/agent/requests.log` and forwarding to an external logging service (e.g., Datadog or CloudWatch)

Ask AI: "Create a Pod manifest with a FastAPI agent and a Fluent Bit sidecar that collects logs and forwards them to [your logging service]."

Review AI's response:
- Is the sidecar configured as an init container with `restartPolicy: Always`?
- Do both containers mount the same log volume?
- Is the Fluent Bit configuration correct for your logging service?
- Are environment variables set correctly for API keys or endpoints?

Tell AI: "The logging service requires authentication. I need to pass credentials securely."

Ask: "Update the manifest to inject credentials using a Kubernetes Secret."

**Reflection:**
- How does the sidecar access the Secret?
- What would break if the logging service becomes unavailable?
- Could you test this locally with Minikube?

### Scenario 2: Multi-Sidecar Deployment

**Your task:**
Your agent now needs:
1. Logging sidecar (Fluent Bit)
2. Metrics sidecar (Prometheus)
3. Security sidecar (mTLS termination with Linkerd or Istio)

Ask AI: "Create a Pod manifest with three sidecars: logging, metrics, and security. Explain the startup and shutdown ordering."

AI might suggest:
- All sidecars as init containers with `restartPolicy: Always`
- Separate ConfigMaps for each sidecar's configuration
- Shared volumes for log files, shared network for metrics scraping

Ask: "What happens if the metrics sidecar crashes? Will my agent stop handling requests?"

**Reflection:**
- Why are sidecars better than separate Pods?
- How would you test sidecar failures locally?
- What's the memory and CPU cost of adding a third sidecar?

### Scenario 3: Troubleshoot Sidecar Failures

**Your task:**
You deployed a Pod with an agent and logging sidecar. The Pod shows `2/2 READY`, but logs aren't being collected. The Fluent Bit sidecar is healthy, but no data is appearing in your logging service.

Ask AI: "My logging sidecar is running but not collecting logs. The agent container is writing to `/var/log/agent/requests.log`. What could be wrong?"

AI might suggest:
- Verify the volume mount path is correct
- Check Fluent Bit configuration syntax
- Confirm the logging service credentials are valid
- Inspect Fluent Bit logs with `kubectl logs -c logging-sidecar`

Ask: "Show me the exact kubectl commands to debug this."

**Reflection:**
- What would you look for in the sidecar logs?
- How would you verify the agent is actually writing to the shared volume?
- Could a permissions issue prevent the sidecar from reading the log file?
