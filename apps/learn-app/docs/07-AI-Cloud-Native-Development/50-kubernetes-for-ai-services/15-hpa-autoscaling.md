---
sidebar_position: 15
chapter: 50
lesson: 15
duration_minutes: 40
title: "Horizontal Pod Autoscaler for AI Agents"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Learn HPA mechanics through hands-on scaling configuration"
cognitive_load:
  concepts_count: 8
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Configure metrics-server in Minikube to enable resource monitoring"
    bloom_level: "Apply"
  - id: LO2
    description: "Create HPA resources that automatically scale Pods based on CPU utilization"
    bloom_level: "Apply"
  - id: LO3
    description: "Analyze HPA scaling behavior under load"
    bloom_level: "Analyze"
  - id: LO4
    description: "Configure scaling behavior with stabilization windows"
    bloom_level: "Apply"
---

# Horizontal Pod Autoscaler for AI Agents

Your agent deployment is running smoothly on Kubernetes. Traffic arrives in waves: quiet periods, then sudden bursts when multiple users send inference requests simultaneously.

If you set replicas to handle peak load, you're wasting money on idle Pods during quiet periods. If you set replicas for average load, you're rejecting requests during spikes.

Kubernetes solves this with the Horizontal Pod Autoscaler (HPA). It watches CPU usage (or memory, or custom metrics) and automatically scales Pods up when demand rises and down when demand falls. You specify min and max replicas; HPA keeps the cluster balanced between them.

This lesson teaches you how HPA works, why it matters for AI agents, and how to configure it on Minikube.

---

## Concept 1: Why Autoscaling Matters for AI Agents

AI inference workloads have unpredictable demand. Unlike web servers that handle requests with minimal CPU, inference requests are CPU-intensive:

- **During quiet periods**: 2-3 agent replicas handle light traffic efficiently. Extra replicas waste compute.
- **During spike**: 20+ users send inference requests simultaneously. Without autoscaling, requests queue and timeout.
- **Custom metrics**: Some agents measure "queue depth" or "inference latency" as scaling triggers, not just CPU.

HPA solves this by:

1. **Continuous monitoring**: Kubernetes measures resource usage of every Pod
2. **Automatic scaling**: When CPU exceeds threshold (e.g., 50%), scale up
3. **Stabilization**: Wait before scaling down to prevent thrashing (replicas flickering up/down constantly)
4. **Cost optimization**: Run only as many replicas as needed

The mental model: HPA is a feedback loop that keeps your cluster right-sized to current demand.

---

## Concept 2: Metrics Server — The Eyes of the Cluster

Before HPA can scale, it needs to measure Pod resource usage. This is the job of the **metrics-server**, a cluster component that collects CPU and memory metrics from every container.

### How Metrics Flow

```
kubelet (on each node)
  ↓ reads actual CPU/memory
  ↓
metrics-server
  ↓ aggregates metrics
  ↓
HPA controller
  ↓ reads metrics, decides to scale
  ↓
Deployment
  ↓ scales replicas up/down
```

**Key insight**: HPA without metrics-server cannot function. The cluster won't scale because it has nothing to measure.

### Checking if metrics-server is running

On your Minikube cluster:

```bash
kubectl get deployment -n kube-system | grep metrics-server
```

**Output**:
```
metrics-server   1/1     1            1           2m
```

If metrics-server is not listed, you need to install it.

---

## Concept 3: HPA Resources — The Configuration

An HPA resource is a declarative configuration that tells Kubernetes:
- Which Deployment to scale
- What metric to monitor (CPU, memory, custom)
- What threshold to trigger scaling (e.g., "when CPU exceeds 50%")
- Min and max replica limits

### Anatomy of an HPA

```yaml
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: agent-scaler
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: my-agent
  minReplicas: 2          # Never scale below 2
  maxReplicas: 10         # Never scale above 10
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 50  # Scale up when avg CPU > 50%
  behavior:                      # Advanced: control scaling speed
    scaleDown:
      stabilizationWindowSeconds: 300
    scaleUp:
      stabilizationWindowSeconds: 0
```

**Breaking it down**:

- **scaleTargetRef**: Which Deployment does this HPA manage?
- **minReplicas/maxReplicas**: Bounds on how many Pods can exist
- **metrics**: What to measure and at what threshold
- **behavior**: Fine-tune scaling speed (we'll explain this next)

---

## Concept 4: Scaling Behavior — scaleUp vs scaleDown

HPA scales in two directions. They have different behaviors to prevent problems.

### scaleUp Behavior

When CPU exceeds the target (50% in our example):

1. **Calculate desired replicas**: If 3 Pods are using 70% CPU each, and target is 50%, we need 3 × (70/50) = 4.2 → round to 5 Pods
2. **Stabilization window**: By default, scale up immediately (stabilizationWindowSeconds: 0)
3. **Max scale-up per minute**: Kubernetes limits how fast replicas can increase (prevents runaway)

**Example**:
- Current: 3 replicas, 70% CPU each
- Target: 50% CPU
- Desired: 5 replicas
- Action: Create 2 new Pods immediately

### scaleDown Behavior

When CPU falls below the target:

1. **Calculate desired replicas**: If 10 Pods are using 20% CPU each, and target is 50%, we need 10 × (20/50) = 4 Pods
2. **Stabilization window**: Wait 300 seconds (5 minutes) before scaling down
3. **Why the wait?**: Prevent thrashing. If a spike is brief (2 minutes), you don't want to scale down then back up immediately.

**Example**:
- Current: 10 replicas, 20% CPU each
- Target: 50% CPU
- Desired: 4 replicas
- Decision: Wait 5 minutes, then remove 6 Pods

**Key principle**: Scale up fast (respond to spikes), scale down slow (don't over-react to dips).

---

## Concept 5: Stabilization Windows — Preventing Thrashing

A stabilization window prevents HPA from making rapid, contradictory scaling decisions.

### The Problem Without Stabilization

Imagine no stabilization:

```
Time    CPU   Replicas   Decision
0s      75%   3          Scale to 5
30s     48%   5          Scale down to 2
60s     80%   2          Scale to 4
90s     35%   4          Scale down to 2
```

Replicas are constantly changing. This causes:
- Pod churn (containers starting/stopping repeatedly)
- Cascading failures (new Pods crash due to overload)
- Cost thrashing (your bill fluctuates wildly)

### The Solution: Stabilization Windows

**scaleUp stabilization**: Watch for at least N seconds of sustained high CPU before scaling up
**scaleDown stabilization**: Wait N seconds after CPU drops before removing Pods

With 300-second scaleDown stabilization:

```
Time    CPU   Replicas   Decision
0s      75%   3          Scale to 5
30s     48%   5          (wait 300s before considering scale down)
120s    40%   5          (still waiting)
300s    35%   5          Now eligible for scale down → scale to 2
```

In this scenario, the brief 30-second dip doesn't trigger scale down. Only sustained low CPU causes reduction.

---

## Practice 1: Install metrics-server on Minikube

Most Minikube clusters come with metrics-server disabled. Enable it:

```bash
minikube addons enable metrics-server
```

**Output**:
```
✓ metrics-server is enabled
```

Verify it's running:

```bash
kubectl get deployment metrics-server -n kube-system
```

**Output**:
```
NAME             READY   UP-TO-DATE   AVAILABLE   AGE
metrics-server   1/1     1            1           10s
```

Wait 30 seconds for metrics to accumulate, then check if metrics are available:

```bash
kubectl top nodes
```

**Output**:
```
NAME       CPU(cores)   CPU%   MEMORY(Mi)   MEMORY%
minikube   542m         13%    1247Mi       16%
```

If this shows CPU and memory percentages, metrics-server is working. If it shows `<unknown>`, wait another 30 seconds and retry.

---

## Practice 2: Create a Deployment to Scale

Create a simple deployment that consumes CPU when stressed.

**Manifest** (save as `agent-deployment.yaml`):

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: my-agent
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
        image: python:3.11-slim
        command: ["python", "-c"]
        args:
          - |
            import time
            while True:
              # Simple busy loop to consume CPU
              for i in range(1000000):
                _ = i * 2
              time.sleep(0.1)
        resources:
          requests:
            cpu: 100m
            memory: 128Mi
          limits:
            cpu: 500m
            memory: 256Mi
```

Deploy it:

```bash
kubectl apply -f agent-deployment.yaml
```

**Output**:
```
deployment.apps/my-agent created
```

Verify Pods are running:

```bash
kubectl get pods -l app=agent
```

**Output**:
```
NAME                       READY   STATUS    RESTARTS   AGE
my-agent-7c8f8d4b5-abc12   1/1     Running   0          10s
my-agent-7c8f8d4b5-def45   1/1     Running   0          10s
```

Check CPU usage:

```bash
kubectl top pods -l app=agent
```

**Output**:
```
NAME                       CPU(m)   MEMORY(Mi)
my-agent-7c8f8d4b5-abc12   450m     45Mi
my-agent-7c8f8d4b5-def45   420m     42Mi
```

Each Pod is using ~450m CPU (450 millicores). Since the limit is 500m, they're near their maximum.

---

## Practice 3: Create an HPA

Now create an HPA that scales this deployment based on CPU.

**Manifest** (save as `agent-hpa.yaml`):

```yaml
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: agent-scaler
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: my-agent
  minReplicas: 2
  maxReplicas: 10
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 50
  behavior:
    scaleDown:
      stabilizationWindowSeconds: 300
    scaleUp:
      stabilizationWindowSeconds: 0
```

Deploy the HPA:

```bash
kubectl apply -f agent-hpa.yaml
```

**Output**:
```
horizontalpodautoscaler.autoscaling/agent-scaler created
```

Check the HPA status:

```bash
kubectl get hpa
```

**Output**:
```
NAME           REFERENCE           TARGETS   MINPODS   MAXPODS   REPLICAS   AGE
agent-scaler   Deployment/my-agent 90%/50%   2         10        2          15s
```

The `TARGETS` column shows `90%/50%`. This means:
- **Actual usage**: 90% (average of all Pods' CPU)
- **Target**: 50%

Since 90% > 50%, the HPA should decide to scale up. Let's watch it happen:

```bash
kubectl get hpa -w
```

**Output** (watch continuously updates):
```
NAME           REFERENCE           TARGETS   MINPODS   MAXPODS   REPLICAS   AGE
agent-scaler   Deployment/my-agent 90%/50%   2         10        2          20s
agent-scaler   Deployment/my-agent 90%/50%   2         10        3          22s
agent-scaler   Deployment/my-agent 85%/50%   2         10        4          25s
agent-scaler   Deployment/my-agent 70%/50%   2         10        5          30s
agent-scaler   Deployment/my-agent 55%/50%   2         10        6          35s
agent-scaler   Deployment/my-agent 48%/50%   2         10        6          40s
agent-scaler   Deployment/my-agent 48%/50%   2         10        6          45s
```

**What happened**:

1. **Time 20s**: HPA detected 90% CPU > 50% target
2. **Time 22s**: Scaled from 2 to 3 replicas
3. **Time 25s**: Scaled to 4 replicas
4. **Time 30s**: Scaled to 5 replicas
5. **Time 35s**: Scaled to 6 replicas
6. **Time 40s+**: CPU stabilized at 48%, which is below target. HPA stops scaling.

Check the Pods:

```bash
kubectl get pods -l app=agent
```

**Output**:
```
NAME                       READY   STATUS    RESTARTS   AGE
my-agent-7c8f8d4b5-abc12   1/1     Running   0          3m
my-agent-7c8f8d4b5-def45   1/1     Running   0          3m
my-agent-7c8f8d4b5-gh123   1/1     Running   0          2m
my-agent-7c8f8d4b5-ij456   1/1     Running   0           1m 50s
my-agent-7c8f8d4b5-kl789   1/1     Running   0           1m 20s
my-agent-7c8f8d4b5-op012   1/1     Running   0           50s
```

Six Pods are running. CPU is now distributed: each Pod uses less CPU because work is spread across more containers.

---

## Practice 4: Trigger Scale Down

Now let's reduce CPU load and watch HPA scale down.

Delete the Deployment (which stops the busy loop):

```bash
kubectl delete deployment my-agent
```

**Output**:
```
deployment.apps "my-agent" deleted
```

Check the HPA:

```bash
kubectl get hpa
```

**Output**:
```
NAME           REFERENCE           TARGETS         MINPODS   MAXPODS   REPLICAS   AGE
agent-scaler   Deployment/my-agent <unknown>/50%   2         10        6          3m
```

The `TARGETS` column shows `<unknown>` because the Deployment no longer exists. HPA can't scale it.

Let's recreate the Deployment with a lower-CPU workload:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: my-agent
spec:
  replicas: 1
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
        image: python:3.11-slim
        command: ["python", "-c"]
        args:
          - |
            import time
            while True:
              time.sleep(1)
        resources:
          requests:
            cpu: 100m
            memory: 128Mi
          limits:
            cpu: 500m
            memory: 256Mi
```

Deploy it:

```bash
kubectl apply -f agent-deployment.yaml
```

**Output**:
```
deployment.apps/my-agent created
```

Watch HPA:

```bash
kubectl get hpa -w
```

**Output**:
```
NAME           REFERENCE           TARGETS   MINPODS   MAXPODS   REPLICAS   AGE
agent-scaler   Deployment/my-agent 5%/50%    2         10        6          3m
agent-scaler   Deployment/my-agent 5%/50%    2         10        6          3m 30s
agent-scaler   Deployment/my-agent 5%/50%    2         10        6          4m 30s
agent-scaler   Deployment/my-agent 5%/50%    2         10        2          5m 20s
```

**What happened**:

1. **0-4m 30s**: CPU is 5%, below 50% target. HPA calculates desired replicas: 6 × (5/50) = 0.6 → rounds to 1. But minReplicas is 2, so HPA can't scale below 2.
2. **4m 30s-5m**: HPA waits during stabilization window (300 seconds = 5 minutes)
3. **5m 20s**: Stabilization complete, HPA scales down to 2 replicas (the minimum)

The stabilization window prevented HPA from immediately thrashing. It waited 5 minutes of sustained low CPU before scaling down.

---

## Understanding Scaling Math

HPA calculates desired replicas with this formula:

```
desired_replicas = ceil(current_replicas × (current_usage / target))
```

**Example 1: Scale up**
- Current replicas: 2
- Current CPU usage: 80%
- Target: 50%
- Desired: ceil(2 × (80/50)) = ceil(3.2) = 4 replicas

**Example 2: Scale down**
- Current replicas: 6
- Current CPU usage: 20%
- Target: 50%
- Desired: ceil(6 × (20/50)) = ceil(2.4) = 3 replicas

This explains why HPA scales up faster than down:
- **Up**: 2 Pods at 80% → need 4 Pods (2x increase)
- **Down**: 6 Pods at 20% → need 3 Pods (only 50% reduction)

HPA always prefers to provision more Pods than fewer, ensuring requests don't get rejected.

---

## Best Practices for AI Agent Autoscaling

**1. Set requests and limits appropriately**

```yaml
resources:
  requests:
    cpu: 200m
    memory: 256Mi
  limits:
    cpu: 1000m
    memory: 1Gi
```

HPA scales based on **requested** CPU, not used CPU. Requests must reflect what your agent actually needs at baseline.

**2. Use a target utilization between 50-80%**

```yaml
target:
  averageUtilization: 70  # Good balance
```

- Too low (30%): Over-provisioning, high cost
- Too high (90%): Slow response to spikes, rejections

**3. Adjust stabilization windows based on workload**

For bursty inference workloads:

```yaml
behavior:
  scaleUp:
    stabilizationWindowSeconds: 0      # Respond immediately to spikes
  scaleDown:
    stabilizationWindowSeconds: 600    # Wait 10 minutes before scaling down
```

For steady-state services:

```yaml
behavior:
  scaleUp:
    stabilizationWindowSeconds: 60     # Wait 1 minute before scaling up
  scaleDown:
    stabilizationWindowSeconds: 300    # Wait 5 minutes before scaling down
```

**4. Monitor scaling events**

```bash
kubectl describe hpa agent-scaler
```

**Output** (relevant section):
```
Events:
  Type    Reason             Age   Message
  ----    ------             ---   -------
  Normal  SuccessfulRescale  2m    New size: 4; reason: cpu resource utilization (percentage of request) above target
  Normal  SuccessfulRescale  1m    New size: 6; reason: cpu resource utilization (percentage of request) above target
  Normal  SuccessfulRescale  30s   New size: 2; reason: All metrics below target
```

Events show every scaling decision and why it was made. Use this to verify HPA is responding correctly.

---

## Troubleshooting HPA

**Problem**: HPA shows `<unknown>` for TARGETS

```bash
kubectl get hpa
```

**Output**:
```
NAME           REFERENCE           TARGETS         MINPODS   MAXPODS   REPLICAS
agent-scaler   Deployment/my-agent <unknown>/50%   2         10        2
```

**Causes**:
1. Metrics-server not running
2. Pods haven't been running long enough for metrics to accumulate
3. Deployment doesn't exist or is in wrong namespace

**Fix**:
```bash
# Verify metrics-server
kubectl get deployment metrics-server -n kube-system

# Wait 1-2 minutes for metrics to accumulate
kubectl top pods

# Verify Deployment exists
kubectl get deployment my-agent
```

**Problem**: HPA not scaling despite high CPU

```bash
kubectl describe hpa agent-scaler
```

**Output**:
```
Current Metrics:   <some value>
Desired Replicas:  10
Min/Max Replicas:  2/10
Status:            "Waiting for deployment..."
```

**Cause**: HPA calculated it should scale to 10, but Deployment can't create Pods fast enough.

**Fix**:
- Check if nodes have capacity: `kubectl top nodes`
- Check Pod events: `kubectl describe pod <pod-name>`
- Increase resource limits in Deployment if Pods are OOMKilled

---

## Cleanup

When done experimenting:

```bash
kubectl delete hpa agent-scaler
kubectl delete deployment my-agent
```

**Output**:
```
horizontalpodautoscaler.autoscaling "agent-scaler" deleted
deployment.apps "my-agent" deleted
```

---

## Try With AI

You now understand how HPA scales based on CPU. Real-world systems often need more sophisticated scaling based on **custom metrics** — things like queue depth, inference latency, or model confidence scores.

**Setup**: You're deploying an agent that processes inference requests from a queue. Load varies wildly: sometimes empty, sometimes 100 requests queued.

Your challenge: Design an HPA configuration that scales based on **queue depth** (a custom metric) instead of CPU.

**Your Assignment**:

1. **Research custom metrics**

   Ask AI: "How do I configure Kubernetes HPA to scale based on custom metrics like queue depth? What components are needed beyond the standard metrics-server?"

2. **Design the configuration**

   Based on AI's explanation, design an HPA manifest that:
   - Scales a deployment based on queue depth
   - Scales to 10 replicas if queue has 50+ items
   - Scales down to 2 if queue is empty
   - Has a 1-minute stabilization window for scale-down

   Ask AI: "Here's my desired scaling behavior: [describe above]. Write an HPA manifest that achieves this using custom metrics."

3. **Understand the components**

   Ask AI: "In a custom metrics setup, what's the relationship between Prometheus, custom-metrics-api, and HPA? How does the data flow?"

4. **Iterate on thresholds**

   Discuss with AI: "If an agent takes 30 seconds to process one request, and I want maximum 2-second response time, what queue depth should trigger scaling to 10?"

   Work through the math together:
   - Expected throughput: 1 request / 30 seconds = 2 requests/minute per Pod
   - With 10 Pods: 20 requests/minute
   - Queue depth to maintain 2-second latency: (20 requests/min) × (2 seconds) / 60 = 0.67 requests

   Refine your threshold based on this analysis.

**Expected outcome**: You'll understand that CPU-based scaling is simple but crude. Custom metrics enable precise control over system behavior. You don't scale "when CPU is high"—you scale "when queue depth exceeds healthy levels."

