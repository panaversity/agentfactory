---
sidebar_position: 7
chapter: 50
lesson: 7
duration_minutes: 40
title: "Resource Management and Debugging"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Hands-on debugging builds cluster troubleshooting skills"
cognitive_load:
  concepts_count: 8
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Set resource requests and limits for Pods"
    bloom_level: "Apply"
  - id: LO2
    description: "Understand QoS classes and eviction priority"
    bloom_level: "Understand"
  - id: LO3
    description: "Diagnose common Pod failures (CrashLoopBackOff, ImagePullBackOff, Pending, OOMKilled)"
    bloom_level: "Analyze"
  - id: LO4
    description: "Use kubectl describe, logs, and exec for interactive debugging"
    bloom_level: "Apply"
  - id: LO5
    description: "Interpret cluster events to identify failure root causes"
    bloom_level: "Analyze"
  - id: LO6
    description: "Fix resource-related issues through manifest adjustment"
    bloom_level: "Apply"
digcomp_mapping:
  - objective_id: "LO1"
    competency_area: "5. Problem Solving"
    competency: "5.3 Using digital tools to solve problems"
  - objective_id: "LO2"
    competency_area: "5. Problem Solving"
    competency: "5.4 Identifying digital competence gaps"
  - objective_id: "LO3"
    competency_area: "5. Problem Solving"
    competency: "5.3 Using digital tools to solve problems"
---

# Resource Management and Debugging

Your Kubernetes cluster is running Pods. Everything works perfectly in development. Then you deploy to production.

Your Pod crashes immediately. Or it stays Pending forever. Or it consumes all memory and gets evicted. You don't know why—you just see error states and no explanation.

This lesson teaches you to read what the cluster is trying to tell you. Kubernetes provides signals about Pod failures: status fields, events, logs, and resource constraints. Learning to interpret these signals is the difference between a 5-minute fix and hours of frustration.

---

## Concept 1: Resource Requests and Limits

Before diving into debugging, you need to understand how Kubernetes allocates resources.

### The Mental Model: Requests vs Limits

Think of resource management like renting an apartment:

- **Request**: "I need at least 2 bedrooms"—the landlord won't accept your application if they have fewer. This is your GUARANTEED minimum.
- **Limit**: "My apartment can have at most 3 bedrooms"—you don't need more than this. If you try to use 4, you get evicted.

In Kubernetes:

```yaml
resources:
  requests:
    memory: "256Mi"    # Guaranteed minimum
    cpu: "100m"        # for scheduling decisions
  limits:
    memory: "512Mi"    # Maximum allowed
    cpu: "500m"        # can't exceed this
```

**Key Principle**: A Pod cannot be scheduled on a node unless that node has at least the REQUESTED amount of free resources. Limits prevent a Pod from monopolizing node resources.

### Why This Matters

Requests enable fair scheduling. If you have 3 Pods:
- Pod A requests 1 CPU
- Pod B requests 1 CPU
- Pod C requests 1 CPU

Kubernetes won't schedule all three on a 2-CPU node. Request prevents overcommitment.

Limits enable isolation. If Pod A starts consuming 2 CPUs (more than its limit), Kubernetes throttles it. Pod B doesn't starve because Pod A went rogue.

### CPU and Memory Units

**CPU**:
- `1000m` = 1 CPU core
- `100m` = 0.1 CPU cores (100 millicores)
- `0.5` = half a CPU core (also written as `500m`)

**Memory**:
- `1Mi` = 1 mebibyte (~1 million bytes, technically 1048576 bytes)
- `1Gi` = 1 gibibyte (~1 billion bytes)
- `256Mi` = 256 mebibytes (typical for small services)
- `1Gi` = 1 gibibyte (typical for memory-intensive services)

Always use `Mi` and `Gi` (binary) not `MB` and `GB` (decimal) in Kubernetes manifests. They're different.

---

## Concept 2: Quality of Service (QoS) Classes

Kubernetes prioritizes which Pods to evict when a node runs out of resources. This priority is determined by the Pod's **QoS class**.

### The Three QoS Classes

**Guaranteed** (Highest Priority)

```yaml
resources:
  requests:
    memory: "256Mi"
    cpu: "100m"
  limits:
    memory: "256Mi"
    cpu: "100m"
```

When requests equal limits, the Pod is Guaranteed. Kubernetes evicts Guaranteed Pods LAST. Use this for critical workloads (databases, control planes).

**Burstable** (Medium Priority)

```yaml
resources:
  requests:
    memory: "256Mi"
    cpu: "100m"
  limits:
    memory: "512Mi"
    cpu: "500m"
```

When requests < limits, the Pod is Burstable. Kubernetes evicts Burstable Pods second. Use this for normal workloads (most services, agents).

**BestEffort** (Lowest Priority)

```yaml
resources: {}
  # No requests or limits
```

When a Pod has no requests or limits, it's BestEffort. Kubernetes evicts these FIRST when memory pressure occurs. Only use this for batch jobs, not for Pods that need to stay running.

### The Eviction Decision

Imagine your cluster is out of memory and needs to evict a Pod:

1. Check BestEffort Pods first—evict one
2. If memory pressure continues, evict Burstable Pods
3. Only if nothing else works, evict Guaranteed Pods

This hierarchy ensures critical workloads stay running when resources are tight.

---

## Concept 3: Common Pod Failure States

Pods fail in predictable ways. Each failure state has a specific cause and fix pattern.

### CrashLoopBackOff

**What you see**:
```
NAME    READY   STATUS             RESTARTS   AGE
myapp   0/1     CrashLoopBackOff   5          2m
```

**What it means**: The container started, then crashed, then restarted, then crashed again. This cycle repeats 5+ times.

**Root causes**:
1. Application error (bug in code)
2. Missing environment variable
3. Missing configuration file
4. Port already in use
5. Out of memory

**Fix pattern**:
1. Check logs: `kubectl logs <pod-name>` (shows why it crashed)
2. Check if limit was hit: `kubectl describe pod <pod-name>` (look for OOMKilled)
3. Fix the underlying issue in your manifest or code
4. Delete and recreate the Pod

### ImagePullBackOff

**What you see**:
```
NAME    READY   STATUS             RESTARTS   AGE
myapp   0/1     ImagePullBackOff   0          1m
```

**What it means**: Kubernetes tried to pull the container image but failed.

**Root causes**:
1. Image doesn't exist in registry
2. Wrong image name or tag
3. Registry credentials missing (private images)
4. Network unreachable (can't reach registry)

**Fix pattern**:
1. Check the event: `kubectl describe pod <pod-name>` (look for "Failed to pull image")
2. Verify image name: `kubectl get pod -o yaml <pod-name>` (find exact image reference)
3. Test locally: `docker pull <image-name>` (can you pull it on your laptop?)
4. Fix the image reference in your manifest
5. Apply the manifest again

### Pending

**What you see**:
```
NAME    READY   STATUS    RESTARTS   AGE
myapp   0/1     Pending   0          5m
```

**What it means**: Kubernetes is trying to schedule the Pod, but no node has enough resources.

**Root causes**:
1. Requested resources exceed cluster capacity
2. Node affinity requirements not met
3. Pod is waiting for PersistentVolume
4. Node taint prevents Pod scheduling

**Fix pattern**:
1. Check events: `kubectl describe pod <pod-name>` (shows why scheduling failed)
2. Check node resources: `kubectl top nodes` (are nodes overcommitted?)
3. Reduce Pod requests if they're too high
4. Add more nodes to the cluster
5. Check tolerations and affinity rules

### OOMKilled

**What you see** (in describe output):

```
State:       Waiting
  Reason:    CrashLoopBackOff
LastState:   Terminated
  Reason:    OOMKilled
  Exit Code: 137
```

**What it means**: The container consumed more memory than its limit, so Kubernetes forcefully terminated it.

**Root causes**:
1. Application has a memory leak
2. Limit is too low for the workload
3. Processing unexpectedly large dataset

**Fix pattern**:
1. Increase memory limit (if limit is genuinely too low)
2. Profile the application (use tools like `pprof` for Python/Go)
3. Fix the memory leak in code
4. Change the application to process data in chunks instead of all at once

---

## Concept 4: The Debugging Pattern

Kubernetes provides four signals for debugging. Learn to read them in order.

### Signal 1: Pod Status

```bash
kubectl get pods
```

**Output**:
```
NAME              READY   STATUS             RESTARTS   AGE
nginx-good        1/1     Running            0          5m
nginx-crash       0/1     CrashLoopBackOff   3          2m
nginx-pending     0/1     Pending            0          1m
```

Status tells you WHAT is wrong (CrashLoopBackOff, Pending, etc.). But not WHY. Continue to Signal 2.

### Signal 2: Events

```bash
kubectl describe pod <pod-name>
```

**Output** (partial):
```
Name:         nginx-crash
Namespace:    default
...
Events:
  Type    Reason     Age    Message
  ----    ------     ---    -------
  Normal  Created    2m20s  Created container nginx
  Normal  Started    2m19s  Started container nginx
  Warning BackOff    2m10s  Back-off restarting failed container
```

Events show WHEN things happened and provide clues (like "restarting failed container"). For Pending Pods, events reveal scheduling reasons.

### Signal 3: Logs

```bash
kubectl logs <pod-name>
```

**Output** (if app crashed):
```
Traceback (most recent call last):
  File "app.py", line 5, in <module>
    connect_to_db()
  File "app.py", line 2, in connect_to_db
    raise Exception("Database not found")
Exception: Database not found
```

Logs show the APPLICATION'S error message. This is where you find the root cause (missing env var, code bug, etc.).

For Pending Pods, logs are empty (Pod never started). Check events instead.

### Signal 4: Interactive Access

```bash
kubectl exec -it <pod-name> -- /bin/bash
```

When the above three signals aren't enough, jump into the running Pod and investigate directly.

```bash
# Inside the Pod
$ env          # Check environment variables
$ ls -la       # Check filesystem
$ ps aux       # Check running processes
$ curl localhost:8080  # Test internal services
```

This is your "last resort" debugging tool. Use when you need to poke around interactively.

---

## Putting It Together: The Debugging Workflow

When a Pod fails:

1. **Get status**: `kubectl get pods` (What's the state?)
2. **Describe**: `kubectl describe pod <name>` (Why is it in that state? Are there events?)
3. **Check logs**: `kubectl logs <name>` (What did the application say?)
4. **Investigate interactively**: `kubectl exec -it <name> -- /bin/bash` (What's actually happening inside?)
5. **Fix**: Modify manifest or code based on findings
6. **Apply**: `kubectl apply -f manifest.yaml`
7. **Verify**: `kubectl get pods` (Did it work?)

This pattern works for 95% of Kubernetes debugging.

---

## Practice 1: Diagnose CrashLoopBackOff

Create a Pod that crashes due to a missing environment variable.

**Manifest** (save as `crash-loop.yaml`):

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: crash-loop-app
spec:
  containers:
  - name: app
    image: python:3.11-slim
    command: ["python", "-c"]
    args:
      - |
        import os
        db_url = os.environ['DATABASE_URL']
        print(f"Connecting to {db_url}")
    resources:
      requests:
        memory: "64Mi"
        cpu: "50m"
      limits:
        memory: "128Mi"
        cpu: "100m"
  restartPolicy: Always
```

**Deploy it**:

```bash
kubectl apply -f crash-loop.yaml
```

**Output**:
```
pod/crash-loop-app created
```

**Check status** (after 30 seconds):

```bash
kubectl get pods
```

**Output**:
```
NAME              READY   STATUS             RESTARTS   AGE
crash-loop-app    0/1     CrashLoopBackOff   2          35s
```

**Describe to see events**:

```bash
kubectl describe pod crash-loop-app
```

**Output** (relevant section):
```
Events:
  Type     Reason            Age   Message
  ----     ------            ---   -------
  Normal   Scheduled         45s   Successfully assigned default/crash-loop-app
  Normal   Created           44s   Created container app
  Normal   Started           43s   Started container app
  Warning  BackOff           20s   Back-off restarting failed container
```

**Check logs to see the actual error**:

```bash
kubectl logs crash-loop-app
```

**Output**:
```
Traceback (most recent call last):
  File "<string>", line 2, in <module>
db_url = os.environ['DATABASE_URL']
KeyError: 'DATABASE_URL'
```

**Diagnosis**: The application expects `DATABASE_URL` environment variable but it's not set.

**Fix**: Add the environment variable to the manifest:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: crash-loop-app
spec:
  containers:
  - name: app
    image: python:3.11-slim
    command: ["python", "-c"]
    args:
      - |
        import os
        db_url = os.environ['DATABASE_URL']
        print(f"Connecting to {db_url}")
    env:
    - name: DATABASE_URL
      value: "postgres://localhost:5432/mydb"
    resources:
      requests:
        memory: "64Mi"
        cpu: "50m"
      limits:
        memory: "128Mi"
        cpu: "100m"
  restartPolicy: Always
```

**Apply the fix**:

```bash
kubectl apply -f crash-loop.yaml
```

**Output**:
```
pod/crash-loop-app configured
```

**Check status**:

```bash
kubectl get pods
```

**Output**:
```
NAME              READY   STATUS    RESTARTS   AGE
crash-loop-app    1/1     Running   0          5s
```

**Check logs to confirm it's working**:

```bash
kubectl logs crash-loop-app
```

**Output**:
```
Connecting to postgres://localhost:5432/mydb
```

Notice the Pod transitions from CrashLoopBackOff to Running. The restart counter resets because the underlying issue is fixed.

**Clean up**:

```bash
kubectl delete pod crash-loop-app
```

---

## Practice 2: Diagnose Pending Pod Due to Insufficient Resources

Create a Pod that requests more resources than available.

**Manifest** (save as `pending-pod.yaml`):

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: memory-hog
spec:
  containers:
  - name: app
    image: python:3.11-slim
    command: ["sleep", "3600"]
    resources:
      requests:
        memory: "100Gi"    # Way more than any node has
        cpu: "50"          # Way more cores than available
      limits:
        memory: "100Gi"
        cpu: "50"
  restartPolicy: Always
```

**Deploy it**:

```bash
kubectl apply -f pending-pod.yaml
```

**Output**:
```
pod/memory-hog created
```

**Check status**:

```bash
kubectl get pods
```

**Output**:
```
NAME         READY   STATUS    RESTARTS   AGE
memory-hog   0/1     Pending   0          10s
```

**Describe to see why it's Pending**:

```bash
kubectl describe pod memory-hog
```

**Output** (relevant section):
```
Events:
  Type     Reason            Age   Message
  ----     ------            ---   -------
  Warning  FailedScheduling  15s   0/1 nodes are available: 1 Insufficient memory (requires 100Gi, but nodes only have ~5Gi free).
```

**Diagnosis**: The Pod requests 100Gi of memory, but no node has that much available. Kubernetes cannot schedule it.

**Fix**: Reduce the resource requests to reasonable values:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: memory-hog
spec:
  containers:
  - name: app
    image: python:3.11-slim
    command: ["sleep", "3600"]
    resources:
      requests:
        memory: "256Mi"    # Reasonable request
        cpu: "100m"
      limits:
        memory: "512Mi"
        cpu: "500m"
  restartPolicy: Always
```

**Apply the fix**:

```bash
kubectl apply -f pending-pod.yaml
```

**Output**:
```
pod/memory-hog configured
```

**Check status**:

```bash
kubectl get pods
```

**Output**:
```
NAME         READY   STATUS    RESTARTS   AGE
memory-hog   1/1     Running   0          5s
```

Pod transitions from Pending to Running.

**Clean up**:

```bash
kubectl delete pod memory-hog
```

---

## Practice 3: Diagnose OOMKilled and Adjust Limits

Create a Pod that exceeds its memory limit.

**Manifest** (save as `oom-pod.yaml`):

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: memory-leak-app
spec:
  containers:
  - name: app
    image: python:3.11-slim
    command: ["python", "-c"]
    args:
      - |
        import time
        memory = []
        while True:
            # Allocate 50MB every 100ms
            memory.append(bytearray(50 * 1024 * 1024))
            time.sleep(0.1)
    resources:
      requests:
        memory: "256Mi"
        cpu: "100m"
      limits:
        memory: "256Mi"    # Limit memory to 256Mi
        cpu: "500m"
  restartPolicy: Always
```

**Deploy it**:

```bash
kubectl apply -f oom-pod.yaml
```

**Output**:
```
pod/memory-leak-app created
```

**Check status immediately**:

```bash
kubectl get pods
```

**Output** (within a few seconds):
```
NAME               READY   STATUS             RESTARTS   AGE
memory-leak-app    0/1     CrashLoopBackOff   1          5s
```

**Describe to see the termination reason**:

```bash
kubectl describe pod memory-leak-app
```

**Output** (relevant section):
```
State:          Waiting
  Reason:       CrashLoopBackOff
Last State:     Terminated
  Reason:       OOMKilled
  Exit Code:    137
  Message:      The container was killed due to an out-of-memory condition.
```

**Diagnosis**: The application consumes memory faster than the 256Mi limit allows. Kubernetes kills it with OOMKilled.

**Fix Options**:

1. **Increase the limit** (if the application actually needs more memory):

```yaml
resources:
  requests:
    memory: "512Mi"
    cpu: "100m"
  limits:
    memory: "1Gi"      # Increase limit
    cpu: "500m"
```

2. **Fix the memory leak** (if there's a bug):

```yaml
args:
  - |
    import time
    memory = []
    while True:
        # Only keep last 5 allocations (500MB max)
        if len(memory) > 5:
            memory.pop(0)
        memory.append(bytearray(50 * 1024 * 1024))
        time.sleep(0.1)
```

For this example, let's increase the limit:

```bash
kubectl apply -f oom-pod.yaml
```

**Check status**:

```bash
kubectl get pods
```

**Output**:
```
NAME               READY   STATUS    RESTARTS   AGE
memory-leak-app    1/1     Running   0          5s
```

If the Pod stays running (not crashing), the limit was the issue. If it still crashes with the higher limit, there's a memory leak in the code that needs fixing.

**Clean up**:

```bash
kubectl delete pod memory-leak-app
```

---

## Resource Management Best Practices

**1. Always set requests and limits for production Pods**

```yaml
resources:
  requests:
    memory: "256Mi"
    cpu: "100m"
  limits:
    memory: "512Mi"
    cpu: "500m"
```

This ensures fair scheduling and prevents Pod eviction.

**2. Make requests equal to limits for critical workloads** (Guaranteed QoS)

```yaml
resources:
  requests:
    memory: "512Mi"
    cpu: "250m"
  limits:
    memory: "512Mi"
    cpu: "250m"
```

This prevents your Pod from being evicted when other Pods fail.

**3. Start conservative and increase based on monitoring**

Set requests low (`100m` CPU, `128Mi` memory) for new services, then monitor actual usage:

```bash
kubectl top pods    # Shows actual CPU/memory usage
```

Increase requests based on observed usage + 20% headroom.

**4. Use namespaces to isolate resource quotas**

```yaml
apiVersion: v1
kind: Namespace
metadata:
  name: ai-services
---
apiVersion: v1
kind: ResourceQuota
metadata:
  name: ai-quota
  namespace: ai-services
spec:
  hard:
    requests.memory: "10Gi"    # All Pods in namespace combined
    requests.cpu: "5"
```

---

## Try With AI

You now have the mental models and debugging workflow. Let's collaborate with AI to troubleshoot a complex scenario.

**Setup**: Deploy a multi-container Pod with intentional resource and configuration issues. Use kubectl commands to inspect it, then iterate with AI to fix problems.

**Your Assignment**:

Create this manifest (save as `complex-pod.yaml`):

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: multi-container-app
spec:
  containers:
  - name: web
    image: nginx:1.25
    ports:
    - containerPort: 8080
    env:
    - name: ENVIRONMENT
      value: "production"
    resources:
      requests:
        memory: "64Mi"
        cpu: "50m"
      limits:
        memory: "128Mi"
        cpu: "100m"
  - name: sidecar
    image: curlimages/curl:latest
    command: ["sleep", "3600"]
    resources:
      requests:
        memory: "32Mi"
        cpu: "25m"
      limits:
        memory: "64Mi"
        cpu: "50m"
  restartPolicy: Always
```

**Step 1: Deploy and diagnose**

```bash
kubectl apply -f complex-pod.yaml
kubectl get pods
kubectl describe pod multi-container-app
kubectl logs multi-container-app -c web
kubectl logs multi-container-app -c sidecar
```

**Step 2: Ask AI for analysis**

Tell AI:
"I've deployed a multi-container Pod with nginx and curl sidecar. Here are the kubectl outputs: [paste describe and logs]. What QoS class is this Pod? How would you monitor resource usage? What would happen if CPU requests were set to 50 instead of 50m?"

**Step 3: Interactive exploration**

Jump into the Pod and verify:

```bash
kubectl exec -it multi-container-app -c web -- /bin/sh
$ ps aux              # Check if nginx is running
$ netstat -tlnp       # Check port bindings
$ env                 # Verify environment variables
$ exit
```

**Step 4: Propose a modification**

Based on AI's suggestions, modify the manifest to:
- Change sidecar image to a production-ready one
- Add a liveness probe to the web container
- Adjust resource requests based on typical nginx usage

Ask AI: "Given that nginx typically uses 20-50m CPU and 50-100Mi memory in production, what requests and limits would you recommend? Should this be Guaranteed or Burstable?"

**Step 5: Validate and explain**

Apply your modified manifest:

```bash
kubectl apply -f complex-pod.yaml
kubectl get pods
kubectl top pod multi-container-app     # View actual resource usage
```

Explain to AI:
- Why your resource choices match the QoS class you selected
- What signals you'd monitor to detect problems before they become critical
- How you'd adjust resources if you observed Pod eviction in a high-load scenario

**Clean up**:

```bash
kubectl delete pod multi-container-app
```

