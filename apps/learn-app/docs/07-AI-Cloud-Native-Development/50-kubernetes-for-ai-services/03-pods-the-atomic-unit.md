---
sidebar_position: 3
chapter: 50
lesson: 3
duration_minutes: 40
title: "Pods: The Atomic Unit"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Hands-on Pod creation builds core Kubernetes mental model"
cognitive_load:
  concepts_count: 9
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Define what a Pod is and why it exists beyond containers"
    bloom_level: "Understand"
  - id: LO2
    description: "Write production-grade Pod manifests in YAML"
    bloom_level: "Apply"
  - id: LO3
    description: "Deploy Pods using kubectl apply"
    bloom_level: "Apply"
  - id: LO4
    description: "Inspect Pod status, events, and logs"
    bloom_level: "Apply"
  - id: LO5
    description: "Explain Pod lifecycle states and transitions"
    bloom_level: "Understand"
  - id: LO6
    description: "Understand Pod networking and ephemeral IP assignment"
    bloom_level: "Understand"
---

# Pods: The Atomic Unit

Your Minikube cluster is running. Now let's deploy something to it.

In Docker, you ran `docker run my-agent:v1` to start a container. In Kubernetes, you don't run containers directly—you create **Pods**. A Pod wraps one or more containers and adds the production features Kubernetes needs: shared networking, health management, resource guarantees, and co-location for tightly coupled processes.

This lesson teaches you to write Pod manifests by hand, deploy them with `kubectl apply`, and inspect them with `kubectl describe` and `kubectl logs`. By the end, you'll have deployed your first workload to Kubernetes and understand why Pods (not containers) are the atomic unit of deployment.

---

## What Is a Pod? (Beyond the Container)

### Docker vs Kubernetes Thinking

When you worked with Docker, you thought in containers:

```
┌─────────────────┐
│   Container A   │
│  (your app)     │
└─────────────────┘
```

When you work with Kubernetes, you think in Pods:

```
┌─────────────────────────────┐
│        Pod                  │
│  ┌─────────────────────┐   │
│  │   Container A       │   │
│  │  (your app)         │   │
│  └─────────────────────┘   │
│  - Shared network ns        │
│  - Shared volumes           │
│  - Lifecycle mgmt           │
│  - Ephemeral IP             │
└─────────────────────────────┘
```

### A Closer Analogy

Think of a Pod like an **apartment**:
- The apartment (Pod) is the unit Kubernetes deploys
- Containers are like roommates inside the apartment
- Roommates share the kitchen (network namespace)
- Roommates share the bathroom (storage volumes)
- The apartment has an address (IP address)
- When the apartment is evicted, all roommates leave together

**Key Insight**: Roommates can't coordinate at 2 AM from separate apartments. They're in the same apartment for a reason. Similarly, containers in a Pod are co-located because they need tight coordination.

### Pod Networking: Localhost Just Works

The most counterintuitive feature of Pods: **Containers in the same Pod share localhost.**

If you have two containers in one Pod:
- Container A listens on `localhost:8080`
- Container B can reach it at `localhost:8080` (not `container-a-host:8080`)

This works because containers share the Pod's network namespace. There's no bridge or service discovery needed for intra-Pod communication—they're literally on the same network interface.

---

## Concept 1: Pod Manifests in YAML

Kubernetes uses declarative YAML manifests instead of imperative Docker commands. Instead of:

```bash
docker run nginx:alpine -p 8080:80
```

You write a manifest describing what you want:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: nginx-pod
  labels:
    app: nginx
    tier: frontend
spec:
  containers:
  - name: nginx
    image: nginx:alpine
    ports:
    - containerPort: 80
    resources:
      limits:
        memory: "128Mi"
        cpu: "500m"
      requests:
        memory: "64Mi"
        cpu: "250m"
```

### Field Breakdown

**apiVersion & kind**: Kubernetes API version and resource type
- `v1` = stable core API
- `Pod` = we're creating a Pod

**metadata**: Identification and labeling
- `name`: Must be unique in the namespace
- `labels`: Arbitrary key-value pairs for organization (not unique)
  - Use labels to tag Pods (dev/prod, frontend/backend, etc.)

**spec.containers**: What actually runs
- `name`: Container identifier within the Pod
- `image`: Docker image (from Docker Hub or private registry)
- `ports`: Which container ports Kubernetes should expose
  - Note: This is declarative—it documents intention but doesn't bind ports to the host

**resources**: CPU and memory guarantees
- `limits`: Maximum resources container can use
  - If container exceeds limit, Kubernetes kills it
- `requests`: Guaranteed minimum resources
  - Used for scheduling decisions (place Pod on nodes with available resources)

### Why YAML, Not Docker Commands?

YAML enables:
1. **Version control**: Check manifests into Git, track changes
2. **Reproducibility**: Same manifest → same Pod every time
3. **Infrastructure as code**: Automation and policy checking
4. **GitOps**: Push code, Git webhook triggers deployment

This is the shift from imperative (Docker: "run this") to declarative (Kubernetes: "this should exist").

---

## Concept 2: Deploying a Pod with kubectl

Create a file `nginx-pod.yaml` with the manifest above, then:

```bash
kubectl apply -f nginx-pod.yaml
```

**Output:**
```
pod/nginx-pod created
```

This tells Kubernetes: "Make sure this Pod exists. If it doesn't, create it. If it does but the spec changed, update it."

### Verifying Deployment

Check if the Pod was created:

```bash
kubectl get pods
```

**Output:**
```
NAME        READY   STATUS    RESTARTS   AGE
nginx-pod   1/1     Running   0          12s
```

Columns explained:
- **NAME**: Pod name from metadata
- **READY**: `1/1` means 1 container requested, 1 container running
- **STATUS**: Current state (Pending, Running, Succeeded, Failed, etc.)
- **RESTARTS**: How many times container crashed and restarted
- **AGE**: How long this Pod has been alive

---

## Concept 3: Pod Lifecycle States

When you create a Pod, it doesn't run instantly. Kubernetes goes through several states:

```
Pending → Running → (Succeeded or Failed)
```

### State Breakdown

**Pending**: Pod created but not yet running
- Kubernetes is scheduling (finding a node)
- Container image is being pulled
- Pod waiting for volumes to attach
- Duration: Usually <30 seconds, but can be longer if image is large or cluster is full

**Running**: At least one container is running
- Pod is on a node
- Health checks starting
- Your application is active

**Succeeded**: All containers completed successfully
- Only for batch Jobs (not long-running services)
- Pod stops running

**Failed**: At least one container failed
- Container exited with non-zero status
- Pod won't restart (unless you configure RestartPolicy)

### Viewing State Transitions

Watch a Pod's journey from creation to running:

```bash
kubectl get pods -w
```

The `-w` flag means "watch"—stream updates as they happen.

---

## Concept 4: Inspecting Pods in Detail

### Getting More Information: kubectl describe

```bash
kubectl describe pod nginx-pod
```

**Output** (abbreviated):
```
Name:         nginx-pod
Namespace:    default
Priority:     0
Node:         worker-node-2
Start Time:   Mon, 22 Dec 2025 14:30:15 +0000
Status:       Running
IP:           10.244.1.52
IPs:
  IP:  10.244.1.52

Containers:
  nginx:
    Container ID:   containerd://abc123...
    Image:          nginx:alpine
    Image ID:       docker.io/library/nginx@sha256:...
    Port:           80/TCP
    State:          Running
      Started:      Mon, 22 Dec 2025 14:30:20 +0000
    Ready:          True
    Restart Count:  0
    Limits:
      cpu:      500m
      memory:   128Mi
    Requests:
      cpu:      250m
      memory:   64Mi

Events:
  Type    Reason     Age    From               Message
  ----    ------     ----   ----               -------
  Normal  Scheduled  1m     default-scheduler  Successfully assigned default/nginx-pod to worker-node-2
  Normal  Pulling    1m     kubelet            Pulling image "nginx:alpine"
  Normal  Pulled     45s    kubelet            Successfully pulled image "nginx:alpine"
  Normal  Created    45s    kubelet            Created container nginx
  Normal  Started    45s    kubelet            Started container nginx
```

**Key information**:
- **IP**: Unique IP within cluster (ephemeral—changes if Pod restarts)
- **Node**: Which worker node the Pod landed on
- **Containers**: Detailed status of each container
- **Events**: Timeline of what happened (scheduling, image pull, start)

### Reading Logs

See what your application is outputting:

```bash
kubectl logs nginx-pod
```

**Output** (nginx startup logs):
```
/docker-entrypoint.sh: /docker-entrypoint.d/ is not empty, will attempt to apply all files in /docker-entrypoint.d/:
/docker-entrypoint.sh: Looking for shell scripts in /docker-entrypoint.d/:
/docker-entrypoint.sh: Launching /docker-entrypoint.d/10-listen-on-ipv6-by-default.sh
10-listen-on-ipv6-by-default.sh: info: Getting the checksum of /etc/nginx/conf.d/default.conf
10-listen-on-ipv6-by-default.sh: info: Enabled ipv6 for in /etc/nginx/conf.d/default.conf
/docker-entrypoint.sh: Launching /docker-entrypoint.d/20-envsubst-on-templates.sh
/docker-entrypoint.sh: Launching /docker-entrypoint.d/30-tune-worker-processes.sh
2025-12-22T14:30:20.531919Z 0 notice: signal process started
```

For multi-container Pods, view logs from a specific container:

```bash
kubectl logs nginx-pod -c nginx
```

---

## Concept 5: Pod Networking Deep Dive

### Each Pod Gets a Unique IP

When Kubernetes creates your Pod, it assigns an IP address from the cluster network (usually 10.0.0.0/8):

```
Pod A: 10.244.1.52
Pod B: 10.244.2.104
Pod C: 10.244.1.53
```

These IPs are **ephemeral**—they change when the Pod restarts. This is critical:

**❌ WRONG**: Store Pod IPs in configuration files
**✅ RIGHT**: Use Kubernetes Services (next lesson) for stable networking

### How Containers in Same Pod Communicate

Create a Pod with two containers (web app and log shipper):

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: multi-container-pod
spec:
  containers:
  - name: web
    image: nginx:alpine
    ports:
    - containerPort: 8080

  - name: log-shipper
    image: filebeat:latest
    # log-shipper can reach web at localhost:8080
```

The log-shipper can reach the web container at `localhost:8080` because they share the Pod's network namespace.

### Network Namespaces Visualized

```
┌──────────────────────────────────────────────┐
│  Pod "multi-container-pod"                   │
│                                              │
│  Network namespace: 10.244.1.52              │
│  ├─ eth0: 10.244.1.52                        │
│  ├─ loopback: 127.0.0.1                      │
│                                              │
│  ┌──────────────┐  ┌──────────────┐         │
│  │ Container    │  │ Container    │         │
│  │ (web)        │  │ (log-ship)   │         │
│  │              │  │              │         │
│  │ Listen: 8080 │  │ Connect:     │         │
│  │              │  │ localhost:80 │         │
│  └──────────────┘  └──────────────┘         │
│                                              │
└──────────────────────────────────────────────┘
```

Both containers see:
- Same hostname
- Same localhost
- Same network interface (eth0)
- Access to shared volumes

---

## Concept 6: Single vs Multi-Container Pods

### Single-Container Pods (Most Common)

Most Pods contain one container:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: api-pod
spec:
  containers:
  - name: api
    image: myapp:1.0
```

This is the common case. One container = one concern.

### Multi-Container Pods (Sidecar Pattern)

Two containers in one Pod when they need tight coupling:

**Use case 1: Log Shipper Sidecar**

Main container writes logs to stdout. Sidecar ships logs somewhere:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: app-with-logging
spec:
  containers:
  - name: app
    image: myapp:1.0
    volumeMounts:
    - name: logs
      mountPath: /var/log

  - name: log-shipper
    image: filebeat:latest
    volumeMounts:
    - name: logs
      mountPath: /var/log

  volumes:
  - name: logs
    emptyDir: {}  # Temporary volume shared between containers
```

Both containers share the `logs` volume. App writes to `/var/log`, log-shipper reads from `/var/log`.

**Use case 2: Init Container (Setup)**

An init container runs before main containers:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: app-with-init
spec:
  initContainers:
  - name: setup
    image: setup-script:1.0
    # Runs first, sets up environment

  containers:
  - name: app
    image: myapp:1.0
    # Runs after init container completes
```

Init containers always complete before app containers start.

### When NOT to Use Multi-Container Pods

Don't force unrelated containers into one Pod:

**❌ WRONG**:
```yaml
containers:
  - name: web-api
    image: api:1.0
  - name: database
    image: postgres:15
    # These don't belong together!
```

Each should be its own Pod. Multi-container is for **tightly coupled** responsibilities (logging, monitoring, security).

---

## Concept 7: Pod Lifecycle: Creation to Termination

### Full Pod Lifecycle

```
1. kubectl apply -f pod.yaml
   ↓
2. Kubernetes creates Pod object (status: Pending)
   ↓
3. Scheduler assigns Pod to a Node (status: Pending)
   ↓
4. Kubelet on Node pulls image (status: Pending)
   ↓
5. Kubelet starts container (status: Running)
   ↓
6. App is ready to serve traffic
   ↓
7. Pod continues running until:
   - Container exits (Failed state)
   - kubectl delete pod (terminated)
   - Node shuts down (evicted)
   ↓
8. Pod is gone, IP is reclaimed
```

### RestartPolicy: What Happens When Container Crashes

When a container exits, Kubernetes decides what to do based on RestartPolicy:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: app-with-restart-policy
spec:
  restartPolicy: Always  # Always restart on exit
  containers:
  - name: app
    image: myapp:1.0
```

**RestartPolicy options**:
- `Always`: Restart container if it exits (default)
  - Useful for long-running services
- `OnFailure`: Restart only if exit code was non-zero
  - Useful for batch jobs that might fail temporarily
- `Never`: Don't restart
  - Useful for one-shot jobs

---

## Concept 8: Pods Are Mortal (Ephemeral Design)

**Critical insight**: Pods are NOT pets. They're cattle.

When a Pod terminates:
- IP address is gone
- Data in the Pod is lost (unless stored on a persistent volume)
- Kubernetes does NOT automatically restart it (unless it's managed by a higher-level controller like Deployment—covered in next lesson)

```bash
kubectl delete pod nginx-pod
```

**Output:**
```
pod "nginx-pod" deleted
```

The Pod is **gone**. It doesn't come back unless:
1. You manually create it again with `kubectl apply`
2. It's managed by a Deployment/StatefulSet/Job that respawns it automatically

### Implications for Data

**❌ WRONG**:
```yaml
containers:
- name: app
  image: myapp:1.0
  # Storing important data here? LOST when Pod dies!
```

**✅ RIGHT**:
```yaml
containers:
- name: app
  image: myapp:1.0
  volumeMounts:
  - name: data
    mountPath: /data

volumes:
- name: data
  persistentVolumeClaim:
    claimName: my-data-pvc  # Persists beyond Pod lifecycle
```

---

## Concept 9: Resource Requests vs Limits

You saw this in the manifest earlier. It's critical for production:

```yaml
resources:
  requests:
    memory: "64Mi"
    cpu: "250m"
  limits:
    memory: "128Mi"
    cpu: "500m"
```

### Requests: Guaranteed Resources

`requests` tells Kubernetes: "This Pod needs at least this much."

Kubernetes uses requests to **schedule** (place Pods on nodes with available capacity):

```
Node 1 (4 CPU, 8Gi memory):
  - Pod A requests 1 CPU, 2Gi → Scheduled here
  - Pod B requests 1 CPU, 2Gi → Scheduled here
  - Pod C requests 2 CPU, 4Gi → Not enough space, goes to Node 2
```

Without requests, Kubernetes can overcommit (overload a node).

### Limits: Hard Ceiling

`limits` tells Kubernetes: "This Pod can use at most this much."

If container exceeds limits, Kubernetes **kills it**:

```
Container uses 200Mi memory (limit: 128Mi)
  → Out of Memory (OOM)
  → Kubernetes kills container
  → If restartPolicy is Always, restarts it
  → If keeps hitting OOM, Pod enters CrashLoopBackOff
```

### Best Practice Settings

For most applications:
- `requests.memory` = typical memory usage
- `requests.cpu` = typical CPU usage
- `limits.memory` = requests × 1.5 (allow spikes)
- `limits.cpu` = requests × 2 (allow temporary spikes)

Example for a Python FastAPI app:

```yaml
resources:
  requests:
    memory: "256Mi"
    cpu: "500m"
  limits:
    memory: "512Mi"
    cpu: "1000m"
```

---

## Hands-On Exercise: Create and Manage a Pod

### Step 1: Write a Pod Manifest

Create file `hello-api.yaml`:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: hello-api
  labels:
    app: hello
    version: v1
spec:
  containers:
  - name: api
    image: python:3.11-alpine
    command: ["python", "-m", "http.server", "8000"]
    ports:
    - containerPort: 8000
    resources:
      requests:
        memory: "32Mi"
        cpu: "100m"
      limits:
        memory: "64Mi"
        cpu: "200m"
```

### Step 2: Deploy the Pod

```bash
kubectl apply -f hello-api.yaml
```

**Output:**
```
pod/hello-api created
```

### Step 3: Check Status

```bash
kubectl get pods
```

**Output:**
```
NAME        READY   STATUS    RESTARTS   AGE
hello-api   1/1     Running   0          8s
```

### Step 4: Inspect Details

```bash
kubectl describe pod hello-api
```

Look for:
- IP address assigned
- Node it's running on
- Events showing creation timeline

### Step 5: View Logs

```bash
kubectl logs hello-api
```

**Output:**
```
Serving HTTP on 0.0.0.0 port 8000 (http://0.0.0.0:8000/) ...
```

### Step 6: Simulate a Pod Restart

Stop the Pod:

```bash
kubectl delete pod hello-api
```

**Output:**
```
pod "hello-api" deleted
```

Notice: No Pod respawns. It's gone permanently.

Why? Because we created it directly. In real deployments, you'd use a **Deployment** (next lesson) that automatically respawns Pods when they fail.

---

## Common Mistakes and How to Avoid Them

### Mistake 1: Storing Important Data in Pods

**❌ WRONG**:
```yaml
containers:
- name: app
  image: myapp:1.0
  # If Pod dies, database is lost
```

**✅ RIGHT**:
Use persistent volumes or external databases (PostgreSQL in Cloud SQL, etc.)

### Mistake 2: Ignoring Resource Limits

**❌ WRONG**:
```yaml
containers:
- name: app
  image: myapp:1.0
  # No limits = possible node overload
```

**✅ RIGHT**:
Always set requests and limits based on actual application needs.

### Mistake 3: Hard-Coding Pod IPs

**❌ WRONG**:
```python
# In your app code
backend_url = "10.244.1.52:8000"  # This IP changes!
```

**✅ RIGHT**:
Use Kubernetes Services (next lesson) for stable DNS names.

### Mistake 4: Multi-Container Pods for Unrelated Services

**❌ WRONG**:
```yaml
containers:
- name: web
  image: nginx:latest
- name: postgres
  image: postgres:15
  # Completely unrelated!
```

**✅ RIGHT**:
Create separate Pods for separate services. Only use multi-container for tight coupling (sidecars).

---

## Try With AI

Now that you understand Pods manually, explore deeper questions with AI:

**Part 1: Pod Architecture**

Ask AI: "Why would I run multiple containers in the same Pod instead of creating separate Pods? Give me 3 real-world examples."

Expected: AI should explain sidecar patterns (logging, monitoring, security sidecar) and why tight network coupling matters.

**Part 2: Networking Implications**

Ask AI: "In a Pod with two containers, Container A listens on port 8080 and Container B tries to reach it—what's the address that Container B should use?"

Expected: AI should explain that localhost:8080 works because they share the network namespace.

**Part 3: Lifecycle and Persistence**

Ask AI: "I deployed a Pod that crashed. The Pod is gone. How is this different from a Docker container that exited? What solves the 'Pod keeps crashing but doesn't respawn' problem?"

Expected: AI should explain ephemeral nature of Pods, mention Deployments as the solution for automatic respawning.

**Part 4: Resource Limits**

Ask AI: "My application uses about 200Mi of memory under normal load. What values should I set for memory `requests` and `limits`? Why are they different?"

Expected: AI should explain the distinction (requests for scheduling, limits for hard ceiling) and suggest reasonable safety margins.

