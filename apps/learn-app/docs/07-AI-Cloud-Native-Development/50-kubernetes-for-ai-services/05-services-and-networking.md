---
sidebar_position: 5
chapter: 50
lesson: 5
duration_minutes: 40
title: "Services and Networking: Stable Access to Dynamic Pods"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Manual Service creation builds understanding of stable networking abstractions"
cognitive_load:
  concepts_count: 8
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Explain why Services exist (Pod IP ephemerality problem)"
    bloom_level: "Understand"
  - id: LO2
    description: "Distinguish between Service types: ClusterIP, NodePort, LoadBalancer"
    bloom_level: "Understand"
  - id: LO3
    description: "Understand label selectors as the mechanism for Services to target Pods"
    bloom_level: "Understand"
  - id: LO4
    description: "Create and expose a Deployment via ClusterIP Service"
    bloom_level: "Apply"
  - id: LO5
    description: "Convert a ClusterIP Service to NodePort for external access"
    bloom_level: "Apply"
  - id: LO6
    description: "Explain Kubernetes DNS discovery pattern (service.namespace.svc.cluster.local)"
    bloom_level: "Understand"
  - id: LO7
    description: "Debug Service connectivity issues using label selector analysis"
    bloom_level: "Apply"
  - id: LO8
    description: "Design Service strategy based on access pattern requirements (internal vs external)"
    bloom_level: "Apply"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "5. Problem Solving"
    competency: "5.2 Identifying needs and technological responses"
  - objective_id: LO2
    competency_area: "1. Information and Data Literacy"
    competency: "1.2 Understanding digital concepts and terminology"
  - objective_id: LO3
    competency_area: "1. Information and Data Literacy"
    competency: "1.2 Understanding digital concepts and terminology"
  - objective_id: LO4
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO5
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO6
    competency_area: "1. Information and Data Literacy"
    competency: "1.2 Understanding digital concepts and terminology"
  - objective_id: LO7
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
  - objective_id: LO8
    competency_area: "5. Problem Solving"
    competency: "5.3 Using digital tools to solve problems"
---

# Services and Networking: Stable Access to Dynamic Pods

Your Deployment from Lesson 4 is running Pods. But there's a problem: **Pods are ephemeral. When a Pod crashes and is replaced, it gets a new IP address. Your application clients can't track these constant changes.**

Enter Services. A Service is a stable, virtual IP address that routes to a dynamic set of Pods. It's like a phone number that rings to whoever happens to be on duty today—not tied to one person, but always available.

In this lesson, you'll understand why Services are necessary, learn the three main Service types, and master the mechanism that makes it all work: label selectors.

---

## The Networking Problem: Pods Are Ephemeral

In Docker (Chapter 49), a container had a stable network identity. You'd run a container, learn its IP, and talk to it.

Kubernetes is different. Pods are designed to be replaced:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: example-pod
spec:
  containers:
  - name: nginx
    image: nginx:alpine
```

When this Pod crashes and Kubernetes restarts it, a new Pod takes its place. But the new Pod gets a **new IP address**. The old IP becomes invalid.

### The Direct IP Problem

Imagine your frontend application tries to talk to your backend:

```python
# In a frontend Pod
response = requests.get("http://10.0.2.45:5000/data")
```

If the backend Pod gets replaced (it will), `10.0.2.45` no longer exists. The request fails.

**This is unacceptable in production.** You need a stable address that routes to the current set of healthy backend Pods, regardless of which specific Pods are running.

**Enter Services.**

---

## Services: Stable Virtual IP Addresses

A **Service** is a Kubernetes abstraction that provides:

1. **A stable virtual IP address** (ClusterIP) that doesn't change
2. **Load balancing** across Pods matching the Service's label selector
3. **Service discovery** through Kubernetes DNS

Think of a Service like a phone number. The number stays the same, but the person who answers might change. Call the same number and get connected to whoever is currently on duty.

### How Services Work

A Service uses **label selectors** to determine which Pods receive traffic. Here's the mechanism:

```yaml
apiVersion: v1
kind: Service
metadata:
  name: backend-service
spec:
  selector:
    app: backend        # <-- Label selector: "find Pods with label app=backend"
  ports:
  - port: 80           # <-- Service listens on port 80
    targetPort: 5000   # <-- Forward to port 5000 in the Pod
  type: ClusterIP      # <-- Service type (discussed below)
```

When you apply this Service:

1. Kubernetes creates a virtual IP address (e.g., `10.96.0.1`)
2. It identifies all Pods matching `app: backend`
3. It load-balances traffic to those Pods' port 5000
4. If Pods are added/removed, the Service updates automatically

Your frontend Pod now talks to:

```python
response = requests.get("http://backend-service:80/data")
```

Even when backend Pods crash and restart, the Service name and IP stay constant. Kubernetes automatically updates the endpoints.

---

## The Three Service Types

Kubernetes offers three main Service types. Choose based on your access pattern.

### 1. ClusterIP: Internal Communication (Default)

**Use when**: Pods need to talk to other Pods within the cluster.

```yaml
apiVersion: v1
kind: Service
metadata:
  name: database-service
spec:
  selector:
    app: database
  ports:
  - port: 5432
    targetPort: 5432
  type: ClusterIP    # Default if omitted
```

**Characteristics**:
- Virtual IP only accessible from within the cluster
- No external access possible
- Lightweight and efficient
- Most common type (used internally between Pods)

**Example traffic flow**:
```
Frontend Pod → (requests) → ClusterIP Service (10.96.0.5:5432)
  → (routes to) → Database Pod
```

### 2. NodePort: External Access via Host Port

**Use when**: You need external clients to access your application (development/testing).

```yaml
apiVersion: v1
kind: Service
metadata:
  name: api-service
spec:
  selector:
    app: api
  ports:
  - port: 80              # Service port (internal)
    targetPort: 8000      # Pod port
    nodePort: 30007       # Host port (must be 30000-32767)
  type: NodePort
```

**Characteristics**:
- Allocates a port on every cluster node (30000-32767 range)
- External clients access `<node-ip>:<nodePort>`
- Useful for development and testing
- Not recommended for production (traffic through nodes)

**Example traffic flow**:
```
External Client → (connects to) → Node IP:30007
  → (routes to) → Service (10.96.0.1:80)
  → (load balances to) → API Pods
```

### 3. LoadBalancer: Production External Access

**Use when**: You need production-grade external access with load balancing.

```yaml
apiVersion: v1
kind: Service
metadata:
  name: frontend-service
spec:
  selector:
    app: frontend
  ports:
  - port: 80
    targetPort: 3000
  type: LoadBalancer
```

**Characteristics**:
- Provisions a cloud load balancer (AWS ELB, Azure LB, GCP LB)
- Gives your Service a public IP address
- Load balancer distributes traffic across cluster nodes
- Recommended for production (handles high traffic, automatic scaling)

**Example traffic flow**:
```
External Client → (connects to) → Cloud LB Public IP:80
  → (routes to) → Service (10.96.0.2:80)
  → (load balances to) → Frontend Pods
```

### Decision Framework: Choosing Service Type

| Requirement | Type | Why |
|-------------|------|-----|
| Pod-to-Pod communication | ClusterIP | No external access needed, most efficient |
| Manual external testing | NodePort | Quick, no cloud LB needed |
| Production external access | LoadBalancer | Cloud LB handles scalability, reliability |
| Multi-environment consistency | ClusterIP + Ingress | (Ingress covered in later chapter) |

---

## Understanding Label Selectors

Label selectors are **the mechanism that connects Services to Pods**. They're critical to understand because misconfigured selectors cause Services to have zero endpoints.

### How Label Selectors Work

When you define a Service selector, Kubernetes continuously asks: "Which Pods match these labels?"

```yaml
# Service definition
apiVersion: v1
kind: Service
metadata:
  name: web-service
spec:
  selector:
    app: web           # <-- Kubernetes finds Pods with app: web
    version: v1        # <-- AND version: v1
  ports:
  - port: 80
    targetPort: 8080
```

This Service routes traffic to Pods labeled **both** `app: web` **and** `version: v1`.

### Example: Label Matching

**Pod 1: Matches the selector**
```yaml
apiVersion: v1
kind: Pod
metadata:
  name: web-pod-1
  labels:
    app: web        # <-- Matches
    version: v1     # <-- Matches
    env: prod       # <-- Extra label (OK)
spec:
  containers:
  - name: nginx
    image: nginx:alpine
```

**Pod 2: Does NOT match**
```yaml
apiVersion: v1
kind: Pod
metadata:
  name: api-pod-1
  labels:
    app: api        # <-- Does NOT match (wrong value)
    version: v1     # <-- Matches but doesn't satisfy both
spec:
  containers:
  - name: flask
    image: flask:latest
```

**Pod 3: Does NOT match**
```yaml
apiVersion: v1
kind: Pod
metadata:
  name: web-pod-2
  labels:
    app: web        # <-- Matches
    version: v2     # <-- Does NOT match (wrong version)
spec:
  containers:
  - name: nginx
    image: nginx:alpine
```

The Service routes to **Pod 1 only**. Pods 2 and 3 are ignored.

---

## Hands-On: Creating and Exposing Services

You'll create a Deployment, then expose it via different Service types. Start with your Minikube cluster running.

### Step 1: Create a Deployment with Clear Labels

Create a file called `web-deployment.yaml`:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: web-deployment
spec:
  replicas: 3
  selector:
    matchLabels:
      app: web          # <-- Key label
  template:
    metadata:
      labels:
        app: web        # <-- Pod gets this label
    spec:
      containers:
      - name: nginx
        image: nginx:alpine
        ports:
        - containerPort: 80
```

Apply it:

```bash
kubectl apply -f web-deployment.yaml
```

**Output:**
```
deployment.apps/web-deployment created
```

Verify Pods are running with labels displayed:

```bash
kubectl get pods --show-labels
```

**Output:**
```
NAME                              READY   STATUS    RESTARTS   AGE   LABELS
web-deployment-5d5f4c7f5b-abc12   1/1     Running   0          10s   app=web,pod-template-hash=5d5f4c7f5b
web-deployment-5d5f4c7f5b-def45   1/1     Running   0          10s   app=web,pod-template-hash=5d5f4c7f5b
web-deployment-5d5f4c7f5b-ghi78   1/1     Running   0          10s   app=web,pod-template-hash=5d5f4c7f5b
```

Notice each Pod has the label `app=web` (the key label the Service will use). Get their IP addresses:

```bash
kubectl get pods -o wide
```

**Output:**
```
NAME                              READY   STATUS    RESTARTS   AGE   IP           NODE
web-deployment-5d5f4c7f5b-abc12   1/1     Running   0          10s   10.244.0.3   minikube
web-deployment-5d5f4c7f5b-def45   1/1     Running   0          10s   10.244.0.4   minikube
web-deployment-5d5f4c7f5b-ghi78   1/1     Running   0          10s   10.244.0.5   minikube
```

Each Pod has a unique IP: `10.244.0.3`, `10.244.0.4`, `10.244.0.5`. If a Pod crashes, Kubernetes replaces it with a new Pod at a new IP. This is why Services matter—they provide stable routing regardless of which Pod IPs are currently active.

### Step 2: Expose via ClusterIP Service

Create `web-service-clusterip.yaml`:

```yaml
apiVersion: v1
kind: Service
metadata:
  name: web-service
spec:
  selector:
    app: web          # <-- Targets Pods with app: web
  ports:
  - port: 80
    targetPort: 80
  type: ClusterIP
```

Apply it:

```bash
kubectl apply -f web-service-clusterip.yaml
```

**Output:**
```
service/web-service created
```

Inspect the Service:

```bash
kubectl get services
```

**Output:**
```
NAME          TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)
web-service   ClusterIP   10.96.0.234     <none>        80/TCP
```

The Service has a stable virtual IP: `10.96.0.234`. This IP never changes, even if Pods are replaced.

### Step 3: Access the Service Internally

Launch a temporary Pod inside the cluster to test internal access:

```bash
kubectl run -it test-pod --image=alpine --rm -- sh
```

From inside the Pod, curl the Service:

```bash
apk add curl
curl http://web-service
```

**Output:**
```
<!DOCTYPE html>
<html>
<head>
    <title>Welcome to nginx!</title>
</head>
...
```

Success! The Service routed your request to one of the backend Pods. Try it again:

```bash
curl http://web-service
```

Same output. Kubernetes load-balanced across Pods transparently. You didn't need to know which specific Pod answered—the Service handled it.

Exit the test Pod:

```bash
exit
```

### Step 4: Convert to NodePort for External Access

Update `web-service-clusterip.yaml`:

```yaml
apiVersion: v1
kind: Service
metadata:
  name: web-service
spec:
  selector:
    app: web
  ports:
  - port: 80
    targetPort: 80
    nodePort: 30007      # <-- Add this (external port)
  type: NodePort         # <-- Change this
```

Apply the update:

```bash
kubectl apply -f web-service-clusterip.yaml
```

**Output:**
```
service/web-service configured
```

Verify the change:

```bash
kubectl get services
```

**Output:**
```
NAME          TYPE       CLUSTER-IP      EXTERNAL-IP   PORT(S)
web-service   NodePort   10.96.0.234     <none>        80:30007/TCP
```

The Service now listens on node port 30007. Get your Minikube IP:

```bash
minikube ip
```

**Output:**
```
192.168.49.2
```

Access the Service from your laptop:

```bash
curl http://192.168.49.2:30007
```

**Output:**
```
<!DOCTYPE html>
<html>
<head>
    <title>Welcome to nginx!</title>
</head>
...
```

Success! External traffic now reaches your Deployment through the NodePort Service.

---

## Kubernetes DNS: Service Discovery

Your Pods don't need to know the Service IP address. Kubernetes provides **automatic DNS discovery**.

Every Service gets a DNS name in the format:

```
<service-name>.<namespace>.svc.cluster.local
```

For the `web-service` in the `default` namespace:

```
web-service.default.svc.cluster.local
```

Within the same namespace, you can shorten it to:

```
web-service
```

### DNS in Action

Launch a test Pod and resolve the DNS name:

```bash
kubectl run -it test-pod --image=alpine --rm -- sh
```

From inside the Pod, install curl and test DNS resolution:

```bash
apk add curl dnsutils
nslookup web-service
```

**Output:**
```
Name:      web-service
Address 1: 10.96.0.234
```

Kubernetes DNS returned the Service's virtual IP. Your application can connect without hardcoding IPs. Test it:

```bash
curl -I http://web-service
```

**Output:**
```
HTTP/1.1 200 OK
Server: nginx/1.26.2
Date: Fri, 22 Dec 2023 10:15:32 GMT
Content-Type: text/html
Content-Length: 612
Connection: keep-alive
```

The request succeeded! Kubernetes DNS resolved `web-service` to the virtual IP, and the Service routed to a backend Pod.

You can also use the full DNS name from different namespaces:

```bash
curl -I http://web-service.default.svc.cluster.local
```

**Output:**
```
HTTP/1.1 200 OK
Server: nginx/1.26.2
Date: Fri, 22 Dec 2023 10:15:35 GMT
Content-Type: text/html
Content-Length: 612
Connection: keep-alive
```

Both work. Kubernetes DNS resolves either form to the virtual IP. Exit the test Pod:

```bash
exit
```

---

## Debugging: Service Selector Mismatch

The most common Service problem: **a Service has no endpoints**. This happens when the label selector doesn't match any Pods.

### Scenario: Misconfigured Selector

Create a Deployment with `app: backend`:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: backend-deployment
spec:
  replicas: 2
  selector:
    matchLabels:
      app: backend
  template:
    metadata:
      labels:
        app: backend
    spec:
      containers:
      - name: flask
        image: flask:latest
        ports:
        - containerPort: 5000
```

Apply it:

```bash
kubectl apply -f backend-deployment.yaml
```

**Output:**
```
deployment.apps/backend-deployment created
```

Now create a Service with the **wrong** selector:

```yaml
apiVersion: v1
kind: Service
metadata:
  name: backend-service
spec:
  selector:
    app: api          # <-- WRONG: Looking for app: api, but Pods have app: backend
  ports:
  - port: 80
    targetPort: 5000
  type: ClusterIP
```

Apply it:

```bash
kubectl apply -f backend-service-wrong.yaml
```

**Output:**
```
service/backend-service created
```

Check the Service endpoints:

```bash
kubectl get endpoints backend-service
```

**Output:**
```
NAME               ENDPOINTS   AGE
backend-service    <none>      10s
```

**No endpoints!** The Service exists but routes to nothing. Any traffic sent to this Service fails because there's no Pod to handle it.

### Fixing the Selector

Update the Service with the correct selector:

```yaml
apiVersion: v1
kind: Service
metadata:
  name: backend-service
spec:
  selector:
    app: backend      # <-- FIXED: Now matches the Pod labels
  ports:
  - port: 80
    targetPort: 5000
  type: ClusterIP
```

Apply the fix:

```bash
kubectl apply -f backend-service-wrong.yaml
```

**Output:**
```
service/backend-service configured
```

Check endpoints again:

```bash
kubectl get endpoints backend-service
```

**Output:**
```
NAME               ENDPOINTS                              AGE
backend-service    10.244.0.6:5000,10.244.0.7:5000       5s
```

Now the Service has 2 endpoints (the 2 backend Pods). Traffic will route correctly.

### Debugging Process

When a Service has no endpoints, follow this systematic check:

1. **List the Pods and their labels**:

```bash
kubectl get pods --show-labels
```

**Output:**
```
NAME                              READY   LABELS
backend-deployment-abcd1234-xyz   1/1     app=backend,pod-template-hash=abcd1234
backend-deployment-abcd1234-uvw   1/1     app=backend,pod-template-hash=abcd1234
```

2. **Check the Service selector**:

```bash
kubectl describe service backend-service
```

**Output:**
```
Name:              backend-service
Namespace:         default
Labels:            <none>
Annotations:       <none>
Selector:          app: api              # <-- Wrong! Should be app: backend
Type:              ClusterIP
IP:                10.96.0.1
Port:              <unset>  80/TCP
TargetPort:        5000/TCP
Endpoints:         <none>
Session Affinity:  None
```

3. **Compare**: The Pods have `app=backend` but the Service selector is `app=api`. That's the mismatch. Fix the Service selector to match the Pod labels.

4. **Verify the fix**:

```bash
kubectl describe service backend-service
```

**Output:**
```
Name:              backend-service
Namespace:         default
Selector:          app: backend          # <-- Fixed!
Type:              ClusterIP
IP:                10.96.0.1
Port:              <unset>  80/TCP
TargetPort:        5000/TCP
Endpoints:         10.244.0.6:5000,10.244.0.7:5000   # <-- Endpoints now populated!
Session Affinity:  None
```

Now the Service has endpoints. Traffic will route correctly to the backend Pods.

---

## Service Networking Summary

| Concept | Purpose | Example |
|---------|---------|---------|
| **Service** | Stable virtual IP routing to dynamic Pods | `web-service` (IP: 10.96.0.234) |
| **Label Selector** | Mechanism to match Pods | `app: web`, `version: v1` |
| **ClusterIP** | Internal Pod-to-Pod communication | Backend database accessed by frontend |
| **NodePort** | External access via host port | Testing tool accessing API on port 30007 |
| **LoadBalancer** | Production external access via cloud LB | Public users accessing frontend |
| **DNS** | Service discovery by name | `web-service.default.svc.cluster.local` |

---

## Try With AI

**Challenge**: You have a FastAPI agent running in Kubernetes (from Chapter 49 containerization). Expose it so:
- Other Pods in the cluster can call its API
- External clients can access it for testing

You'll describe the requirements, and you and AI will iterate on the Service configuration.

**Setup**:
- Your agent Deployment is named `agent-deployment` with label `app: agent`
- It listens on port 8000
- You'll deploy to your Minikube cluster

**Part 1: Internal Access Requirement**

Tell AI: "I need a Service that lets other Pods access my agent at port 8000. The agent Deployment has label `app: agent`."

Record AI's Service specification for internal access.

**Part 2: Evaluate Internal Design**

Review AI's response:
- Does the Service selector match the Deployment labels?
- Is the port mapping correct (80 → 8000)?
- Is the type appropriate for internal access?

Ask yourself: What would you change to make this better?

**Part 3: External Access Requirement**

Tell AI: "Now I also need external clients to test the agent. Should I convert to NodePort or LoadBalancer? I'm testing locally with Minikube."

Note AI's reasoning about the choice.

**Part 4: Design Validation**

Ask AI: "Show me the complete Service YAML for external testing access on Minikube."

Apply the YAML to your cluster:

```bash
kubectl apply -f agent-service.yaml
```

Test external access:

```bash
minikube ip  # Get your Minikube IP
curl http://<minikube-ip>:<nodeport>/docs  # Your FastAPI agent's Swagger docs
```

**Part 5: Reflection**

Compare your initial understanding to what emerged:
- Did you initially consider both internal and external access?
- What would you do differently when deploying to a cloud cluster instead of Minikube?
- When would you choose LoadBalancer over NodePort?

These questions activate your reasoning for future Service design decisions in production environments.
