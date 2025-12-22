---
sidebar_position: 17
chapter: 50
lesson: 17
duration_minutes: 45
title: "Kubernetes Security for AI Services"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Manual security configuration builds understanding of production hardening"
cognitive_load:
  concepts_count: 9
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Configure Pods to run as non-root users"
    bloom_level: "Apply"
  - id: LO2
    description: "Set up SecurityContext for containers"
    bloom_level: "Apply"
  - id: LO3
    description: "Configure read-only root filesystem"
    bloom_level: "Apply"
  - id: LO4
    description: "Use Network Policies to isolate agent traffic"
    bloom_level: "Apply"
  - id: LO5
    description: "Understand Pod Security Standards (Restricted, Baseline, Privileged)"
    bloom_level: "Understand"
  - id: LO6
    description: "Scan container images for vulnerabilities (awareness)"
    bloom_level: "Understand"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "4. Safety"
    competency: "4.1 Protecting devices"
  - objective_id: LO2
    competency_area: "4. Safety"
    competency: "4.1 Protecting devices"
  - objective_id: LO3
    competency_area: "4. Safety"
    competency: "4.1 Protecting devices"
  - objective_id: LO4
    competency_area: "4. Safety"
    competency: "4.1 Protecting devices"
  - objective_id: LO5
    competency_area: "4. Safety"
    competency: "4.1 Protecting devices"
  - objective_id: LO6
    competency_area: "4. Safety"
    competency: "4.2 Protecting personal data and privacy"
---

# Kubernetes Security for AI Services

Your FastAPI agent is now deployed to Kubernetes (Lesson 4). It's running, scaling, exposed to traffic. But here's the question: **who can access what?**

In a production cluster, your agent container might handle sensitive data—user conversations, API keys, model weights. A compromised container could leak everything. This lesson builds the security foundations that protect your agent in production: non-root execution, read-only filesystems, network isolation, and vulnerability scanning.

By the end, your agent will run with minimal privileges, reject requests from unauthorized namespaces, and expose zero unnecessary attack surface.

---

## The Security Specification

Before we write a single YAML line, let's define what "secure" means for your agent.

**Security Intent**: AI agent handling sensitive user data must not run as root, must have read-only filesystem, must be isolated from other namespaces, and must reject unauthorized network traffic.

**Success Criteria**:
- ✅ Container runs as non-root user (UID > 1000)
- ✅ Root filesystem is read-only
- ✅ Container cannot gain elevated privileges
- ✅ Agent Pod only receives traffic from authorized namespaces
- ✅ Pod adheres to Restricted Pod Security Standard

**Constraints**:
- Must preserve application functionality (logging, tmp files still work)
- Container must have specific user ID pre-built into the image
- Network Policies require matching labels for routing

**Non-Goals**:
- Encrypting data at rest (handled by encrypted persistent volumes)
- Secret rotation automation (handled by external secret managers)
- Pod-to-Pod TLS encryption (handled by service mesh)

This specification guides every decision we make in this lesson.

---

## SecurityContext: Running as Non-Root

The first line of defense is **SecurityContext**—a Kubernetes configuration that controls how a container runs at the OS level.

### Why Non-Root Matters

By default, containers inherit the permissions of the user who created the image. In many base images (Python, Node), that user is root. This means:

```
Root container compromised → Attacker has root on the entire container
                         → Attacker can modify any file in the image
                         → Attacker can modify the kernel
```

Running as non-root doesn't prevent compromise, but it limits what an attacker can do after gaining access.

### Building a Secure Image

First, create an image with a non-root user. In your Dockerfile:

```dockerfile
FROM python:3.11-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt

# Create a non-root user
RUN useradd -m -u 1001 agentuser

# Set working directory permissions
RUN chown -R agentuser:agentuser /app

USER agentuser

COPY --chown=agentuser:agentuser . .

CMD ["python", "agent.py"]
```

**Key points**:
- `useradd -m -u 1001 agentuser` creates a user with ID 1001 (non-root, > 1000)
- `chown` transfers ownership to the new user
- `USER agentuser` makes the container run as this user by default
- `--chown` flag on `COPY` preserves ownership during the build

Build and push this image:

```bash
docker build -t myregistry/agent:v1-secure .
docker push myregistry/agent:v1-secure
```

**Output:**
```
Successfully built sha256:abc123...
Successfully tagged myregistry/agent:v1-secure
The push refers to repository [myregistry/agent]
v1-secure: digest: sha256:def456... size: 45MB
```

### Enforcing Non-Root with SecurityContext

Even if your image runs as a non-root user, Kubernetes can enforce it with SecurityContext. This prevents accidentally running a container as root.

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: agent-secure
spec:
  securityContext:
    runAsNonRoot: true
    runAsUser: 1001
    fsGroup: 1001
  containers:
  - name: agent
    image: myregistry/agent:v1-secure
    securityContext:
      allowPrivilegeEscalation: false
      readOnlyRootFilesystem: false
      capabilities:
        drop:
          - ALL
```

**What each setting does**:
- **`runAsNonRoot: true`**: Kubernetes rejects any container that tries to run as root (UID 0)
- **`runAsUser: 1001`**: Force the container to run as user 1001, even if the image says otherwise
- **`fsGroup: 1001`**: Group ownership of volumes mounted to the Pod
- **`allowPrivilegeEscalation: false`**: Container cannot become root later (blocks `sudo`, setuid)
- **`drop: ALL`**: Remove ALL Linux capabilities (no raw sockets, no device access)

Apply this Pod:

```bash
kubectl apply -f agent-secure.yaml
```

**Output:**
```
pod/agent-secure created
```

Verify the container runs as the correct user:

```bash
kubectl exec agent-secure -- id
```

**Output:**
```
uid=1001(agentuser) gid=1001(agentuser) groups=1001(agentuser)
```

The container is now running as user 1001, not root.

---

## Read-Only Root Filesystem

The second layer: **read-only root filesystem**. An attacker who gains code execution inside the container can modify files on disk, install backdoors, or change the application logic. A read-only filesystem blocks this attack vector.

### Understanding Filesystem Layers

When you mount a read-only filesystem, the application can still write to specific locations using **emptyDir** volumes. These are temporary, per-Pod directories that disappear when the Pod restarts—perfect for logs, temp files, and caches.

### Configuring Read-Only Root with Writable Volume

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: agent-app
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
      securityContext:
        runAsNonRoot: true
        runAsUser: 1001
      containers:
      - name: agent
        image: myregistry/agent:v1-secure
        securityContext:
          readOnlyRootFilesystem: true
          allowPrivilegeEscalation: false
          capabilities:
            drop:
              - ALL
        volumeMounts:
        - name: tmp-volume
          mountPath: /tmp
        - name: logs-volume
          mountPath: /var/log/agent
        ports:
        - containerPort: 8000
      volumes:
      - name: tmp-volume
        emptyDir: {}
      - name: logs-volume
        emptyDir: {}
```

**How this works**:
- **`readOnlyRootFilesystem: true`**: Root filesystem cannot be modified
- **`volumeMounts`**: Mount writable locations for the app
- **`emptyDir: {}`**: A temporary directory backed by the host's disk (or memory, depending on config)

The application can write to `/tmp` and `/var/log/agent`, but every other file is read-only.

Deploy this:

```bash
kubectl apply -f agent-deployment-readonly.yaml
```

**Output:**
```
deployment.apps/agent-app created
```

Verify read-only filesystem by trying to write to root:

```bash
kubectl exec -it deployment/agent-app -- sh -c "touch /test.txt"
```

**Output:**
```
Read-only file system
```

But writing to the mounted volume works:

```bash
kubectl exec -it deployment/agent-app -- sh -c "echo 'test' > /tmp/test.txt && cat /tmp/test.txt"
```

**Output:**
```
test
```

The filesystem is now locked down. Your application can write only to designated temporary locations.

---

## Network Policies: Isolating Agent Traffic

The third layer: **Network Policies**. By default, Kubernetes allows all Pods to communicate with each other. A compromised Pod in one namespace could reach Pods in another.

Network Policies enforce segmentation: your agent only receives traffic from authorized namespaces and services.

### Denying All Traffic, Then Allowing Specific Sources

```yaml
# Step 1: Deny all ingress traffic by default
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: agent-default-deny
  namespace: agents
spec:
  podSelector: {}
  policyTypes:
  - Ingress

---
# Step 2: Allow traffic only from the api-gateway in the ingress namespace
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: agent-allow-from-gateway
  namespace: agents
spec:
  podSelector:
    matchLabels:
      app: agent
  policyTypes:
  - Ingress
  ingress:
  - from:
    - namespaceSelector:
        matchLabels:
          name: ingress
      podSelector:
        matchLabels:
          app: api-gateway
    ports:
    - protocol: TCP
      port: 8000

---
# Step 3: Allow agent Pods to reach external APIs
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: agent-allow-egress
  namespace: agents
spec:
  podSelector:
    matchLabels:
      app: agent
  policyTypes:
  - Egress
  egress:
  - to:
    - namespaceSelector: {}
    ports:
    - protocol: TCP
      port: 443
```

**What each policy does**:
- **`agent-default-deny`**: All Pods in the `agents` namespace reject all incoming traffic by default
- **`agent-allow-from-gateway`**: Only accept traffic from Pods labeled `app: api-gateway` in the `ingress` namespace, on port 8000
- **`agent-allow-egress`**: Allow agent to initiate outbound connections on port 443 (HTTPS) to external services

The key insight: **explicit allow**. Every connection must match a rule or it's blocked.

First, make sure your namespaces are labeled:

```bash
kubectl label namespace ingress name=ingress
kubectl label namespace agents name=agents
```

**Output:**
```
namespace/ingress labeled
namespace/agents labeled
```

Apply the Network Policies:

```bash
kubectl apply -f agent-network-policy.yaml
```

**Output:**
```
networkpolicy.networking.k8s.io/agent-default-deny created
networkpolicy.networking.k8s.io/agent-allow-from-gateway created
networkpolicy.networking.k8s.io/agent-allow-egress created
```

Test the policy by trying to reach the agent from an unauthorized Pod:

```bash
# From a different namespace without permission
kubectl run test-pod --image=curlimages/curl -n other -- sleep 3600
kubectl exec test-pod -n other -- curl http://agent-service.agents.svc.cluster.local:8000
```

**Output:**
```
command terminated with exit code 1
```

The connection is denied. But traffic from the authorized gateway succeeds:

```bash
# From the authorized api-gateway Pod
kubectl exec deployment/api-gateway -n ingress -- curl http://agent-service.agents.svc.cluster.local:8000/health
```

**Output:**
```
{"status": "healthy", "model": "gpt-4o", "uptime_seconds": 3600}
```

Your agent is now isolated—only authorized services can reach it.

---

## Pod Security Standards

Kubernetes provides **Pod Security Standards**—three tiers that codify security best practices. This lesson's agent should adhere to the Restricted standard.

### The Three Standards

| Standard | Use Case | Restrictions |
|----------|----------|--------------|
| **Privileged** | System components needing OS access | None—allows everything |
| **Baseline** | General-purpose applications | Disallows privileged containers, host access |
| **Restricted** | High-security applications (agents, APIs handling sensitive data) | Requires non-root, read-only, no privilege escalation, drop all capabilities |

Your agent should meet **Restricted** because it handles sensitive data.

### Enforcing Pod Security Standards

Label your namespace to enforce the Restricted standard:

```bash
kubectl label namespace agents pod-security.kubernetes.io/enforce=restricted
```

**Output:**
```
namespace/agents labeled
```

Now, any Pod in the `agents` namespace that violates Restricted standards is rejected. For example, this Pod would be rejected:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: violates-pss
  namespace: agents
spec:
  containers:
  - name: app
    image: myimage:v1
    securityContext:
      runAsUser: 0  # root—violates Restricted
```

Apply it:

```bash
kubectl apply -f violating-pod.yaml
```

**Output:**
```
error: pods "violates-pss" is forbidden: violates PodSecurity "restricted:latest":
  allowPrivilegeEscalation != false (container "app" must set allowPrivilegeEscalation to false)
  runAsNonRoot != true (container "app" must set runAsNonRoot to true)
  securityContext.runAsUser must be > 0
```

The Pod is rejected. Your agent (built with proper SecurityContext) passes validation automatically.

---

## Container Image Security: Vulnerability Scanning

The fourth layer: **image security**. Before your container runs, scan it for known vulnerabilities in dependencies.

### Why Scanning Matters

Your agent image includes:
- Python 3.11 (security updates released regularly)
- pip packages (FastAPI, NumPy, OpenAI SDK)
- OS libraries (libssl, libcrypto)

Each could contain CVEs (Common Vulnerabilities and Exposures). Scanning detects them before deployment.

### Tools for Scanning

Popular options:

| Tool | Approach | Best For |
|------|----------|----------|
| **Trivy** (Aqua Security) | Container image scanning | Local development, CI/CD pipelines |
| **Grype** (Anchore) | Vulnerability database | Supply chain security |
| **Snyk** | SaaS scanning | Developer-first security |
| **Harbor** | Registry integration | Preventing vulnerable images from being pushed |

For this lesson, we use **Trivy** (open-source, easy to integrate).

### Scanning Your Agent Image

Install Trivy (or use Docker image):

```bash
trivy image myregistry/agent:v1-secure
```

**Output:**
```
myregistry/agent:v1-secure (linux/amd64)
==================================
Total: 8 Vulnerabilities
┌───────────────────────────────────────────┬──────────┬──────────┐
│ Library                  │ Vulnerability ID │ Severity │
├──────────────────────────┼──────────────────┼──────────┤
│ libssl1.1                │ CVE-2023-2976    │ MEDIUM   │
│ libcrypto1.1             │ CVE-2023-3817    │ MEDIUM   │
│ pip packages             │ CVE-2024-123     │ LOW      │
└──────────────────────────┴──────────────────┴──────────┘
```

Most vulnerabilities are in the base Python image. To fix them:

```dockerfile
FROM python:3.11-slim
RUN apt-get update && apt-get upgrade -y
```

Upgrading base packages often resolves vulnerabilities.

### Integration into CI/CD

Add a scanning step to your deployment pipeline:

```bash
# Before pushing to registry
trivy image --severity HIGH,CRITICAL myregistry/agent:v1
if [ $? -ne 0 ]; then
  echo "High/critical vulnerabilities found. Fix before deploying."
  exit 1
fi

docker push myregistry/agent:v1
```

This prevents pushing vulnerable images.

---

## Putting It All Together: A Secure Agent Deployment

Here's the complete, security-hardened Deployment combining all layers:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: agent-prod
  namespace: agents
spec:
  replicas: 3
  selector:
    matchLabels:
      app: agent
  template:
    metadata:
      labels:
        app: agent
    spec:
      securityContext:
        runAsNonRoot: true
        runAsUser: 1001
        fsGroup: 1001
      containers:
      - name: agent
        image: myregistry/agent:v1-secure
        securityContext:
          readOnlyRootFilesystem: true
          allowPrivilegeEscalation: false
          capabilities:
            drop:
              - ALL
        resources:
          requests:
            cpu: 500m
            memory: 512Mi
          limits:
            cpu: 1000m
            memory: 1Gi
        volumeMounts:
        - name: tmp
          mountPath: /tmp
        - name: logs
          mountPath: /var/log/agent
        ports:
        - containerPort: 8000
          name: http
        livenessProbe:
          httpGet:
            path: /health
            port: 8000
          initialDelaySeconds: 10
          periodSeconds: 10
      volumes:
      - name: tmp
        emptyDir: {}
      - name: logs
        emptyDir: {}

---
apiVersion: v1
kind: Service
metadata:
  name: agent-service
  namespace: agents
spec:
  selector:
    app: agent
  type: ClusterIP
  ports:
  - port: 8000
    targetPort: 8000

---
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: agent-deny-all
  namespace: agents
spec:
  podSelector: {}
  policyTypes:
  - Ingress

---
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: agent-allow-ingress
  namespace: agents
spec:
  podSelector:
    matchLabels:
      app: agent
  policyTypes:
  - Ingress
  ingress:
  - from:
    - namespaceSelector:
        matchLabels:
          name: ingress
```

Deploy everything:

```bash
kubectl apply -f agent-production.yaml
```

**Output:**
```
deployment.apps/agent-prod created
service/agent-service created
networkpolicy.networking.k8s.io/agent-deny-all created
networkpolicy.networking.k8s.io/agent-allow-ingress created
```

Verify security settings:

```bash
kubectl get pod -n agents -o jsonpath='{.items[0].spec.securityContext}'
```

**Output:**
```
{"fsGroup":1001,"runAsNonRoot":true,"runAsUser":1001}
```

Your agent is now deployed with:
- Non-root execution
- Read-only root filesystem
- Network isolation
- Vulnerable images prevented at CI/CD
- Pod Security Standards enforced

---

## Try With AI

**Setup**: You have a running Kubernetes cluster with an agent Deployment (from Lesson 4 or above). Your AI assistant can help audit the deployment for security gaps.

**Task 1: Audit an Existing Deployment for Security**

Describe your current agent Deployment:

```
I have a Deployment running a FastAPI agent in Kubernetes.
The Pod spec looks like:

spec:
  containers:
  - name: agent
    image: myregistry/agent:v1.0
    # No securityContext configured
```

Ask Claude:

```
My agent runs in Kubernetes without security hardening.
Help me identify security gaps and prioritize fixes based on impact.
```

Reflect: What vulnerabilities did Claude identify? Which fixes address the highest-risk scenarios first?

**Task 2: Generate Security Configuration**

Ask Claude:

```
Based on our Deployment, generate:
1. A secure Dockerfile that creates a non-root user
2. SecurityContext settings for the Pod
3. A Network Policy isolating the agent to the ingress namespace
4. A scanning step for our CI/CD pipeline
```

Reflect: What constraints does Claude ask about? What assumptions does it make?

**Task 3: Validate Against Pod Security Standards**

After applying your security changes, verify compliance:

```bash
kubectl label namespace agents pod-security.kubernetes.io/enforce=restricted
kubectl apply -f agent-deployment.yaml
```

Ask Claude:

```
I got this error when applying my Pod. Help me understand why it violates Restricted standard
and what changes are needed.
```

Your agent is now secure by default, with AI-assisted hardening and validation.
