---
sidebar_position: 8
chapter: 50
lesson: 8
duration_minutes: 40
title: "Namespaces: Virtual Clusters for AI Workloads"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Manual namespace management builds understanding of cluster organization"
cognitive_load:
  concepts_count: 9
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Explain Namespaces as virtual cluster partitions"
    bloom_level: "Understand"
  - id: LO2
    description: "Create and manage Namespaces with kubectl"
    bloom_level: "Apply"
  - id: LO3
    description: "Deploy resources to specific Namespaces"
    bloom_level: "Apply"
  - id: LO4
    description: "Configure ResourceQuotas to limit namespace resource usage"
    bloom_level: "Apply"
  - id: LO5
    description: "Configure LimitRanges for default container limits"
    bloom_level: "Apply"
  - id: LO6
    description: "Implement multi-environment strategy (dev/staging/prod)"
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
    competency: "5.3 Using digital tools creatively"
---

# Namespaces: Virtual Clusters for AI Workloads

You've deployed services to Kubernetes and exposed them for traffic. But what happens when your dev team's experimental agent consumes all the cluster's GPU memory? Or when a prod bug causes one team's deployment to spiral out of control, bringing down everyone else's workloads? In shared clusters, resource contention becomes inevitable without boundaries.

Namespaces solve this. They're virtual clusters within a single Kubernetes cluster—isolated spaces where you can enforce quotas, set default limits, and partition workloads by environment, team, or project. When you deploy the same agent to dev, staging, and production, namespaces keep each environment separate and constrained.

## What Are Namespaces?

A namespace is a logical partition of a Kubernetes cluster. Think of it like having separate apartments in a building—they share the same foundation and utilities, but each has its own door, locks, and boundaries.

**Three key properties of namespaces:**

1. **Resource Scoping**: Pods, services, and deployments live in namespaces. When you reference a service, you implicitly reference the one in the current namespace (unless you specify otherwise)
2. **Quota Enforcement**: You can limit CPU, memory, and storage per namespace so one team's workloads can't starve others
3. **Identity Isolation**: Service accounts and RBAC policies are namespaced, enabling per-environment security

**Kubernetes provides three built-in namespaces:**

- `default` - Where your resources go if you don't specify a namespace
- `kube-system` - Core Kubernetes components (API server, controller manager, DNS)
- `kube-public` - Publicly readable resources (cluster info ConfigMaps)

The problem without namespaces: All deployments share one resource pool. A runaway pod in development can consume cluster memory, causing prod services to crash. The solution: Use namespaces to partition the cluster by environment and enforce quotas.

## Creating and Managing Namespaces

Let's create namespaces for dev, staging, and production environments. The simplest approach is using kubectl:

```bash
kubectl create namespace dev
kubectl create namespace staging
kubectl create namespace prod
```

**Output:**
```
namespace/dev created
namespace/staging created
namespace/prod created
```

Verify the namespaces exist:

```bash
kubectl get namespaces
```

**Output:**
```
NAME              STATUS   AGE
default           Active   2d15h
dev               Active   45s
kube-node-lease   Active   2d15h
kube-public       Active   2d15h
kube-system       Active   2d15h
prod              Active   42s
staging           Active   43s
```

Now let's deploy a simple agent deployment to the dev namespace:

```bash
kubectl apply -f - <<EOF
apiVersion: apps/v1
kind: Deployment
metadata:
  name: agent-api
  namespace: dev
spec:
  replicas: 2
  selector:
    matchLabels:
      app: agent-api
      env: dev
  template:
    metadata:
      labels:
        app: agent-api
        env: dev
    spec:
      containers:
      - name: api
        image: agent-api:latest
        ports:
        - containerPort: 8000
        resources:
          requests:
            cpu: 100m
            memory: 128Mi
          limits:
            cpu: 500m
            memory: 512Mi
EOF
```

**Output:**
```
deployment.apps/agent-api created
```

Verify the deployment exists in the dev namespace:

```bash
kubectl get deployments --namespace=dev
```

**Output:**
```
NAME        READY   UP-TO-DATE   AVAILABLE   AGE
agent-api   2/2     2            2           15s
```

Notice the `--namespace=dev` flag. Without it, kubectl defaults to the `default` namespace. You can also use the shorthand `-n dev`.

## ResourceQuotas: Limiting Namespace Resource Usage

Namespaces become powerful when you enforce quotas. A ResourceQuota limits the total CPU, memory, and other resources a namespace can consume. This prevents one team's workloads from starving others.

Create a ResourceQuota for the dev namespace:

```bash
kubectl apply -f - <<EOF
apiVersion: v1
kind: ResourceQuota
metadata:
  name: dev-quota
  namespace: dev
spec:
  hard:
    requests.cpu: "2"           # Max 2 CPUs across all pods
    requests.memory: "2Gi"      # Max 2GB RAM across all pods
    limits.cpu: "4"             # Max 4 CPUs in limits across all pods
    limits.memory: "4Gi"        # Max 4GB in limits across all pods
    pods: "20"                  # Max 20 pods in this namespace
    requests.storage: "10Gi"    # Max 10GB storage
EOF
```

**Output:**
```
resourcequota/dev-quota created
```

Verify the quota was created:

```bash
kubectl get resourcequota --namespace=dev
```

**Output:**
```
NAME        AGE   REQUEST.CPU   REQUEST.MEMORY   LIMITS.CPU   LIMITS.MEMORY   PODS
dev-quota   8s    200m          256Mi            1            1Gi             2/20
```

The output shows current usage (200m CPU, 256Mi memory) from the two agent-api pods we created earlier. The quota allows up to 2 CPUs and 2GB memory in requests.

**Why this matters for AI workloads:** GPU-intensive agents require reserved resources. A ResourceQuota prevents experimental agents in dev from consuming the GPU memory that staging needs for final testing.

## LimitRanges: Default Limits for Containers

ResourceQuotas set namespace-wide limits. LimitRanges set per-container defaults and boundaries. When developers deploy containers without specifying requests/limits, LimitRange provides sensible defaults.

Create a LimitRange for the dev namespace:

```bash
kubectl apply -f - <<EOF
apiVersion: v1
kind: LimitRange
metadata:
  name: dev-limits
  namespace: dev
spec:
  limits:
  - type: Container
    default:
      cpu: 250m
      memory: 256Mi
    defaultRequest:
      cpu: 100m
      memory: 128Mi
    max:
      cpu: 500m
      memory: 512Mi
    min:
      cpu: 10m
      memory: 32Mi
EOF
```

**Output:**
```
limitrange/dev-limits created
```

Verify the LimitRange:

```bash
kubectl get limitrange --namespace=dev
```

**Output:**
```
NAME        CREATED AT
dev-limits  2025-01-20T14:32:15Z
```

Describe the LimitRange to see the full configuration:

```bash
kubectl describe limitrange dev-limits --namespace=dev
```

**Output:**
```
Name:       dev-limits
Namespace:  dev
Type        Resource  Min    Max    Default Request  Default Limit
----        --------  ---    ---    ---------------  -------------
Container   cpu       10m    500m   100m             250m
Container   memory    32Mi   512Mi  128Mi            256Mi
```

Now, if a developer deploys a pod without specifying requests/limits, Kubernetes automatically applies the `defaultRequest` and `default` values from the LimitRange. This ensures every pod in the namespace has reasonable resource boundaries.

**Example**: Deploy a pod without explicit limits—the LimitRange provides defaults:

```bash
kubectl apply -f - <<EOF
apiVersion: v1
kind: Pod
metadata:
  name: agent-worker
  namespace: dev
spec:
  containers:
  - name: worker
    image: agent-worker:latest
    ports:
    - containerPort: 8001
EOF
```

**Output:**
```
pod/agent-worker created
```

Check the pod's actual requests/limits:

```bash
kubectl get pod agent-worker --namespace=dev -o jsonpath='{.spec.containers[0].resources}'
```

**Output:**
```
{"limits":{"cpu":"250m","memory":"256Mi"},"requests":{"cpu":"100m","memory":"128Mi"}}
```

The LimitRange automatically injected the default values even though we didn't specify them in the pod definition.

## Multi-Environment Strategy: Deploying to Dev, Staging, Prod

The real power of namespaces emerges when you deploy the same application across environments with identical configuration but different resource quotas. Here's how to structure a multi-environment setup:

**1. Create staging and prod namespaces with their own quotas:**

```bash
# Staging: More resources than dev, production-like constraints
kubectl apply -f - <<EOF
apiVersion: v1
kind: ResourceQuota
metadata:
  name: staging-quota
  namespace: staging
spec:
  hard:
    requests.cpu: "4"
    requests.memory: "4Gi"
    limits.cpu: "8"
    limits.memory: "8Gi"
    pods: "30"
---
apiVersion: v1
kind: LimitRange
metadata:
  name: staging-limits
  namespace: staging
spec:
  limits:
  - type: Container
    default:
      cpu: 500m
      memory: 512Mi
    defaultRequest:
      cpu: 250m
      memory: 256Mi
    max:
      cpu: 1000m
      memory: 1Gi
    min:
      cpu: 50m
      memory: 64Mi
EOF
```

**Output:**
```
resourcequota/staging-quota created
limitrange/staging-limits created
```

**2. Production: Highest quotas and strictest limits:**

```bash
kubectl apply -f - <<EOF
apiVersion: v1
kind: ResourceQuota
metadata:
  name: prod-quota
  namespace: prod
spec:
  hard:
    requests.cpu: "8"
    requests.memory: "8Gi"
    limits.cpu: "16"
    limits.memory: "16Gi"
    pods: "50"
---
apiVersion: v1
kind: LimitRange
metadata:
  name: prod-limits
  namespace: prod
spec:
  limits:
  - type: Container
    default:
      cpu: 750m
      memory: 768Mi
    defaultRequest:
      cpu: 500m
      memory: 512Mi
    max:
      cpu: 2000m
      memory: 2Gi
    min:
      cpu: 100m
      memory: 128Mi
EOF
```

**Output:**
```
resourcequota/prod-quota created
limitrange/prod-limits created
```

**3. Deploy your agent to all three environments:**

Save this as `agent-deployment.yaml`:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: agent-api
spec:
  replicas: 2
  selector:
    matchLabels:
      app: agent-api
  template:
    metadata:
      labels:
        app: agent-api
    spec:
      containers:
      - name: api
        image: agent-api:latest
        ports:
        - containerPort: 8000
        resources:
          requests:
            cpu: 200m
            memory: 256Mi
          limits:
            cpu: 500m
            memory: 512Mi
```

Deploy to all three namespaces:

```bash
kubectl apply -f agent-deployment.yaml --namespace=dev
kubectl apply -f agent-deployment.yaml --namespace=staging
kubectl apply -f agent-deployment.yaml --namespace=prod
```

**Output:**
```
deployment.apps/agent-api created
deployment.apps/agent-api created
deployment.apps/agent-api created
```

Verify deployments in each namespace:

```bash
kubectl get deployments --all-namespaces | grep agent-api
```

**Output:**
```
dev            agent-api   2/2     2            2           3m20s
prod           agent-api   2/2     2            2           3m15s
staging        agent-api   2/2     2            2           3m18s
```

Now each environment has the same agent running with resource isolation. A memory leak in dev won't affect prod.

## Cross-Namespace Communication

By default, services in one namespace cannot directly reference services in another namespace. But if you need cross-namespace communication, use the fully qualified DNS name: `<service-name>.<namespace-name>.svc.cluster.local`

For example, if the `logging` service in the `observability` namespace needs to reach the `agent-api` service in `prod`:

```bash
curl http://agent-api.prod.svc.cluster.local:8000/health
```

The DNS name breaks down as:
- `agent-api` - service name
- `prod` - namespace
- `svc.cluster.local` - cluster DNS suffix (always this)

This pattern is useful when you have shared services (logging, monitoring, authentication) in a central namespace that other namespaces need to access.

## Try With AI

You now understand how namespaces provide virtual cluster partitions with resource quotas and per-container limits. The AI collaboration begins when you need to design namespace strategies for complex multi-team scenarios.

### Exercise 1: Design Namespace Strategy for Multi-Team Setup

**Scenario**: Your organization has 3 teams deploying AI agents to the same cluster:

- **Data Pipeline Team**: Runs long-running training jobs (GPU-intensive, high memory)
- **Model Serving Team**: Runs inference APIs (low latency requirements, moderate CPU)
- **Experimentation Team**: Runs short-lived development agents (highly variable, experimental)

**Your task**: Design a namespace strategy that isolates these teams while allowing shared observability services (Prometheus, Loki) to monitor all of them.

**Parts to consider:**

1. How many namespaces would you create?
2. What ResourceQuota would you set for each team's namespace?
3. How would you expose monitoring/logging endpoints so all teams can reach shared services?
4. What LimitRange defaults would prevent runaway pods in each environment?

**Ask AI to evaluate your strategy**: Describe your namespace design, ResourceQuotas, and cross-namespace access patterns. AI can review whether your quotas are realistic for the workload types and suggest adjustments based on team requirements.

