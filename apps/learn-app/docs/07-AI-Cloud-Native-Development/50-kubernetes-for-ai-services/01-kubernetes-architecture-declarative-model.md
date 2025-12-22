---
sidebar_position: 1
chapter: 50
lesson: 1
duration_minutes: 35
title: "Kubernetes Architecture and the Declarative Model"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Conceptual foundations before any hands-on commands"
cognitive_load:
  concepts_count: 8
  scaffolding_level: "Heavy"
learning_objectives:
  - id: LO1
    description: "Explain why container orchestration exists (Docker limitations in production)"
    bloom_level: "Understand"
  - id: LO2
    description: "Describe the declarative model: desired state vs observed state"
    bloom_level: "Understand"
  - id: LO3
    description: "Identify control plane components (API server, scheduler, etcd, controller manager)"
    bloom_level: "Remember"
  - id: LO4
    description: "Identify worker node components (kubelet, kube-proxy, container runtime)"
    bloom_level: "Remember"
  - id: LO5
    description: "Explain the reconciliation loop as Kubernetes' core operating principle"
    bloom_level: "Understand"
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
    competency_area: "1. Information and Data Literacy"
    competency: "1.2 Understanding digital concepts and terminology"
  - objective_id: LO5
    competency_area: "5. Problem Solving"
    competency: "5.3 Using digital tools to solve problems"
---

# Kubernetes Architecture and the Declarative Model

Your FastAPI agent from Part 6 is now containerized. In Chapter 49, you packaged it into a Docker image—portable, reproducible, ready to run anywhere. But "anywhere" in production means dozens of servers, not your laptop.

Here's the problem: You run `docker run my-agent:v1` on one server. Great. Now what happens when:
- That server crashes at 3 AM?
- Traffic spikes and one container can't handle the load?
- You need to deploy version v2 without dropping user requests?

With Docker alone, YOU are the "someone" who handles all of this manually. With Kubernetes, the cluster handles it automatically.

This lesson builds the mental model you need before running any kubectl commands: why orchestration exists, how Kubernetes thinks declaratively about state, and what happens inside a cluster when things go wrong.

---

## Why Docker Isn't Enough

In Chapter 49, you learned Docker's core value: package your application into an image, run it as a container anywhere. Docker solved the "it works on my machine" problem.

But Docker alone doesn't solve:

**1. Multi-machine deployment**: `docker run` starts one container on one machine. Production needs containers across 10, 100, or 1000 machines.

**2. Self-healing**: If a container crashes, Docker doesn't restart it automatically (unless you use restart policies—and even those are limited). Your application stays down until someone notices.

**3. Scaling**: When traffic increases, you need more container copies. With Docker alone, you'd manually run `docker run` on multiple machines and somehow balance traffic between them.

**4. Updates without downtime**: Deploying a new version means stopping the old container, then starting the new one. Users see an error during that gap.

**5. Service discovery**: Your frontend needs to talk to your backend. Both are containers, but their IP addresses change every time they restart. How do they find each other?

### The Gap Between Docker and Production

```
Development (Docker alone)          Production (what you need)
--------------------------          -------------------------
One container, one laptop           100+ containers, many machines
Manual restarts                     Automatic recovery
Static configuration                Dynamic scaling
Down during updates                 Zero-downtime deployments
Hard-coded IPs                      Automatic service discovery
```

Container orchestration fills this gap. Kubernetes is the industry standard for orchestrating containers at scale.

---

## The Declarative Model: You Describe WHAT, Kubernetes Figures Out HOW

The most important concept in Kubernetes is the **declarative model**.

### Imperative vs Declarative

**Imperative** (Docker-style): You give step-by-step instructions.

```bash
docker run nginx       # Step 1: Start nginx
docker run nginx       # Step 2: Start another nginx
docker run nginx       # Step 3: Start a third nginx
# Wait, one crashed. Now I need to figure out which one and restart it.
```

**Declarative** (Kubernetes-style): You describe the end state you want.

```yaml
apiVersion: apps/v1
kind: Deployment
spec:
  replicas: 3          # I want 3 nginx containers
  template:
    spec:
      containers:
      - name: nginx
        image: nginx:1.25
```

You declare: "I want 3 nginx containers running." Kubernetes makes it happen. If one crashes, Kubernetes notices the mismatch (desired: 3, actual: 2) and creates a new one.

### Desired State vs Observed State

This is the core loop:

```
┌─────────────────────────────────────────┐
│                                         │
│  YOU: "I want 3 replicas"  (DESIRED)    │
│           │                             │
│           ▼                             │
│  KUBERNETES: Observes 2 running (OBSERVED)
│           │                             │
│           ▼                             │
│  MISMATCH DETECTED: 3 ≠ 2              │
│           │                             │
│           ▼                             │
│  ACTION: Create 1 new replica           │
│           │                             │
│           ▼                             │
│  OBSERVE AGAIN: Now 3 running          │
│           │                             │
│           ▼                             │
│  MATCH: 3 = 3, no action needed        │
│           │                             │
│           └─────────────────────────────┘
│                (loop continues)
```

This loop runs continuously. Kubernetes controllers are always watching, comparing desired to observed, and taking action to close the gap.

### Why This Matters

The declarative model means:

1. **Self-healing is built-in**: Crashes create mismatches that controllers automatically fix.
2. **Scaling is one number change**: Update `replicas: 3` to `replicas: 10`. Kubernetes handles the rest.
3. **Rollbacks are instant**: Your previous desired state is stored. Reverting means pointing back to it.
4. **Intent is documented**: Your YAML files describe what should exist. Version control them like code.

---

## Kubernetes Architecture: The Big Picture

A Kubernetes cluster has two types of machines:

```
┌─────────────────────────────────────────────────────────────┐
│                     KUBERNETES CLUSTER                      │
├──────────────────────────┬──────────────────────────────────┤
│     CONTROL PLANE        │         WORKER NODES             │
│                          │                                  │
│  ┌──────────────────┐   │   ┌─────────────────────────┐   │
│  │   API Server     │◄──┼───│  kubelet                │   │
│  └────────┬─────────┘   │   │  (manages containers)   │   │
│           │             │   └─────────────────────────┘   │
│  ┌────────▼─────────┐   │                                  │
│  │   Scheduler      │   │   ┌─────────────────────────┐   │
│  │  (places Pods)   │   │   │  kube-proxy             │   │
│  └────────┬─────────┘   │   │  (network routing)      │   │
│           │             │   └─────────────────────────┘   │
│  ┌────────▼─────────┐   │                                  │
│  │  Controller      │   │   ┌─────────────────────────┐   │
│  │  Manager         │   │   │  Container Runtime      │   │
│  └────────┬─────────┘   │   │  (runs containers)      │   │
│           │             │   └─────────────────────────┘   │
│  ┌────────▼─────────┐   │                                  │
│  │   etcd           │   │                                  │
│  │  (cluster state) │   │                                  │
│  └──────────────────┘   │                                  │
├──────────────────────────┴──────────────────────────────────┤
│  The "brains"            │  The "muscles"                   │
│  Decides WHAT to do      │  Actually runs your containers   │
└─────────────────────────────────────────────────────────────┘
```

**Control Plane**: The brains. Makes decisions about what should run where.

**Worker Nodes**: The muscles. Actually run your containers.

---

## Control Plane Components

The control plane runs on dedicated machines (or VMs) and manages the entire cluster.

### API Server

The front door to Kubernetes. Every operation—creating a deployment, scaling replicas, checking status—goes through the API server.

When you run `kubectl apply`, you're sending a request to the API server. When the dashboard shows your pods, it queried the API server.

The API server:
- Validates incoming requests
- Stores objects in etcd
- Notifies controllers about changes

### etcd

A distributed key-value store that holds the cluster's state:
- What deployments exist
- How many replicas each should have
- Pod configurations
- Secrets and ConfigMaps

etcd is the single source of truth. If etcd loses data, you lose your cluster configuration.

### Scheduler

When you create a Pod, someone needs to decide which worker node runs it. That's the scheduler.

The scheduler considers:
- **Resource availability**: Does the node have enough CPU and memory?
- **Affinity rules**: Should this Pod run near (or far from) other Pods?
- **Taints and tolerations**: Is the node restricted to certain workloads?

It doesn't start the Pod—it just assigns the Pod to a node. The kubelet on that node does the actual starting.

### Controller Manager

Controllers implement the reconciliation loop. Each controller watches specific resources:

- **Deployment Controller**: Ensures the right number of ReplicaSets exist
- **ReplicaSet Controller**: Ensures the right number of Pods exist
- **Node Controller**: Monitors node health, marks nodes as unavailable when they stop responding
- **Job Controller**: Manages batch jobs that run to completion

When a Pod crashes, the ReplicaSet controller detects the mismatch (desired vs observed) and creates a replacement.

---

## Worker Node Components

Worker nodes run your actual containers. Each node has three critical components.

### kubelet

The agent running on every worker node. It:
- Receives Pod specifications from the API server
- Tells the container runtime to start containers
- Monitors container health
- Reports status back to the API server

When you deploy a Pod, the scheduler assigns it to a node, and the kubelet on that node makes it happen.

### kube-proxy

Handles network routing on each node. When your frontend Pod needs to reach your backend Pod, kube-proxy manages the routing.

It implements Kubernetes Services—stable network endpoints that route to the right Pods even when Pod IPs change.

### Container Runtime

The software that actually runs containers. Kubernetes supports multiple runtimes:
- **containerd**: Most common in production
- **CRI-O**: Red Hat's container runtime
- **Docker**: Was default, now deprecated as a runtime (though Docker-built images still work)

The kubelet talks to the container runtime through a standard interface (CRI—Container Runtime Interface), so Kubernetes doesn't care which runtime you use.

---

## The Reconciliation Loop: Kubernetes' Heartbeat

Everything in Kubernetes operates through reconciliation loops:

```
                    ┌────────────────────────────┐
                    │                            │
                    │  1. OBSERVE                │
                    │     Check actual state     │
                    │                            │
                    └────────────┬───────────────┘
                                 │
                                 ▼
                    ┌────────────────────────────┐
                    │                            │
                    │  2. COMPARE                │
                    │     Actual vs Desired      │
                    │                            │
                    └────────────┬───────────────┘
                                 │
                    ┌────────────┴───────────────┐
                    │                            │
             ┌──────▼──────┐              ┌──────▼──────┐
             │             │              │             │
             │  Match:     │              │  Mismatch:  │
             │  Do nothing │              │  Take action│
             │             │              │             │
             └──────┬──────┘              └──────┬──────┘
                    │                            │
                    │                            │
                    └──────────┬─────────────────┘
                               │
                               ▼
                    ┌────────────────────────────┐
                    │                            │
                    │  3. WAIT                   │
                    │     Brief pause            │
                    │                            │
                    └────────────┬───────────────┘
                                 │
                                 └────────► (back to OBSERVE)
```

This loop runs constantly for every resource type. Here's what happens when you deploy:

1. **You apply a Deployment manifest**: API server stores it in etcd.
2. **Deployment controller notices**: A new Deployment exists with `replicas: 3`.
3. **Deployment controller creates ReplicaSet**: Desired state: 3 Pods.
4. **ReplicaSet controller notices**: 0 Pods exist, but 3 are desired.
5. **ReplicaSet controller creates Pods**: 3 Pod objects in etcd.
6. **Scheduler notices**: 3 Pods need nodes.
7. **Scheduler assigns Pods to nodes**: Pod objects updated with node assignments.
8. **kubelets notice**: Each kubelet sees a Pod assigned to its node.
9. **kubelets start containers**: Actual containers running.
10. **Observation continues**: All controllers keep watching for mismatches.

---

## What Happens When a Container Crashes?

Let's trace through a failure scenario:

1. **Container exits** (application bug, out of memory, etc.)
2. **kubelet detects the exit**: Marks the Pod as failed.
3. **kubelet restarts the container**: Based on Pod's restart policy.
4. **If container keeps crashing**: Pod enters `CrashLoopBackOff`.
5. **ReplicaSet controller**: Sees desired 3, but one is unhealthy.
6. **If Pod is terminated**: ReplicaSet creates a replacement Pod.
7. **Scheduler assigns new Pod** to a node (possibly different than before).
8. **kubelet starts new container**: Service restored.

All of this happens without human intervention. The reconciliation loop handles it.

---

## Core Terminology Summary

**Cluster**: A set of machines (nodes) running Kubernetes.

**Control Plane**: The components that manage the cluster (API server, scheduler, etcd, controller manager).

**Worker Node**: A machine that runs your containers.

**Pod**: The smallest deployable unit. Contains one or more containers.

**Deployment**: A higher-level resource that manages Pods through ReplicaSets.

**Desired State**: What you specify in YAML manifests.

**Observed State**: What actually exists in the cluster right now.

**Reconciliation**: The process of detecting mismatches and taking action.

**Controller**: A loop that watches resources and ensures desired state matches observed state.

---

## Mental Model: Kubernetes as an Operating System

Think of Kubernetes as an operating system for containers:

| Traditional OS | Kubernetes |
|---------------|-----------|
| Manages one machine | Manages many machines |
| Schedules processes | Schedules containers |
| Process dies → restart | Container dies → restart + reschedule |
| Networking between processes | Networking between containers |
| File systems | Persistent volumes |
| Environment variables | ConfigMaps and Secrets |

The key difference: a traditional OS manages one machine. Kubernetes manages a fleet of machines as a single system.

---

## What You've Learned

Before touching any commands, you now understand:

- **Why orchestration exists**: Docker alone can't handle multi-machine deployment, self-healing, scaling, or service discovery.
- **The declarative model**: You describe WHAT you want; Kubernetes figures out HOW.
- **Desired vs observed state**: Kubernetes continuously compares and reconciles.
- **Control plane components**: API server (entry point), etcd (state store), scheduler (placement), controller manager (reconciliation).
- **Worker node components**: kubelet (runs Pods), kube-proxy (networking), container runtime (runs containers).
- **The reconciliation loop**: The heartbeat that makes Kubernetes self-healing.

In Lesson 2, you'll set up Minikube—a local Kubernetes cluster—and run your first kubectl commands.

---

## Try With AI

Before moving to hands-on work, solidify your understanding with these prompts:

**Prompt 1: Test your declarative model understanding**

Ask AI: "I have a Kubernetes Deployment with replicas: 3. One Pod crashes and gets stuck in CrashLoopBackOff. Explain what each Kubernetes component does in response."

Expected answer should mention: kubelet detects crash, restarts container, ReplicaSet controller monitors Pod status, scheduler not involved unless Pod is fully terminated.

**Prompt 2: Control plane vs worker nodes**

Ask AI: "If I have 3 worker nodes and the control plane node goes down, what happens to my running containers? What can't I do anymore?"

Expected answer: Running containers continue (kubelet keeps them alive), but you can't deploy new workloads, scale, or make changes (no API server).

**Prompt 3: Why etcd matters**

Ask AI: "If etcd loses all its data in a Kubernetes cluster, what happens? What's lost?"

Expected answer: Cluster state is lost—all Deployments, Services, ConfigMaps, Secrets. Running containers might continue but can't be managed. This is why etcd backups are critical.

**Prompt 4: Reconciliation in practice**

Ask AI: "I change my Deployment from replicas: 3 to replicas: 5. Walk me through exactly which controllers notice and what actions they take."

Expected answer should trace: Deployment controller creates/updates ReplicaSet, ReplicaSet controller sees 3 Pods but wants 5, creates 2 Pod objects, scheduler assigns them, kubelets start containers.

**Prompt 5: Kubernetes vs Docker**

Ask AI: "My teammate says 'just use Docker Compose for production.' What's missing compared to Kubernetes?"

Expected answer: Multi-machine orchestration, automatic failover across nodes, rolling updates across a fleet, service discovery at scale, built-in load balancing.
