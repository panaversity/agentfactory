---
sidebar_position: 2
chapter: 50
lesson: 2
duration_minutes: 25
title: "Setting Up Minikube"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Hands-on installation builds familiarity with kubectl"
cognitive_load:
  concepts_count: 6
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Install Minikube on macOS, Windows, or Linux"
    bloom_level: "Apply"
  - id: LO2
    description: "Start a local Kubernetes cluster with minikube start"
    bloom_level: "Apply"
  - id: LO3
    description: "Verify cluster health with kubectl commands"
    bloom_level: "Apply"
  - id: LO4
    description: "Access the Kubernetes dashboard"
    bloom_level: "Apply"
  - id: LO5
    description: "Understand kubectl context and kubeconfig"
    bloom_level: "Understand"
  - id: LO6
    description: "Troubleshoot common Minikube startup issues"
    bloom_level: "Apply"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO2
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO3
    competency_area: "5. Problem Solving"
    competency: "5.3 Using digital tools to solve problems"
  - objective_id: LO4
    competency_area: "1. Information and Data Literacy"
    competency: "1.2 Understanding digital concepts and terminology"
  - objective_id: LO5
    competency_area: "1. Information and Data Literacy"
    competency: "1.2 Understanding digital concepts and terminology"
  - objective_id: LO6
    competency_area: "5. Problem Solving"
    competency: "5.2 Identifying needs and technological responses"
---

# Setting Up Minikube

Your Docker agent from Chapter 49 is containerized and ready. But a single container running on your laptop isn't production—it's one process, one crash away from total failure. Production systems need orchestration: multiple containers, health checking, automatic restarts, rolling updates, resource management.

That's Kubernetes. But learning Kubernetes on a cloud cluster is expensive and overwhelming. Minikube is a local Kubernetes cluster that runs on your laptop, using the same API and concepts as production Kubernetes. By the end of this lesson, you'll have a working cluster running kubectl commands just like production engineers do—without any cloud bills.

---

## What Is Minikube?

Minikube is a lightweight Kubernetes distribution designed for learning and development. Here's what you need to know:

### Minikube vs Production Kubernetes

| Feature | Minikube | Production Kubernetes |
|---------|----------|----------------------|
| **Location** | Your laptop (VM or container) | Cloud or data center |
| **Nodes** | Single node (control plane + workers combined) | Multiple nodes (separate control planes and workers) |
| **Cost** | Free | Cloud compute bills |
| **Use Case** | Development, learning, local testing | Serving real users, handling failures |
| **API** | Identical Kubernetes API | Same API |
| **kubectl** | Exact same commands | Same commands |

**Key insight**: Minikube is NOT a toy. It's a real Kubernetes cluster with the same API as production. Everything you learn here transfers directly to cloud deployments. The only difference is scale—Minikube runs one node, production runs many.

### How Minikube Works

Minikube runs a single Kubernetes node inside a virtual machine or container on your laptop:

```
┌──────────────────────────────────────┐
│        Your Laptop (macOS/Win/Linux) │
├──────────────────────────────────────┤
│  ┌────────────────────────────────┐  │
│  │   Minikube VM (or Container)   │  │
│  ├────────────────────────────────┤  │
│  │  Kubernetes Control Plane      │  │
│  │  (API Server, Scheduler, etc)  │  │
│  ├────────────────────────────────┤  │
│  │  Kubernetes Worker Node        │  │
│  │  (kubelet, container runtime)  │  │
│  ├────────────────────────────────┤  │
│  │  Your Containers (Pods)        │  │
│  └────────────────────────────────┘  │
└──────────────────────────────────────┘
```

Everything you deploy to Minikube runs inside that single VM/container. Your laptop stays clean—Minikube handles all the Kubernetes infrastructure.

---

## Prerequisites

Before installing Minikube, you need:

1. **Docker or another container runtime** — Minikube needs something to run containers inside. Docker Desktop (macOS/Windows) or Docker Engine (Linux) is the easiest choice.
2. **kubectl** — The Kubernetes command-line tool. You'll install this alongside Minikube.
3. **2GB of free disk space** — Minikube VM/container is about 2GB
4. **4GB of RAM available** — Minikube default allocation is 2GB; you may want 4GB+ for AI workloads

Check what you have:

```bash
docker version
```

**Output** (if Docker is installed and running):
```
Client: Docker Desktop 25.0.1
Server: Docker Desktop 25.0.1
```

If Docker isn't running, start Docker Desktop before continuing.

---

## Installing Minikube

Minikube works on macOS, Windows, and Linux. Choose your platform:

### macOS

Using Homebrew (simplest):

```bash
brew install minikube
```

**Output**:
```
==> Downloading https://homebrew.bintray.com/bottles/minikube-1.32.0.big_sur.bottle.tar.gz
==> Pouring minikube-1.32.0.big_sur.bottle.tar.gz
🍺  /usr/local/Cellar/minikube/1.32.0: 8 files, 84MB
```

Verify installation:

```bash
minikube version
```

**Output**:
```
minikube version: v1.32.0
commit: 8220a6eb95f0a4d5f864e64e6ea061899a56c6a3
```

### Windows (PowerShell as Administrator)

Using winget (recommended):

```powershell
winget install minikube
```

**Output**:
```
Found Minikube [Minikube.Minikube] Version 1.32.0
Starting package install...
Successfully installed
```

Verify installation:

```powershell
minikube version
```

**Output**:
```
minikube version: v1.32.0
commit: 8220a6eb95f0a4d5f864e64e6ea061899a56c6a3
```

Alternatively, download from https://github.com/kubernetes/minikube/releases and add to your PATH.

### Linux

Download and install:

```bash
curl -LO https://storage.googleapis.com/minikube/releases/latest/minikube-linux-amd64
sudo install minikube-linux-amd64 /usr/local/bin/minikube
```

**Output**:
```
  % Total    % Received % Xferd  Average Speed   Time    Time     Time  Current
                                 Dload  Upload   Total   Spent    Left Speed
100   84.3M  100   84.3M    0     0  15.2M      0  --:-- --:-- --:--:-- 15.2M
```

Verify installation:

```bash
minikube version
```

**Output**:
```
minikube version: v1.32.0
commit: 8220a6eb95f0a4d5f864e64e6ea061899a56c6a3
```

### Installing kubectl

kubectl is the Kubernetes command-line tool. Minikube includes it, but you can also install it separately.

**macOS** (using Homebrew):

```bash
brew install kubectl
```

**Output**:
```
==> Downloading https://homebrew.bintray.com/bottles/kubectl-1.28.3.big_sur.bottle.tar.gz
==> Pouring kubectl-1.28.3.big_sur.bottle.tar.gz
🍺  /usr/local/Cellar/kubectl/1.28.3: 8 files, 224MB
```

**Windows** (PowerShell as Administrator):

```powershell
winget install kubectl
```

**Linux**:

```bash
curl -LO "https://dl.k8s.io/release/$(curl -L -s https://dl.k8s.io/release/stable.txt)/bin/linux/amd64/kubectl"
sudo install -o root -g root -m 0755 kubectl /usr/local/bin/kubectl
```

Verify kubectl is installed:

```bash
kubectl version --client
```

**Output**:
```
Client Version: v1.28.3
Kustomize Version: v5.0.4-0.20230601165947-6ce0bf390ce3
```

---

## Starting Your Kubernetes Cluster

Now that Minikube and kubectl are installed, start your cluster:

```bash
minikube start
```

**Output** (this takes 1-2 minutes):
```
😄  minikube v1.32.0 on Darwin 14.0
✨  Automatically selected the docker driver
📌  Using Docker Desktop driver with root privileges
👍  Starting control plane node minikube in cluster minikube
🚜  Pulling base image ...
🔥  Creating docker container (CPUs=2, Memory=4000MB) ...
🐳  Preparing Kubernetes v1.28.3 on Docker 24.0.7 ...
🔗  Configuring bridge CNI (Container Networking Interface) ...
🎓  Installing StorageClass addon ...
🔎  Verifying Kubernetes components...
🌟  Enabled addons: storage-provisioner, default-storageclass
🏄  Done! kubectl is now configured to use "minikube" cluster
```

**What just happened:**
- Minikube created a Docker container running Kubernetes v1.28.3
- It allocated 2 CPUs and 4GB of memory by default
- It configured networking (CNI) so your pods can communicate
- It enabled storage provisioning so you can create persistent volumes
- It configured kubectl to point to your local Minikube cluster

Your Kubernetes cluster is now running.

---

## Verifying Your Cluster

Verify that Kubernetes is running and healthy:

```bash
kubectl cluster-info
```

**Output**:
```
Kubernetes control plane is running at https://127.0.0.1:55000
CoreDNS is running at https://127.0.0.1:55000/api/v1/namespaces/kube-system/services/kube-dns:dns/proxy

To further debug and diagnose cluster problems, use 'kubectl cluster-info dump'.
```

This shows:
- **Control plane**: API server is running at 127.0.0.1:55000 (localhost)
- **CoreDNS**: Service discovery is working (pods can find each other by name)

Check the nodes in your cluster:

```bash
kubectl get nodes
```

**Output**:
```
NAME       STATUS   ROLES           AGE   VERSION
minikube   Ready    control-plane   2m    v1.28.3
```

This shows:
- **NAME**: Your single node is called "minikube"
- **STATUS**: Ready (healthy, accepting workloads)
- **ROLES**: control-plane (runs both control plane and worker responsibilities)
- **AGE**: 2 minutes since startup
- **VERSION**: Kubernetes v1.28.3

Get more detailed information about your node:

```bash
kubectl describe node minikube
```

**Output** (partial):
```
Name:               minikube
Roles:              control-plane
Status:             Ready
Capacity:
  cpu:              2
  memory:           3880Mi
  pods:             110
Allocatable:
  cpu:              2
  memory:           3358Mi
  pods:             110
```

This shows your node has:
- **2 CPUs** available (Minikube default)
- **3.8GB of memory** available
- Can run up to 110 pods (containers)

---

## Understanding kubectl Context and kubeconfig

kubectl needs to know which Kubernetes cluster to talk to. It stores this information in kubeconfig.

### What is kubeconfig?

kubeconfig is a file that tells kubectl how to connect to a Kubernetes cluster. It contains:
- **Cluster information** (API server URL, certificate authority)
- **User credentials** (how to authenticate)
- **Context** (which cluster + user to use by default)

### Where is kubeconfig?

kubeconfig lives at `~/.kube/config`:

```bash
cat ~/.kube/config
```

**Output** (partial):
```yaml
apiVersion: v1
clusters:
- cluster:
    certificate-authority: /Users/you/.minikube/ca.crt
    server: https://127.0.0.1:55000
  name: minikube
contexts:
- context:
    cluster: minikube
    user: minikube
  name: minikube
current-context: minikube
kind: Config
preferences: {}
users:
- name: minikube
  user:
    client-certificate: /Users/you/.minikube/client.crt
    client-key: /Users/you/.minikube/client.key
```

**Key parts**:
- **clusters.server**: `https://127.0.0.1:55000` (where your local cluster API server runs)
- **current-context**: `minikube` (which context kubectl uses by default)

### What is a context?

A context combines:
- **Cluster**: Which Kubernetes cluster to talk to
- **User**: What credentials to use
- **Namespace**: Which namespace to use (default is `default`)

See your current context:

```bash
kubectl config current-context
```

**Output**:
```
minikube
```

See all available contexts:

```bash
kubectl config get-contexts
```

**Output**:
```
CURRENT   NAME       CLUSTER    AUTHINFO   NAMESPACE
*         minikube   minikube   minikube   default
```

The `*` marks your current context (minikube).

If you later work with cloud clusters (GKE, EKS, AKS), you'll have multiple contexts. Switching between them is simple:

```bash
kubectl config use-context gke-cluster
```

---

## Accessing the Kubernetes Dashboard

Kubernetes provides a web-based dashboard for viewing cluster information. Open it:

```bash
minikube dashboard
```

**Output**:
```
🔌  Enabling dashboard ...
⏳  Verifying dashboard health ...
🚀  Launching proxy ...
🎉  Opening http://127.0.0.1:54321/api/v1/namespaces/kubernetes-dashboard/services/http:kubernetes-dashboard:/proxy/ in your default browser ...
```

This opens your browser to the Kubernetes Dashboard. You'll see:
- **Cluster Overview**: Node status, pod counts, resource usage
- **Workloads**: Deployments, StatefulSets, DaemonSets, Pods
- **Services & Ingress**: Networking configuration
- **Storage**: PersistentVolumes and PersistentVolumeClaims
- **Configuration**: ConfigMaps and Secrets

The dashboard is a GUI for everything you can do with kubectl commands. It's useful for learning and quick status checks, but kubectl is more powerful for automation.

---

## Useful Minikube Commands

### Check cluster status:

```bash
minikube status
```

**Output**:
```
minikube
type: Control Plane
host: Running
kubelet: Running
apiserver: Running
kubeconfig: Configured
```

### Stop the cluster (preserves state):

```bash
minikube stop
```

**Output**:
```
⏸️  Stopping node "minikube" ...
🛑  Powering off "minikube" via SSH ...
🛑  1 node stopped.
```

Your cluster is paused. Restart it later with `minikube start`. All your deployments and data persist.

### Delete the cluster (removes everything):

```bash
minikube delete
```

**Output**:
```
🔥  Deleting "minikube" cluster ...
🔥  Removing /Users/you/.minikube/profiles/minikube ...
✅  Successfully deleted profile "minikube"
```

This removes the Minikube VM/container completely. Use this when you want a clean slate.

### SSH into the Minikube node:

```bash
minikube ssh
```

**Output**:
```
                         _             _
            _         _( )( )_       _( )( )_
           (_)       (_    _  )_   _(_  _    )
             _   _    ( )_( )(_) (_   ( )_
            ( ) ( )   (  _  _  ) (  _ (  _
            |(_)|_)    (_( )(_))  (_) (_)
 __________________________GNU/Linux__________
```

You're now inside the Minikube VM (running Linux). You can inspect the node, check logs, etc. Type `exit` to return to your laptop.

---

## Troubleshooting Common Issues

### Issue: Docker not running

Error:
```
X  Exiting due to DRV_AS_ROOT: Docker is not running on this host. Please install and start Docker to continue: exec: "docker": executable file not found in PATH
```

Solution: Start Docker Desktop (macOS/Windows) or Docker Engine (Linux), then run `minikube start` again.

### Issue: Port conflicts

If Minikube fails to start with a port conflict error, specify different ports:

```bash
minikube start --ports=8080:80
```

This maps port 8080 on your laptop to port 80 inside Minikube.

### Issue: Insufficient memory

If you have limited RAM and Minikube fails, allocate less memory:

```bash
minikube start --memory=2048
```

This allocates only 2GB instead of the default 4GB. Adjust based on your available RAM.

### Issue: Slow startup

Minikube pulls a base image on first startup (takes time). On subsequent starts, it's much faster. If it hangs, check:

```bash
minikube logs
```

This shows detailed startup logs. If there's an error, the logs will help diagnose it.

---

## What You've Accomplished

You now have:

- ✅ Minikube installed and running on your laptop
- ✅ kubectl configured and communicating with your cluster
- ✅ A working single-node Kubernetes cluster (same API as production)
- ✅ Understanding of kubeconfig and kubectl contexts
- ✅ Access to the Kubernetes Dashboard
- ✅ Knowledge of basic Minikube commands

Your local Kubernetes cluster is ready. Next lesson, you'll deploy your first containers to this cluster using Kubernetes manifests.

---

## Try With AI

Now that your cluster is running, explore it with AI assistance.

### Setup

- **Tool**: kubectl + your running Minikube cluster
- **Context**: You can run any kubectl command. Ask questions about cluster concepts or kubectl syntax

### Prompts to Explore

**Prompt 1: Understand cluster internals**

Ask your AI: "Explain the difference between 'minikube stop' and 'minikube delete'. When would I use each?"

Expected answer: stop pauses and preserves state; delete removes everything and is useful for clean starts.

**Prompt 2: Resource configuration**

Ask your AI: "How do I increase the memory allocated to my Minikube cluster to 8GB?"

Expected answer: Use `minikube start --memory=8192` or reconfigure existing cluster with `minikube config set memory 8192`

**Prompt 3: kubectl context fundamentals**

Ask your AI: "Explain what a kubectl context is and why it matters. How would I switch to a different Kubernetes cluster?"

Expected answer: Context = cluster + user + namespace. Switch with `kubectl config use-context <context-name>`. Useful when managing multiple clusters (local + cloud).

**Prompt 4: Dashboard exploration**

Ask your AI: "I see my pods in the Kubernetes Dashboard. What does 'Ready' mean, and what does the 'Status' column tell me?"

Expected answer: Ready pods are healthy and handling traffic. Status shows pending (starting), running (active), or failed (crashed).

**Prompt 5: Cluster capacity**

Ask your AI: "I ran 'kubectl describe node minikube' and see 'Allocatable' resources. What's the difference between 'Capacity' and 'Allocatable'?"

Expected answer: Capacity is total resources. Allocatable is what's available for your workloads (some reserved for system components).

Experiment with different kubectl commands and ask your AI for explanations. The cluster is yours—you can't break anything by exploring.
