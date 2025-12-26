---
sidebar_position: 6
title: "ArgoCD Architecture & Installation"
description: "Install ArgoCD 3.x on Minikube and understand its component architecture"
keywords: [argocd, installation, helm, minikube, architecture, kubernetes, gitops]
---

# ArgoCD Architecture & Installation

You've learned GitOps principles in Lesson 5. Now it's time to install the controller that enforces those principles: ArgoCD. ArgoCD is a declarative, GitOps continuous delivery tool that reconciles your Git repository with your Kubernetes cluster.

The journey is straightforward:

1. **Understand** the architecture (what components work together)
2. **Install** ArgoCD on your Minikube cluster using Helm
3. **Access** the UI and CLI
4. **Verify** the installation works

By the end of this lesson, you'll have a running ArgoCD instance that watches a Git repository and automatically syncs changes to your cluster.

## The Problem ArgoCD Solves

Before ArgoCD, your deployment workflow looked like this:

- Developer pushes code
- CI pipeline builds and tests
- You manually run `kubectl apply -f deployment.yaml`
- You manually check if the deployment succeeded
- If something broke, you manually rolled back

With ArgoCD:

- Developer pushes manifests to Git
- ArgoCD watches that Git repository
- ArgoCD **automatically** detects changes
- ArgoCD reconciles the cluster to match Git
- If the cluster drifts from Git, ArgoCD corrects it
- Rollback means reverting a Git commit

Git becomes your source of truth. ArgoCD becomes the enforcement mechanism.

## ArgoCD Architecture: Five Core Components

When you install ArgoCD, you get five components working together. Understanding each one helps you troubleshoot when something breaks.

### 1. API Server

**What it does**: Exposes the REST API that the UI and CLI communicate with

**Port**: 8080 (internal), exposed to 443 externally

**Example interaction**: When you click "Sync" in the UI, the UI sends a request to the API Server. The API Server validates the request and instructs the Application Controller to reconcile.

### 2. Repository Server

**What it does**: Clones your Git repository and generates Kubernetes manifests

**Why separate**: Cloning repositories and rendering templates is resource-intensive. Keeping it in a separate pod lets it scale independently.

**Flow**:
- ArgoCD fetches your Git repo
- Repo Server runs `helm template`, `kustomize build`, or just uses plain YAML
- Repo Server returns rendered manifests to the Application Controller

### 3. Application Controller

**What it does**: The reconciliation loop—compares Git state with cluster state and syncs when they differ

**Core logic**:
```
Loop every 3 seconds (default):
  1. Get target state from Git (via Repo Server)
  2. Get current state from cluster (via kubectl)
  3. Compare them
  4. If different, apply changes
  5. Health check resources
```

### 4. Redis

**What it does**: Cache for Git state and cluster state

**Why it matters**: Without caching, ArgoCD would fetch Git and query the cluster constantly. Redis stores intermediate data so reconciliation is fast.

### 5. Dex

**What it does**: OIDC provider for authentication (optional, but configured by default)

**When you use it**: When you integrate ArgoCD with GitHub, GitLab, or another OAuth provider for multi-user access

**For now**: You'll skip OAuth and just use the built-in admin user.

## ArgoCD Custom Resource Definitions (CRDs)

ArgoCD introduces three new Kubernetes resource types you'll use constantly:

### Application

Defines what to sync and where. Example:

```yaml
apiVersion: argoproj.io/v1alpha1
kind: Application
metadata:
  name: my-agent
  namespace: argocd
spec:
  project: default
  source:
    repoURL: https://github.com/yourname/your-agent.git
    path: k8s/  # Manifests are in k8s/ directory
    targetRevision: HEAD
  destination:
    server: https://kubernetes.default.svc  # Current cluster
    namespace: default
  syncPolicy:
    automated:
      prune: true
      selfHeal: true
```

**What this means**: "Watch the `k8s/` directory in my GitHub repo. If anything changes, apply those manifests to my cluster. If the cluster drifts, fix it."

### AppProject

Defines access control and constraints. You'll use this in Lesson 11 (RBAC), but know it exists. For now, all applications use the `default` project.

### ApplicationSet

Scales applications across multiple clusters or environments. You'll learn this in Lesson 10. For now, know that one `Application` = one repo + one cluster. `ApplicationSet` = one template applied to many clusters.

## Installing ArgoCD with Helm

You'll install ArgoCD 3.x using Helm (from Chapter 51). Make sure your Minikube cluster is running first.

### Step 1: Create the ArgoCD Namespace

```bash
kubectl create namespace argocd
```

**Output:**
```
namespace/argocd created
```

### Step 2: Add the ArgoCD Helm Repository

```bash
helm repo add argo https://argoproj.github.io/argo-helm
helm repo update
```

**Output:**
```
"argo" has been added to your repositories
Hang tight while we grab the latest from your chart repositories...
...Successfully got an update from the "argo" chart repository
```

### Step 3: Install ArgoCD Using Helm

```bash
helm install argocd argo/argo-cd \
  --namespace argocd \
  --version 7.7.0 \
  --set server.service.type=LoadBalancer \
  --set redis.enabled=true
```

**Output:**
```
NAME: argocd
LAST DEPLOYED: Tue Dec 23 10:30:00 2025
NAMESPACE: argocd
STATUS: deployed
REVISION: 1
NOTES:
1. Get your 'admin' user password by running:
   kubectl -n argocd get secret argocd-initial-admin-secret -o jsonpath="{.data.password}" | base64 -d; echo
```

Wait for all pods to be ready (this takes 30-60 seconds):

```bash
kubectl get pods -n argocd --watch
```

**Output:**
```
NAME                                              READY   STATUS    RESTARTS   AGE
argocd-application-controller-0                   1/1     Running   0          45s
argocd-dex-server-5c8b4f7d6c-8vx9n                1/1     Running   0          45s
argocd-redis-master-0                             1/1     Running   0          45s
argocd-repo-server-68b4f9c8d5-4kxh2               1/1     Running   0          45s
argocd-server-7d8f9e6c4b-2x5h3                    1/1     Running   0          45s
argocd-metrics-server-5b7c4d6e3f-9k2x8            1/1     Running   0          45s
```

Press `Ctrl+C` when all pods show `1/1 Running`.

## Accessing the ArgoCD UI

### Step 1: Port-Forward to the ArgoCD Server

```bash
kubectl port-forward svc/argocd-server -n argocd 8080:443
```

**Output:**
```
Forwarding from 127.0.0.1:8080 -> 8080
Forwarding from [::1]:8080 -> 8080
```

Leave this running in a terminal tab.

### Step 2: Get the Admin Password

In a new terminal:

```bash
kubectl -n argocd get secret argocd-initial-admin-secret -o jsonpath="{.data.password}" | base64 -d; echo
```

**Output:**
```
aBcDeF1234GhIjKl5678MnOpQrStUvWxYz
```

Your actual password will be different.

### Step 3: Open the UI

Navigate to `https://localhost:8080` in your browser.

**What you'll see**:
- A login screen
- Username: `admin`
- Password: (the value from Step 2)

The browser will warn "This connection is not private" (because it's a self-signed certificate). Click "Advanced" and "Proceed to localhost:8080".

After logging in, you'll see:

- **Welcome page** showing "Applications" (currently empty)
- **Navigation menu** on the left: Applications, Repositories, Settings
- **User menu** in the top right (your user icon)

## Installing the ArgoCD CLI

The CLI lets you manage ArgoCD from the command line. You can sync applications, create new applications, and view status without opening the browser.

### Step 1: Install the CLI

::::os-tabs

::macos
```bash
brew install argocd
```

**Output:**
```
==> Downloading https://github.com/argoproj/argo-cd/releases/download/v3.2.2/argocd-darwin-amd64
==> Pouring argocd-3.2.2.arm64_bottle.tar.gz
🍺  /opt/homebrew/Cellar/argocd/3.2.2: 8 files, 125.5MB
```

::linux
```bash
curl -sSL -o /usr/local/bin/argocd https://github.com/argoproj/argo-cd/releases/download/v3.5.0/argocd-linux-amd64
chmod +x /usr/local/bin/argocd
```

**Output:**
```
(no output on success)
```

::windows
```powershell
# Download from GitHub releases
Invoke-WebRequest -Uri "https://github.com/argoproj/argo-cd/releases/download/v3.5.0/argocd-windows-amd64.exe" -OutFile argocd.exe
Move-Item argocd.exe C:\Windows\System32\argocd.exe
```

Or using Chocolatey:
```powershell
choco install argocd-cli
```

::::

Verify the installation:

```bash
argocd version
```

**Output:**
```
argocd: v3.5.0+unknown
argocd: v3.5.0+unknown
```

### Step 2: Authenticate the CLI

You need to tell the CLI how to connect to your ArgoCD server.

```bash
argocd login localhost:8080 \
  --username admin \
  --password $(kubectl -n argocd get secret argocd-initial-admin-secret -o jsonpath="{.data.password}" | base64 -d) \
  --insecure
```

**Output:**
```
'admin' logged in successfully
```

The `--insecure` flag tells the CLI to skip certificate verification (safe for localhost).

Now test that the CLI works:

```bash
argocd app list
```

**Output:**
```
NAME  CLUSTER                         NAMESPACE  PROJECT  STATUS     HEALTH   SYNCPOLICY  CONDITIONS  AGE
```

The output is empty (no applications yet), which is expected.

## Verifying the Installation

You now have a working ArgoCD instance. Let's verify all components are healthy.

### Check Pod Status

```bash
kubectl get pods -n argocd
```

**Output:**
```
NAME                                              READY   STATUS    RESTARTS   AGE
argocd-application-controller-0                   1/1     Running   0          2m
argocd-dex-server-5c8b4f7d6c-8vx9n                1/1     Running   0          2m
argocd-redis-master-0                             1/1     Running   0          2m
argocd-repo-server-68b4f9c8d5-4kxh2               1/1     Running   0          2m
argocd-server-7d8f9e6c4b-2x5h3                    1/1     Running   0          2m
argocd-metrics-server-5b7c4d6e3f-9k2x8            1/1     Running   0          2m
```

All should show `1/1` ready and `Running` status.

### Check the API Server

```bash
kubectl logs -n argocd -l app.kubernetes.io/name=argocd-server --tail=5
```

**Output:**
```
2025-12-23 10:35:22 INFO: Starting ArgoCD Server v3.5.0
2025-12-23 10:35:22 INFO: Loaded 1 app project(s)
2025-12-23 10:35:23 INFO: Server started listening on :8080
2025-12-23 10:35:25 INFO: TLS listener starting
```

The API Server is running if you see "Server started listening".

### Check the Application Controller

```bash
kubectl logs -n argocd -l app.kubernetes.io/name=application-controller --tail=5
```

**Output:**
```
2025-12-23 10:35:30 INFO: Application controller started
2025-12-23 10:35:30 INFO: Git poller interval: 3s
2025-12-23 10:35:30 INFO: Cluster polling interval: 5s
2025-12-23 10:35:30 INFO: Application reconciliation loop running
```

The controller is reconciling if you see the reconciliation loop message.

## What You've Accomplished

You now have:

1. **A running ArgoCD instance** with all five components deployed
2. **A working UI** that you can access at localhost:8080
3. **A working CLI** that can query and manage applications
4. **Understanding of the architecture** (API Server, Repo Server, Controller, Redis, Dex)
5. **Knowledge of the CRDs** that define applications, projects, and ApplicationSets

In Lesson 7, you'll create your first `Application` resource that points to a GitHub repository. ArgoCD will watch that repository and sync changes to your cluster automatically.

## Try With AI

Ask Claude: "I'm trying to set up ArgoCD on my Minikube cluster, but the argocd-repo-server pod keeps crashing with 'OOMKilled'. My Minikube has 4GB of RAM. What should I do?"

When Claude responds, it should suggest:

- Checking pod logs with `kubectl logs -n argocd -l app.kubernetes.io/name=repo-server`
- Checking resource requests with `kubectl describe pod -n argocd <pod-name>`
- Adjusting Helm values to lower memory requests: `--set repoServer.resources.requests.memory=256Mi`
- Potentially increasing Minikube memory with `minikube delete && minikube start --memory=8192`

Ask a follow-up: "I see the repo-server pod requesting 1Gi of memory. Can I safely lower this to 512Mi if I'm only deploying small applications?"

Use Claude's response to understand which components are resource-sensitive and when it's safe to optimize.
