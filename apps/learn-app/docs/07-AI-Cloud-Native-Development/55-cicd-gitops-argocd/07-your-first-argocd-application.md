---
sidebar_position: 7
title: "Your First ArgoCD Application"
description: "Create an Application CRD that syncs a Helm chart from Git to your cluster"
keywords: [argocd, application, crd, helm, sync, gitops, kubernetes]
---

# Your First ArgoCD Application

You've installed ArgoCD and understand its architecture. Now comes the core GitOps practice: creating an `Application` resource that tells ArgoCD what to sync, where to sync it, and how often to check for changes.

An Application is a Kubernetes Custom Resource Definition (CRD) that acts as a contract between Git and your cluster. When you create an Application, you're saying: "Watch this Git repository for changes. When something changes, apply it to my cluster."

In this lesson, you'll:

1. **Understand** the Application CRD structure (source, destination, syncPolicy)
2. **Create** an Application three ways (UI, CLI, declarative YAML)
3. **Watch** ArgoCD detect your manifests from Git
4. **Sync** the application to your cluster
5. **Monitor** sync status (OutOfSync, Synced) and health status (Healthy, Degraded)

By the end, you'll have your Part 6 FastAPI agent running on your cluster, managed by ArgoCD.

## The Application CRD: Your GitOps Contract

An Application resource looks like this:

```yaml
apiVersion: argoproj.io/v1alpha1
kind: Application
metadata:
  name: fastapi-agent
  namespace: argocd                          # Applications live in the argocd namespace
spec:
  project: default                            # Which AppProject controls this app (Lesson 11)

  source:
    repoURL: https://github.com/yourname/agent-helm-chart.git  # Git repository URL
    path: charts/fastapi-agent                # Path within repo containing manifests
    targetRevision: HEAD                      # Git ref to sync (HEAD, main, v1.0.0, etc.)
    helm:
      values: |                               # Helm values override
        replicas: 2
        image:
          tag: "1.0.0"

  destination:
    server: https://kubernetes.default.svc   # Target cluster URL
    namespace: production                     # Target namespace

  syncPolicy:
    automated:
      prune: true                             # Delete resources removed from Git
      selfHeal: true                          # Reconcile if cluster drifts from Git
    syncOptions:
    - CreateNamespace=true                    # Create namespace if it doesn't exist
```

Let's break down each section:

### Source: Where to Sync From

**repoURL**: The Git repository containing your manifests. This can be:
- Public HTTPS: `https://github.com/yourname/repo.git`
- SSH: `git@github.com:yourname/repo.git` (requires SSH key in ArgoCD)
- Private HTTPS with credentials (Lesson 14 covers secrets)

**path**: The directory within the repository containing Kubernetes manifests or Helm charts. Examples:
- `k8s/` for plain YAML manifests
- `charts/my-app` for a Helm chart directory
- `.` for manifests in the root

**targetRevision**: Which Git ref to sync from:
- `HEAD` — Latest on the current branch
- `main` or `master` — Latest on main branch
- `v1.0.0` — A git tag for versioned releases
- `feature/new-config` — A specific branch

**helm** (optional): If syncing a Helm chart, override values without modifying Git:
```yaml
helm:
  values: |
    image:
      tag: "latest"
    replicas: 3
  parameters:
  - name: service.type
    value: LoadBalancer
```

### Destination: Where to Sync To

**server**: The Kubernetes API server URL:
- `https://kubernetes.default.svc` — Current cluster (where ArgoCD is running)
- `https://other-cluster-api.example.com:6443` — Remote cluster

**namespace**: The target namespace. ArgoCD creates it if it doesn't exist (with `CreateNamespace=true`).

### Sync Policy: How to Sync

**Manual sync** (default):
```yaml
syncPolicy: {}  # Don't sync automatically
```
You click "Sync" manually in the UI or run `argocd app sync fastapi-agent` in the CLI.

**Automated sync** (this lesson's approach):
```yaml
syncPolicy:
  automated:
    prune: true      # Delete resources removed from Git
    selfHeal: true   # Fix cluster drift
```

**prune**: If you remove a resource from Git, ArgoCD deletes it from the cluster.

**selfHeal**: If someone manually edits a resource in the cluster (or it crashes), ArgoCD detects the drift and reapplies the Git state.

## Create an Application via the UI

The ArgoCD UI provides a visual way to create applications. This is useful for learning and one-off applications, but production typically uses declarative YAML (GitOps).

### Step 1: Access the UI

Make sure you have port-forwarding running (from Lesson 6):

```bash
kubectl port-forward svc/argocd-server -n argocd 8080:443
```

**Output:**
```
Forwarding from 127.0.0.1:8080 -> 8080
```

Navigate to `https://localhost:8080` and log in.

### Step 2: Click "Create Application"

You'll see an empty Applications list. Click the "+ NEW APP" button in the top right.

A form appears with sections:

1. **GENERAL** — Application name, project
2. **SOURCE** — Git repository details
3. **DESTINATION** — Cluster and namespace
4. **SYNC POLICY** — Manual or automated

### Step 3: Fill in GENERAL

- **Application Name**: `fastapi-agent`
- **Project**: `default` (you'll learn AppProject in Lesson 11)
- **Sync Policy**: Choose `Automatic` (you can change later)

### Step 4: Fill in SOURCE

For this lesson, we'll use a public Helm chart as an example. In practice, you'd use your own repository.

- **Repository URL**: `https://github.com/panaversity/fastapi-agent-helm.git` (public example repo)
- **Revision**: `HEAD`
- **Path**: `charts/fastapi-agent`

If your repository is private, you'll configure Git credentials in Lesson 14.

### Step 5: Fill in DESTINATION

- **Cluster**: `https://kubernetes.default.svc` (your local cluster)
- **Namespace**: `production` (ArgoCD creates it with `CreateNamespace=true`)

### Step 6: Configure SYNC POLICY

In the SYNC POLICY section, you'll see checkboxes:

- **Prune Resources**: Check this (delete removed resources)
- **Self Heal**: Check this (fix drift)

Then click "CREATE".

ArgoCD now watches your repository. The Application status starts as **OutOfSync** because the cluster doesn't have the manifests yet.

### Step 7: Trigger the First Sync

Click the Application card to view details. You'll see:

- **Status**: `OutOfSync` (Git has manifests, cluster doesn't)
- **Health**: `Unknown` (no resources deployed yet)

Click "SYNC" in the top right. A dialog appears:

- **Revision**: `HEAD` (sync the latest)
- **Namespace**: `production`

Click "SYNCHRONIZE".

ArgoCD now:
1. Fetches the Git repository
2. Renders the Helm chart
3. Applies manifests to your cluster
4. Updates status to `Synced`

Watch the UI update. After 30 seconds, the **Status** changes to `Synced` and **Health** updates to `Healthy` (or `Progressing` if pods are still starting).

## Create an Application via CLI

The CLI approach is faster for experts and enables scripting. You'll use `argocd app create`.

### Step 1: Authenticate the CLI

If you haven't already, authenticate:

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

### Step 2: Create the Application

```bash
argocd app create fastapi-agent-cli \
  --repo https://github.com/panaversity/fastapi-agent-helm.git \
  --path charts/fastapi-agent \
  --dest-server https://kubernetes.default.svc \
  --dest-namespace production \
  --auto-prune \
  --self-heal
```

**Output:**
```
application 'fastapi-agent-cli' created
```

The `--auto-prune` and `--self-heal` flags enable automated sync. Without them, sync would be manual.

### Step 3: Sync the Application

```bash
argocd app sync fastapi-agent-cli
```

**Output:**
```
TIMESTAMP PROGRESS NAME                                  HEALTH STATUS
2025-12-23 syncing fastapi-agent-cli                                  SYNCING
2025-12-23 synced  fastapi-agent-cli                                  SYNCED
```

### Step 4: Check Status

```bash
argocd app get fastapi-agent-cli
```

**Output:**
```
Name:               fastapi-agent-cli
Namespace:          argocd
Project:            default
Server:             https://kubernetes.default.svc
Namespace:          production
URL:                https://localhost:8080/applications/fastapi-agent-cli
Repo:               https://github.com/panaversity/fastapi-agent-helm.git
Target:             HEAD
Path:               charts/fastapi-agent
Sync Policy:        Automated (Prune: true, SelfHeal: true)
Sync Status:        Synced
Health Status:      Healthy

NAMESPACE     NAME                                    REVISION   TYPE    HEALTH   STATUS
production    fastapi-agent-cli                       1.0.0      Helm    Healthy  Synced
```

The Application is now synced and healthy.

## Create an Application via Declarative YAML

This is the GitOps way: define your Application as YAML in a Git repository, then apply it with `kubectl`.

### Step 1: Write the Application Manifest

Create a file `argocd/fastapi-agent.yaml`:

```yaml
---
apiVersion: argoproj.io/v1alpha1
kind: Application
metadata:
  name: fastapi-agent
  namespace: argocd
spec:
  project: default

  source:
    repoURL: https://github.com/panaversity/fastapi-agent-helm.git
    path: charts/fastapi-agent
    targetRevision: HEAD
    helm:
      values: |
        image:
          tag: "1.0.0"
        replicas: 2
        service:
          type: ClusterIP
          port: 8000

  destination:
    server: https://kubernetes.default.svc
    namespace: production

  syncPolicy:
    automated:
      prune: true
      selfHeal: true
    syncOptions:
    - CreateNamespace=true
```

### Step 2: Apply the Manifest

```bash
kubectl apply -f argocd/fastapi-agent.yaml
```

**Output:**
```
application.argoproj.io/fastapi-agent created
```

Kubernetes creates the Application resource. The ArgoCD Application Controller detects it and immediately starts syncing.

### Step 3: Watch the Sync Progress

```bash
kubectl get application -n argocd
```

**Output:**
```
NAME              SYNC STATUS  HEALTH STATUS  AGE
fastapi-agent     Syncing      Progressing    5s
```

Wait 10-20 seconds:

```bash
kubectl get application -n argocd
```

**Output:**
```
NAME              SYNC STATUS  HEALTH STATUS  AGE
fastapi-agent     Synced       Healthy        20s
```

### Step 4: View Detailed Status

```bash
kubectl describe application fastapi-agent -n argocd
```

**Output:**
```
Name:         fastapi-agent
Namespace:    argocd
Labels:       <none>
Annotations:  <none>
API Version:  argoproj.io/v1alpha1
Kind:         Application

Spec:
  Destination:
    Namespace:  production
    Server:     https://kubernetes.default.svc
  Project:      default
  Source:
    Path:               charts/fastapi-agent
    Repo Url:           https://github.com/panaversity/fastapi-agent-helm.git
    Target Revision:    HEAD
  Sync Policy:
    Automated:
      Prune:      true
      Self Heal:  true

Status:
  Conditions:
  - Last Transition Time:  2025-12-23T10:45:00Z
    Message:               Sync operation completed successfully
    Type:                  Synced
  Health:
    Status:  Healthy
  Resources:
  - Health:
      Status:  Healthy
    Kind:      Deployment
    Name:      fastapi-agent
    Namespace: production
    Status:    Synced
  Sync:
    Status:  Synced
```

Your Application is now synced and healthy, managed entirely by GitOps.

## Understanding Sync Status

When ArgoCD compares Git with the cluster, it produces a **Sync Status**.

### OutOfSync

**What it means**: Git has resources that the cluster doesn't have, or Git has removed resources the cluster still has.

**Why it happens**:
- You just created the Application (first sync not run yet)
- You pushed new manifests to Git
- You removed manifests from Git (and prune is disabled)

**How to fix**: Click "Sync" in the UI or run `argocd app sync <app-name>` in the CLI.

**Example**:
```bash
argocd app get fastapi-agent
```

**Output**:
```
Sync Status: OutOfSync
```

### Syncing

**What it means**: ArgoCD is actively applying changes to the cluster.

**Why it happens**:
- You just clicked "Sync"
- Auto-sync detected a Git change and is reconciling

**How to monitor**: Watch the UI or run:
```bash
argocd app get fastapi-agent --refresh
```

### Synced

**What it means**: The cluster state matches the Git state. Everything is in sync.

**Why it's good**: Your cluster is exactly as defined in Git. You have a single source of truth.

**Example**:
```bash
argocd app get fastapi-agent
```

**Output**:
```
Sync Status: Synced
```

## Understanding Health Status

**Sync Status** tells you if Git and cluster match. **Health Status** tells you if the deployed resources are actually working.

### Healthy

**What it means**: All resources are running and reporting healthy. Deployments have desired replicas running, Services have endpoints, StatefulSets are ready.

**Example**:
```bash
argocd app get fastapi-agent
```

**Output**:
```
Health Status: Healthy
```

### Progressing

**What it means**: Resources are deploying. Pods are starting, but not all replicas are ready yet.

**Why it happens**:
- New Deployment was just synced
- Pods are pulling images
- Pods are initializing

**How to monitor**: This usually resolves within 30-60 seconds. If it stays `Progressing` for more than 5 minutes, check pod logs:

```bash
kubectl logs -n production -l app=fastapi-agent
```

### Degraded

**What it means**: Resources are deployed but not functioning. Pods are CrashLooping, Services have no endpoints, or Deployments have failed replicas.

**Why it happens**:
- Image doesn't exist or failed to pull
- Container is crashing
- Liveness probe is failing
- Database connection is broken

**How to fix**: Check pod logs and events:

```bash
kubectl describe pod -n production $(kubectl get pod -n production -l app=fastapi-agent -o jsonpath='{.items[0].metadata.name}')
```

### Unknown

**What it means**: ArgoCD hasn't checked health yet, or it can't determine health.

**Why it happens**:
- Application just created (first health check is 3-5 seconds away)
- Custom resources without health rules
- Network issues between ArgoCD and cluster

**How to fix**: Wait 10 seconds and refresh. If it stays `Unknown`, check ArgoCD logs:

```bash
kubectl logs -n argocd -l app.kubernetes.io/name=argocd-server --tail=20
```

### Missing

**What it means**: A resource defined in Git doesn't exist in the cluster.

**Why it happens**:
- Sync failed
- Prune is disabled and you removed the manifest from Git
- RBAC permissions prevent ArgoCD from creating the resource

**How to fix**: Trigger a sync or check ArgoCD logs for RBAC errors.

## Inspecting the Synced Resources

After your Application is synced, verify the actual resources were created:

```bash
kubectl get all -n production
```

**Output**:
```
NAME                                    READY   STATUS    RESTARTS   AGE
pod/fastapi-agent-5d7f9c8b4c-k2m9x     1/1     Running   0          45s
pod/fastapi-agent-5d7f9c8b4c-n3p8w     1/1     Running   0          45s

NAME                       TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)          AGE
service/fastapi-agent      ClusterIP   10.96.123.45    <none>        8000/TCP         45s

NAME                               READY   UP-TO-DATE   AVAILABLE   AGE
deployment.apps/fastapi-agent      2/2     2            2           45s

NAME                                          DESIRED   CURRENT   READY   AGE
replicaset.apps/fastapi-agent-5d7f9c8b4c     2         2         2       45s
```

These resources came from the Helm chart that ArgoCD rendered and applied. You can verify this by comparing with Git:

```bash
helm template fastapi-agent \
  https://github.com/panaversity/fastapi-agent-helm.git/charts/fastapi-agent \
  --values values.yaml
```

This shows you exactly what ArgoCD rendered before applying.

## Application Lifecycle: From Git Change to Cluster

Here's the complete flow when you push changes to Git:

### 1. Developer Pushes to Git

```bash
git add k8s/deployment.yaml
git commit -m "Bump image tag to 1.0.1"
git push origin main
```

### 2. ArgoCD Detects the Change

The Repository Server fetches your repo every 3 seconds (default interval, configurable). When it detects a new commit, the Application status changes to **OutOfSync**.

### 3. Auto-Sync (If Enabled)

If you have `automated: { enabled: true }` in syncPolicy, ArgoCD immediately syncs:
- Renders the Helm chart
- Compares rendered output with current cluster state
- Applies changes

### 4. Sync Status Updates

**Syncing** → **Synced** as changes are applied.

### 5. Health Check

ArgoCD checks if all resources are healthy (Deployments have desired replicas, Services have endpoints, etc.).

**Progressing** → **Healthy** (assuming no errors).

### 6. Your Service is Updated

Users see the new version. If something breaks, you revert the Git commit and push again. Rollback is just `git revert <commit>`.

## Customizing Source Configuration

### Source Type: Plain YAML

If your repository contains plain Kubernetes YAML files (no Helm, no Kustomize):

```yaml
source:
  repoURL: https://github.com/yourname/manifests.git
  path: k8s/
  targetRevision: main
```

ArgoCD uses the YAML as-is. No templating.

### Source Type: Helm Chart

If you're syncing a Helm chart, specify the chart values:

```yaml
source:
  repoURL: https://github.com/yourname/charts.git
  path: charts/fastapi-agent
  targetRevision: v1.0.0
  helm:
    values: |
      image:
        repository: myregistry/agent
        tag: "1.0.1"
      replicas: 3
    parameters:
    - name: service.type
      value: LoadBalancer
```

ArgoCD runs `helm template` with these values before applying.

### Source Type: Kustomize

If your manifests use Kustomize:

```yaml
source:
  repoURL: https://github.com/yourname/kustomize.git
  path: overlays/production
  targetRevision: main
  kustomize:
    images:
    - name: fastapi-agent
      newTag: "1.0.1"
```

ArgoCD runs `kustomize build` before applying.

## Customizing Destination Configuration

### Single Cluster (This Lesson)

```yaml
destination:
  server: https://kubernetes.default.svc  # Current cluster
  namespace: production
```

### Multiple Clusters (Lesson 15)

You can register multiple clusters and deploy to them:

```yaml
destinations:
- server: https://cluster1.example.com
  namespace: production
- server: https://cluster2.example.com
  namespace: production
```

ArgoCD then syncs the same manifests to multiple clusters.

## What You've Accomplished

You now understand and have practiced:

1. **Application CRD anatomy** — source, destination, syncPolicy
2. **Three ways to create applications** — UI (visual), CLI (scripting), declarative YAML (GitOps)
3. **Sync status** — OutOfSync, Syncing, Synced
4. **Health status** — Healthy, Progressing, Degraded, Unknown, Missing
5. **GitOps workflow** — Git is source of truth, ArgoCD enforces it
6. **Automated sync** — ArgoCD detects changes and syncs automatically (with auto-prune and self-heal)

In Lesson 8, you'll master sync strategies: controlling the order resources sync in, running hooks before/after sync, and handling advanced scenarios.

## Try With AI

Ask Claude: "I've created an ArgoCD Application that syncs a Helm chart. The chart includes a Deployment, Service, and ConfigMap. After I pushed a new image tag to Git, the Deployment updated but the Pods are stuck in ImagePullBackOff. How would I diagnose this?"

Before accepting Claude's response, verify it suggests:
- Checking pod logs with `kubectl logs` in the production namespace
- Using `kubectl describe pod` to see the ImagePullBackOff event
- Verifying the image repository and tag are correct in the Helm values
- Checking if image pull secrets are configured (Lesson 14)

Then ask: "The image is correct in the registry. But I pushed to my private registry and ArgoCD doesn't have credentials. How do I give ArgoCD access to pull from my private registry?"

Notice how Claude's response guides you toward the Repository credentials section (Lesson 14 topic), helping you understand that Applications depend on ArgoCD's ability to authenticate with registries and repositories.
