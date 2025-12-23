---
sidebar_position: 17
title: "Capstone: End-to-End Agent Pipeline"
description: "Build a complete CI/CD pipeline from code push to running agent using GitHub Actions and ArgoCD"
keywords: [capstone, ci/cd, github actions, argocd, cicd pipeline, fastapi, gitops, automation]
duration_minutes: 90
proficiency_level: "B1-B2"
layer: "L4"
---

# Capstone: End-to-End Agent Pipeline

From code commit to running agent. That's the entire journey this capstone walks you through.

You've built your FastAPI agent in Part 6. You've containerized it with Docker (Lesson 49). You've deployed it with Kubernetes (Chapter 50). You've managed it with Helm (Chapter 51). Now you'll automate the entire path: push code, trigger tests, build a container, push to a registry, detect the new image in Git, and sync it automatically to your cluster.

This capstone is **specification-first**. Before writing a single YAML file, you'll write the specification that defines what your pipeline does, when it triggers, and how it validates. Then you'll implement it using GitHub Actions (CI) and ArgoCD (CD). Finally, you'll demonstrate the complete flow: code push → deployment → validation → rollback.

This lesson produces a **fully automated, auditable pipeline** for your agent. Every deployment is version-controlled, every rollback is a git revert, and every step is automated.

## Pipeline Specification

### Intent

Deploy your FastAPI agent from code push to accessible endpoint with full automation:

- **Trigger**: Every push to main branch automatically starts the pipeline
- **CI Stage**: Run tests, build container image, push to registry with SHA tag
- **CD Stage**: ArgoCD detects new image, syncs automatically, health checks confirm service is ready
- **Validation**: Agent endpoint responds to requests within 30 seconds of deployment
- **Rollback**: Git revert triggers ArgoCD to deploy previous version

### Success Criteria

- ✅ Push to main triggers GitHub Actions within 30 seconds
- ✅ Tests pass with minimum 80% coverage before image build
- ✅ Container image pushed to registry in under 10 minutes
- ✅ ArgoCD detects new image and syncs within 3 minutes of registry push
- ✅ Agent endpoint returns HTTP 200 after deployment
- ✅ Rollback via `git revert` restores previous version within 5 minutes
- ✅ Complete flow logs are visible in GitHub Actions UI and ArgoCD UI

### Components

| Component | Purpose | Ownership |
|-----------|---------|-----------|
| `.github/workflows/ci.yaml` | Build, test, push image | GitHub Actions |
| `gitops-repo/fastapi-agent/` | Git source of truth | Git repository |
| `gitops-repo/kustomization.yaml` | Kustomize configuration | Git repository |
| `argocd/fastapi-agent.yaml` | ArgoCD Application | Kubernetes cluster |
| Image registry (GHCR/DockerHub) | Container storage | External service |

### Non-Goals

- Multi-environment deployments (dev/staging/prod) — Lesson 10 covers this with ApplicationSets
- Progressive delivery (canary/blue-green) — Lesson 13 covers this with Argo Rollouts
- Secrets management — Lesson 14 covers sealed secrets and external secrets
- Multi-cluster sync — Lesson 15 covers hub-spoke patterns

---

## Part 1: GitHub Actions CI Workflow

Your GitHub Actions workflow automates the **CI stage**: trigger → test → build → push.

**The flow**:

```
Push to main
    ↓
[GitHub Actions triggered]
    ↓
[Run pytest: coverage >80%?]
    ↓ NO → Pipeline fails
    ↓ YES
[Build Docker image with SHA tag]
    ↓
[Push to GHCR]
    ↓
[Update image reference in Git]
    ↓
[Done — ArgoCD watches Git and syncs]
```

### GitHub Actions Workflow File

Create `.github/workflows/ci.yaml`:

```yaml
name: CI - Build and Push Agent

on:
  push:
    branches:
      - main
    paths:
      - 'app/**'
      - 'tests/**'
      - 'pyproject.toml'
      - 'poetry.lock'
      - '.github/workflows/ci.yaml'
  pull_request:
    branches:
      - main

env:
  REGISTRY: ghcr.io
  IMAGE_NAME: ${{ github.repository }}/fastapi-agent

jobs:
  test:
    name: Run Tests
    runs-on: ubuntu-latest
    steps:
      - name: Checkout code
        uses: actions/checkout@v4

      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: "3.11"

      - name: Install dependencies
        run: |
          pip install poetry
          poetry install

      - name: Run pytest with coverage
        run: |
          poetry run pytest tests/ \
            --cov=app \
            --cov-report=xml \
            --cov-report=term \
            --cov-fail-under=80

      - name: Upload coverage to Codecov
        uses: codecov/codecov-action@v3
        with:
          files: ./coverage.xml
          flags: unittests

  build-and-push:
    name: Build and Push Image
    needs: test
    runs-on: ubuntu-latest
    permissions:
      contents: read
      packages: write

    steps:
      - name: Checkout code
        uses: actions/checkout@v4

      - name: Set up QEMU (multi-arch builds)
        uses: docker/setup-qemu-action@v3

      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v3

      - name: Log in to Container Registry
        uses: docker/login-action@v3
        with:
          registry: ${{ env.REGISTRY }}
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}

      - name: Extract metadata
        id: meta
        uses: docker/metadata-action@v5
        with:
          images: ${{ env.REGISTRY }}/${{ env.IMAGE_NAME }}
          tags: |
            type=sha,prefix={{branch}}-
            type=ref,event=branch
            type=semver,pattern={{version}}
            type=semver,pattern={{major}}.{{minor}}

      - name: Build and push image
        uses: docker/build-push-action@v5
        with:
          context: .
          platforms: linux/amd64,linux/arm64
          push: true
          tags: ${{ steps.meta.outputs.tags }}
          labels: ${{ steps.meta.outputs.labels }}
          cache-from: type=gha
          cache-to: type=gha,mode=max

      - name: Output image digest
        run: echo "Image pushed with digest ${{ steps.build.outputs.digest }}"

  update-gitops:
    name: Update GitOps Repository
    needs: build-and-push
    runs-on: ubuntu-latest
    steps:
      - name: Checkout main repo
        uses: actions/checkout@v4

      - name: Checkout GitOps repo
        uses: actions/checkout@v4
        with:
          repository: ${{ github.repository_owner }}/gitops-repo
          path: gitops-repo
          token: ${{ secrets.GITOPS_REPO_TOKEN }}

      - name: Update image tag in kustomization
        run: |
          cd gitops-repo/fastapi-agent

          # Extract new image SHA
          IMAGE_SHA=${{ github.sha }}

          # Update kustomization.yaml with new image
          kustomize edit set image fastapi-agent=${{ env.REGISTRY }}/${{ env.IMAGE_NAME }}:main-${IMAGE_SHA:0:7}

          cat kustomization.yaml

      - name: Commit and push update
        run: |
          cd gitops-repo

          git config user.name "github-actions[bot]"
          git config user.email "github-actions[bot]@users.noreply.github.com"

          git add fastapi-agent/kustomization.yaml
          git commit -m "chore: update fastapi-agent image to ${{ github.sha }}"
          git push origin main

      - name: Create deployment notification
        run: |
          echo "Deployment started"
          echo "Image: ${{ env.REGISTRY }}/${{ env.IMAGE_NAME }}:main-${{ github.sha }}"
          echo "GitOps commit: $(cd gitops-repo && git rev-parse HEAD)"
```

**Output when successful**:

```
✓ Test stage passed (coverage: 92%)
✓ Image built: ghcr.io/org/repo/fastapi-agent:main-a1b2c3d
✓ Image pushed to GHCR
✓ GitOps repository updated with new image tag
✓ ArgoCD watching gitops-repo detects change
→ ArgoCD begins syncing new image
```

### What This Workflow Does

1. **Trigger**: Every push to main (and PRs for safety checks)
2. **Test Job**: Runs pytest, enforces 80% coverage, uploads to Codecov
3. **Build Job**: Waits for test success, builds multi-platform image, caches layers
4. **Push Job**: Authenticates to GHCR with GitHub token, pushes image with SHA tag
5. **Update Job**: Checks out GitOps repo, updates image reference in kustomization.yaml, commits and pushes

The entire flow is **auditable**: every commit links to a workflow run, which links to a container image, which links to a GitOps commit.

---

## Part 2: GitOps Repository Structure

ArgoCD watches a **Git repository** for your desired state. The structure tells ArgoCD what to deploy.

Create a separate repository called `gitops-repo` (not the code repo):

```
gitops-repo/
├── README.md
├── fastapi-agent/
│   ├── kustomization.yaml      ← Updates here from CI
│   ├── base/
│   │   ├── deployment.yaml      ← K8s Deployment
│   │   ├── service.yaml         ← K8s Service
│   │   └── kustomization.yaml   ← Base configuration
│   └── overlays/
│       └── production/
│           ├── kustomization.yaml
│           └── patches/
│               └── replicas.yaml
├── argocd/
│   └── applications/
│       └── fastapi-agent.yaml   ← ArgoCD watches this
└── helmcharts/
    └── [future: helm-based deployments]
```

### Kustomization Configuration

File: `gitops-repo/fastapi-agent/base/kustomization.yaml`

```yaml
apiVersion: kustomize.config.k8s.io/v1beta1
kind: Kustomization

namespace: agents

resources:
  - deployment.yaml
  - service.yaml

commonLabels:
  app: fastapi-agent
  part: part-6
  managed-by: argocd

images:
  - name: fastapi-agent
    newName: ghcr.io/org/repo/fastapi-agent
    newTag: main-a1b2c3d  # ← CI updates this
```

File: `gitops-repo/fastapi-agent/base/deployment.yaml`

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: fastapi-agent
  labels:
    app: fastapi-agent
spec:
  replicas: 2
  strategy:
    type: RollingUpdate
    rollingUpdate:
      maxSurge: 1
      maxUnavailable: 0
  selector:
    matchLabels:
      app: fastapi-agent
  template:
    metadata:
      labels:
        app: fastapi-agent
      annotations:
        prometheus.io/scrape: "true"
        prometheus.io/port: "8000"
        prometheus.io/path: "/metrics"
    spec:
      serviceAccountName: fastapi-agent
      containers:
        - name: agent
          image: fastapi-agent:latest  # ← Kustomize overwrites this
          imagePullPolicy: IfNotPresent
          ports:
            - name: http
              containerPort: 8000
              protocol: TCP
          env:
            - name: ENVIRONMENT
              value: "production"
          livenessProbe:
            httpGet:
              path: /health
              port: 8000
            initialDelaySeconds: 5
            periodSeconds: 10
            failureThreshold: 3
          readinessProbe:
            httpGet:
              path: /ready
              port: 8000
            initialDelaySeconds: 3
            periodSeconds: 5
            failureThreshold: 2
          resources:
            requests:
              memory: "256Mi"
              cpu: "250m"
            limits:
              memory: "512Mi"
              cpu: "500m"
      affinity:
        podAntiAffinity:
          preferredDuringSchedulingIgnoredDuringExecution:
            - weight: 100
              podAffinityTerm:
                labelSelector:
                  matchExpressions:
                    - key: app
                      operator: In
                      values:
                        - fastapi-agent
                topologyKey: kubernetes.io/hostname
```

File: `gitops-repo/fastapi-agent/base/service.yaml`

```yaml
apiVersion: v1
kind: Service
metadata:
  name: fastapi-agent
  labels:
    app: fastapi-agent
spec:
  type: ClusterIP
  ports:
    - port: 80
      targetPort: http
      protocol: TCP
      name: http
  selector:
    app: fastapi-agent
```

**Output when kustomize applies this**:

```
namespace: agents (created if not exists)
Deployment fastapi-agent (2 replicas)
  - Image: ghcr.io/org/repo/fastapi-agent:main-a1b2c3d
  - Health checks enabled
  - Resource requests: 256Mi RAM, 250m CPU
Service fastapi-agent (ClusterIP on port 80)
```

---

## Part 3: ArgoCD Application

ArgoCD watches the GitOps repository and syncs changes to your cluster.

File: `argocd/applications/fastapi-agent.yaml`

```yaml
apiVersion: argoproj.io/v1alpha1
kind: Application
metadata:
  name: fastapi-agent
  namespace: argocd
  # Finalizers ensure proper cleanup on deletion
  finalizers:
    - resources-finalizer.argocd.argoproj.io
spec:
  project: default

  # Where to deploy FROM (the Git repo)
  source:
    repoURL: https://github.com/org/gitops-repo
    targetRevision: main
    path: fastapi-agent/base

  # WHERE to deploy TO (your cluster)
  destination:
    server: https://kubernetes.default.svc
    namespace: agents

  # HOW to sync
  syncPolicy:
    # Auto-sync on Git changes (required for CI → CD integration)
    automated:
      prune: true      # Delete K8s resources if removed from Git
      selfHeal: true   # Resync if cluster diverges from Git
      allowEmpty: false # Prevent accidental deletion of all resources

    # Sync options
    syncOptions:
      - CreateNamespace=true  # Create namespace if missing
      - RespectIgnoreDifferences=true

    # Wait for resources to reach healthy state
    retry:
      limit: 5
      backoff:
        duration: 5s
        factor: 2
        maxDuration: 3m

  # Health assessment
  ignoreDifferences:
    - group: apps
      kind: Deployment
      jsonPointers:
        # Ignore timestamp fields that change on every sync
        - /spec/template/metadata/annotations/deployment.kubernetes.io/revision

  # Notifications (optional: requires Slack/webhook setup from Lesson 12)
  info:
    - name: "GitHub"
      value: "https://github.com/org/repo"
    - name: "Slack"
      value: "#deployments"
```

**Apply this to your cluster**:

```bash
kubectl apply -f argocd/applications/fastapi-agent.yaml
```

**Output**:

```
application.argoproj.io/fastapi-agent created

Verify with:
argocd app get fastapi-agent

Name:               fastapi-agent
Namespace:          argocd
Project:            default
Server:             https://kubernetes.default.svc
Namespace:          agents
Repo:               https://github.com/org/gitops-repo
Target Branch:      main
Path:               fastapi-agent/base
Sync Status:        Synced
Health Status:      Healthy
```

---

## Part 4: Image Update Automation

When CI pushes a new image to the registry, **how does ArgoCD know?**

The answer: The CI workflow updates `kustomization.yaml` in the GitOps repo. ArgoCD watches Git. When Git changes, ArgoCD syncs.

### Kustomize Image Updater

The `update-gitops` job in your CI workflow updates the image tag:

```yaml
# In .github/workflows/ci.yaml
update-gitops:
  name: Update GitOps Repository
  needs: build-and-push
  runs-on: ubuntu-latest
  steps:
    - name: Checkout GitOps repo
      uses: actions/checkout@v4
      with:
        repository: ${{ github.repository_owner }}/gitops-repo
        path: gitops-repo
        token: ${{ secrets.GITOPS_REPO_TOKEN }}

    - name: Update image tag
      run: |
        cd gitops-repo/fastapi-agent
        kustomize edit set image fastapi-agent=ghcr.io/org/repo/fastapi-agent:main-${GITHUB_SHA:0:7}

    - name: Commit and push
      run: |
        cd gitops-repo
        git config user.name "github-actions[bot]"
        git config user.email "github-actions[bot]@users.noreply.github.com"
        git add fastapi-agent/kustomization.yaml
        git commit -m "chore: update image to ${{ github.sha }}"
        git push origin main
```

**What kustomize edit does**:

```bash
kustomize edit set image fastapi-agent=ghcr.io/org/repo/fastapi-agent:main-a1b2c3d
```

**Output**: `fastapi-agent/base/kustomization.yaml` now contains:

```yaml
images:
  - name: fastapi-agent
    newName: ghcr.io/org/repo/fastapi-agent
    newTag: main-a1b2c3d
```

ArgoCD sees this change in Git, pulls the new image reference, and syncs.

### Alternative: Helm Values Update

If using Helm instead of Kustomize:

```yaml
update-gitops:
  steps:
    - name: Update Helm values
      run: |
        cd gitops-repo/fastapi-agent

        # Update values.yaml
        sed -i "s/tag:.*/tag: main-${GITHUB_SHA:0:7}/" values.yaml

        cat values.yaml
```

This updates `values.yaml`:

```yaml
image:
  repository: ghcr.io/org/repo/fastapi-agent
  tag: main-a1b2c3d  # ← Updated by CI
  pullPolicy: IfNotPresent
```

---

## Part 5: End-to-End Validation

After deployment, validate that your pipeline worked.

### Step 1: Trigger the Pipeline

Push code to main:

```bash
cd your-agent-repo
git add app/main.py
git commit -m "feat: add new endpoint"
git push origin main
```

**Expected output**: GitHub Actions workflow starts automatically (check Actions tab in GitHub).

### Step 2: Monitor GitHub Actions

**In your GitHub repository → Actions tab:**

```
✓ CI - Build and Push Agent [00:02 started]
  ├─ test (in progress)
  │  ├─ Checkout code ✓
  │  ├─ Set up Python ✓
  │  ├─ Install dependencies ✓
  │  └─ Run pytest (3/5 tests passing...)
  ├─ build-and-push (waiting for test)
  └─ update-gitops (waiting for build-and-push)
```

**Wait for all three jobs to complete**. Total time: 8-12 minutes.

**Output when complete**:

```
✓ test (2:34 elapsed)
✓ build-and-push (4:12 elapsed)
✓ update-gitops (1:45 elapsed)

Summary:
  Test coverage: 92%
  Image: ghcr.io/org/repo/fastapi-agent:main-a1b2c3d
  GitOps commit: 3f2a1c0 "chore: update image to a1b2c3d"
```

### Step 3: Verify Image in Registry

```bash
# List images in GHCR
curl -H "Authorization: Bearer $GITHUB_TOKEN" \
  https://ghcr.io/v2/org/repo/fastapi-agent/tags/list

# Output:
# {
#   "name": "org/repo/fastapi-agent",
#   "tags": ["main-a1b2c3d", "main-9z8y7x6"]
# }
```

### Step 4: Verify GitOps Repository Updated

```bash
cd gitops-repo
git log --oneline | head -5

# Output:
# 3f2a1c0 chore: update image to a1b2c3d
# 8e7d6c5 Initial commit
```

Check the image tag in kustomization:

```bash
grep -A 3 "^images:" fastapi-agent/base/kustomization.yaml

# Output:
# images:
# - name: fastapi-agent
#   newName: ghcr.io/org/repo/fastapi-agent
#   newTag: main-a1b2c3d
```

### Step 5: Verify ArgoCD Synced

```bash
# Get ArgoCD app status
argocd app get fastapi-agent

# Output:
Name:               fastapi-agent
Project:            default
Repo:               https://github.com/org/gitops-repo
Branch:             main
Path:               fastapi-agent/base
Sync Status:        Synced      ← Git and cluster match
Health Status:      Healthy     ← All pods running
```

### Step 6: Verify Agent is Running

**Get the Service IP:**

```bash
kubectl get svc fastapi-agent -n agents

# Output:
NAME              TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)   AGE
fastapi-agent     ClusterIP   10.98.123.45    <none>        80/TCP    2m
```

**Test the agent endpoint** (port-forward or use external IP):

```bash
kubectl port-forward -n agents svc/fastapi-agent 8000:80 &

curl -s http://localhost:8000/health | jq .

# Output:
# {
#   "status": "healthy",
#   "version": "1.0.0",
#   "timestamp": "2025-12-23T14:32:10Z"
# }
```

**Test an agent endpoint:**

```bash
curl -s -X POST http://localhost:8000/tasks \
  -H "Content-Type: application/json" \
  -d '{"title": "Test task", "description": "Validate pipeline"}' | jq .

# Output:
# {
#   "id": 1,
#   "title": "Test task",
#   "description": "Validate pipeline",
#   "status": "created",
#   "created_at": "2025-12-23T14:32:15Z"
# }
```

### Validation Checklist

- ✅ GitHub Actions workflow completed with all three jobs passing
- ✅ Test coverage ≥ 80%
- ✅ Container image pushed to registry with correct SHA tag
- ✅ GitOps repository updated with new image reference
- ✅ ArgoCD app status shows "Synced" and "Healthy"
- ✅ All Deployment pods are Running and Ready (2/2 replicas)
- ✅ Agent endpoint responds with HTTP 200
- ✅ Health check endpoint returns healthy status
- ✅ Agent can process requests (create tasks, etc.)

If all checks pass, your **complete CI/CD pipeline is working**. From code push to running agent: fully automated.

---

## Part 6: Rollback Demonstration

Production issue? A commit introduced a bug? Kubernetes makes rollback trivial: revert the Git commit.

### Scenario: Detect an Issue

Your agent is running with the new deployment. You get a bug report:

```
User: "Agent is timing out on long requests"
Version: main-a1b2c3d (the latest you just deployed)
```

### Step 1: Identify the Problem Commit

```bash
cd your-agent-repo
git log --oneline | head -10

# Output:
# a1b2c3d feat: add timeout handling (← this broke things)
# 9z8y7x6 feat: improve response caching
# ...
```

### Step 2: Revert the Commit

```bash
# Revert the problematic commit
git revert a1b2c3d --no-edit

# Output:
# [main 2d4e5f6] Revert "feat: add timeout handling"
#  1 file changed, 5 insertions(+), 5 deletions(-)
```

### Step 3: Push the Revert

```bash
git push origin main
```

**Expected output**: GitHub Actions triggers automatically.

### Step 4: Monitor the Rollback

**In GitHub Actions:**

```
✓ test - (pytest passes with reverted code)
✓ build-and-push - (new image built with reverted code)
✓ update-gitops - (GitOps repo updated with previous image version)
```

**Image built**: `ghcr.io/org/repo/fastapi-agent:main-2d4e5f6` (the reverted version)

### Step 5: Watch ArgoCD Sync Old Version

```bash
# Monitor ArgoCD
argocd app wait fastapi-agent --sync

# Or watch in real-time:
watch -n 2 "argocd app get fastapi-agent | grep -E 'Sync|Health'"

# Output:
# Sync Status:   Syncing   (ArgoCD detecting Git change)
# Sync Status:   Synced    (old image deployed)
# Health Status: Progressing
# Health Status: Healthy   (pods healthy with reverted code)
```

### Step 6: Verify Rollback Successful

```bash
# Check deployment image
kubectl get deployment fastapi-agent -n agents -o yaml | grep image:

# Output:
# image: ghcr.io/org/repo/fastapi-agent:main-2d4e5f6  ← Previous version

# Verify pods restarted with old image
kubectl get pods -n agents -l app=fastapi-agent

# Output:
# NAME                              READY   STATUS    RESTARTS   AGE
# fastapi-agent-7f8c9d3e2-abc12     1/1     Running   0          45s
# fastapi-agent-7f8c9d3e2-def56     1/1     Running   0          48s
```

**Test the endpoint again:**

```bash
curl -s http://localhost:8000/health | jq .

# Output should be successful again:
# {
#   "status": "healthy",
#   "version": "1.0.0",
#   "timestamp": "2025-12-23T14:33:45Z"
# }
```

### Timeline of Rollback

```
14:32:10 - Bug detected in production
14:32:15 - Developer identifies commit: a1b2c3d
14:32:30 - Run: git revert a1b2c3d
14:32:45 - Run: git push origin main
14:33:00 - GitHub Actions triggered (automatically)
14:36:15 - New image built (reverted code)
14:36:30 - ArgoCD detects Git change
14:37:45 - Old image deployed and pods healthy
14:38:00 - Rollback complete, service restored

Total time: 6 minutes from detection to deployed fix
```

---

## Complete Pipeline Reference

Here's the entire automation flow in one diagram:

```
┌─────────────────────────────────────────────────────────────┐
│ 1. Developer pushes code to main                            │
│    git push origin main                                     │
└──────────────────┬──────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────────────────┐
│ 2. GitHub Actions triggered (within 30 seconds)             │
│    Workflow: .github/workflows/ci.yaml                      │
└──────────────────┬──────────────────────────────────────────┘
                   ↓
         ┌─────────┴─────────┐
         ↓                   ↓
    ┌─────────┐          ┌─────────┐
    │ TEST    │          │  BUILD  │
    │ Stage   │ (runs    │ + PUSH  │
    │ ────    │  first)  │ Stage   │
    │ pytest  │          │ ────────│
    │ >80%    │          │ Docker  │
    │        │  ──→  │ image   │
    └─────────┘          │ SHA tag │
         ↓               │ GHCR    │
    PASS? YES            └────┬────┘
         │                    ↓
         ↓            PUSH? SUCCESS?
         └──────┬─────────────┤YES
                ↓             ↓
         ┌────────────────────────────┐
         │ UPDATE GITOPS              │
         │ ────────────────────       │
         │ • Checkout gitops-repo     │
         │ • kustomize edit set image │
         │ • git commit + push        │
         └─────────┬──────────────────┘
                   ↓
         ┌────────────────────────────┐
         │ GIT UPDATED                │
         │ fastapi-agent/base/        │
         │   kustomization.yaml:      │
         │   newTag: main-a1b2c3d     │
         └─────────┬──────────────────┘
                   ↓
         ┌────────────────────────────┐
         │ ARGOCD WATCHES GIT         │
         │ ────────────────────       │
         │ (every 3 seconds checks    │
         │  if cluster matches Git)   │
         └─────────┬──────────────────┘
                   ↓
         ┌────────────────────────────┐
         │ CHANGE DETECTED!           │
         │ New image in kustomization │
         └─────────┬──────────────────┘
                   ↓
         ┌────────────────────────────┐
         │ ARGOCD SYNCS               │
         │ ────────────────────       │
         │ kubectl apply -k ...       │
         │ pulls new image            │
         │ updates Deployment         │
         │ rolls pods                 │
         └─────────┬──────────────────┘
                   ↓
         ┌────────────────────────────┐
         │ HEALTH CHECKS              │
         │ ────────────────────       │
         │ Pods start                 │
         │ /ready endpoint → 200?     │
         │ /health endpoint → 200?    │
         └─────────┬──────────────────┘
                   ↓
         ┌────────────────────────────┐
         │ DEPLOYMENT COMPLETE        │
         │ ────────────────────       │
         │ Status: Synced + Healthy   │
         │ Agent responds to requests │
         └────────────────────────────┘

TIMELINE: Code push → Deployment: ~8 minutes
```

---

## Capstone Completion Checklist

Use this checklist to verify you've completed the entire capstone:

### GitHub Actions Setup
- ✅ `.github/workflows/ci.yaml` created with all three jobs
- ✅ Tests run and enforce 80% coverage minimum
- ✅ Multi-platform build configured (linux/amd64, linux/arm64)
- ✅ Image pushed to GHCR with SHA tag
- ✅ GitOps repository token configured in GitHub Secrets

### GitOps Repository
- ✅ Separate `gitops-repo` created (not code repo)
- ✅ Directory structure: `fastapi-agent/base/` and `fastapi-agent/overlays/`
- ✅ `kustomization.yaml` configures image and namespace
- ✅ `deployment.yaml` includes health checks and resource limits
- ✅ `service.yaml` exposes agent on port 80
- ✅ CI workflow updates image tag on every push

### ArgoCD Configuration
- ✅ ArgoCD Application created at `argocd/applications/fastapi-agent.yaml`
- ✅ Application points to GitOps repo (`https://github.com/org/gitops-repo`)
- ✅ Auto-sync enabled (prune + selfHeal)
- ✅ Namespace created automatically (CreateNamespace=true)
- ✅ Application syncs within 3 minutes of Git update

### End-to-End Validation
- ✅ Pushed code to main, triggered workflow automatically
- ✅ Tests passed with coverage ≥ 80%
- ✅ Image built and pushed to GHCR
- ✅ GitOps repo updated with new image tag
- ✅ ArgoCD synced (status: Synced + Healthy)
- ✅ Agent pods running (2/2 Ready)
- ✅ Agent endpoint responds (HTTP 200)
- ✅ Health check passes (`/health` → 200)

### Rollback Demonstration
- ✅ Identified a problem commit
- ✅ Ran `git revert` to revert the commit
- ✅ Pushed revert, triggered workflow automatically
- ✅ New image built with reverted code
- ✅ ArgoCD synced old image to cluster
- ✅ Agent pods restarted with previous version
- ✅ Verified agent works again
- ✅ Total rollback time < 10 minutes

### Advanced Extensions (Optional)
- ⭐ Add ApplicationSet for multi-environment deployment (dev/staging/prod)
- ⭐ Configure Slack notifications (Lesson 12)
- ⭐ Implement canary/blue-green with Argo Rollouts (Lesson 13)
- ⭐ Add sealed secrets for database credentials (Lesson 14)
- ⭐ Scale to multiple clusters with hub-spoke (Lesson 15)

---

## Try With AI

You've built a complete, automated CI/CD pipeline. Now collaborate with Claude to troubleshoot and extend it.

### Setup for This Section

You'll need:
- Your FastAPI agent code in a GitHub repository
- A working Kubernetes cluster (Minikube, GKE, EKS, etc.)
- ArgoCD installed with admin access
- Container registry access (GHCR, Docker Hub, etc.)
- GitHub repository with Actions enabled

### Scenario 1: Pipeline is Stuck

**Your situation**: You pushed code, GitHub Actions started, but the workflow is hanging on the build step.

**Exploration prompt**:

"I pushed code 15 minutes ago and GitHub Actions is still building. The log shows:

```
[+] Building 45.3s (12/25)
 => [internal] load build definition from Dockerfile
 => => transferring dockerfile: 36B
 => [internal] load .dockerignore
 => ...
```

It seems stuck. What are common reasons Docker builds hang in CI? How do I debug this?"

**Ask Claude about**:
- Network issues (registry timeout, DNS problems)
- Disk space in the runner
- Memory constraints
- Large dependency downloads without caching
- How to add debugging output to your workflow

**Refine your question**:
- Build output logging: `docker buildx build --progress=plain`
- Timeout settings: `timeout: 15m` on the step
- Debug environment variables: `BUILDX_BUILDER_OUTPUT_VERBOSE=true`

### Scenario 2: ArgoCD Not Syncing

**Your situation**: The GitOps repo has the new image tag, but ArgoCD says "out of sync."

**Exploration prompt**:

"ArgoCD shows my app is 'OutOfSync' but the kustomization.yaml has the correct image tag:

```
images:
  - name: fastapi-agent
    newName: ghcr.io/org/repo/fastapi-agent
    newTag: main-a1b2c3d
```

Why isn't it syncing? The app repo is set to auto-sync. What am I missing?"

**Ask Claude about**:
- Is the app watching the right Git branch? (Check targetRevision)
- Is the path correct? (Check path in Application spec)
- Are there differing fields? (Use `argocd app diff`)
- Is the Git token working? (Check SSH key or HTTPS token)
- Run: `argocd app sync --force` to force reconciliation

**Refine your question**:
- Diagnose with: `argocd app diff fastapi-agent`
- Check Git access: `argocd repo add ...`
- Enable debug logging: `--loglevel debug`

### Scenario 3: Image Never Gets Updated

**Your situation**: CI completes successfully, image is pushed to GHCR, but kustomization.yaml never updates in the GitOps repo.

**Exploration prompt**:

"My workflow completed:
- ✅ Tests passed
- ✅ Image pushed: ghcr.io/org/repo/fastapi-agent:main-a1b2c3d
- ❌ But GitOps repo kustomization.yaml still has the old tag: main-old

The update-gitops job logs show 'success' but the commit never appeared. What could be wrong?"

**Ask Claude about**:
- Is `secrets.GITOPS_REPO_TOKEN` configured? (Authentication fails silently)
- Does the token have write access to the GitOps repo?
- Is the branch protection rule blocking commits?
- Is the git config correct? (user.name, user.email)
- How to check GitHub Actions logs for actual error

**Refine your question**:
- Add explicit error handling: `git push || { echo "Push failed"; exit 1; }`
- Verify token permissions: List allowed actions
- Test locally: `git clone` with the token to verify it works
- Add verbose logging: `git config core.logallrefupdates true`

### Scenario 4: Rollback Didn't Work

**Your situation**: You reverted a commit and pushed, but ArgoCD is deploying the new image instead of the old one.

**Exploration prompt**:

"I ran:

```bash
git revert a1b2c3d
git push origin main
```

But ArgoCD is deploying image `main-newSHA` (the reverted commit) instead of `main-oldSHA` (the version before the reverted commit).

The kustomization.yaml shows the correct image tag. Why is it deploying the wrong one?"

**Ask Claude about**:
- Image cache issues (pullPolicy should be Always, not IfNotPresent)
- Pod using cached local image instead of pulling new one
- ArgoCD showing correct image but deployment pulling cached version
- Using `kubectl describe pod -n agents` to check actual image

**Refine your question**:
- Force image pull: Change to `imagePullPolicy: Always`
- Delete old pods: `kubectl delete pods -n agents -l app=fastapi-agent`
- Verify image exists in registry: `crane ls ghcr.io/org/repo/fastapi-agent`
- Check pull secret if registry is private

### Scenario 5: Add Multi-Environment Deployment

**Your situation**: You want to deploy to dev and prod with different replicas and resources.

**Exploration prompt**:

"I have a working pipeline deploying to production. Now I need to deploy the same agent to a dev environment with:
- 1 replica instead of 2
- Smaller resource limits (128Mi RAM instead of 256Mi)
- Same code, different configuration

Should I create another Application? Use ApplicationSets? How do I keep them in sync?"

**Ask Claude about**:
- ApplicationSet with List generator (one entry per environment)
- Overlays pattern (base + dev/prod patches)
- Environment variables for different configs
- Helm values overrides

**Refine your question**:
- ApplicationSet YAML with List generator for `dev`, `prod`
- Kustomize overlays: `base/` + `overlays/dev/` + `overlays/prod/`
- How to parameterize replicas and resources
- How CI deploys to both environments in sequence
