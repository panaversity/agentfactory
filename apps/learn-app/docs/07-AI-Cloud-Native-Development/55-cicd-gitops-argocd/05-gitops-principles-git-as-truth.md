---
sidebar_position: 5
title: "GitOps Principles: Git as Truth"
description: "Understand declarative infrastructure, reconciliation loops, and drift detection"
keywords: [gitops, declarative, reconciliation, drift detection, git, kubernetes, argocd]
---

# GitOps Principles: Git as Truth

You've built a CI pipeline that tests and pushes images automatically. Now comes a harder question: **how do you deploy?**

In Lessons 1-4, you learned that CI automates building and testing. But once tests pass, who decides when and how to deploy? If you're running `kubectl apply -f deployment.yaml` manually after each image push, you're not really automating deployment—you're automating only the preparation for deployment.

This lesson introduces **GitOps**, a radically different approach: instead of running manual kubectl commands, you declare your desired infrastructure in Git. A controller watches Git and makes the cluster match it automatically. This transforms deployments from imperative commands ("do this, then do that") into declarative declarations ("this is the desired state").

By the end of this lesson, you'll understand why Git-as-truth enables auditability, rollback, and collaboration—and why it's fundamentally safer than manual kubectl apply.

## The Imperative Trap: Why Manual Deployments Fail at Scale

Let's start with what you probably do today: manual deployments.

You have a Kubernetes deployment YAML:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: task-agent
  namespace: default
spec:
  replicas: 3
  selector:
    matchLabels:
      app: task-agent
  template:
    metadata:
      labels:
        app: task-agent
    spec:
      containers:
      - name: agent
        image: myregistry.azurecr.io/task-agent:sha-abc123
        ports:
        - containerPort: 8000
```

**Output:**
```
Deployment manifest defined with 3 replicas of task-agent
```

To deploy a new image, you update the image tag and run:

```bash
kubectl apply -f deployment.yaml
```

**Output:**
```
deployment.apps/task-agent configured
```

Seems simple. But here's what happens at scale:

1. **No audit trail**: Who changed the deployment? When? Why? Git doesn't know—it's only in kubectl's terminal history.
2. **No versioning**: You can't easily revert to the previous replica count if someone manually changed it.
3. **Cluster becomes the source of truth**: Someone ssh's into a node and changes a config file. Now the cluster doesn't match your YAML.
4. **Manual kubectl is error-prone**: One typo in `kubectl apply` and you've deployed to the wrong namespace with the wrong image.

In a team, this breaks down entirely. If three people are running kubectl commands independently, the cluster becomes a mess of undocumented changes.

### The Real Problem: Imperative vs Declarative

When you run `kubectl apply`, you're being **imperative**:

```bash
# Imperative: "Here's a sequence of commands to execute"
kubectl apply -f deployment.yaml
# (If it exists, Kubernetes patches it. If not, creates it.)
```

Imperative thinking is about **steps and sequences**. "Do this first, then that." It's how you think when writing bash scripts or manual processes.

Declarative thinking is completely different:

```yaml
# Declarative: "Here's the desired state. Make it so."
apiVersion: apps/v1
kind: Deployment
metadata:
  name: task-agent
spec:
  replicas: 3
  selector:
    matchLabels:
      app: task-agent
  # ... (Kubernetes makes this state exist)
```

You don't say "create a deployment"; you say "the desired state is a deployment with 3 replicas." Kubernetes's job is to **reconcile**—to make the cluster match that declaration.

**Output:**
```
Kubernetes reconciles: desired state (3 replicas) vs actual state (current replicas)
If they match, do nothing. If they don't, take action.
```

Imperative requires humans to execute steps. Declarative requires a controller to observe and reconcile continuously.

## Git as Single Source of Truth

Here's the GitOps insight: **Git is already declarative**. Your YAML files declare desired state. But you're treating Git as optional—the real source of truth is the cluster, which you mutate with imperative commands.

Flip that relationship: **Make Git the source of truth.** Write your desired infrastructure state in Git. A controller watches Git and says:

> "What's in Git is the desired state. What's in the cluster is the actual state. They don't match? I'll fix that."

This is radically simpler:

```
Developer commits deployment.yaml to Git
         ↓
GitOps Controller (ArgoCD) watches Git
         ↓
Controller reads: "desired state is 3 replicas of image:tag-v1.2.0"
         ↓
Controller checks cluster: "actual state is 2 replicas of image:tag-v1.1.9"
         ↓
Controller patches cluster: "Updating to 3 replicas, pulling new image"
         ↓
Cluster converges to desired state
         ↓
Everyone can audit Git history: who changed what, when, why
```

**Output:**
```
Audit trail: 10:05 dev@company.com pushed deployment.yaml
Desired state now in Git, actual cluster state in sync
Rollback: git revert to previous commit, controller deploys old version
```

Notice what just happened:

1. **Git is auditable**: Every change is committed with author, timestamp, and message.
2. **Cluster state is reproducible**: You can rebuild the cluster from Git at any point in history.
3. **Changes are reviewed**: Deployments happen via pull requests, not anonymous kubectl commands.
4. **Rollback is trivial**: `git revert` instead of "figure out what was running before."

Compare this to imperative deployments:

| Aspect | Imperative (kubectl) | Declarative (GitOps) |
|--------|---------------------|----------------------|
| **Source of truth** | Cluster state | Git repository |
| **Audit trail** | None (terminal history lost) | Full Git history with author/timestamp |
| **Change review** | None (kubectl can run anytime) | Pull request review before merge |
| **Rollback** | Manual effort (restore from backups?) | `git revert` + controller auto-deploys |
| **Team collaboration** | Risky (concurrent kubectl runs) | Safe (Git merge conflicts caught) |
| **Reproducibility** | Cluster is fragile (hard to rebuild) | Cluster is ephemeral (rebuild from Git) |

### Why Git Specifically?

You might ask: "Why not store desired state in a database?"

Git has three properties that make it unique for infrastructure:

1. **Version control**: Every change is committed with a full history. You can see who changed what, when, and why.
2. **Distributed and resilient**: Git works offline. You can have multiple copies. A Git server going down doesn't stop deployments.
3. **Human-readable and diffable**: You can review changes before they're applied. `git diff` shows exactly what will change.

A database can store current state, but it can't easily show history or allow offline operation. Git excels at both.

**Output:**
```
Git advantages for infrastructure:
- History: git log shows every change
- Diffs: git diff shows what's changing
- Offline: developers can work without network
- Mergeability: team changes can be coordinated
```

## The Reconciliation Loop: Observe, Diff, Act

A GitOps controller runs continuously and executes the same loop forever:

```
1. OBSERVE: Read Git repository
   "What is the desired state?"

2. DIFF: Compare Git to cluster
   "Where do desired and actual state differ?"

3. ACT: Make the cluster match Git
   "Apply any changes needed"

(Repeat every N seconds)
```

This is why we call it **reconciliation**: the controller is constantly trying to reconcile the gap between desired (Git) and actual (cluster).

Here's a concrete example:

```yaml
# Commit to Git: spec.replicas: 3
apiVersion: apps/v1
kind: Deployment
metadata:
  name: task-agent
spec:
  replicas: 3
```

**Output:**
```
Git desired state: 3 replicas
```

Controller observes Git, sees `replicas: 3`. Checks cluster, sees 3 pods running. **No action needed.**

Now, someone manually scales the deployment:

```bash
kubectl scale deployment task-agent --replicas=5
```

**Output:**
```
Cluster now has 5 replicas
Git still declares: 3 replicas
```

The reconciliation loop detects the drift:

```
OBSERVE: Git says 3 replicas
DIFF:    Cluster has 5 replicas (mismatch!)
ACT:     Scale cluster back to 3 replicas
```

**Output:**
```
Reconciliation detected drift. Scaling to 3 replicas.
Cluster converges back to Git desired state.
```

This is the power of reconciliation: **the cluster self-heals toward the declared state automatically**. No human has to notice the manual change and fix it. The controller notices and fixes it.

## Drift Detection: When Manual Changes Break Everything

"Drift" means the cluster state has drifted away from Git. Here's why it matters:

**Scenario 1: Environment variable was manually changed**

A developer ssh's into a pod and changes an environment variable to debug an issue:

```bash
kubectl exec -it task-agent-xyz -- bash
# (inside container) export DEBUG=true
```

This works for debugging, but the developer forgets to revert it. Now:
- Git says: `DEBUG=false` (or undefined)
- Cluster has: `DEBUG=true`
- **Drift exists**: The running container doesn't match Git

The next time the controller reconciles, it sees the mismatch. It restarts the pod with the Git-declared environment, reverting the debug change. Production is back in sync with Git.

**Output:**
```
Drift detected: Pod environment differs from Git spec
Controller restarts pod with correct environment variables
```

**Scenario 2: Resource was manually deleted**

Someone accidentally deletes a service:

```bash
kubectl delete service task-agent
```

Git still declares the service should exist. Controller detects:
- Git says: Service should exist
- Cluster has: Service deleted
- **Drift exists**

Controller recreates the service from Git. Your application is healthy again.

**Output:**
```
Service deleted from cluster.
Controller detects missing resource, recreates from Git spec.
```

**Scenario 3: Image was manually changed**

A desperate developer manually changes the image tag to test a fix:

```bash
kubectl set image deployment/task-agent agent=myregistry.azurecr.io/task-agent:hotfix-abc123
```

Git still declares the original image. Drift detected. Controller restarts pods with the Git-declared image.

**Output:**
```
Drift: Cluster image is hotfix-abc123, Git declares tag-v1.0.0
Controller rolls back to Git-declared image.
```

In each case, the reconciliation loop acts as a **safety net**: manual changes automatically drift back toward the declared state. This doesn't prevent mistakes, but it prevents them from persisting.

## Declarative Infrastructure as a Decision Framework

At this point, you understand the mechanics. But the real shift is philosophical: **declarative infrastructure changes how you think about deployments**.

Imperative thinking: "How do I change the cluster right now?"

Declarative thinking: "What should the desired state be?"

This is actually more powerful. Instead of worrying about the sequence of commands ("patch this, then update that, then restart pods"), you just describe the end state. The controller figures out how to get there.

For your FastAPI agent:

**Imperative approach**:
```bash
# Update image tag
kubectl set image deployment/task-agent agent=myregistry.azurecr.io/task-agent:tag-v2.0.0 --record

# Wait for rollout
kubectl rollout status deployment/task-agent

# If something breaks, manually kubectl rollout undo
kubectl rollout undo deployment/task-agent
```

**Declarative approach**:
```yaml
# In Git: update image tag
apiVersion: apps/v1
kind: Deployment
metadata:
  name: task-agent
spec:
  replicas: 3
  template:
    spec:
      containers:
      - name: agent
        image: myregistry.azurecr.io/task-agent:tag-v2.0.0
```

Commit, push. Controller does the rest. Rollback is `git revert`. That's it.

**Output:**
```
Imperative: 3 commands to deploy, 2 to rollback
Declarative: 1 git push to deploy, 1 git revert to rollback
```

## Comparison: kubectl apply vs GitOps Reconciliation

Here's the full picture side by side:

| Step | kubectl apply | GitOps (ArgoCD) |
|------|---------------|-----------------|
| **1. Change code** | Developer edits deployment.yaml | Developer edits deployment.yaml in Git |
| **2. Review & approve** | None (developer runs kubectl themselves) | Pull request → code review → merge |
| **3. Deploy** | Developer runs `kubectl apply` manually | Controller watches Git, auto-deploys |
| **4. Audit trail** | None; user history is ephemeral | Git commit with author, timestamp, message |
| **5. Rollback** | Manual investigation, then kubectl revert | `git revert` + auto-deploy |
| **6. Drift detection** | None; cluster diverges silently | Controller detects and corrects drift |
| **7. Team coordination** | Risky; multiple people running kubectl | Safe; merges prevent concurrent changes |

The shift from column 1 to column 2 is transformative. You move from **doing** deployments to **declaring** them.

## GitOps in 30 Seconds

Here's the essence:

1. **Write infrastructure as code in Git** (YAML files describing desired state)
2. **A controller watches Git** (ArgoCD in this chapter)
3. **Controller continuously reconciles**: desired state (Git) vs actual state (cluster)
4. **If they differ, controller makes them match**
5. **Result**: Cluster automatically converges to Git, drift is self-healed, and every change is audited

This is GitOps: using Git as the source of truth for infrastructure, with a controller enforcing it.

Starting in Lesson 6, you'll install **ArgoCD**, the controller that watches Git and deploys to your Minikube cluster. But now you understand the philosophy: you'll never run manual kubectl apply again. Instead, you'll commit to Git and let ArgoCD handle deployment.

## Try With AI

Ask Claude: "I'm using GitOps with ArgoCD. One of my developers manually scales a deployment to 10 replicas for testing, then forgets to undo it before committing to Git. What happens?"

Evaluate the response:

- Does it explain that the reconciliation loop will detect the mismatch?
- Does it describe how ArgoCD will revert to the Git-declared replicas?
- Does it mention the value of having an audit trail in Git?

Iterate with: "How would my approach to incident response change if I'm using GitOps instead of manual kubectl?"

Expected evaluation points:
- **Source of truth clarity**: Git is the single source of truth, not the cluster
- **Auditability**: Every change is in Git history, reviewable
- **Automation**: Drift detection and correction is automatic, not manual
- **Reproducibility**: Cluster state can be reconstructed from Git at any point in time
