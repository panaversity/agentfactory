---
sidebar_position: 8
title: "Sync Strategies and Policies"
description: "Configure auto-sync, auto-prune, self-heal, and sync windows for precise control over ArgoCD deployments"
keywords: [argocd, sync, auto-sync, prune, self-heal, gitops, deployment, syncPolicy]
---

# Sync Strategies and Policies

You've created your first ArgoCD Application in Lesson 7. Now comes a critical decision: **When and how should ArgoCD sync changes to your cluster?**

By default, ArgoCD doesn't automatically sync. You must click "Sync" in the UI or run `argocd app sync`. This gives you control but requires constant attention. Real GitOps teams use automated sync policies to eliminate manual intervention while still maintaining safety.

In this lesson, you'll learn how to configure sync strategies that match your deployment goals:

- **Manual sync** - You explicitly trigger deployments (good for learning, risky for production)
- **Auto-sync** - ArgoCD automatically applies changes from Git (true GitOps automation)
- **Auto-prune** - Automatically delete resources that were removed from Git (cleanup)
- **Self-heal** - Automatically revert manual cluster changes back to Git truth (drift correction)
- **Replace vs Apply** - How to update immutable resources (force overwrite)
- **Sync windows** - Time-based restrictions on deployments (maintenance windows)

By the end of this lesson, you'll understand the tradeoffs and be able to configure sync policies that work for your team.

## Manual Sync: When You Control Everything

Manual sync is the default. You must explicitly trigger deployments. This makes sense when:

- You're learning and want to see exactly what happens
- You need approval before every deployment
- You deploy infrequently and can check each one manually

### Setting Manual Sync

When you create an Application in Lesson 7, if you don't specify `syncPolicy`, it defaults to manual:

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
    path: k8s/
    targetRevision: HEAD
  destination:
    server: https://kubernetes.default.svc
    namespace: default
  # No syncPolicy means manual sync only
```

**Output:**
```
Application 'my-agent' created
Sync status: OutOfSync (because Git and cluster differ)
```

### Triggering Sync Manually

#### Via CLI:

```bash
argocd app sync my-agent
```

**Output:**
```
NAMESPACE  NAME                         TYPE     VERSION  CREATED  STATUS
default    my-agent-deployment          Apps     apps/v1  5m       OutOfSync
default    my-agent-service             Service  v1       5m       OutOfSync

syncing
...
Application 'my-agent' synced successfully
```

#### Via UI:

1. Open ArgoCD (port-forward: `kubectl port-forward -n argocd svc/argocd-server 8080:443`)
2. Click on your Application
3. Click the blue "Sync" button
4. Select "Sync Policy: All" and click "Synchronize"

### Why This Works for Learning

- You see exactly when and what changes
- You can review Git changes before applying them
- You practice the reconciliation workflow: compare → approve → apply → verify

### Why Manual Doesn't Scale

In production:
- Every developer waits for you to sync
- You become the bottleneck
- A critical bug needs deploying NOW - manual review takes time
- You might forget to sync for hours, leaving the cluster out of date

## Auto-Sync: GitOps Automation

Auto-sync means: "Whenever Git changes, automatically apply those changes to the cluster." This is true GitOps.

### Enabling Auto-Sync

Add `syncPolicy.automated` to your Application:

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
    path: k8s/
    targetRevision: HEAD
  destination:
    server: https://kubernetes.default.svc
    namespace: default
  syncPolicy:
    automated: {}  # Enable auto-sync with defaults
```

**Output:**
```
Application 'my-agent' created
Sync status: Synced (changes applied automatically within seconds)
```

### What Happens Automatically

1. ArgoCD polls Git every 3 seconds (default)
2. A change appears in Git (new version of `deployment.yaml`)
3. ArgoCD detects the change
4. ArgoCD applies the change with `kubectl apply`
5. The cluster updates within seconds

All without manual intervention.

### Auto-Sync Settings

You can fine-tune auto-sync behavior:

```yaml
syncPolicy:
  automated:
    prune: false      # Keep deleted resources (safe default)
    selfHeal: false   # Don't revert manual changes
    allowEmpty: false # Require at least one resource
```

Each setting addresses a specific concern:

- **prune: true** - Automatically delete resources removed from Git
- **selfHeal: true** - Automatically revert manual cluster changes
- **allowEmpty: false** - Prevent syncing if the Git path is empty (safety check)

We'll explore prune and selfHeal below. For now, know that `automated: {}` means:
- prune: false (default)
- selfHeal: false (default)
- allowEmpty: false (default)

## Auto-Prune: Cleaning Up Deleted Resources

When you remove a resource from your Git repository, what happens to the instance already running in the cluster?

**Without auto-prune**: The resource stays in the cluster even though it's not in Git anymore. You have orphaned resources.

**With auto-prune**: ArgoCD automatically deletes the resource.

### Example: Removing a Service

You have a service in Git:

```yaml
# k8s/service.yaml
apiVersion: v1
kind: Service
metadata:
  name: my-agent-service
spec:
  ports:
  - port: 8080
    targetPort: 8080
  selector:
    app: my-agent
```

Your Application has it synced:

```bash
kubectl get svc -n default
```

**Output:**
```
NAME               TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)    AGE
my-agent-service   ClusterIP   10.96.123.45    <none>        8080/TCP   5m
```

Now you delete `k8s/service.yaml` from your Git repo and push.

**If auto-prune is DISABLED**:

```bash
argocd app sync my-agent
```

**Output:**
```
syncing
Application 'my-agent' synced successfully

kubectl get svc -n default
NAME               TYPE        CLUSTER-IP      EXTERNAL-IP   PORT(S)    AGE
my-agent-service   ClusterIP   10.96.123.45    <none>        8080/TCP   5m  # Still there!
```

The service still exists. ArgoCD only cares about applying what's in Git, not removing what's not.

**If auto-prune is ENABLED**:

```yaml
syncPolicy:
  automated:
    prune: true  # Delete resources removed from Git
```

Push the same deletion to Git. ArgoCD applies changes:

```bash
argocd app sync my-agent
```

**Output:**
```
syncing
Pruning 1 resource (removing: Service my-agent-service)
Application 'my-agent' synced successfully

kubectl get svc -n default
NAME      TYPE        CLUSTER-IP   EXTERNAL-IP   PORT(S)    AGE
# Service is gone
```

### When to Prune

**Enable prune when**:
- Your Git repo is the source of truth for ALL resources
- You want cleanup to be automatic
- You're confident your Git push won't accidentally delete important things

**Disable prune when**:
- You manually create resources in the cluster and want to keep them
- You're still learning (safer to delete by hand)
- You're worried about accidental deletions

### Pruning Safely

Start with `prune: false` and increase your git discipline:

```yaml
syncPolicy:
  automated:
    prune: false  # Start here
```

Once you're confident:
- All resources are in Git
- You review deletions in code review
- You have tests that catch broken deletions

Then enable:

```yaml
syncPolicy:
  automated:
    prune: true  # Only after mastering Git-as-truth
```

## Self-Heal: Reverting Unauthorized Changes

Sometimes you or a colleague manually changes something in the cluster:

```bash
# Someone manually edits the deployment
kubectl patch deployment my-agent -p '{"spec":{"replicas":10}}'
```

Now the cluster state (10 replicas) doesn't match Git state (3 replicas). You have drift.

**Without self-heal**: The cluster keeps the manual change. Your Git is out of sync.

**With self-heal**: ArgoCD detects the drift and reverts the cluster back to what Git says.

### Example: Manual Edit Gets Reverted

Your Application currently has:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: my-agent
spec:
  replicas: 3  # Git says 3
```

Someone manually scales it:

```bash
kubectl patch deployment my-agent -p '{"spec":{"replicas":10}}'
```

Check the cluster:

```bash
kubectl get deployment my-agent
```

**Output:**
```
NAME       READY   UP-TO-DATE   AVAILABLE   AGE
my-agent   10/10   10           10          5m
```

The cluster now has 10 replicas (manual change).

**If self-heal is DISABLED**:

```yaml
syncPolicy:
  automated:
    selfHeal: false
```

The cluster stays at 10 replicas. ArgoCD doesn't care about drift—it only syncs when Git changes. Your Git and cluster are out of sync.

**If self-heal is ENABLED**:

```yaml
syncPolicy:
  automated:
    selfHeal: true
```

ArgoCD checks every 3 seconds (default). When it detects the drift:

```bash
# ArgoCD automatically reverts the change
# Within a few seconds:
kubectl get deployment my-agent
```

**Output:**
```
NAME       READY   UP-TO-DATE   AVAILABLE   AGE
my-agent   3/3     3            3           5m  # Back to 3 (what Git says)
```

The cluster is healed back to Git state.

### When to Enable Self-Heal

**Enable when**:
- Your team agrees: Git is the only source of truth
- You want to prevent manual changes from sticking
- You deploy to clusters that should match Git exactly

**Disable when**:
- You make emergency manual changes in production
- You're testing something and don't want it reverted
- You're learning and want to understand manual changes

### Self-Heal Risks

Self-heal automatically reverts changes. This is powerful but risky:

- Emergency hotfix: You manually change something critical. Self-heal reverts it. Bad timing = outage.
- Testing: You change a config to test something. Self-heal reverts your change immediately.
- Debugging: You change something to debug an issue. Self-heal interferes.

**Safety strategy**: Keep self-heal off in staging until you're confident. Test it thoroughly. Only enable in production once you trust it.

## Replace vs Apply: Handling Immutable Fields

Sometimes you need to change a resource that Kubernetes won't let you modify with `kubectl apply`. This is where the **syncStrategy** comes in.

### The Problem: Immutable Fields

Kubernetes immutability rules vary by resource type. For example, you can't change a Service's `selector` with `kubectl apply`:

```yaml
apiVersion: v1
kind: Service
metadata:
  name: my-agent-service
spec:
  selector:
    app: my-old-name  # You want to change this to my-agent
  ports:
  - port: 8080
```

If you try to apply a new Service with a different selector:

```bash
kubectl apply -f service.yaml
```

**Output:**
```
The Service "my-agent-service" is invalid: spec.selector: Invalid value: map[string]string{"app":"my-agent"}:
field is immutable
```

### Solution: Replace Strategy

Tell ArgoCD to delete and recreate the resource instead of applying changes:

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
    path: k8s/
    targetRevision: HEAD
  destination:
    server: https://kubernetes.default.svc
    namespace: default
  syncPolicy:
    automated:
      prune: false
      selfHeal: false
    syncOptions:
    - Replace=true  # Delete and recreate resources with immutable field changes
```

Now when you change the selector, ArgoCD:

1. Deletes the old Service
2. Creates the new Service with the new selector

### Apply vs Replace: The Tradeoff

| Strategy | Good For | Risk |
|----------|----------|------|
| **Apply** (default) | Most resources, strategic merge | Can't update immutable fields |
| **Replace** | Immutable fields, clean slate | Brief downtime while resource is recreated |

For a Service, Replace means a few seconds of downtime (clients can't reach the service). For a Deployment, Replace means pods restart.

**Use Replace only when necessary** (immutable field changes). Use Apply (default) for most updates.

## Sync Windows: Restricting Deployments to Specific Times

Sometimes you need to say: "Don't deploy between 9 AM and 10 AM because we're doing live demos."

Sync windows let you restrict when deployments can happen.

### Setting a Sync Window

Add `syncPolicy.syncWindows`:

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
    path: k8s/
    targetRevision: HEAD
  destination:
    server: https://kubernetes.default.svc
    namespace: default
  syncPolicy:
    automated:
      prune: false
      selfHeal: false
    syncWindows:
    - schedule: "0 2 * * *"       # 2 AM daily (cron format)
      duration: 2h                 # Allow syncs for 2 hours (2-4 AM)
      timeZone: "UTC"
      kind: allow                  # This is when syncs ARE allowed
    - schedule: "0 9 * * 1-5"      # 9 AM Mon-Fri (cron format)
      duration: 1h                 # Block syncs for 1 hour (9-10 AM)
      timeZone: "America/New_York"
      kind: deny                   # This is when syncs are BLOCKED
```

### How Sync Windows Work

The Application has two windows defined:

1. **Allow window**: Mon-Fri 2-4 AM UTC - syncs are allowed
2. **Deny window**: Mon-Fri 9-10 AM America/New_York - syncs are blocked

If a deployment is triggered outside these windows:

- If an "allow" window is defined and it's outside that window → blocked
- If a "deny" window is defined and you're inside that window → blocked
- If no windows match → deployment proceeds (no restriction)

### Practical Examples

#### Example 1: Maintenance Window (Only Deploy During Maintenance)

```yaml
syncWindows:
- schedule: "0 2 * * 0"      # Sundays 2 AM UTC
  duration: 4h               # 2-6 AM UTC
  kind: allow                # Only allow syncs during this window
```

This means: "I only want deployments on Sunday mornings from 2-6 AM. Any other time, block."

#### Example 2: Blackout Window (Never Deploy During Business Hours)

```yaml
syncWindows:
- schedule: "0 9 * * 1-5"    # Mon-Fri 9 AM
  duration: 8h               # 9 AM - 5 PM
  kind: deny                 # Block syncs during business hours
```

This means: "Don't deploy Mon-Fri 9-5 (business hours). Any other time, deploy freely."

### Cron Format Reference

Sync windows use cron syntax: `minute hour day month dayOfWeek`

| Example | Meaning |
|---------|---------|
| `0 2 * * *` | 2:00 AM every day |
| `0 14 * * 1-5` | 2:00 PM Mon-Fri |
| `*/30 * * * *` | Every 30 minutes |
| `0 0 1 * *` | Midnight on the 1st of each month |

Use [crontab.guru](https://crontab.guru) to build cron expressions.

### Syncing During a Deny Window

What if a critical bug needs deploying during a deny window?

Option 1: Manual sync overrides the window

```bash
# Force a sync even during a deny window
argocd app sync my-agent --force
```

Option 2: Edit the Application to remove the sync window temporarily

```yaml
# Remove the syncWindows section, sync, then add it back
```

## Putting It All Together: Sync Policy Comparison

Here's a quick reference for common scenarios:

| Scenario | Manual | Auto-Sync | Prune | Self-Heal | Sync Window |
|----------|--------|-----------|-------|-----------|------------|
| Learning | ✓ | | | | |
| Staging (frequent deploys) | | ✓ | ✓ | ✓ | |
| Production (auto-deploy) | | ✓ | ✓ | ✓ | ✓ |
| Production (conservative) | | ✓ | | | ✓ |
| Live demo (don't interrupt) | | ✓ | ✓ | ✓ | ✓ (deny 9-10 AM) |

### Common Policy Configurations

#### Configuration 1: Learning (Default)

```yaml
syncPolicy:
  automated: {}  # No options = manual sync only
```

No automation. You control everything. Good for understanding the workflow.

#### Configuration 2: Staging Automation

```yaml
syncPolicy:
  automated:
    prune: true
    selfHeal: true
  syncOptions:
  - Prune=true
```

Full automation in staging. Let it fail fast so you catch issues before production.

#### Configuration 3: Production (Safe Automation)

```yaml
syncPolicy:
  automated:
    prune: false  # Keep accidentally deleted resources
    selfHeal: true
  syncWindows:
  - schedule: "0 2 * * 0"  # Sunday 2 AM
    duration: 4h
    kind: allow
```

Auto-sync with self-heal, but:
- No auto-prune (safer if you accidentally delete something)
- Sync window restricts deployments to Sunday maintenance window

#### Configuration 4: Production (Conservative)

```yaml
syncPolicy:
  syncOptions:
  - CreateNamespace=true
  # No automated - require manual approval
```

Manual sync for every deployment. Gives you control but requires active management.

## Updating Your Application with New Sync Policies

Once you understand the policies, you can update an existing Application:

```bash
# Edit the Application to add auto-sync
kubectl edit application my-agent -n argocd
```

Add or modify the `syncPolicy` section. Save and exit your editor.

**Output:**
```
application.argoproj.io/my-agent edited
Sync status changes will apply on the next reconciliation (within 3 seconds)
```

Verify the change:

```bash
argocd app get my-agent
```

**Output:**
```
Name:               my-agent
Project:            default
Sync Policy:        automated (prune=false, selfHeal=false)
Sync Status:        Synced
Health Status:      Healthy
```

## Try With AI

Now that you understand sync strategies, ask Claude to help you navigate tradeoffs:

**Setup**: Open your Application YAML from Lesson 7 (or create a sample one)

**Prompt 1**: "I want to enable auto-sync for my staging environment, but I'm worried about auto-prune deleting important resources. Should I enable auto-prune? What's a safer approach?"

When Claude responds, evaluate:
- Does it explain the risk of orphaned resources?
- Does it suggest monitoring or testing prune behavior first?
- Does it mention the difference between staging and production?

**Prompt 2**: "Our team keeps making emergency manual changes to production, but self-heal keeps reverting them. How should we structure our sync policies to allow emergency changes while still having Git as truth?"

Evaluate the response:
- Does it acknowledge the tension between automation and emergency fixes?
- Does it suggest solutions like disabling self-heal or creating separate Applications?
- Does it recommend practices like "change Git immediately after the emergency fix"?

**Prompt 3**: "Write a sync window that only allows deployments on weekdays between 2-4 AM UTC, except during the first Monday of each month when we do major releases (allow anytime that day)."

Review the YAML:
- Is the cron syntax correct for weekdays 2-4 AM?
- Does it handle the exception correctly (hint: this might need multiple windows)?
- Could you apply this to your Application?

**Iterate**: Take Claude's answers and apply them to your actual Application. Verify the sync behavior matches your expectations by:
1. Pushing a change to Git
2. Watching ArgoCD sync (or waiting for the sync window to open)
3. Making a manual change and verifying self-heal reverts it (if enabled)
4. Checking the sync logs: `argocd app logs my-agent`
