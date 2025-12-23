---
sidebar_position: 13
title: "Progressive Delivery Overview"
description: "Understand canary and blue-green deployments with Argo Rollouts"
keywords: [argocd, argo rollouts, canary, blue-green, progressive delivery, deployment]
---

# Progressive Delivery Overview

You've deployed applications to Kubernetes using ArgoCD. That works for many scenarios, but you've faced a critical limitation: **all-or-nothing deployments**. When you apply a new Deployment, Kubernetes rolls out the new version to all replicas at once. If there's a bug in that new version, all users see it. You've swapped a working version for a broken one in seconds.

In production, this is unacceptable. You need **progressive delivery**—the ability to deploy new versions safely by rolling them out gradually, watching them carefully, and rolling back if problems appear.

This lesson introduces two complementary approaches to progressive delivery: **canary deployments** (gradually shift traffic to new version) and **blue-green deployments** (run both versions, switch all traffic instantly). We'll then introduce **Argo Rollouts**, the Kubernetes-native tool that automates these patterns.

By the end of this lesson, you'll understand why AI agents especially benefit from progressive delivery (behavior changes are subtle), the mechanics of canary vs blue-green strategies, and how Argo Rollouts implements them.

## Why Progressive Delivery Matters for AI Agents

Before we dive into mechanics, let's establish why progressive delivery is non-negotiable for AI-powered services.

### Traditional Deployments: All-or-Nothing Risk

When you deploy a new version of your FastAPI agent with a standard Kubernetes Deployment:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: task-agent
spec:
  replicas: 3
  strategy:
    type: RollingUpdate
    rollingUpdate:
      maxSurge: 1
      maxUnavailable: 0
  template:
    metadata:
      labels:
        app: task-agent
    spec:
      containers:
      - name: agent
        image: myregistry/task-agent:v2.0
```

**Output:**
```
Deployment configuration with rolling update strategy
RollingUpdate creates 1 new pod while keeping all others running
Kubernetes gradually replaces old pods with new pods
```

Kubernetes replaces old pods with new ones gradually (rolling update). But here's the problem: **the entire system is running v2.0 within minutes**. If v2.0 has a subtle bug—maybe it incorrectly calculates priorities, or misses edge cases in task decomposition—every single user-facing request encounters that bug.

For traditional services (static content, CRUD APIs), this is manageable. You monitor error rates, and if something breaks, you rollback.

For AI agents, this is catastrophic.

### The AI Agent Problem: Subtle Failures

AI agent behavior changes are fundamentally different from traditional software bugs.

**Traditional bug**: "Function throws exception when input is null"
- **Detection**: Immediate (exception logged)
- **Impact scope**: Specific input condition
- **Fix**: Patch the function

**Agent behavior change**: "New prompt template makes agent more aggressive in scheduling tasks"
- **Detection**: Gradual (users notice subtly different task prioritization over hours)
- **Impact scope**: ALL requests, ALL users, cascading downstream effects
- **Fix**: Requires investigation, retraining, prompt refinement

With all-or-nothing deployments, you can't distinguish between:
- "This version is working well, let's keep it"
- "This version works but behaves differently than expected"
- "This version is broken"

Progressive delivery solves this by allowing **gradual, observable rollouts** where you can:

1. **Deploy to a small percentage of users** and observe behavior
2. **Run experiments** (e.g., canary traffic) and compare metrics
3. **Rollback instantly** if something goes wrong
4. **Promote gradually** as confidence increases

This is especially critical for agent behavior because the failures are:
- **Subtle**: Not obvious in logs or error rates
- **Wide-impact**: Affect all users when fully deployed
- **Observable through metrics**: Task completion rates, execution times, user satisfaction

Progressive delivery lets you catch these before they reach everyone.

## The Two Approaches to Progressive Delivery

There are two main strategies for safely deploying new versions: **canary** and **blue-green**. They solve the same problem but with different tradeoffs.

### Canary Deployments: Gradual Traffic Shift

**Concept**: Deploy the new version alongside the old version, then gradually shift traffic from old to new. Monitor metrics at each step. If something goes wrong, rollback by returning all traffic to the old version.

**Visual model**:

```
Initial state: 100% traffic to v1.0 (3 pods)
                │
                v
Step 1: Deploy v2.0 (1 pod), shift 20% traffic
        v1.0 (3 pods) ← 80% traffic
        v2.0 (1 pod)  ← 20% traffic
                │
                v
Step 2: Shift 50% traffic
        v1.0 (2 pods) ← 50% traffic
        v2.0 (2 pods) ← 50% traffic
                │
                v
Step 3: Shift 100% traffic
        v2.0 (3 pods) ← 100% traffic
                │
                v
        Scale down v1.0 (0 pods)
```

**Process**:

1. **Deploy new version**: Run new version in parallel with old
2. **Route percentage of traffic**: Send X% to new, (100-X)% to old
3. **Monitor metrics**: Watch error rates, latency, task completion in the canary traffic
4. **Increase traffic**: If metrics are healthy, increase percentage to new version
5. **Promote or rollback**: If metrics degrade, immediately return all traffic to old version

**Example timeline for a 5-step canary**:

```
Time 0:00 - Deploy v2.0, shift 10% traffic
          - Watch metrics for 5 minutes

Time 0:05 - Shift 25% traffic (error rate stable, latency OK)
          - Watch metrics for 5 minutes

Time 0:10 - Shift 50% traffic (still healthy)

Time 0:15 - Shift 75% traffic (task completion rate matches v1.0)

Time 0:20 - Shift 100% traffic (all metrics passing)
          - Scale down v1.0 (v2.0 now fully deployed)
```

**If something goes wrong**:

```
Time 0:08 - Shift to 25% traffic
          - Error rate spikes to 10% (abnormal)
          - ROLLBACK: immediately return 100% traffic to v1.0
          - Investigation: debug v2.0, find issue, redeploy
```

**Advantages**:
- **Gradual validation**: Real traffic validates new version before full deployment
- **Early detection**: Catch behavior changes while only affecting small percentage of users
- **Flexibility**: Adjust timing and percentages per deployment

**Disadvantages**:
- **Dual-stack overhead**: Both versions run simultaneously, increasing resource usage
- **Stateful behavior complexity**: Requests from same user may hit different versions
- **Requires metrics**: You must have alerting/metrics to detect problems in canary traffic

### Blue-Green Deployments: Instant Switch

**Concept**: Run two complete versions of your application (blue and green). All traffic currently goes to blue. Deploy green with the new version, fully test it, then switch ALL traffic to green instantly. If green has problems, switch back to blue.

**Visual model**:

```
Initial state: All traffic to BLUE (v1.0)
        BLUE (v1.0): 3 pods ← 100% traffic
        GREEN (idle): 0 pods
                │
                v
Deploy GREEN: Deploy v2.0 to green environment
        BLUE (v1.0): 3 pods ← 100% traffic
        GREEN (v2.0): 3 pods ← 0% traffic (testing phase)
                │
                v
Test GREEN: Validate all functionality before traffic switch
        (Run synthetic tests, manual smoke tests)
                │
                v
Switch traffic: Instant switch to GREEN
        BLUE (v1.0): 3 pods ← 0% traffic
        GREEN (v2.0): 3 pods ← 100% traffic
                │
                v
Rollback (if needed): Instant switch back to BLUE
        BLUE (v1.0): 3 pods ← 100% traffic
        GREEN (v2.0): 3 pods ← 0% traffic
```

**Process**:

1. **Deploy green**: Full parallel deployment of new version
2. **Pre-flight validation**: Synthetic tests, health checks, smoke tests (NO real traffic)
3. **Traffic switch**: Route 100% traffic from blue to green instantly
4. **Observe**: Watch metrics in green
5. **Rollback or promote**: If something goes wrong, switch back to blue. If healthy, scale down blue.

**Example timeline for blue-green**:

```
Time 0:00 - Deploy v2.0 to GREEN (parallel to BLUE)
          - Start health checks

Time 0:05 - GREEN health checks pass
          - Run synthetic smoke tests (deploy test pod, hit endpoints)

Time 0:10 - All tests pass
          - Switch router: 100% traffic to GREEN (instant)

Time 0:15 - Monitor GREEN metrics
          - Error rate: 0.1% (normal)
          - Task completion: 99.8% (healthy)

Time 0:30 - GREEN stable for 15 minutes
          - Scale down BLUE
          - v2.0 fully deployed
```

**If something goes wrong**:

```
Time 0:12 - 2 minutes after traffic switch
          - Error rate in GREEN: 8% (abnormal)
          - IMMEDIATE rollback: switch 100% traffic back to BLUE
          - Investigation: debug v2.0, find issue, redeploy
```

**Advantages**:
- **Zero-downtime**: Traffic never interrupted, just switched
- **Instant rollback**: If something goes wrong, switch back in seconds
- **No dual-stack overhead**: Only two full replicas (one active, one hot)
- **Clear before/after**: Tests run before any real traffic hits new version

**Disadvantages**:
- **Large resource footprint**: Must run both versions fully (2x resource usage)
- **Limited real-traffic validation**: Tests are synthetic, not real user patterns
- **Stateless requirement**: Works best for stateless services (agents are typically stateless)

## Canary vs Blue-Green: Which to Choose?

There's no universal "better" strategy. The choice depends on your constraints:

| Factor | Canary | Blue-Green |
|--------|--------|-----------|
| **Resource usage** | High (dual-stack, gradual) | Very high (2x full deployment) |
| **Time to deploy** | Slow (5-20 minutes) | Fast (5-10 minutes) |
| **Real-traffic validation** | Yes (best for catching agent behavior changes) | No (synthetic only) |
| **Rollback time** | Fast (instant) | Instant |
| **Best for** | Services where subtle behavior matters | Services requiring zero-downtime with less resource |
| **Example use case** | AI agents (need real-traffic validation) | Web APIs (behavior is well-defined) |

**For AI agents**, canary deployments are often superior because:
- **Subtle behavior changes** require real traffic to validate
- **Gradual exposure** minimizes impact if something goes wrong
- **Metrics collection** lets you detect agent behavior differences (task completion, latency, satisfaction)

## Introducing Argo Rollouts

Now that you understand the concepts, let's introduce the tool: **Argo Rollouts**.

Argo Rollouts is a Kubernetes controller that automates progressive delivery. Instead of writing your own canary/blue-green logic, you declare a **Rollout** resource (similar to Deployment) with a **strategy** (canary or blue-green) and **steps** (traffic percentages, timing, analysis).

Argo Rollouts then:
- Manages the old and new versions
- Routes traffic according to your strategy
- Monitors metrics and health checks
- Promotes or rolls back based on analysis

**Basic structure**:

```yaml
apiVersion: argoproj.io/v1alpha1
kind: Rollout
metadata:
  name: task-agent
spec:
  # How many replicas total
  replicas: 3

  # What version are we running
  template:
    spec:
      containers:
      - name: agent
        image: myregistry/task-agent:v2.0

  # Progressive delivery strategy
  strategy:
    canary:
      steps:
      - setWeight: 20  # 20% traffic to new version
      - pause: { duration: 5m }  # Wait 5 minutes
      - setWeight: 50  # 50% traffic
      - pause: { duration: 5m }
      - setWeight: 100  # 100% traffic (promotion)

      # Analysis: check metrics to promote/rollback
      analysis:
        interval: 30s
        threshold: 5
        metrics:
        - name: error_rate
          query: rate(http_requests_total{status=~"5.."}[5m])
          successCriteria: '< 0.01'  # Less than 1% error rate
```

**Output:**
```
Rollout resource configured with canary strategy
Steps define: 20% traffic → 5min pause → 50% traffic → 5min pause → 100% traffic
Analysis checks error_rate < 1% at 30-second intervals
```

When you apply this Rollout:

```bash
kubectl apply -f task-agent-rollout.yaml
```

**Output:**
```
rollout.argoproj.io/task-agent created
```

Argo Rollouts takes over:

1. **Deploys v2.0**: Creates pods with new version
2. **Shifts traffic**: At step 1, routes 20% to new version, 80% to old
3. **Monitors analysis**: Every 30 seconds, checks error_rate metric
4. **Promotes or waits**: If analysis passes, waits 5 minutes, then moves to next step
5. **Full promotion**: After all steps pass analysis, promotes v2.0 to 100%

If analysis fails at any step:

```bash
kubectl get rollout task-agent
```

**Output:**
```
NAME         REPLICAS   UPDATED   READY   AVAILABLE   PHASE
task-agent   3          1         3       3           Degraded
```

You can immediately rollback:

```bash
kubectl rollout undo rollout/task-agent
```

**Output:**
```
rollout.argoproj.io/task-agent rolled back
```

And Argo Rollouts returns all traffic to the previous version.

## The Rollout CRD: Key Concepts

Let's map the Rollout resource to the concepts you know:

**Deployment vs Rollout** (conceptual comparison):

```yaml
# Traditional Deployment (all-or-nothing)
apiVersion: apps/v1
kind: Deployment
metadata:
  name: task-agent
spec:
  replicas: 3
  strategy:
    type: RollingUpdate  # "Gradually replace pods"
  template:
    spec:
      containers:
      - image: myregistry/task-agent:v2.0
```

**Output:**
```
Deployment rolls out all 3 pods with new version
Uses RollingUpdate strategy: replaces old pods gradually
No traffic management, no analysis, no safe rollback
```

```yaml
# Argo Rollout (progressive delivery)
apiVersion: argoproj.io/v1alpha1
kind: Rollout
metadata:
  name: task-agent
spec:
  replicas: 3
  template:
    spec:
      containers:
      - image: myregistry/task-agent:v2.0

  strategy:
    canary:
      steps:
      - setWeight: 20
      - pause: { duration: 5m }
      - setWeight: 50
      # ... more steps
```

**Output:**
```
Rollout gradually shifts traffic: 20% → 50% → 100%
Pauses at each step to validate metrics
Supports instant rollback if analysis fails
```

**Key differences**:

| Aspect | Deployment | Rollout |
|--------|-----------|---------|
| **Traffic management** | None (all pods get traffic) | Explicit (shift %, pause, analyze) |
| **Validation** | Health checks only | Health checks + custom metrics |
| **Rollback** | Manual `kubectl rollout undo` | Automatic on analysis failure |
| **Safe for agents** | No (all-or-nothing risk) | Yes (gradual, observable) |

## Integration with ArgoCD: The Full Picture

You might be wondering: "I'm already using ArgoCD to deploy applications. How does Argo Rollouts fit in?"

**ArgoCD deploys resources to Kubernetes. Argo Rollouts is a resource that ArgoCD can deploy.**

The architecture looks like:

```
Your Git Repository
    ↓
    ├─ deployment.yaml (or rollout.yaml)
    ├─ service.yaml
    └─ values.yaml

    ↓

ArgoCD (watches Git)
    │
    └─→ Applies resources to cluster
        (kubectl apply)

    ↓

Kubernetes Cluster
    │
    ├─ Deployment / Rollout (manages pods)
    │
    └─ Service (routes traffic)
        ↓
        └─→ If Rollout: Argo Rollouts controller
                        manages progressive delivery
```

**Example workflow**:

1. **You commit a new version to Git**:
   ```bash
   git commit -m "feat: improve task decomposition in agent"
   git push origin main
   ```

2. **GitHub Actions CI/CD pipeline runs**:
   - Builds new image: `myregistry/task-agent:v2.0`
   - Tests it
   - Pushes to registry
   - **Updates rollout.yaml**: Changes image to v2.0

3. **ArgoCD detects Git change**:
   ```bash
   argocd app get task-agent
   ```

   **Output:**
   ```
   Name:               task-agent
   Namespace:          default
   Status:             OutOfSync
   Repository:         https://github.com/you/agent-repo
   ```

4. **ArgoCD syncs (applies rollout.yaml)**:
   ```bash
   argocd app sync task-agent
   ```

   **Output:**
   ```
   SYNCED - Rollout task-agent created with image v2.0
   ```

5. **Argo Rollouts controller takes over**:
   - Starts canary deployment
   - Shifts traffic: 20% → 50% → 100%
   - Monitors metrics
   - Completes deployment or rolls back

This is the **GitOps + Progressive Delivery** integration: Git is your source of truth, ArgoCD keeps the cluster in sync with Git, and Argo Rollouts automates safe deployment strategies.

## Canary vs Blue-Green Revisited: Implementation with Argo Rollouts

Now let's see how each strategy looks as a Rollout resource.

### Canary Rollout

```yaml
apiVersion: argoproj.io/v1alpha1
kind: Rollout
metadata:
  name: task-agent
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
        image: myregistry/task-agent:v2.0
        ports:
        - containerPort: 8000

  strategy:
    canary:
      steps:
      - setWeight: 20    # Send 20% traffic to new version
      - pause:
          duration: 5m   # Wait 5 minutes
      - setWeight: 50    # Send 50% traffic
      - pause:
          duration: 5m
      - setWeight: 100   # Send 100% (promotion complete)

      # Traffic split: maintained by service mesh or ingress
      # (Argo Rollouts coordinates with your networking layer)
```

**Output:**
```
Rollout configured for canary deployment
Step 1: Route 20% to new version, pause 5m
Step 2: Route 50%, pause 5m
Step 3: Route 100% (complete)
```

### Blue-Green Rollout

```yaml
apiVersion: argoproj.io/v1alpha1
kind: Rollout
metadata:
  name: task-agent
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
        image: myregistry/task-agent:v2.0

  strategy:
    blueGreen:
      activeService: task-agent-active    # Current version (blue)
      previewService: task-agent-preview  # New version (green)

      prePromotionAnalysis:
        interval: 30s
        threshold: 3
        metrics:
        - name: success_rate
          query: |
            sum(rate(http_requests_total{status="200"}[5m])) /
            sum(rate(http_requests_total[5m]))
          successCriteria: '> 0.99'

      autoPromotionEnabled: true
      autoPromotionSeconds: 300  # Promote automatically after 5 min if analysis passes
```

**Output:**
```
Rollout configured for blue-green deployment
Active service routes to current version (blue)
Preview service routes to new version (green)
Pre-promotion analysis runs before traffic switch
Auto-promotion after 5 min if success_rate > 99%
```

## Summary: The Progressive Delivery Stack

Here's how everything fits together:

```
Your AI Agent Code
    ↓
GitHub Actions (Lesson 2)
    ├─ Build image
    ├─ Run tests
    └─ Push to registry
    ↓
Git repository with rollout.yaml
    ↓
ArgoCD (Lessons 5-12)
    └─ Watches Git, applies Rollout to cluster
    ↓
Kubernetes Cluster
    └─ Argo Rollouts controller (Lesson 13 - THIS LESSON)
        ├─ Manages canary or blue-green
        ├─ Shifts traffic safely
        └─ Validates with metrics
    ↓
Your service: Progressive, observable, safe deployments
```

When you deploy a new version, it goes through:

1. **Code commit** → CI pipeline validates
2. **Image push** → Registry stores artifact
3. **Git commit** (rollout.yaml) → ArgoCD notices
4. **Argo Rollouts deployment** → Canary or blue-green strategy executes
5. **Metrics collection** → System validates new version with real traffic
6. **Promotion or rollback** → Safe transition or instant recovery

This is production-grade deployment for AI agents.

## Key Concepts to Remember

Before moving to implementation lessons, internalize these:

**Canary deployments**:
- Gradual traffic shift (10% → 25% → 50% → 100%)
- Requires real traffic for validation
- Best for agent behavior changes
- Uses dual-stack (both versions running)

**Blue-green deployments**:
- Instant traffic switch (0% → 100%)
- Tests before any real traffic
- Best for zero-downtime updates
- Requires 2x resources temporarily

**Argo Rollouts**:
- Kubernetes controller (CRD: Rollout)
- Automates canary/blue-green strategies
- Monitors metrics, promotes or rolls back
- Integrates with ArgoCD

**Why progressive delivery matters for AI agents**:
- Agent behavior changes are subtle (not obvious in error logs)
- Real traffic needed to validate
- Rollback must be instant
- Observability (metrics) is critical

## Try With AI

**Setup**: Open your agent repository in your editor. You'll work with Argo Rollouts concepts.

**Prompt 1 - Understanding Your Current Risk**

Ask AI: "I currently deploy my FastAPI agent using a standard Kubernetes Deployment with RollingUpdate strategy. Explain what happens if the new version has a subtle bug in task prioritization logic—how quickly will all users see it, and what's my rollback process?"

This helps you understand the all-or-nothing risk in your current approach.

**Prompt 2 - Canary vs Blue-Green Decision**

Ask AI: "My agent processes user requests in isolation (no session state). For deploying a new agent version with improved prompt templates, should I use canary or blue-green strategy? Explain the tradeoffs for my use case."

The point here: understand which strategy fits YOUR constraints and workload characteristics.

**Prompt 3 - Translating Concepts to Manifests**

Ask AI: "Show me a Rollout manifest for a canary deployment of an agent with 3 replicas. The canary should shift traffic: 20% for 5m → 50% for 5m → 100%. Include a simple success metric (error rate should stay below 1%)."

Use this to see how Argo Rollouts concepts map to actual YAML configuration.

**Reflection Questions**

As you work through these prompts, ask yourself:
- How is progressive delivery different from traditional rolling updates?
- Why is gradual validation important for AI agent behavior changes?
- What metrics would you actually monitor when deploying a new agent version?
- How would you explain blue-green deployment to a teammate unfamiliar with it?
