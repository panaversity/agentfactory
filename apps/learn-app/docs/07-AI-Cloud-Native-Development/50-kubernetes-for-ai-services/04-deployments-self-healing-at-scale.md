---
sidebar_position: 4
chapter: 50
lesson: 4
duration_minutes: 45
title: "Deployments: Self-Healing at Scale"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Manual deployment manifest construction builds understanding of declarative workload management"
cognitive_load:
  concepts_count: 10
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Explain why direct Pod deployment is rare in production (ephemeral nature, lack of self-healing)"
    bloom_level: "Understand"
  - id: LO2
    description: "Create a Deployment manifest with proper replica configuration and pod templates"
    bloom_level: "Apply"
  - id: LO3
    description: "Understand the Deployment → ReplicaSet → Pod hierarchy and resource ownership"
    bloom_level: "Understand"
  - id: LO4
    description: "Perform rolling updates by modifying image versions and monitoring rollout progress"
    bloom_level: "Apply"
  - id: LO5
    description: "Rollback failed deployments using kubectl rollout undo"
    bloom_level: "Apply"
  - id: LO6
    description: "Demonstrate self-healing behavior by deleting pods and observing controller reconciliation"
    bloom_level: "Apply"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "5. Problem Solving"
    competency: "5.2 Identifying needs and technological responses"
  - objective_id: LO2
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO3
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
  - objective_id: LO4
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO5
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
  - objective_id: LO6
    competency_area: "5. Problem Solving"
    competency: "5.3 Using digital tools to solve problems"
---

# Deployments: Self-Healing at Scale

Delete the Pod you created in Lesson 3:

```bash
kubectl delete pod nginx-pod
```

**Output:**
```
pod "nginx-pod" deleted
```

Now check if it came back:

```bash
kubectl get pods
```

**Output:**
```
No resources found in default namespace.
```

It didn't come back. That's the problem with bare Pods—they're mortal. Delete one, it's gone forever. Your application is down until you manually recreate it.

**Deployments** fix this. A Deployment is a manager that continuously monitors your Pods and ensures the desired count always exists. Delete a Pod? The Deployment creates a replacement. Node crashes? Pods get rescheduled elsewhere. This lesson teaches you to declare WHAT you want (3 replicas of your agent), and let Kubernetes handle HOW to maintain it.

---

## The Problem with Direct Pod Deployment

Direct Pod deployment is like hiring workers without a manager. You manage each worker individually:

- Worker 1 quits? Hire someone new.
- Need more workers? Interview and hire each one.
- Want to update their uniforms? Replace each worker one at a time.

This manual approach doesn't scale.

### Why Pods Are Ephemeral

Recall from Lesson 2 that Pods are the smallest deployable unit in Kubernetes. But Pods have a critical limitation: **they are designed to be ephemeral (temporary)**.

A Pod can be deleted, evicted, or crash at any time:

```
Pod lifecycle:
┌──────┐    ┌────────┐    ┌──────────┐    ┌──────┐
│Pending│ → │ Running│ → │Succeeded │ → │Deleted│
└──────┘    └────────┘    │or Failed │    └──────┘
             (working)     └──────────┘
```

If you deploy a Pod directly with `kubectl run hello --image=nginx`, and the Pod crashes, Kubernetes does NOT automatically create a new Pod. That container is gone. Your service is down.

### The Analogy: Deployment as Manager

A Deployment is a manager that guarantees a desired state:

```
You: "I want 3 workers doing X jobs"

Deployment Manager:
- Checks: Do we have 3 workers?
- No: Creates new worker
- Yes: Checks if they're healthy
- Unhealthy: Replaces them
- Done: Moves to next check
```

The manager runs continuously, observing reality and fixing mismatches. This is Kubernetes' declarative model in action.

---

## The Deployment Abstraction: A Hierarchy

Deployments don't directly manage Pods. They use an intermediate abstraction called a **ReplicaSet**.

```
Deployment (what you create)
    ↓ manages
ReplicaSet (intermediate controller)
    ↓ manages
Pods (actual running containers)
```

### Why This Hierarchy?

**Deployment**: High-level abstraction for updates, rollbacks, scaling.

**ReplicaSet**: Low-level abstraction that says "keep N copies of this Pod template running."

**Pods**: Actual containers running your workload.

The hierarchy allows different responsibilities:
- A Deployment owns ReplicaSets
- Each ReplicaSet owns multiple Pods
- When you update a Deployment's image version, it creates a NEW ReplicaSet with new Pods, leaving the old ReplicaSet in place (for rollbacks)

---

## Creating Your First Deployment

Let's create a Deployment manifest for a simple nginx service.

### Deployment YAML Structure

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: hello-deployment
  labels:
    app: hello
spec:
  replicas: 3
  selector:
    matchLabels:
      app: hello
  template:
    metadata:
      labels:
        app: hello
    spec:
      containers:
      - name: hello
        image: nginx:1.24
        ports:
        - containerPort: 80
        resources:
          limits:
            memory: "128Mi"
            cpu: "500m"
```

**Output:**
(This is just the manifest structure; we'll apply it next)

### Understanding Each Field

**`replicas: 3`**
Desired number of Pod copies. Kubernetes ensures this many exist at all times.

**`selector.matchLabels.app: hello`**
The Deployment finds its Pods by looking for labels matching `app: hello`. This is how the Deployment knows which Pods belong to it.

**`template`**
The Pod template. This is identical to a Pod spec from Lesson 2. Every Pod created by this Deployment uses this template.

**Important**: The labels in `template.metadata.labels` MUST match the selector in `selector.matchLabels`. If they don't match, Kubernetes can't find the Pods, and the Deployment creates an infinite number of new Pods trying to satisfy the replicas requirement.

**`containers`**
One or more containers running in each Pod. Here we have a single nginx container on port 80, with memory and CPU limits.

---

## Deploying and Verifying

Save the manifest above as `deployment.yaml` and deploy it:

```bash
kubectl apply -f deployment.yaml
```

**Output:**
```
deployment.apps/hello-deployment created
```

Check the Deployment status:

```bash
kubectl get deployments
```

**Output:**
```
NAME               READY   UP-TO-DATE   AVAILABLE   AGE
hello-deployment   3/3     3            3           30s
```

**What each column means:**
- **READY**: 3 of 3 desired replicas are ready
- **UP-TO-DATE**: 3 replicas are running the desired image version
- **AVAILABLE**: 3 replicas are ready to accept traffic
- **AGE**: Deployment was created 30 seconds ago

Check the Pods created by this Deployment:

```bash
kubectl get pods -l app=hello
```

**Output:**
```
NAME                               READY   STATUS    RESTARTS   AGE
hello-deployment-7d4b8c9f5-abc12   1/1     Running   0          30s
hello-deployment-7d4b8c9f5-def34   1/1     Running   0          30s
hello-deployment-7d4b8c9f5-ghi56   1/1     Running   0          30s
```

Notice the Pod names: `hello-deployment-[ReplicaSet-hash]-[random-id]`. The ReplicaSet is embedded in the name.

Check the ReplicaSet:

```bash
kubectl get replicasets
```

**Output:**
```
NAME                         DESIRED   CURRENT   READY   AGE
hello-deployment-7d4b8c9f5   3         3         3       30s
```

The ReplicaSet `hello-deployment-7d4b8c9f5` is responsible for ensuring 3 Pods exist.

---

## Self-Healing in Action

Now demonstrate Kubernetes' self-healing. Intentionally delete one Pod:

```bash
kubectl delete pod hello-deployment-7d4b8c9f5-abc12
```

**Output:**
```
pod "hello-deployment-7d4b8c9f5-abc12" deleted
```

Wait a few seconds, then check Pods again:

```bash
kubectl get pods -l app=hello
```

**Output:**
```
NAME                               READY   STATUS    RESTARTS   AGE
hello-deployment-7d4b8c9f5-def34   1/1     Running   0          2m45s
hello-deployment-7d4b8c9f5-ghi56   1/1     Running   0          2m45s
hello-deployment-7d4b8c9f5-xyzab   1/1     Running   0          5s
```

**What happened:**
1. You deleted Pod `abc12`
2. Kubernetes' ReplicaSet controller detected: "I should have 3 Pods, but I only have 2"
3. It immediately created a new Pod `xyzab` (timestamp: 5 seconds ago)
4. The Deployment still shows 3/3 ready replicas

This is self-healing: Kubernetes automatically recovers from Pod failures without human intervention.

---

## Scaling Deployments

Increase the replica count from 3 to 5:

```bash
kubectl scale deployment hello-deployment --replicas=5
```

**Output:**
```
deployment.apps/hello-deployment scaled
```

Check the result:

```bash
kubectl get deployments hello-deployment
```

**Output:**
```
NAME               READY   UP-TO-DATE   AVAILABLE   AGE
hello-deployment   5/5     5            5           5m
```

Check Pods:

```bash
kubectl get pods -l app=hello
```

**Output:**
```
NAME                               READY   STATUS    RESTARTS   AGE
hello-deployment-7d4b8c9f5-def34   1/1     Running   0          5m30s
hello-deployment-7d4b8c9f5-ghi56   1/1     Running   0          5m30s
hello-deployment-7d4b8c9f5-xyzab   1/1     Running   0          4m
hello-deployment-7d4b8c9f5-pqr78   1/1     Running   0          10s
hello-deployment-7d4b8c9f5-stu90   1/1     Running   0          10s
```

Two new Pods (timestamps: 10 seconds ago) were created to reach 5 replicas.

Scale back down:

```bash
kubectl scale deployment hello-deployment --replicas=3
```

**Output:**
```
deployment.apps/hello-deployment scaled
```

Kubernetes will terminate 2 Pods gracefully.

---

## Rolling Updates: Upgrading Your Application

Your application needs to upgrade from nginx 1.24 to nginx 1.25. Update the image:

```bash
kubectl set image deployment/hello-deployment hello=nginx:1.25
```

**Output:**
```
deployment.apps/hello-deployment image updated
```

Watch the rollout:

```bash
kubectl rollout status deployment/hello-deployment
```

**Output:**
```
Waiting for deployment "hello-deployment" rollout to finish: 1 out of 3 new replicas have been updated...
Waiting for deployment "hello-deployment" rollout to finish: 2 out of 3 new replicas have been updated...
deployment "hello-deployment" successfully rolled out
```

What Kubernetes did:
1. Created a new ReplicaSet with image nginx:1.25
2. Started a new Pod with the new image
3. Once the new Pod was ready, terminated an old Pod
4. Repeated until all 3 Pods were running nginx 1.25
5. Left the old ReplicaSet in place (for rollback)

Check the ReplicaSets:

```bash
kubectl get replicasets
```

**Output:**
```
NAME                         DESIRED   CURRENT   READY   AGE
hello-deployment-7d4b8c9f5   0         0         0       10m
hello-deployment-8f9c2k3m   3         3         3       2m
```

The old ReplicaSet has 0 desired replicas (no Pods). The new ReplicaSet has 3 Pods running.

---

## Rollback: Recovering from Failed Updates

If the update introduced a bug and the new version doesn't work, rollback immediately:

```bash
kubectl rollout undo deployment/hello-deployment
```

**Output:**
```
deployment.apps/hello-deployment rolled back
```

Kubernetes:
1. Recreates the old ReplicaSet with nginx 1.24
2. Scales down the new ReplicaSet
3. Your Pods are back on the stable version

Check status:

```bash
kubectl rollout status deployment/hello-deployment
```

**Output:**
```
deployment.apps/hello-deployment successfully rolled back
```

View the history of updates:

```bash
kubectl rollout history deployment/hello-deployment
```

**Output:**
```
deployment.apps/hello-deployment
REVISION  CHANGE-CAUSE
1         <none>
2         <none>
3         <none>
```

Each revision is a ReplicaSet. You can even rollback to a specific revision:

```bash
kubectl rollout undo deployment/hello-deployment --to-revision=1
```

---

## The Declarative Model at Work

Reflect on what happened:

| Action | What You Specified | What Kubernetes Did |
|--------|-------------------|-------------------|
| Create Deployment | "I want 3 replicas of nginx:1.24" | Created ReplicaSet, created 3 Pods, monitored health |
| Delete Pod | (pod deleted) | ReplicaSet detected mismatch: 2 instead of 3, created new Pod |
| Scale to 5 | "I want 5 replicas" | ReplicaSet scaled up, created 2 new Pods |
| Update image | "I want nginx:1.25" | Created new ReplicaSet, performed rolling update, old ReplicaSet available for rollback |
| Rollback | "Go back" | Reactivated old ReplicaSet, terminated new Pods |

You never said HOW. You declared WHAT you wanted, and Kubernetes' controllers continuously worked to achieve that state.

This is the power of Kubernetes: **declare your desired state, and Kubernetes keeps you there.**

---

## Key Concepts Summary

**Deployment**: High-level Kubernetes resource that manages ReplicaSets and enables rolling updates, rollbacks, and scaling.

**ReplicaSet**: Controller that ensures a desired number of Pod copies exist. Deployments use ReplicaSets internally.

**Replicas**: The desired number of Pod copies. Kubernetes maintains this count automatically.

**Selector**: Labels used to identify which Pods belong to a Deployment. Must match Pod labels.

**Rolling Update**: Update strategy where old Pods are replaced gradually, ensuring availability during updates.

**Self-Healing**: Automatic recovery when Pods fail. The ReplicaSet controller detects failure and creates replacement Pods.

**Reconciliation Loop**: Kubernetes' control plane continuously compares desired state (replicas: 3) with actual state (2 Pods running), and takes action to close the gap.

---

## Try With AI

Open a terminal and work through these scenarios with an AI assistant's help:

### Scenario 1: Design a Deployment

**Your task:**
Create a Deployment manifest for a Python Flask application that:
- Runs 2 replicas
- Uses image `myregistry.azurecr.io/flask-app:v2.1`
- Exposes port 5000
- Requires 256Mi memory and 250m CPU

Ask AI: "Create a Deployment manifest for a Flask app with these requirements: [list your requirements]"

Review AI's response:
- Does the replica count match your requirement (2)?
- Is the port correct (5000)?
- Are resource limits set correctly?
- Are labels and selectors consistent?

Tell AI your constraints: "The image needs to be pulled with a secret called my-registry-secret because it's in a private registry."

Ask AI: "Update the manifest to handle the private registry secret."

**Reflection:**
- What changed in the manifest?
- Why is imagePullSecrets needed for private registries?
- Could you deploy this immediately, or do other prerequisites exist?

### Scenario 2: Troubleshoot a Deployment

**Your task:**
You deployed a Deployment, but `kubectl get deployments` shows `0/3 READY`. The manifest looks correct, but Pods aren't running.

Ask AI: "I deployed a Deployment for my app, but the READY status shows 0/3. What could be wrong?"

AI might suggest checking:
- Pod status with `kubectl describe pod`
- Container logs with `kubectl logs`
- Events with `kubectl get events`

Ask: "Show me the exact commands I should run to diagnose this."

**Reflection:**
- What troubleshooting steps did AI suggest?
- Which command would tell you if the image pull failed?
- What would you look for in Pod events to diagnose the issue?

### Scenario 3: Plan a Zero-Downtime Update

**Your task:**
You need to update your Deployment from version 1.0 to 2.0, but the service cannot have downtime. You're unsure about the rolling update strategy.

Ask AI: "Explain the rolling update process. I have 3 replicas—walk me through exactly what happens when I update the image."

AI should explain:
1. New ReplicaSet created
2. One Pod replaced at a time (default MaxSurge/MaxUnavailable)
3. Health checks before continuing
4. Old ReplicaSet retained for rollback

Ask: "What happens if the new version has a bug and Pods crash? How do I recover?"

**Reflection:**
- Did AI's explanation match the commands you ran earlier?
- What's the advantage of retaining the old ReplicaSet?
- How quickly can you rollback if needed?
