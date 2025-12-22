---
sidebar_position: 13
chapter: 50
lesson: 13
duration_minutes: 45
title: "StatefulSets: When Your Agent Needs Identity"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Manual StatefulSet configuration builds understanding of stateful workload patterns"
cognitive_load:
  concepts_count: 9
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Explain when to use StatefulSets vs Deployments"
    bloom_level: "Understand"
  - id: LO2
    description: "Understand stable network identity (pod-0, pod-1, pod-2)"
    bloom_level: "Understand"
  - id: LO3
    description: "Understand stable storage with volumeClaimTemplates"
    bloom_level: "Understand"
  - id: LO4
    description: "Create StatefulSets with ordered Pod creation/deletion"
    bloom_level: "Apply"
  - id: LO5
    description: "Configure headless Services for StatefulSet DNS"
    bloom_level: "Apply"
  - id: LO6
    description: "Perform rolling updates on StatefulSets"
    bloom_level: "Apply"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "5. Problem Solving"
    competency: "5.2 Identifying needs and technological responses"
  - objective_id: LO2
    competency_area: "1. Information and Data Literacy"
    competency: "1.3 Managing data, information and digital content"
  - objective_id: LO3
    competency_area: "1. Information and Data Literacy"
    competency: "1.3 Managing data, information and digital content"
  - objective_id: LO4
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO5
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO6
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
---

# StatefulSets: When Your Agent Needs Identity

You've deployed stateless agents with Deployments—web services that don't care which replica handles their request. But what about vector databases? Qdrant, Milvus, or other stateful services that require stable identity and ordered startup.

Try deploying a Qdrant vector database with a standard Deployment. When Pods scale down, which replica loses its data? When they scale up, which replica is the primary? These questions break Deployment's "any replica is interchangeable" assumption.

**StatefulSets** solve this: Pods get stable hostnames (qdrant-0, qdrant-1, qdrant-2), ordered lifecycle (always start 0 first, then 1, then 2), and persistent storage that follows them. Your vector database nodes maintain their identity even when they restart.

---

## Why Deployments Aren't Enough

Deployments excel at stateless agents: API servers, workers, load-balanced services where "pod-abc123" crashing is fine because "pod-def456" is identical. The power of Deployments comes from treating Pods as disposable. When a Pod crashes, the Deployment controller creates a replacement with a new random name, and nothing cares because the application layer doesn't depend on Pod identity.

But some workloads violate this assumption. Vector databases, distributed caches, and stateful AI services need something different:

| Characteristic | Deployment | StatefulSet |
|---|---|---|
| **Pod Identity** | Random (pod-abc123) | Stable (qdrant-0, qdrant-1) |
| **Scaling Order** | Parallel (all at once) | Ordered (0, then 1, then 2) |
| **Storage** | Ephemeral or shared | Per-Pod, persistent |
| **Network** | Dynamic IP, service routes | Stable DNS: pod-0.service-name |
| **Use Case** | Stateless agents, APIs | Databases, message brokers, AI inference replicas |

### Why Pod Identity Matters

When you run Qdrant (a distributed vector database), each replica maintains a shard of your embedding index. The cluster topology is fixed: replica 0 owns shards A-C, replica 1 owns D-F, replica 2 owns G-I. Other replicas need to contact "qdrant-0" to access shard A. If replica 0 crashes and is replaced with a new Pod named "qdrant-xyz123", the topology breaks. Other nodes can't find the data they need.

### The Problem Scenario

You want to deploy Qdrant with 3 replicas for distributed vector search. Each replica maintains shards of your embedding index:

```bash
# With Deployment, Pod IPs change constantly
kubectl get pods -l app=qdrant
```

**Output:**
```
NAME                      READY   STATUS    RESTARTS   AGE
qdrant-66d4cb8c9c-abc12   1/1     Running   0          2m
qdrant-66d4cb8c9c-def45   1/1     Running   0          1m
qdrant-66d4cb8c9c-ghi67   1/1     Running   0          30s
```

When pod `qdrant-66d4cb8c9c-abc12` crashes, Kubernetes replaces it with `qdrant-66d4cb8c9c-zyx98`. But Qdrant expects `qdrant-0`, `qdrant-1`, `qdrant-2` to remain stable. The cluster topology breaks.

---

## Stable Network Identity

StatefulSets guarantee three critical things:

1. **Stable Hostname**: Pod 0 is always `qdrant-0`, Pod 1 is always `qdrant-1`. Even if the Pod crashes and restarts, it keeps the same name.
2. **Stable DNS**: Access via `qdrant-0.qdrant-service.default.svc.cluster.local`. Kubernetes DNS always resolves this to the current Pod.
3. **Ordered Lifecycle**: Pod 0 starts first, followed by Pod 1, then Pod 2. When scaling down, Pod 2 terminates first, then Pod 1, then Pod 0. This predictability is essential for stateful services that need initialization order.

This combination solves the distributed systems problem: services can discover each other by name, and that name never changes.

### Headless Service

The mechanism is a **headless Service** (no ClusterIP, just DNS). Instead of creating a single virtual IP that load-balances across Pods, a headless Service tells Kubernetes: "Don't create a virtual IP. Just point DNS directly at each Pod individually."

```yaml
apiVersion: v1
kind: Service
metadata:
  name: qdrant-service
spec:
  clusterIP: None  # Headless: no single IP, direct to Pods
  selector:
    app: qdrant
  ports:
  - port: 6333
    name: api
```

The `serviceName: qdrant-service` in the StatefulSet must match this Service name exactly.

**Apply and verify:**

```bash
kubectl apply -f qdrant-service.yaml
kubectl get service qdrant-service
```

**Output:**
```
NAME              TYPE        CLUSTER-IP   EXTERNAL-IP   PORT(S)    AGE
qdrant-service    ClusterIP   None         <none>        6333/TCP   2s
```

The `None` ClusterIP (instead of something like `10.96.0.10`) tells Kubernetes: "Don't create a virtual IP. This is headless."

### Stable Pod DNS

When you create a StatefulSet with this Service, each Pod gets a stable DNS name:

```bash
# From inside any Pod, query DNS
nslookup qdrant-0.qdrant-service.default.svc.cluster.local
```

**Output:**
```
Server:    10.96.0.10
Address 1: 10.96.0.10 kube-dns.kube-system.svc.cluster.local

Name:      qdrant-0.qdrant-service.default.svc.cluster.local
Address 1: 10.244.0.5
```

Pod `qdrant-0` is always accessible at that hostname, even if its IP changes internally.

---

## Creating a StatefulSet

StatefulSets are similar to Deployments in structure, but with critical additions: `serviceName` (must match the headless Service), `volumeClaimTemplates` (creates a PVC per Pod), and guaranteed ordering.

Here's a StatefulSet for Qdrant with 3 replicas, each with persistent storage:

```yaml
apiVersion: apps/v1
kind: StatefulSet
metadata:
  name: qdrant
spec:
  serviceName: qdrant-service  # Must match headless Service name
  replicas: 3
  selector:
    matchLabels:
      app: qdrant
  template:
    metadata:
      labels:
        app: qdrant
    spec:
      containers:
      - name: qdrant
        image: qdrant/qdrant:latest
        ports:
        - containerPort: 6333
          name: api
        volumeMounts:
        - name: qdrant-data
          mountPath: /qdrant/storage
        resources:
          requests:
            cpu: 500m
            memory: 512Mi
          limits:
            cpu: 1000m
            memory: 1Gi
  volumeClaimTemplates:
  - metadata:
      name: qdrant-data
    spec:
      accessModes: [ "ReadWriteOnce" ]
      resources:
        requests:
          storage: 10Gi
```

**Apply the StatefulSet:**

```bash
kubectl apply -f qdrant-statefulset.yaml
kubectl get statefulsets
```

**Output:**
```
NAME     READY   AGE
qdrant   0/3     3s
```

**Watch the ordered creation:**

```bash
kubectl get pods -l app=qdrant --watch
```

**Output:**
```
NAME      READY   STATUS            RESTARTS   AGE
qdrant-0  0/1     Init:0/1          0          2s
qdrant-1  0/1     Pending           0          1s   # Waits for qdrant-0 Ready
qdrant-2  0/1     Pending           0          0s   # Waits for qdrant-1 Ready

# After qdrant-0 is Ready:
qdrant-0  1/1     Running           0          10s
qdrant-1  0/1     ContainerCreating 0          9s
qdrant-2  0/1     Pending           0          8s

# After qdrant-1 is Ready:
qdrant-0  1/1     Running           0          15s
qdrant-1  1/1     Running           0          14s
qdrant-2  0/1     ContainerCreating 0          13s

# All ready:
qdrant-0  1/1     Running           0          20s
qdrant-1  1/1     Running           0          19s
qdrant-2  1/1     Running           0          18s
```

Each Pod waits for the previous one to be Ready. This ensures proper cluster initialization.

---

## Ordered Scaling

Scale down a StatefulSet, and Pod indices scale down in **reverse** order:

```bash
kubectl scale statefulset qdrant --replicas=2
kubectl get pods
```

**Output:**
```
NAME      READY   STATUS        RESTARTS   AGE
qdrant-0  1/1     Running       0          5m
qdrant-1  1/1     Running       0          4m
qdrant-2  1/1     Terminating   0          3m
```

Pod 2 terminates first. This is critical: the highest indices are transient; lower indices are primary. When you scale down, you lose the most recent replicas first, not random ones.

**Scale back up:**

```bash
kubectl scale statefulset qdrant --replicas=3
```

**Output:**
```
NAME      READY   STATUS    RESTARTS   AGE
qdrant-0  1/1     Running   0          6m
qdrant-1  1/1     Running   0          5m
qdrant-2  1/1     Running   0          1s  # Recreated in order
```

New Pod 2 is created. The StatefulSet re-establishes the predictable topology.

---

## Rolling Updates

Unlike Deployments which can update all replicas in parallel, StatefulSets update one Pod at a time, starting from the highest ordinal and working backward (Pod 2, then Pod 1, then Pod 0). This is safer for stateful workloads but slower.

StatefulSets also support **partition-based rolling updates** to control which Pods get updated. This is critical for testing new versions safely:

```bash
kubectl patch statefulset qdrant -p '{"spec":{"updateStrategy":{"type":"RollingUpdate","rollingUpdate":{"partition":1}}}}'
```

This tells Kubernetes: "Update only Pods with index >= 1 (so 1 and 2). Keep Pod 0 at the old version."

```bash
kubectl set image statefulset/qdrant qdrant=qdrant/qdrant:v1.7.0 --record
kubectl get pods
```

**Output:**
```
NAME      READY   STATUS              RESTARTS   AGE
qdrant-0  1/1     Running             0          8m     # Old version (partition=1)
qdrant-1  0/1     ContainerCreating   0          3s     # New version (updating)
qdrant-2  1/1     Running             0          2m
```

Pod 1 updates first to v1.7.0. When ready, Pod 2 updates. Pod 0 stays at the old version, letting you test new versions safely. If v1.7.0 breaks, you can rollback the partition and Pod 1 reverts immediately without touching Pod 0.

---

## Persistent Storage Per Pod

The `volumeClaimTemplates` section in the StatefulSet is what makes persistent state possible. For each Pod, Kubernetes creates a PersistentVolumeClaim (PVC) with a stable name matching the Pod ordinal.

**Verify the PVCs:**

```bash
kubectl get pvc
```

**Output:**
```
NAME                      STATUS   VOLUME                     CAPACITY   ACCESSMODES
qdrant-data-qdrant-0      Bound    pvc-123abc456def789        10Gi       RWO
qdrant-data-qdrant-1      Bound    pvc-789ghi012jkl345        10Gi       RWO
qdrant-data-qdrant-2      Bound    pvc-456mno789pqr012        10Gi       RWO
```

Notice the naming: `qdrant-data-qdrant-0`, `qdrant-data-qdrant-1`, etc. The pattern is `{volume-name}-{statefulset-name}-{ordinal}`.

Each Pod has its own dedicated storage. When Pod 1 crashes and restarts, Kubernetes automatically reconnects it to `qdrant-data-qdrant-1`, preserving the data and cluster state. This is why StatefulSets are suitable for databases: data isn't lost on Pod restart.

---

## Try With AI

**Setup**: You're deploying a distributed LLM inference service with model replication. Each replica needs stable identity and persistent model cache.

**Scenario**: Your FastAPI agent (from Chapter 49) serves LLM inference. You want to:
- Deploy 3 replicas with stable hostnames: inference-0, inference-1, inference-2
- Each replica caches the LLM model locally (10GB cache per Pod)
- Rolling updates should happen one Pod at a time, starting with replica 2

**Prompts to try:**

1. "Design a StatefulSet for LLM inference with 3 replicas. Each replica caches a 10GB model. The headless Service should be `inference-service`. Show the full manifest with volumeClaimTemplates."

2. "I want rolling updates to start with replica 2 (highest index) and roll backward to replica 0. How do I configure the partition strategy? Show the updated StatefulSet configuration."

3. "One of our inference replicas crashed and has stale cache (corrupted model). We want to delete the PVC for that Pod specifically without affecting the others. What kubectl commands do we run, and what happens to the StatefulSet afterward?"

**Expected outcomes:**
- You describe a StatefulSet with 3 replicas and volumeClaimTemplates
- You explain how partition updates work and why starting with higher indices makes sense
- You understand that deleting a Pod and its PVC causes the StatefulSet to recreate both
