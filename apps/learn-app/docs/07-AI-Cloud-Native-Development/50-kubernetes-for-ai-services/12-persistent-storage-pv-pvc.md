---
sidebar_position: 12
chapter: 50
lesson: 12
duration_minutes: 45
title: "Persistent Storage: PV, PVC, StorageClass"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Manual storage configuration builds understanding of persistent data patterns"
cognitive_load:
  concepts_count: 10
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Explain the PersistentVolume (PV) and PersistentVolumeClaim (PVC) abstraction"
    bloom_level: "Understand"
  - id: LO2
    description: "Create static PersistentVolumes"
    bloom_level: "Apply"
  - id: LO3
    description: "Create PersistentVolumeClaims to request storage"
    bloom_level: "Apply"
  - id: LO4
    description: "Use StorageClasses for dynamic provisioning"
    bloom_level: "Apply"
  - id: LO5
    description: "Configure access modes (ReadWriteOnce, ReadOnlyMany, ReadWriteMany)"
    bloom_level: "Apply"
  - id: LO6
    description: "Mount volumes into agent containers"
    bloom_level: "Apply"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "5. Problem Solving"
    competency: "5.2 Identifying needs and technological responses"
  - objective_id: LO2
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO3
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO4
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO5
    competency_area: "1. Information and Data Literacy"
    competency: "1.3 Managing data, information and digital content"
  - objective_id: LO6
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
---

# Persistent Storage: PV, PVC, StorageClass

Your FastAPI agent runs in a Pod on Kubernetes. But Pods are ephemeral—when they restart, their filesystem disappears. This is a critical problem: Your agent has embedded vector search indexes, model checkpoints, conversation logs. When the Pod crashes and Kubernetes creates a replacement, all that data is gone.

**PersistentVolumes (PVs)** and **PersistentVolumeClaims (PVCs)** solve this by decoupling storage from compute. Storage exists independent of Pods. When a Pod restarts, it reconnects to the same storage and your agent resumes with all previous state intact.

This lesson teaches you to provision persistent storage manually, understand the abstraction that makes Kubernetes storage work, and configure your Pods to use that storage reliably.

---

## The Problem: Data Loss on Pod Restart

Let's see what happens without persistent storage.

Create a simple Pod that writes data to its local filesystem:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: ephemeral-app
spec:
  containers:
  - name: app
    image: busybox:1.28
    command: ['sh']
    args: ['-c', 'echo "Agent state data" > /app/state.txt; sleep 3600']
    volumeMounts:
    - name: app-storage
      mountPath: /app
  volumes:
  - name: app-storage
    emptyDir: {}
```

Create this Pod:

```bash
kubectl apply -f ephemeral-app.yaml
```

**Output:**
```
pod/ephemeral-app created
```

Check that the file exists inside the Pod:

```bash
kubectl exec ephemeral-app -- cat /app/state.txt
```

**Output:**
```
Agent state data
```

Now delete the Pod:

```bash
kubectl delete pod ephemeral-app
```

**Output:**
```
pod "ephemeral-app" deleted
```

The data is gone forever. `emptyDir` (temporary storage) is cleared when the Pod terminates. For embeddings, model weights, and conversation history, you need storage that survives Pod restarts.

---

## The PV/PVC Abstraction: Separation of Concerns

Kubernetes separates storage concerns into two layers:

**PersistentVolume (PV)**: The infrastructure—a chunk of storage that exists in your cluster. A cluster administrator provisions PVs from available storage (local disk, network storage, cloud volumes). PVs are cluster-level resources.

**PersistentVolumeClaim (PVC)**: The request—a developer specifies "I need 10GB of storage with read-write access." Kubernetes finds a matching PV and binds them together. PVCs are namespace-scoped.

This abstraction parallels the CPU/memory model:

- **Node** (infrastructure) vs **Pod** (consumer request)
- **PersistentVolume** (infrastructure) vs **PersistentVolumeClaim** (consumer request)

Think of it like renting office space:

- **Building owner** (cluster admin) provides physical office spaces (PVs)
- **Company manager** (developer) requests an office from the building (PVC)
- **Company** (Pod) uses the office while it exists

When the company moves to a different office building (Pod restarts), the same office (PV) still exists. A new company can occupy it, or the same company can return to the same office after relocation.

---

## Creating a Static PersistentVolume

Let's create a PV manually. We'll use `hostPath`—storage backed by a directory on the Kubernetes node. This is suitable for learning and single-node clusters like Minikube.

First, create a directory on your Minikube node:

```bash
minikube ssh
mkdir -p /mnt/data
echo "stored data" > /mnt/data/test.txt
exit
```

**Output:**
```
                         _             _
            minikube     v1.33.1
            arch: amd64
            driver: docker
            ip: 192.168.49.2

# From inside minikube:
$ mkdir -p /mnt/data
$ echo "stored data" > /mnt/data/test.txt
$ exit
```

Now create a PersistentVolume that points to that directory:

```yaml
apiVersion: v1
kind: PersistentVolume
metadata:
  name: agent-storage-pv
spec:
  capacity:
    storage: 10Gi
  accessModes:
    - ReadWriteOnce
  hostPath:
    path: /mnt/data
```

Apply this manifest:

```bash
kubectl apply -f pv.yaml
```

**Output:**
```
persistentvolume/agent-storage-pv created
```

Check that the PV was created:

```bash
kubectl get pv
```

**Output:**
```
NAME                CAPACITY   ACCESS MODES   RECLAIM POLICY   STATUS      CLAIM   STORAGECLASS   REASON   AGE
agent-storage-pv    10Gi       RWO            Delete           Available            manual         <none>   7s
```

Notice the **STATUS: Available**. The PV exists but is not yet bound to any PVC. The **RECLAIM POLICY: Delete** means that when a PVC is deleted, this PV will be deleted too (other options: Retain, Recycle).

---

## Claiming Storage with PersistentVolumeClaim

A PVC is a request for storage. Create a PVC that claims the PV we just created:

```yaml
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: agent-storage-claim
spec:
  accessModes:
    - ReadWriteOnce
  resources:
    requests:
      storage: 5Gi
```

Apply this manifest:

```bash
kubectl apply -f pvc.yaml
```

**Output:**
```
persistentvolumeclaim/agent-storage-claim created
```

Check that the PVC was created and is bound:

```bash
kubectl get pvc
```

**Output:**
```
NAME                    STATUS   VOLUME              CAPACITY   ACCESS MODES   STORAGECLASS   AGE
agent-storage-claim     Bound    agent-storage-pv    10Gi       RWO            manual         3s
```

Check the PV status again:

```bash
kubectl get pv
```

**Output:**
```
NAME                CAPACITY   ACCESS MODES   RECLAIM POLICY   STATUS   CLAIM                         STORAGECLASS   REASON   AGE
agent-storage-pv    10Gi       RWO            Delete           Bound    default/agent-storage-claim   manual          <none>   47s
```

The PV is now **Bound** to the PVC. They're connected. The binding was automatic based on:
- AccessModes match (both RWO)
- Requested storage (5Gi) is less than available (10Gi)
- No StorageClass specified (defaults to "manual")

Now create a Pod that mounts this PVC:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: agent-with-storage
spec:
  containers:
  - name: agent
    image: busybox:1.28
    command: ['sh']
    args: ['-c', 'cat /agent-data/test.txt && sleep 3600']
    volumeMounts:
    - name: persistent-storage
      mountPath: /agent-data
  volumes:
  - name: persistent-storage
    persistentVolumeClaim:
      claimName: agent-storage-claim
```

Apply this manifest:

```bash
kubectl apply -f agent-pod.yaml
```

**Output:**
```
pod/agent-with-storage created
```

Check the logs to confirm the Pod mounted the storage successfully:

```bash
kubectl logs agent-with-storage
```

**Output:**
```
stored data
```

The Pod successfully read the file we created in `/mnt/data/test.txt` earlier. The storage persists across container restarts because it's backed by the host filesystem, not the container's ephemeral layer.

Delete the Pod and recreate it:

```bash
kubectl delete pod agent-with-storage
```

**Output:**
```
pod "agent-with-storage" deleted
```

```bash
kubectl apply -f agent-pod.yaml
```

**Output:**
```
pod/agent-with-storage created
```

Check the logs again:

```bash
kubectl logs agent-with-storage
```

**Output:**
```
stored data
```

The data is still there. The storage survived the Pod deletion and recreation. This is the core benefit of PersistentVolumes: **data outlives container instances**.

---

## Dynamic Provisioning with StorageClass

Creating PVs manually doesn't scale. In production, you use **StorageClasses** to provision PVs dynamically.

A StorageClass defines:
- **Provisioner**: The component that creates storage (e.g., `kubernetes.io/minikube-hostpath` for Minikube, AWS EBS provisioner for AWS)
- **Parameters**: Storage configuration (IOPS, encryption, filesystem type, etc.)
- **Reclaim Policy**: What happens to storage when the PVC is deleted

First, check what StorageClasses are available in your cluster:

```bash
kubectl get storageclass
```

**Output:**
```
NAME                 PROVISIONER                   RECLAIMPOLICY   VOLUMEBINDINGMODE      ALLOWVOLUMEEXPANSION   AGE
standard (default)   k8s.io/minikube-hostpath     Delete          Immediate              false                  2h
```

Minikube comes with a default StorageClass. Now create a PVC that uses this StorageClass (no PV needed—it's created automatically):

```yaml
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: dynamic-storage-claim
spec:
  storageClassName: standard
  accessModes:
    - ReadWriteOnce
  resources:
    requests:
      storage: 2Gi
```

Apply this manifest:

```bash
kubectl apply -f dynamic-pvc.yaml
```

**Output:**
```
persistentvolumeclaim/dynamic-storage-claim created
```

Check the PVC:

```bash
kubectl get pvc
```

**Output:**
```
NAME                      STATUS   VOLUME                                    CAPACITY   ACCESS MODES   STORAGECLASS   AGE
agent-storage-claim       Bound    agent-storage-pv                          10Gi       RWO            manual         5m
dynamic-storage-claim     Bound    pvc-4a2b1c9d-8f3e-4b5a-9c2d-7e6f5a4b3c   2Gi        RWO            standard       2s
```

**Automatic PV creation**: Kubernetes provisioner created a PV automatically and bound the PVC to it. Notice the PV name is generated (`pvc-4a2b1c9d...`). You don't need to manually create PVs anymore.

Create a Pod using this dynamically-provisioned PVC:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: dynamic-storage-pod
spec:
  containers:
  - name: app
    image: busybox:1.28
    command: ['sh']
    args: ['-c', 'echo "Dynamic storage test" > /data/test.txt && cat /data/test.txt && sleep 3600']
    volumeMounts:
    - name: dynamic-vol
      mountPath: /data
  volumes:
  - name: dynamic-vol
    persistentVolumeClaim:
      claimName: dynamic-storage-claim
```

Apply and check logs:

```bash
kubectl apply -f dynamic-pod.yaml
kubectl logs dynamic-storage-pod
```

**Output:**
```
Dynamic storage test
```

Dynamic provisioning eliminates manual PV management. Developers just declare PVCs with desired storage size and access mode; the provisioner handles infrastructure provisioning.

---

## Access Modes: Who Can Access Storage How?

PersistentVolumes support three access modes:

**ReadWriteOnce (RWO)**: The volume can be mounted as read-write by a **single Pod** (but that Pod's containers can all read and write). Most restrictive mode.

- Use for: Databases, stateful applications that require single writer
- Example: PostgreSQL pod

```yaml
accessModes:
  - ReadWriteOnce
```

**ReadOnlyMany (ROX)**: The volume can be mounted as **read-only by many Pods**. Multiple readers, no writers allowed.

- Use for: Shared configuration, reference data, model weights distributed to many inference Pods
- Example: Vector embeddings used by 100 inference Pods

```yaml
accessModes:
  - ReadOnlyMany
```

**ReadWriteMany (RWX)**: The volume can be mounted as read-write by **many Pods simultaneously**. Requires network storage (not hostPath).

- Use for: Shared logs, distributed training, collaborative applications
- Example: Training data accessed by multiple training pods simultaneously
- Requires: Network filesystem (NFS, SMB) not available in Minikube by default

```yaml
accessModes:
  - ReadWriteMany
```

Create a read-only PVC for agent embeddings:

```yaml
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: embeddings-ro-claim
spec:
  storageClassName: standard
  accessModes:
    - ReadOnlyMany
  resources:
    requests:
      storage: 5Gi
```

This PVC can be mounted by multiple inference Pods. If one embedding update Pod writes to it, the read-only mounting enforces that other Pods cannot accidentally overwrite data.

---

## Reclaim Policies: What Happens When a PVC Deletes?

When you delete a PVC, what happens to the underlying PV? The reclaim policy controls this:

**Delete**: The PV is deleted when the PVC is deleted. Storage is freed immediately. Suitable for dynamic provisioning where storage is cheap.

```yaml
reclaimPolicy: Delete
```

**Retain**: The PV persists after PVC deletion. A cluster admin must manually delete the PV or recycle it. Suitable for important data where you want manual verification before deletion.

```yaml
reclaimPolicy: Retain
```

**Recycle** (deprecated): The PV is wiped and made available for reuse. Avoided in production due to data security concerns.

---

## Putting It Together: Agent with Persistent Embeddings

Here's a realistic Pod configuration for an agent that stores embeddings and checkpoints:

```yaml
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: agent-embeddings-claim
spec:
  storageClassName: standard
  accessModes:
    - ReadWriteOnce
  resources:
    requests:
      storage: 50Gi  # Space for embeddings and checkpoints
---
apiVersion: v1
kind: Pod
metadata:
  name: vector-agent
spec:
  containers:
  - name: agent
    image: my-agent:v1
    env:
    - name: EMBEDDINGS_PATH
      value: /agent-storage/embeddings
    - name: CHECKPOINTS_PATH
      value: /agent-storage/checkpoints
    volumeMounts:
    - name: agent-storage
      mountPath: /agent-storage
    resources:
      requests:
        memory: "2Gi"
        cpu: "500m"
      limits:
        memory: "4Gi"
        cpu: "2"
  volumes:
  - name: agent-storage
    persistentVolumeClaim:
      claimName: agent-embeddings-claim
```

When this Pod runs:
1. The PVC claims storage from the StorageClass
2. Kubernetes provisions a PV automatically
3. The Pod mounts the PV at `/agent-storage`
4. The agent writes embeddings and checkpoints to `/agent-storage/embeddings` and `/agent-storage/checkpoints`
5. If the Pod restarts, it reconnects to the same storage
6. All embeddings and checkpoints survive the restart

Your agent continues serving requests without recomputing embeddings from scratch.

---

## Try With AI

**Setup**: You're designing persistent storage for a multi-agent system. One agent computes and caches vector embeddings. Five other agents need read-only access to those embeddings. A background service periodically updates the embeddings.

**Challenge Prompts**:

Ask AI: "Design a PVC and access mode strategy for this scenario:
- 1 embedding generator Pod writes embeddings weekly
- 5 inference Pods read those embeddings continuously
- I want to ensure inference Pods can't accidentally overwrite embeddings
- I want minimal disk space wastage

What access modes and binding strategy should I use? Should the embeddings and generators use separate PVCs?"

Follow up: "The embedding generator needs to update embeddings without downtime. My inference Pods must continue serving. What reclaim policy and update strategy would work best? Should I use ReadOnlyMany or a different approach?"

Then: "Write a Kubernetes manifest for this architecture. Include the PVC for embeddings, the PVC for the generator (if separate), and Pod definitions for one inference Pod and the generator Pod. Ensure the inference Pod includes volume mounts for the embeddings."

**What to evaluate**:
- Does the design isolate read-only and read-write storage?
- Are access modes correctly matched to each component's needs?
- Would this architecture actually prevent accidental overwrites?
- How would the embeddings update without breaking inference Pods?

Compare your initial understanding of the access modes to what emerged through the conversation. What trade-offs between storage isolation, update frequency, and complexity did you discover?
