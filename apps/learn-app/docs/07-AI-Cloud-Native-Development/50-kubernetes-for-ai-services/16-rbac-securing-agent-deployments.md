---
sidebar_position: 16
chapter: 50
lesson: 16
duration_minutes: 45
title: "RBAC: Securing Your Agent Deployments"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Manual RBAC configuration builds understanding of access control patterns"
cognitive_load:
  concepts_count: 10
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Explain RBAC components: ServiceAccount, Role, ClusterRole, RoleBinding, ClusterRoleBinding"
    bloom_level: "Understand"
  - id: LO2
    description: "Create ServiceAccounts for agent Pods"
    bloom_level: "Apply"
  - id: LO3
    description: "Define Roles with minimal required permissions"
    bloom_level: "Apply"
  - id: LO4
    description: "Bind Roles to ServiceAccounts"
    bloom_level: "Apply"
  - id: LO5
    description: "Audit RBAC with kubectl auth can-i"
    bloom_level: "Apply"
  - id: LO6
    description: "Understand the principle of least privilege"
    bloom_level: "Understand"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "4. Safety"
    competency: "4.1 Protecting devices"
  - objective_id: LO2
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO3
    competency_area: "4. Safety"
    competency: "4.1 Protecting devices"
  - objective_id: LO4
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO5
    competency_area: "4. Safety"
    competency: "4.1 Protecting devices"
  - objective_id: LO6
    competency_area: "4. Safety"
    competency: "4.1 Protecting devices"
---

# RBAC: Securing Your Agent Deployments

In Lesson 4, you deployed a Kubernetes Pod with a containerized agent. By default, that Pod can read any ConfigMap, access any Secret, create Pods, delete Deployments—anything in the cluster. This is a security disaster. If an attacker compromises your agent container, they inherit all those permissions and can pivot through your entire cluster.

Role-Based Access Control (RBAC) solves this by restricting what your agent can do. Instead of giving your agent all permissions, you define exactly which Kubernetes resources it needs to access and which verbs (actions) it can perform on those resources. Your agent can read the ConfigMap it needs but cannot delete Deployments. Another agent can list Pods but cannot create Secrets. This is the principle of least privilege: each workload gets only the permissions it needs, nothing more.

---

## Security Requirement: Agent Access Control

Before configuring RBAC, define what your agent needs:

**Specification:**
- Agent must read ConfigMap "agent-config" in the same namespace
- Agent must read Secret "api-credentials" in the same namespace only (not across namespaces)
- Agent must NOT create, delete, or modify any Kubernetes resources
- Agent must NOT access secrets or configmaps in other namespaces
- Cluster administrator must audit what permissions the agent has

**Why this matters:** A compromised agent cannot become a stepping stone to cluster-wide compromise.

---

## RBAC Components

RBAC consists of five components. Understanding how they connect is critical:

```
┌─────────────────────────────────────────────────────────────┐
│                    RBAC Architecture                         │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ServiceAccount (Identity)                                   │
│  ├─ apiVersion: v1                                          │
│  ├─ kind: ServiceAccount                                    │
│  └─ metadata.name: "agent-sa"                               │
│                    ↓                                         │
│  RoleBinding (Connection)                                    │
│  ├─ roleRef → Role                                          │
│  ├─ subjects → [ServiceAccount]                             │
│  └─ metadata.namespace: "default"                           │
│                    ↓                                         │
│  Role (Permissions in namespace)                             │
│  ├─ apiVersion: rbac.authorization.k8s.io/v1                │
│  ├─ kind: Role                                              │
│  ├─ rules[].resources: ["configmaps", "secrets"]            │
│  ├─ rules[].verbs: ["get", "list"]                          │
│  └─ metadata.namespace: "default"                           │
│                    ↓                                         │
│  Resources (What to access)                                  │
│  └─ ConfigMaps, Secrets, Pods, Deployments...               │
│                                                              │
│  Note: ClusterRole/ClusterRoleBinding for cluster-wide      │
│  permissions (same structure, cluster scope)                │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

**The five components:**

1. **ServiceAccount** - An identity for Pods in a namespace. When your Pod starts, Kubernetes mounts the ServiceAccount's token as a credential file (`/var/run/secrets/kubernetes.io/serviceaccount/token`). This token proves "I am this ServiceAccount."

2. **Role** - A set of permissions (rules) within a namespace. Each rule specifies:
   - `resources`: What Kubernetes objects (configmaps, secrets, pods, deployments)
   - `verbs`: What actions (get, list, create, delete, update, patch, watch)
   - `apiGroups`: Which API group (core api: "", apps: "apps", batch: "batch")

3. **RoleBinding** - Connects a ServiceAccount to a Role within a namespace. "This ServiceAccount gets these permissions."

4. **ClusterRole** - A set of permissions cluster-wide. Same structure as Role, but applies to all namespaces.

5. **ClusterRoleBinding** - Connects a ServiceAccount to a ClusterRole cluster-wide.

---

## Creating a ServiceAccount

A ServiceAccount is your Pod's identity. When your Pod makes a request to the Kubernetes API, it presents its ServiceAccount token.

```bash
kubectl create serviceaccount agent-sa
```

**Output:**
```
serviceaccount/agent-sa created
```

Verify it was created:

```bash
kubectl get serviceaccount agent-sa -o yaml
```

**Output:**
```yaml
apiVersion: v1
kind: ServiceAccount
metadata:
  creationTimestamp: "2025-01-15T14:23:45Z"
  name: agent-sa
  namespace: default
  resourceVersion: "12345"
  uid: abc-def-ghi
secrets:
- name: agent-sa-token-xyz789
```

Notice Kubernetes automatically created a token secret (`agent-sa-token-xyz789`). This token is what your Pod will use when making API requests.

---

## Defining a Role with Minimal Permissions

A Role specifies what actions are allowed. For your agent, it only needs to read ConfigMaps and Secrets:

```yaml
apiVersion: rbac.authorization.k8s.io/v1
kind: Role
metadata:
  name: agent-reader
  namespace: default
rules:
- apiGroups: [""]  # core API group (ConfigMaps, Secrets are here)
  resources: ["configmaps", "secrets"]
  verbs: ["get", "list"]
```

Save this as `agent-role.yaml` and apply it:

```bash
kubectl apply -f agent-role.yaml
```

**Output:**
```
role.rbac.authorization.k8s.io/agent-reader created
```

Verify the Role was created:

```bash
kubectl get role agent-reader -o yaml
```

**Output:**
```yaml
apiVersion: rbac.authorization.k8s.io/v1
kind: Role
metadata:
  creationTimestamp: "2025-01-15T14:24:10Z"
  name: agent-reader
  namespace: default
  resourceVersion: "12356"
  uid: xyz-abc-def
rules:
- apiGroups:
  - ""
  resources:
  - configmaps
  - secrets
  verbs:
  - get
  - list
```

**Understanding the Rule:**
- `apiGroups: [""]` - Empty string means the core API group. ConfigMaps and Secrets live here.
- `resources: ["configmaps", "secrets"]` - The agent can access ConfigMaps and Secrets.
- `verbs: ["get", "list"]` - The agent can GET a specific configmap (`get`) and LIST all configmaps (`list`). It cannot create, delete, update, or patch.

---

## Binding the Role to the ServiceAccount

A Role defines permissions, but it's not assigned to anyone yet. A RoleBinding connects the ServiceAccount (your agent's identity) to the Role (the permissions):

```yaml
apiVersion: rbac.authorization.k8s.io/v1
kind: RoleBinding
metadata:
  name: agent-reader-binding
  namespace: default
roleRef:
  apiGroup: rbac.authorization.k8s.io
  kind: Role
  name: agent-reader
subjects:
- kind: ServiceAccount
  name: agent-sa
  namespace: default
```

Save this as `agent-rolebinding.yaml` and apply it:

```bash
kubectl apply -f agent-rolebinding.yaml
```

**Output:**
```
rolebinding.rbac.authorization.k8s.io/agent-reader-binding created
```

Verify the RoleBinding was created:

```bash
kubectl get rolebinding agent-reader-binding -o yaml
```

**Output:**
```yaml
apiVersion: rbac.authorization.k8s.io/v1
kind: RoleBinding
metadata:
  creationTimestamp: "2025-01-15T14:24:45Z"
  name: agent-reader-binding
  namespace: default
  resourceVersion: "12357"
roleRef:
  apiGroup: rbac.authorization.k8s.io
  kind: Role
  name: agent-reader
subjects:
- kind: ServiceAccount
  name: agent-sa
  namespace: default
```

**What happened:** The RoleBinding now says "ServiceAccount agent-sa in namespace default gets the permissions from Role agent-reader in namespace default."

---

## Assigning ServiceAccount to Your Pod

Your Pod must explicitly use the ServiceAccount you created. When you create a Pod without specifying a serviceAccountName, it uses the default ServiceAccount, which has no permissions. To use your agent-sa:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: agent-pod
  namespace: default
spec:
  serviceAccountName: agent-sa  # Use the agent-sa ServiceAccount
  containers:
  - name: agent
    image: my-agent:latest
    env:
    - name: CONFIG_MAP_NAME
      value: agent-config
    - name: SECRET_NAME
      value: api-credentials
```

When this Pod starts, Kubernetes mounts the agent-sa token into the container at `/var/run/secrets/kubernetes.io/serviceaccount/token`. Your agent can then use this token to authenticate API requests.

Apply the Pod:

```bash
kubectl apply -f agent-pod.yaml
```

**Output:**
```
pod/agent-pod created
```

---

## Auditing Permissions with kubectl auth can-i

How do you verify your agent actually has the permissions you defined? Use `kubectl auth can-i`:

```bash
kubectl auth can-i get configmaps --as=system:serviceaccount:default:agent-sa
```

**Output:**
```
yes
```

The agent can get configmaps. Now test if it can delete deployments (it shouldn't):

```bash
kubectl auth can-i delete deployments --as=system:serviceaccount:default:agent-sa
```

**Output:**
```
no
```

Correct. The agent cannot delete deployments because the agent-reader Role doesn't include the delete verb for deployments.

Test accessing secrets:

```bash
kubectl auth can-i get secrets --as=system:serviceaccount:default:agent-sa
```

**Output:**
```
yes
```

Test creating pods (it shouldn't):

```bash
kubectl auth can-i create pods --as=system:serviceaccount:default:agent-sa
```

**Output:**
```
no
```

Good—the agent is restricted. It can only get and list configmaps and secrets, nothing else.

---

## Principle of Least Privilege

The approach above demonstrates least privilege: the agent gets exactly the permissions it needs and nothing more. Compare two scenarios:

**Anti-pattern (BAD):**
```bash
kubectl create clusterrolebinding agent-admin \
  --clusterrole=cluster-admin \
  --serviceaccount=default:agent-sa
```

This gives your agent admin permissions across the entire cluster. If the agent is compromised, the attacker has full cluster access. **Never do this in production.**

**Best practice (GOOD):**
```yaml
rules:
- apiGroups: [""]
  resources: ["configmaps", "secrets"]
  verbs: ["get", "list"]
  # Additional limit: restrict to specific resource names
  resourceNames: ["agent-config", "api-credentials"]
```

This restricts the agent to only the two configmaps/secrets it needs by name. If the agent requests a different secret, Kubernetes denies it.

---

## Try With AI

**Setup:** You're designing RBAC for a multi-tenant AI deployment. Multiple agents run in the same cluster but in different namespaces:
- Namespace "agents-team-a": Agent needs ConfigMap "config-a" and Secret "creds-a" (team-a namespace only)
- Namespace "agents-team-b": Agent needs ConfigMap "config-b" and Secret "creds-b" (team-b namespace only)
- Both agents need to read the Deployment object to check the current replica count (list, get verbs only)

**Task 1: Design the RBAC structure**

Ask AI: "Design RBAC for a multi-tenant Kubernetes deployment. I have two teams, each in their own namespace. Each team's agent needs:
1. Read-only access to its own namespace's ConfigMap and Secret (by name: config-a/config-b and creds-a/creds-b)
2. Ability to list and get Deployments in its own namespace only
3. No cross-namespace access

I want each agent's permissions isolated so a compromised agent cannot access another team's data. Show me the ServiceAccount, Role, and RoleBinding YAML for team-a. How would team-b's setup differ?"

**Task 2: Audit the permissions**

Ask AI: "After applying the RBAC, how would I use kubectl auth can-i to verify that the team-a agent can:
- Get its own ConfigMap but not team-b's?
- Get Deployments but not create Deployments?
- Not access secrets in other namespaces?

Show me the exact kubectl commands and expected outputs."

**Task 3: Refine for production**

Ask AI: "What additional RBAC considerations should I include for production? Should I use more granular resource restrictions, API groups, or non-resource URLs? Show me an enhanced Role that adds read-only access to Pod logs and metrics."

---

**Expected Outcomes:**
- You understand how ServiceAccounts, Roles, and RoleBindings compose to restrict permissions
- You can design RBAC that enforces least privilege across namespaces
- You can audit what an agent can and cannot do using kubectl auth can-i
- You recognize that over-permissioning is a common security anti-pattern
