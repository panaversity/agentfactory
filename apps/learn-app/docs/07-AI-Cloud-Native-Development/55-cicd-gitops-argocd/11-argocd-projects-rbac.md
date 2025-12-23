---
sidebar_position: 11
title: "ArgoCD Projects and RBAC"
description: "Configure multi-tenancy with Projects, resource restrictions, and role-based access"
keywords: [argocd, projects, rbac, multi-tenancy, security, access control]
---

# ArgoCD Projects and RBAC

You've been deploying your own applications across waves with perfect control. Now imagine a team: frontend engineers deploying to the `frontend` namespace, backend engineers deploying to the `backend` namespace. They can't cross boundaries—a frontend engineer must never touch backend deployments.

This is where ArgoCD Projects and RBAC (Role-Based Access Control) become critical. **Projects** define what repositories your team can deploy from and which clusters/namespaces they can deploy to. **RBAC policies** define who can do what—read-only viewing, deployment approvals, admin access.

Without Projects and RBAC, one engineer's mistake could deploy malicious code to production. With them, you enforce organizational boundaries and isolate team deployments.

## Why Multi-Tenancy Matters for Teams

Consider a team of six engineers:
- 2 frontend engineers (should only touch frontend namespace)
- 2 backend engineers (should only touch backend namespace)
- 1 DevOps engineer (admin, touches all namespaces)
- 1 manager (read-only access for audits)

If ArgoCD had no Projects or RBAC, all six engineers would have access to everything. A misconfiguration by the frontend team could accidentally overwrite the backend database password, breaking production.

**Projects solve this**: Each team gets a Project that restricts source repositories and destination namespaces.

**RBAC solves this**: Each engineer gets a Role that defines what actions they can take.

## The AppProject Custom Resource Definition (CRD)

An **AppProject** is a Kubernetes custom resource that acts as a security boundary. It defines:
1. **Source repositories**: Which Git repositories this project can deploy from
2. **Destinations**: Which clusters and namespaces this project can deploy to
3. **Resource restrictions**: Which resource types can be deployed
4. **Namespaces**: Multitenancy isolation

Let's look at the structure:

```yaml
apiVersion: argoproj.io/v1alpha1
kind: AppProject
metadata:
  name: frontend-team
  namespace: argocd
spec:
  # Repositories this project can deploy from
  sourceRepos:
    - "https://github.com/myorg/frontend-repo"
    - "https://github.com/myorg/shared-charts"

  # Destinations this project can deploy to
  destinations:
    - namespace: "frontend"
      server: "https://kubernetes.default.svc"
    - namespace: "staging"
      server: "https://kubernetes.default.svc"

  # Resource restrictions
  deniedNamespaces: ["argocd", "kube-system"]

  # Which resource types can be deployed
  clusterResourceWhitelist:
    - group: ""
      kind: Namespace
  namespaceResourceBlacklist:
    - group: "policy"
      kind: PodDisruptionBudget
```

**Output:**
```
AppProject frontend-team created
- Source repos: github.com/myorg/frontend-repo, github.com/myorg/shared-charts
- Allowed destinations: frontend namespace, staging namespace
- Blocked namespaces: argocd, kube-system
- Denied resources: PodDisruptionBudget in policy group
```

Notice the hierarchy: **source repos** → **destinations** → **resource restrictions**.

## Source Repository Restrictions

The first security boundary is the repository. Your Project defines which Git repositories its Applications can deploy from.

### Single Repository (Simple Case)

```yaml
apiVersion: argoproj.io/v1alpha1
kind: AppProject
metadata:
  name: agent-project
  namespace: argocd
spec:
  sourceRepos:
    - "https://github.com/myorg/agent-repo"
  destinations:
    - namespace: "*"
      server: "*"
```

**Output:**
```
Project agent-project configured
- Can deploy from: https://github.com/myorg/agent-repo
- Can deploy to: Any namespace, any cluster
- Result: This project is NOT restricted (destinationswildcard)
```

### Multiple Repositories (Team Pattern)

A single project can deploy from multiple repositories. This is useful when your deployment includes both application code and shared infrastructure:

```yaml
apiVersion: argoproj.io/v1alpha1
kind: AppProject
metadata:
  name: platform-team
  namespace: argocd
spec:
  sourceRepos:
    - "https://github.com/myorg/agent-app"
    - "https://github.com/myorg/helm-charts"
    - "https://github.com/myorg/config-repo"
  destinations:
    - namespace: "*"
      server: "*"
```

**Output:**
```
Project platform-team configured
- Can deploy from: 3 repositories
  - agent-app (application source)
  - helm-charts (infrastructure)
  - config-repo (configuration)
- Result: Applications can compose manifests across all three repos
```

## Destination Restrictions: Clusters and Namespaces

The second security boundary is the destination—where your Application can deploy.

### Single Namespace (Frontend Team)

```yaml
apiVersion: argoproj.io/v1alpha1
kind: AppProject
metadata:
  name: frontend-team
  namespace: argocd
spec:
  sourceRepos:
    - "https://github.com/myorg/frontend-repo"
  destinations:
    - namespace: "frontend"
      server: "https://kubernetes.default.svc"
    - namespace: "frontend-staging"
      server: "https://kubernetes.default.svc"
```

**Output:**
```
Project frontend-team configured
- Can deploy to:
  - Namespace: frontend (production)
  - Namespace: frontend-staging (staging)
- Blocked: backend, default, argocd, kube-system
- Result: Frontend team cannot accidentally deploy to backend
```

### Multiple Clusters (Multi-Region)

Some organizations deploy to multiple clusters (us-east, eu-west, etc.). You can restrict Projects to specific clusters:

```yaml
apiVersion: argoproj.io/v1alpha1
kind: AppProject
metadata:
  name: backend-team
  namespace: argocd
spec:
  sourceRepos:
    - "https://github.com/myorg/backend-repo"
  destinations:
    # US Region
    - namespace: "backend"
      server: "https://us-east-api-server:6443"
    - namespace: "backend-staging"
      server: "https://us-east-api-server:6443"
    # EU Region
    - namespace: "backend"
      server: "https://eu-west-api-server:6443"
    - namespace: "backend-staging"
      server: "https://eu-west-api-server:6443"
```

**Output:**
```
Project backend-team configured
- Can deploy to: backend namespace in US and EU regions
- Cannot deploy to: frontend namespaces, other regions
- Result: Backend team can manage multi-region deployments
```

## Resource Whitelists and Blacklists

The third security boundary controls resource types. Some organizations block certain resources from being deployed through ArgoCD.

### Blocking System Namespaces

```yaml
apiVersion: argoproj.io/v1alpha1
kind: AppProject
metadata:
  name: tenant-project
  namespace: argocd
spec:
  sourceRepos:
    - "https://github.com/myorg/app-repo"
  destinations:
    - namespace: "*"
      server: "https://kubernetes.default.svc"
  # Deny cluster-scoped resources to prevent namespace creation
  clusterResourceBlacklist:
    - group: ""
      kind: Namespace
    - group: "rbac.authorization.k8s.io"
      kind: ClusterRole
    - group: "rbac.authorization.k8s.io"
      kind: ClusterRoleBinding
```

**Output:**
```
Project tenant-project configured
- Blocked cluster resources:
  - Namespace (prevent new namespace creation)
  - ClusterRole (prevent privilege escalation)
  - ClusterRoleBinding (prevent permission changes)
- Allowed: Pod, Deployment, Service (namespace-scoped)
- Result: Tenant cannot escape their namespace
```

### Whitelisting Only Safe Resources

Some organizations take the opposite approach—whitelist only resources they trust:

```yaml
apiVersion: argoproj.io/v1alpha1
kind: AppProject
metadata:
  name: restricted-project
  namespace: argocd
spec:
  sourceRepos:
    - "https://github.com/myorg/app-repo"
  destinations:
    - namespace: "staging"
      server: "https://kubernetes.default.svc"
  namespaceResourceWhitelist:
    - group: ""
      kind: Pod
    - group: ""
      kind: Service
    - group: ""
      kind: ConfigMap
    - group: "apps"
      kind: Deployment
```

**Output:**
```
Project restricted-project configured
- Whitelist (only these allowed):
  - Pod, Service, ConfigMap, Deployment
- Blocked: StatefulSet, DaemonSet, CronJob, etc.
- Result: Only simple, stateless applications can be deployed
```

## RBAC: Who Can Do What?

Now that you've defined Projects, you need to define which users can access them. RBAC in ArgoCD uses two policy types:

1. **p policies** (permissions): Define what actions a role can perform
2. **g policies** (groups): Define which users belong to which roles

### RBAC Policy Structure

ArgoCD RBAC is configured in a ConfigMap. The format uses three parts:

```
p, <role>, <resource>, <action>, <object>
g, <user>, <role>
```

### Example: Read-Only Role

Create a read-only role that can view but not modify:

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: argocd-rbac-cm
  namespace: argocd
data:
  policy.csv: |
    # Define the role
    p, role:viewer, applications, get, */*, allow
    p, role:viewer, applications, list, */*, allow
    p, role:viewer, logs, get, */*, allow

    # Assign users to the role
    g, alice@myorg.com, role:viewer
    g, bob@myorg.com, role:viewer

  policy.default: deny
```

**Output:**
```
RBAC policy applied
- Role: viewer
  - Can: View applications (get), list applications, view logs
  - Cannot: Deploy, delete, modify
- Users: alice@myorg.com, bob@myorg.com (viewer role)
- Default: Deny all (deny-by-default)
```

### Example: Project-Scoped Deployment Role

Allow a developer to deploy only within their project:

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: argocd-rbac-cm
  namespace: argocd
data:
  policy.csv: |
    # Admin role (can do everything)
    p, role:admin, *, *, */*, allow

    # Frontend team role (can deploy only within frontend-team project)
    p, role:frontend-dev, applications, create, frontend-team/*, allow
    p, role:frontend-dev, applications, update, frontend-team/*, allow
    p, role:frontend-dev, applications, delete, frontend-team/*, allow
    p, role:frontend-dev, applications, sync, frontend-team/*, allow
    p, role:frontend-dev, applications, get, frontend-team/*, allow
    p, role:frontend-dev, repositories, get, *, allow

    # Backend team role
    p, role:backend-dev, applications, create, backend-team/*, allow
    p, role:backend-dev, applications, update, backend-team/*, allow
    p, role:backend-dev, applications, delete, backend-team/*, allow
    p, role:backend-dev, applications, sync, backend-team/*, allow
    p, role:backend-dev, applications, get, backend-team/*, allow
    p, role:backend-dev, repositories, get, *, allow

    # Assign users to roles
    g, alice@myorg.com, role:frontend-dev
    g, bob@myorg.com, role:frontend-dev
    g, charlie@myorg.com, role:backend-dev
    g, dave@myorg.com, role:backend-dev
    g, admin@myorg.com, role:admin

  policy.default: deny
```

**Output:**
```
RBAC policies applied
- 3 roles configured: admin, frontend-dev, backend-dev
- Role permissions:
  - admin: Full access (*, *, */*)
  - frontend-dev: Create, update, delete, sync only in frontend-team project
  - backend-dev: Create, update, delete, sync only in backend-team project
- Users assigned:
  - Frontend: alice, bob
  - Backend: charlie, dave
  - Admin: admin
- Result: Developers cannot deploy outside their project
```

### Understanding the Wildcards

The policy syntax uses `*` as a wildcard:

```
p, role:viewer, applications, get, */*, allow
         ↓         ↓            ↓    ↓↓
      role       resource     action  (project/app)
```

- `*/` = any project
- `/*` = any application
- `*` alone = any

Examples:
- `frontend-team/*` = any app in frontend-team project
- `*/my-app` = my-app in any project (rarely used)
- `frontend-team/agent-app` = specific project and app

## Project-Scoped Repositories

ArgoCD 3.x adds the ability to restrict repositories at the project level, not just globally.

### Complete Multi-Tenant Project Example

Combine all three boundaries—repositories, destinations, and RBAC:

```yaml
---
# Backend Team Project
apiVersion: argoproj.io/v1alpha1
kind: AppProject
metadata:
  name: backend-team
  namespace: argocd
spec:
  # Only backend repositories
  sourceRepos:
    - "https://github.com/myorg/backend-service"
    - "https://github.com/myorg/shared-helm-charts"

  # Only backend namespaces
  destinations:
    - namespace: "backend"
      server: "https://kubernetes.default.svc"
    - namespace: "backend-staging"
      server: "https://kubernetes.default.svc"

  # Block system namespaces and other team namespaces
  deniedNamespaces:
    - "argocd"
    - "kube-system"
    - "kube-public"
    - "frontend"
    - "frontend-staging"
    - "db-team"

  # Only allow safe resources
  namespaceResourceWhitelist:
    - group: ""
      kind: Pod
    - group: ""
      kind: Service
    - group: ""
      kind: ConfigMap
    - group: "apps"
      kind: Deployment
    - group: "apps"
      kind: StatefulSet
    - group: "batch"
      kind: CronJob

  # Block privileged resources
  clusterResourceBlacklist:
    - group: ""
      kind: Namespace
    - group: "rbac.authorization.k8s.io"
      kind: ClusterRole
    - group: "rbac.authorization.k8s.io"
      kind: ClusterRoleBinding

---
# Frontend Team Project
apiVersion: argoproj.io/v1alpha1
kind: AppProject
metadata:
  name: frontend-team
  namespace: argocd
spec:
  sourceRepos:
    - "https://github.com/myorg/frontend-app"
    - "https://github.com/myorg/shared-helm-charts"

  destinations:
    - namespace: "frontend"
      server: "https://kubernetes.default.svc"
    - namespace: "frontend-staging"
      server: "https://kubernetes.default.svc"

  deniedNamespaces:
    - "argocd"
    - "kube-system"
    - "backend"
    - "backend-staging"
    - "db-team"

  namespaceResourceWhitelist:
    - group: ""
      kind: Pod
    - group: ""
      kind: Service
    - group: ""
      kind: ConfigMap
    - group: "apps"
      kind: Deployment

---
# RBAC Configuration
apiVersion: v1
kind: ConfigMap
metadata:
  name: argocd-rbac-cm
  namespace: argocd
data:
  policy.csv: |
    # Roles
    p, role:admin, *, *, */*, allow
    p, role:backend-engineer, applications, *, backend-team/*, allow
    p, role:frontend-engineer, applications, *, frontend-team/*, allow
    p, role:viewer, applications, get, */*, allow

    # User assignments
    g, charlie@myorg.com, role:backend-engineer
    g, dave@myorg.com, role:backend-engineer
    g, alice@myorg.com, role:frontend-engineer
    g, bob@myorg.com, role:frontend-engineer
    g, manager@myorg.com, role:viewer
    g, admin@myorg.com, role:admin

  policy.default: deny
```

**Output:**
```
Multi-tenant ArgoCD configured
- Projects: backend-team, frontend-team
- Backend team:
  - Repositories: backend-service, shared-helm-charts
  - Destinations: backend, backend-staging namespaces
  - Blocked: frontend, argocd, system namespaces
  - Resources: Pods, Services, Deployments, StatefulSets, CronJobs
  - Blocked: ClusterRoles, Namespaces
- Frontend team:
  - Repositories: frontend-app, shared-helm-charts
  - Destinations: frontend, frontend-staging namespaces
  - Blocked: backend, argocd, system namespaces
  - Resources: Pods, Services, Deployments
- RBAC:
  - 6 users assigned to 3 roles
  - backend-engineer: charlie, dave
  - frontend-engineer: alice, bob
  - viewer: manager (read-only)
  - admin: admin (full access)
- Result: Teams isolated, resources restricted, access controlled
```

## Applying Projects in Your Cluster

Let's apply these projects to your Minikube cluster and verify they work.

### Step 1: Create the Projects

```bash
kubectl apply -f - <<EOF
---
apiVersion: argoproj.io/v1alpha1
kind: AppProject
metadata:
  name: backend-team
  namespace: argocd
spec:
  sourceRepos:
    - "https://github.com/myorg/backend-service"
  destinations:
    - namespace: "backend"
      server: "https://kubernetes.default.svc"
  deniedNamespaces:
    - "argocd"
    - "kube-system"
    - "frontend"
---
apiVersion: argoproj.io/v1alpha1
kind: AppProject
metadata:
  name: frontend-team
  namespace: argocd
spec:
  sourceRepos:
    - "https://github.com/myorg/frontend-app"
  destinations:
    - namespace: "frontend"
      server: "https://kubernetes.default.svc"
  deniedNamespaces:
    - "argocd"
    - "kube-system"
    - "backend"
EOF
```

**Output:**
```
appproject.argoproj.io/backend-team created
appproject.argoproj.io/frontend-team created
```

### Step 2: Verify Projects Exist

```bash
kubectl get appproject -n argocd
```

**Output:**
```
NAME            AGE
backend-team    2m
frontend-team   2m
```

### Step 3: Inspect a Project

```bash
kubectl describe appproject backend-team -n argocd
```

**Output:**
```
Name:         backend-team
Namespace:    argocd
Labels:       <none>
Annotations:  <none>
API Version:  argoproj.io/v1alpha1
Kind:         AppProject
Metadata:
  Creation Timestamp:  2025-01-15T14:32:00Z
Spec:
  Denied Namespaces:
    argocd
    kube-system
    frontend
  Destinations:
    Namespace:  backend
    Server:     https://kubernetes.default.svc
  Source Repos:
    https://github.com/myorg/backend-service
Status:
  JwtTokensByRole:  <empty>
```

### Step 4: Apply RBAC

```bash
kubectl apply -f - <<EOF
apiVersion: v1
kind: ConfigMap
metadata:
  name: argocd-rbac-cm
  namespace: argocd
data:
  policy.csv: |
    p, role:admin, *, *, */*, allow
    p, role:backend-dev, applications, *, backend-team/*, allow
    p, role:frontend-dev, applications, *, frontend-team/*, allow
    p, role:viewer, applications, get, */*, allow
    g, charlie@myorg.com, role:backend-dev
    g, alice@myorg.com, role:frontend-dev
    g, manager@myorg.com, role:viewer
    g, admin@myorg.com, role:admin
  policy.default: deny
EOF
```

**Output:**
```
configmap/argocd-rbac-cm configured
```

### Step 5: Create Applications Within Projects

Now create an Application that uses the backend-team project:

```bash
argocd app create backend-agent \
  --repo https://github.com/myorg/backend-service \
  --path k8s \
  --dest-server https://kubernetes.default.svc \
  --dest-namespace backend \
  --project backend-team
```

**Output:**
```
application 'backend-agent' created
```

## ArgoCD 3.x RBAC Enhancements

ArgoCD 3.x adds fine-grained permission controls that earlier versions lacked.

### v3.0 and Later Features

1. **Repository-level RBAC**: Control who can create Applications from specific repositories
2. **Account tokens with RBAC**: Create service accounts with scoped permissions
3. **GitOps operator patterns**: Use Argo Workflows + ArgoCD for complex permission models

Example of repository-scoped RBAC:

```yaml
p, role:backend-dev, repositories, create, https://github.com/myorg/backend-service, allow
p, role:backend-dev, repositories, get, https://github.com/myorg/backend-service, allow
p, role:frontend-dev, repositories, create, https://github.com/myorg/frontend-app, allow
p, role:frontend-dev, repositories, get, https://github.com/myorg/frontend-app, allow
```

**Output:**
```
Repository-level RBAC configured
- backend-dev: Create/get applications from backend-service repo only
- frontend-dev: Create/get applications from frontend-app repo only
- Result: Developers cannot reference repositories outside their team
```

## Security Best Practices

### 1. Default Deny

Always set your default policy to deny:

```yaml
policy.default: deny
```

This means every access is blocked unless explicitly allowed.

### 2. Least Privilege

Give each role the minimum permissions needed:

```yaml
# Bad: Too permissive
p, role:dev, applications, *, */*, allow

# Good: Specific actions
p, role:dev, applications, get, team-project/*, allow
p, role:dev, applications, sync, team-project/*, allow
```

### 3. Separate Admin Accounts

Create a dedicated admin account with strong authentication:

```yaml
g, devops-admin@myorg.com, role:admin
```

Don't use admin for day-to-day development.

### 4. Regular Audits

Review RBAC policies quarterly:

```bash
kubectl get cm argocd-rbac-cm -n argocd -o yaml
```

Check for unused roles, orphaned users, or overly permissive policies.

## Try With AI

**Setup**: You have your Minikube cluster running with ArgoCD installed. You want to set up multi-team access control.

**Prompts**:

1. **Initial Prompt**:
   ```
   I have two teams: frontend (3 engineers) and backend (2 engineers).
   Design an ArgoCD AppProject and RBAC configuration that:
   - Isolates frontend team to frontend and frontend-staging namespaces
   - Isolates backend team to backend and backend-staging namespaces
   - Prevents teams from deploying outside their namespaces
   - Creates a read-only role for the engineering manager

   Include the full YAML for AppProject definitions and the RBAC ConfigMap.
   ```

2. **Refinement Prompt**:
   ```
   In your RBAC configuration, I want to also allow both teams to read
   (but not modify) logs and sync status. Update the role definitions to
   add these permissions without allowing them to deploy to other teams'
   namespaces.
   ```

3. **Testing Prompt**:
   ```
   Give me kubectl commands to:
   - Apply the AppProject and RBAC ConfigMap to my cluster
   - Create test Applications in both projects
   - Verify that a frontend engineer can sync frontend applications
   - Verify that they cannot create or modify backend applications
   ```

**Expected outcomes**:
- Clear AppProject CRDs defining source/destination boundaries
- RBAC policy.csv with role definitions and user assignments
- kubectl commands that validate the multi-tenant setup
- Understanding of how Projects and RBAC work together to enforce isolation
