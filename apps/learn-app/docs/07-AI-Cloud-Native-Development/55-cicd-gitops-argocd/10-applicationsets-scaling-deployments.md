---
sidebar_position: 10
title: "ApplicationSets: Scaling Deployments"
description: "Deploy to multiple environments with List, Cluster, Matrix, and Git generators"
keywords: [argocd, applicationset, generators, multi-environment, gitops, kubernetes, scaling]
---

# ApplicationSets: Scaling Deployments

You've mastered individual Application CRDs in Lessons 7-9. Now you face a new scale: **deploy your agent to multiple environments**. You need the same Helm chart deployed to dev, staging, and production with different values for each. Or you need to deploy across 5 Kubernetes clusters. Or you need matrix combinations: 3 environments × 2 regions = 6 deployments.

Manually creating 6 Applications with near-identical YAML wastes time and violates DRY principle. **ApplicationSets** solve this by generating Applications from a single template, using generators that parameterize for different environments, clusters, or combinations.

An ApplicationSet is a Kubernetes CRD that says: "Create multiple Applications automatically based on these parameters."

## ApplicationSet CRD Structure

An ApplicationSet has a structure parallel to Application, but focused on **generating** rather than deploying:

```yaml
apiVersion: argoproj.io/v1alpha1
kind: ApplicationSet
metadata:
  name: agent-multi-env
  namespace: argocd
spec:
  # This is the key difference from Application
  generators:
    - list: {}  # Generate from a list (you'll see other generator types)

  template:
    metadata:
      name: '{{.name}}'  # Parameterized—substituted for each generated app
      labels:
        env: '{{.environment}}'
    spec:
      project: default
      source:
        repoURL: https://github.com/yourorg/agent-helm
        targetRevision: HEAD
        path: helm/agent
        helm:
          valuesInline:
            environment: '{{.environment}}'
            replicas: '{{.replicas}}'
      destination:
        server: https://kubernetes.default.svc
        namespace: agent-{{.environment}}
      syncPolicy:
        automated:
          prune: true
          selfHeal: true
```

**Output:**
```
apiVersion: argoproj.io/v1alpha1
kind: ApplicationSet
metadata:
  name: agent-multi-env
status:
  conditions:
    - type: ParametersGenerated
      status: "True"
    - type: ResourcesGenerated
      status: "True"
  applicationStatus: []
```

**Key concept**: The `template` is a standard Application spec, but with `{{.paramName}}` placeholders. The `generators` fill those placeholders, creating N Applications from one template.

## Generator Types

ApplicationSet supports four main generators. Each answers a different scaling question:

| Generator | Scenario | Generates |
|-----------|----------|-----------|
| **List** | "Deploy to dev, staging, prod" | One App per list item |
| **Cluster** | "Deploy across 5 registered clusters" | One App per cluster |
| **Matrix** | "Deploy to 3 envs × 2 regions" | One App per combination |
| **Git** | "Deploy from directories in Git repo" | One App per matching directory |

You'll learn each one through practical examples with your Part 6 agent.

## Generator 1: List Generator (Explicit Environments)

**The List generator** creates one Application per item in a list. Each list item is a dictionary that populates template placeholders.

### Use Case: Three Environments

Your agent needs three environments with different configurations:

- **dev**: 1 replica, rapid iteration, public API
- **staging**: 2 replicas, test like production, internal API
- **prod**: 3 replicas, HA database, restricted API

Instead of three separate Applications, one ApplicationSet with List generator creates all three.

### ApplicationSet with List Generator

```yaml
apiVersion: argoproj.io/v1alpha1
kind: ApplicationSet
metadata:
  name: agent-environments
  namespace: argocd
spec:
  generators:
    - list:
        elements:
          # Each element becomes template parameters
          - name: agent-dev
            environment: dev
            replicas: "1"
            database: sqlite  # Lightweight for dev
            logLevel: debug
          - name: agent-staging
            environment: staging
            replicas: "2"
            database: postgres
            logLevel: info
          - name: agent-prod
            environment: prod
            replicas: "3"
            database: postgres-ha
            logLevel: warn

  template:
    metadata:
      name: '{{.name}}'
      namespace: argocd
      labels:
        app: agent
        environment: '{{.environment}}'
    spec:
      project: default
      source:
        repoURL: https://github.com/youragent/helm-charts
        targetRevision: HEAD
        path: agent
        helm:
          valuesInline:
            replicaCount: {{.replicas}}
            database:
              type: '{{.database}}'
            logging:
              level: '{{.logLevel}}'
            environment: '{{.environment}}'

      destination:
        server: https://kubernetes.default.svc
        namespace: agent-{{.environment}}

      syncPolicy:
        automated:
          prune: true
          selfHeal: true
        syncOptions:
          - CreateNamespace=true
```

**Output:**
```
$ kubectl apply -f agent-environments-applicationset.yaml
applicationset.argoproj.io/agent-environments created

$ argocd app list
NAME              SYNC STATUS  HEALTH STATUS  NAMESPACE         PROJECT
agent-dev         Synced       Healthy        agent-dev         default
agent-staging     Synced       Healthy        agent-staging     default
agent-prod        Synced       Healthy        agent-prod        default

$ kubectl get applicationsets -n argocd
NAME                   DESIRED   CREATED   AGE
agent-environments     3         3         45s
```

ArgoCD created **three Applications automatically** from one ApplicationSet template. Each has different parameters:
- `agent-dev` syncs to `agent-dev` namespace with 1 replica
- `agent-staging` syncs to `agent-staging` namespace with 2 replicas
- `agent-prod` syncs to `agent-prod` namespace with 3 replicas, HA database

**Updating all three**: If you change the Helm chart, all three automatically re-sync without touching the ApplicationSet.

## Generator 2: Cluster Generator (Multi-Cluster Deployments)

**The Cluster generator** creates one Application per cluster registered in ArgoCD. It's perfect for deploying to multiple Kubernetes clusters without maintaining separate ApplicationSets.

### Use Case: Geographic Distribution

Your organization has three clusters:
- US East cluster (production API servers)
- US West cluster (backup region)
- EU cluster (GDPR compliance)

One ApplicationSet deploys your agent to all three.

### Registering Clusters (Prerequisites)

Before creating the ApplicationSet, clusters must be registered:

```bash
# Each cluster is already registered if you ran:
argocd cluster add <context-name>

# Verify clusters:
$ argocd cluster list
NAME                          VERSION  STATUS      MESSAGE
https://kubernetes.default.svc  1.28    Successful  argocd-server's kubernetes cluster
us-east-cluster               1.28    Successful  (external cluster)
us-west-cluster               1.28    Successful  (external cluster)
eu-cluster                    1.28    Successful  (external cluster)
```

### ApplicationSet with Cluster Generator

```yaml
apiVersion: argoproj.io/v1alpha1
kind: ApplicationSet
metadata:
  name: agent-multi-cluster
  namespace: argocd
spec:
  generators:
    - clusters:
        selector:
          matchLabels:
            agent: enabled  # Only clusters with this label

  template:
    metadata:
      name: 'agent-{{cluster.name}}'
      namespace: argocd
      labels:
        cluster: '{{cluster.name}}'
    spec:
      project: default
      source:
        repoURL: https://github.com/youragent/helm-charts
        targetRevision: HEAD
        path: agent
        helm:
          valuesInline:
            cluster:
              name: '{{cluster.name}}'
              region: '{{cluster.metadata.annotations.region}}'

      # Key difference: destination is the cluster itself
      destination:
        server: '{{server}}'
        namespace: agent

      syncPolicy:
        automated:
          prune: true
          selfHeal: true
```

**Output:**
```
$ kubectl apply -f agent-multi-cluster-applicationset.yaml
applicationset.argoproj.io/agent-multi-cluster created

$ argocd app list | grep agent-
NAME                    SYNC STATUS  HEALTH STATUS  CLUSTER
agent-us-east-cluster   Synced       Healthy        us-east-cluster
agent-us-west-cluster   Synced       Healthy        us-west-cluster
agent-eu-cluster        Synced       Healthy        eu-cluster
```

**Setup**: Label clusters for ApplicationSet selection:

```bash
# Label each cluster with agent=enabled
argocd cluster list | grep -v 'local' | while read cluster; do
  kubectl config use-context "$cluster"
  kubectl label nodes -l karpenter.sh/capacity-type=on-demand \
    agent=enabled --overwrite
done
```

**Output when applied**:

```
$ kubectl apply -f agent-multi-cluster-applicationset.yaml
applicationset.argoproj.io/agent-multi-cluster created

$ argocd app list
NAME                    SYNC STATUS  HEALTH STATUS  CLUSTER
agent-us-east-cluster   Synced       Healthy        us-east-cluster
agent-us-west-cluster   Synced       Healthy        us-west-cluster
agent-eu-cluster        Synced       Healthy        eu-cluster
```

Three Applications created, one per registered cluster. Each deploys to its cluster server without duplicating the ApplicationSet.

## Generator 3: Matrix Generator (Environment × Region Combinations)

**The Matrix generator** combines two generators to create the Cartesian product. Perfect for deploying across environments AND regions.

### Use Case: Multi-Environment, Multi-Region

Your agent needs deployment across:
- **Environments**: dev, prod
- **Regions**: us, eu

That's 2 × 2 = 4 combinations, each with different configuration.

### ApplicationSet with Matrix Generator

```yaml
apiVersion: argoproj.io/v1alpha1
kind: ApplicationSet
metadata:
  name: agent-matrix
  namespace: argocd
spec:
  generators:
    # Matrix combines two generators
    - matrix:
        generators:
          # First generator: environments
          - list:
              elements:
                - environment: dev
                  replicas: "1"
                - environment: prod
                  replicas: "3"

          # Second generator: regions
          - list:
              elements:
                - region: us
                  zone: us-east-1
                - region: eu
                  zone: eu-west-1

  template:
    metadata:
      name: 'agent-{{environment}}-{{region}}'
      labels:
        environment: '{{environment}}'
        region: '{{region}}'
    spec:
      project: default
      source:
        repoURL: https://github.com/youragent/helm-charts
        targetRevision: HEAD
        path: agent
        helm:
          valuesInline:
            replicaCount: {{replicas}}
            environment: '{{environment}}'
            region: '{{region}}'
            zone: '{{zone}}'

      destination:
        server: https://kubernetes.default.svc
        namespace: agent-{{environment}}-{{region}}

      syncPolicy:
        automated:
          prune: true
          selfHeal: true
```

**Output when applied**:

```
$ kubectl apply -f agent-matrix-applicationset.yaml
applicationset.argoproj.io/agent-matrix created

$ argocd app list
NAME               SYNC STATUS  HEALTH STATUS  NAMESPACE
agent-dev-us       Synced       Healthy        agent-dev-us
agent-dev-eu       Synced       Healthy        agent-dev-eu
agent-prod-us      Synced       Healthy        agent-prod-us
agent-prod-eu      Synced       Healthy        agent-prod-eu
```

Matrix generated **four Applications** from two lists. Each combination has its own namespace and parameters. Update the generators, and ArgoCD maintains all four without separate ApplicationSet edits.

## Generator 4: Git Generator (Directory-Based Discovery)

**The Git generator** discovers Applications from directory structure in your Git repository. It automatically creates Applications for every directory matching a pattern.

### Use Case: Directory-Per-Environment

Your repository structure:

```
agent-helmcharts/
├── environments/
│   ├── dev/
│   │   └── values.yaml
│   ├── staging/
│   │   └── values.yaml
│   └── prod/
│       └── values.yaml
└── helm/
    └── agent/
        ├── Chart.yaml
        └── templates/
```

The Git generator discovers all three environment directories and creates Applications automatically.

### ApplicationSet with Git Generator

```yaml
apiVersion: argoproj.io/v1alpha1
kind: ApplicationSet
metadata:
  name: agent-git-dirs
  namespace: argocd
spec:
  generators:
    - git:
        repoURL: https://github.com/youragent/helm-charts
        revision: HEAD
        directories:
          # Match all directories under environments/
          - path: 'environments/*'
            # Extract the directory name as environment
            exclude: 'environments/templates'

  template:
    metadata:
      # Extract directory name: dev, staging, prod
      name: 'agent-{{path.basename}}'
      labels:
        environment: '{{path.basename}}'
    spec:
      project: default
      source:
        repoURL: https://github.com/youragent/helm-charts
        targetRevision: HEAD
        path: helm/agent  # Chart path
        helm:
          valuesFiles:
            # Use environment-specific values
            - '../environments/{{path.basename}}/values.yaml'

      destination:
        server: https://kubernetes.default.svc
        namespace: agent-{{path.basename}}

      syncPolicy:
        automated:
          prune: true
          selfHeal: true
```

**Output when applied** (ArgoCD scans the repo, discovers directories):

```
$ kubectl apply -f agent-git-dirs-applicationset.yaml
applicationset.argoproj.io/agent-git-dirs created

$ argocd app list
NAME           SYNC STATUS  HEALTH STATUS  NAMESPACE
agent-dev      Synced       Healthy        agent-dev
agent-staging  Synced       Healthy        agent-staging
agent-prod     Synced       Healthy        agent-prod
```

**Advantage**: Add a new environment directory in Git, and ArgoCD automatically creates a new Application. No ApplicationSet edits needed—pure GitOps.

## Complete Example: Part 6 Agent with List Generator

Here's a production-ready ApplicationSet for your FastAPI agent across three environments:

```yaml
apiVersion: argoproj.io/v1alpha1
kind: ApplicationSet
metadata:
  name: part6-agent
  namespace: argocd
spec:
  generators:
    - list:
        elements:
          - name: part6-agent-dev
            env: dev
            replicas: 1
            image_tag: latest
            api_workers: 2
            db_pool_size: 5
            log_level: DEBUG
            feature_flags: "all_enabled"

          - name: part6-agent-staging
            env: staging
            replicas: 2
            image_tag: v1.0.0-rc.1
            api_workers: 4
            db_pool_size: 15
            log_level: INFO
            feature_flags: "production_only"

          - name: part6-agent-prod
            env: production
            replicas: 3
            image_tag: v1.0.0
            api_workers: 8
            db_pool_size: 30
            log_level: WARN
            feature_flags: "stable_only"

  template:
    metadata:
      name: '{{name}}'
      namespace: argocd
      labels:
        app: part6-agent
        environment: '{{env}}'
        part: "6"

    spec:
      project: default

      source:
        repoURL: https://github.com/yourorg/part6-agent
        targetRevision: HEAD
        path: helm/agent
        helm:
          releaseName: agent-{{env}}
          valuesInline:
            environment: '{{env}}'
            image:
              tag: '{{image_tag}}'
            replicaCount: {{replicas}}
            fastapi:
              workers: {{api_workers}}
            database:
              pool_size: {{db_pool_size}}
            logging:
              level: '{{log_level}}'
            features:
              enabled: '{{feature_flags}}'

      destination:
        server: https://kubernetes.default.svc
        namespace: agent-{{env}}

      syncPolicy:
        automated:
          prune: true
          selfHeal: true
        syncOptions:
          - CreateNamespace=true
          - RespectIgnoreDifferences=true
        retry:
          limit: 5
          backoff:
            duration: 5s
            factor: 2
            maxDuration: 3m
```

**Output**:

```bash
$ kubectl apply -f part6-agent-applicationset.yaml
applicationset.argoproj.io/part6-agent created

$ argocd app list
NAME                   SYNC STATUS  HEALTH STATUS  NAMESPACE
part6-agent-dev        Synced       Healthy        agent-dev
part6-agent-staging    Synced       Healthy        agent-staging
part6-agent-prod       Synced       Healthy        agent-prod

# Check individual app status
$ argocd app get part6-agent-prod
Name:               part6-agent-prod
Project:            default
Sync Policy:        Automated (Prune)
Sync Status:        Synced
Health Status:      Healthy
Repository:         https://github.com/yourorg/part6-agent
Revision:           v1.0.0
Path:               helm/agent
Helm Values:        environment=production, replicas=3, ...
Destination Server: https://kubernetes.default.svc
Destination Namespace: agent-prod
```

## Template Customization Per Environment

Sometimes you need more than just value changes—different templates or resource sets per environment. Use the `goTemplate` or `goText` in ApplicationSet for advanced templating.

### Example: Dev Gets No PDB, Prod Gets PodDisruptionBudget

```yaml
apiVersion: argoproj.io/v1alpha1
kind: ApplicationSet
metadata:
  name: agent-with-custom-resources
  namespace: argocd
spec:
  generators:
    - list:
        elements:
          - name: agent-dev
            env: dev
            include_pdb: "false"
          - name: agent-prod
            env: prod
            include_pdb: "true"

  template:
    metadata:
      name: '{{name}}'
    spec:
      source:
        repoURL: https://github.com/youragent/helm-charts
        path: agent
        helm:
          valuesInline:
            environment: '{{env}}'
            pdb_enabled: {{include_pdb}}
```

In your Helm chart's `values.yaml`:

```yaml
pdb_enabled: false

# templates/pdb.yaml
{{- if .Values.pdb_enabled }}
apiVersion: policy/v1
kind: PodDisruptionBudget
metadata:
  name: {{ include "agent.fullname" . }}
spec:
  minAvailable: 1
  selector:
    matchLabels:
      app: agent
{{- end }}
```

**Output:**
```
# Dev deployment (pdb_enabled: false)
$ kubectl get pdb -n agent-dev
# (No PodDisruptionBudget created)

# Prod deployment (pdb_enabled: true)
$ kubectl get pdb -n agent-prod
NAME        MIN AVAILABLE   AGE
agent-prod  1               2m
```

Dev deploys without PDB (fast iteration). Prod deploys with PDB (HA protection). No ApplicationSet changes—Helm templating handles it.

## Validation Checklist

Before syncing your ApplicationSet, verify:

```bash
# 1. ApplicationSet is valid YAML
$ kubectl apply -f applicationset.yaml --dry-run=client -o yaml | head -20
# Output: Shows valid YAML structure

# 2. Generators match data
$ kubectl get applicationsets agent-multi-env -o jsonpath='{.spec.generators}'
# Output: [{"list":{"elements":[...]}}]

# 3. Applications are created
$ kubectl get applications -n argocd | grep agent
NAME                   SYNC STATUS  HEALTH STATUS
part6-agent-dev        Synced       Healthy
part6-agent-staging    Synced       Healthy
part6-agent-prod       Synced       Healthy

# 4. Applications are syncing
$ argocd app list | grep agent
part6-agent-dev        Synced       Healthy        agent-dev
part6-agent-staging    Synced       Healthy        agent-staging
part6-agent-prod       Synced       Healthy        agent-prod

# 5. Each generated app has correct parameters
$ argocd app get part6-agent-prod --show-params
HELM VALUES
environment: production
replicaCount: 3
image.tag: v1.0.0
database.pool_size: 30
logging.level: WARN
```

**Output:**
All validation checks pass: ApplicationSet created 3 Applications, each synced and healthy, with correct environment-specific parameters.

## Try With AI

**Part 1: Generate ApplicationSet from Requirements**

You need to deploy your agent across: dev (1 replica, debug), staging (2 replicas, warnings only), prod (3 replicas, errors only).

Ask AI: "Create an ApplicationSet that deploys my FastAPI agent to three environments: dev with 1 replica and DEBUG logging, staging with 2 replicas and INFO logging, prod with 3 replicas and ERROR logging. Each environment should be in its own namespace (agent-dev, agent-staging, agent-prod). Use the Helm chart at path 'helm/agent' in the repository."

**Part 2: Critical Evaluation**

Review AI's response:
- Does the template use `{{.paramName}}` placeholders correctly?
- Does each list element have all parameters referenced in the template?
- Are namespace names generated correctly with the environment parameter?
- Does the sync policy enable automation?

**Part 3: Environment-Specific Adjustment**

Based on your evaluation, refine: "In production, I need PodDisruptionBudget and stricter resource requests (4 CPU, 4Gi memory). In dev, allow unlimited resources. How would you modify the ApplicationSet to support this without duplicating the template?"

**Part 4: Validation**

Ask AI: "Generate a kubectl commands sequence that applies this ApplicationSet and verifies all three Applications were created with correct parameters."

**Part 5: Final Check**

After applying in your cluster:
- Did ArgoCD create three Applications automatically?
- Does each app show correct environment in labels?
- Are all three syncing to their respective namespaces?
- Can you modify values in the list elements and have all apps update automatically?

